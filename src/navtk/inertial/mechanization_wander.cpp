#include <navtk/inertial/mechanization_wander.hpp>

#include <navtk/errors.hpp>
#include <navtk/inertial/Inertial.hpp>
#include <navtk/inertial/WanderPosVelAtt.hpp>
#include <navtk/inertial/inertial_functions.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/wgs84.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

using navtk::navutils::OMF2;
using navtk::navutils::OMF4;
using navtk::navutils::SEMI_MAJOR_RADIUS;

/*
 * Calculates the curvature of the N frame at a given location, as
 * described in Savage Vol 1, eq 7.3.1-10 and 5.3-18.
 *
 * @param C_n_to_e N frame to E frame DCM.
 * @param h Ellipsoidal height, meters.
 * @param earth_model How to model the earth when constructing the matrix.
 * Defaults to EarthModels::ELLIPTICAL.
 */
Matrix3 curvature_matrix(const Matrix3& C_n_to_e, double h, EarthModels earth_model) {

	auto fc_n = zeros(3, 3);

	// Eqn 5.1-10
	auto u_up_ye  = C_n_to_e(1, 2);  // y-component of up unit vector in E frame
	auto rs_prime = SEMI_MAJOR_RADIUS / sqrt(1.0 + u_up_ye * u_up_ye * (OMF2 - 1.0));

	switch (earth_model) {
	case EarthModels::SPHERICAL: {  // Calculate radius from center of earth to inertial location
		// Eqn 5.2.1-4
		auto rs = rs_prime * sqrt(1 + u_up_ye * u_up_ye * (OMF4 - 1));

		// From description of FcN from Savage Vol 1, eq 4.1.1-6
		// Calculation only valid when h is an ellipsoidal height
		// (because Rs is magnitude of ellipsoidal pos)
		fc_n(0, 0) = 1.0 / (rs + h);
		fc_n(1, 1) = 1.0 / (rs + h);

		return fc_n;
	}
	default:
		// Default to elliptical case
		if (earth_model != EarthModels::ELLIPTICAL)
			log_or_throw<std::invalid_argument>("Unrecognized earth_model");

		// Savage Vol 1, eq 5.2.4-25
		auto r_ls = OMF2 * pow(rs_prime, 3) / (SEMI_MAJOR_RADIUS * SEMI_MAJOR_RADIUS);

		// Savage Vol 1, eq 5.2.4-37
		auto r_l = r_ls + h;

		// Savage Vol 1, eq 5.3-18
		auto D21 = C_n_to_e(1, 0);
		auto D22 = C_n_to_e(1, 1);
		auto D23 = C_n_to_e(1, 2);

		auto fe  = (OMF2 - 1) / (1 + D23 * D23 * (OMF2 - 1));
		auto fh  = 1 / (1 + h / rs_prime);
		auto feh = fe * fh;

		fc_n(0, 0) = (1 + D21 * D21 * feh) / r_l;
		fc_n(0, 1) = D21 * D22 * feh / r_l;
		fc_n(1, 0) = fc_n(0, 1);
		fc_n(1, 1) = (1 + D22 * D22 * feh) / r_l;

		return fc_n;
	}
}

std::tuple<Matrix3, double, Vector3, Matrix3> mechanization_wander(
    const Vector3& dv_s,
    const Vector3& dth_s,
    double dt,
    const Matrix3& C_n_to_e_0,
    double h0,
    const Vector3& v_n_0,
    const Matrix3& C_s_to_l_0,
    const MechanizationOptions& mech_options,
    AidingAltData* aiding_alt_data) {

	// Matrix that relates  n to l wander frames (or equivalently ENU to NED).
	// Swaps x and y axis and negates the z.
	Matrix3 C_n_to_l{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}};

	// #########################
	//  Attitude Update
	// #########################
	//  Calculate sensor rotation as measured in sensor frame
	auto C_s0_to_s1 = navutils::rot_vec_to_dcm(dth_s);

	// Calculate rotation of l frame

	// Earth rotation rate in the n frame (Eqns 4.1.1-3 and 4.1.1-4)
	// TODO: PNTOS-250 This rate is slightly different from the hardcoded one
	// in the Titterton mechanization, which is being left as-is for now
	// as it matches the old Kotlin version and changing it breaks tests
	// Those tests could be re-accomplished with the different rate as
	// input
	Vector3 w_ie_e{0, navutils::ROTATION_RATE, 0};
	Vector3 w_ie_n = dot(xt::transpose(C_n_to_e_0), w_ie_e);

	// Savage Vol 1 eq 7.1.1.2.1-1 (rho_zN = 0 because wander azimuth)
	auto fc_n = curvature_matrix(C_n_to_e_0, h0, mech_options.earth_model);

	Vector3 u_zn_n{0, 0, 1};                           // Savage Vol 1 eq 5.3-17
	Vector3 w_en_n = dot(fc_n, cross(u_zn_n, v_n_0));  // Same, 4.1.1-6, rho_zN = 0

	Vector3 w_in_n = w_ie_n + w_en_n;
	Vector3 w_il_l = dot(C_n_to_l, w_in_n);

	// Convert to incremental change
	Vector3 zeta_n = w_il_l * dt;

	// Calculate l frame rotation (Savage Vol 1 eq 7.1.1.2-3, noting sign change to
	// account for opposite rotation sense)
	auto C_l0_to_l1 = navutils::rot_vec_to_dcm(-zeta_n);

	// Final update of attitude DCM, using Savage Vol 1, eq 7.1.1-1
	auto C_s_to_l_1 = dot(C_l0_to_l1, dot(C_s_to_l_0, C_s0_to_s1));

	// #########################
	//  Velocity Update
	// #########################
	Vector3 dv_n = dot(C_n_to_l, dot(C_s_to_l_0, dv_s));

	Vector3 gp_n{0.0, 0.0, 0.0};

	// Variable declarations for altitude aiding function calls
	Vector3 g     = zeros(3);
	auto llw      = navutils::C_n_to_e_to_lat_lon_wander(C_n_to_e_0);
	auto r_n      = navutils::meridian_radius(llw[0]);
	auto r_e      = navutils::transverse_radius(llw[0]);
	double r_zero = sqrt(r_n * r_e);

	switch (mech_options.grav_model) {
	case navutils::GravModels::SCHWARTZ: {
		g               = navutils::calculate_gravity_schwartz(h0, llw[0]);
		auto C_ned_to_n = navutils::wander_to_C_ned_to_n(llw[2]);
		gp_n            = dot(C_ned_to_n, g);
		break;
	}
	case navutils::GravModels::SAVAGE: {
		gp_n = navutils::calculate_gravity_savage_n(C_n_to_e_0, h0);
		break;
	}
	default:
		if (mech_options.grav_model != navutils::GravModels::TITTERTON)
			log_or_throw<std::invalid_argument>("Unknown gravity model");

		// Default to Titterton case.
		g               = navutils::calculate_gravity_titterton(h0, llw[0], r_zero);
		auto C_ned_to_n = navutils::wander_to_C_ned_to_n(llw[2]);
		gp_n            = dot(C_ned_to_n, g);
	}

	// Savage Vol 1 eq 7.2-3
	auto dv_grav_corr_n = (gp_n - cross(w_en_n + 2.0 * w_ie_n, v_n_0)) * dt;

	// Apply aiding altitude
	double c1 = 0;
	Vector3 v_n_1;

	if (aiding_alt_data != nullptr) {
		// Derive the acceleration for the current mechanization iteration.
		Vector3 aided_accel = (dv_n + dv_grav_corr_n) / dt;
		// Switch orientation of the acceleration vectors to match the NED frame used in the aiding
		// algorithm. From vertical is UP, to vertical is DOWN.
		aided_accel[2]  = -aided_accel[2];
		auto aided_gp_n = -gp_n;
		c1 = apply_aiding_alt_accel(r_zero, &aided_accel, aiding_alt_data, h0, dt, aided_gp_n);
		// Re-orient back to the original navigation frame, ENU for wander_mech.
		aided_accel[2] = -aided_accel[2];
		v_n_1          = v_n_0 + aided_accel * dt;
	} else {
		// Savage Vol 1 eq 7.2-2
		v_n_1 = v_n_0 + dv_n + dv_grav_corr_n;
	}

	// ###########################
	//  Position Update
	// ###########################

	// Savage Vol 1 eq 7.3.2-1
	// TODO: PNTOS-251 This assumes trapezoidal- we could implement others just as in T+W
	// Section 7.3.3 gives a high-res update algorithm that appears quite complicated
	Vector3 delta_r_n = 0.5 * (v_n_1 + v_n_0) * dt;

	// Adjust the vertical channel position by subtracting the aiding factor, since the coordinate
	// frame has an upward vertical channel, and the algorithm is based on a downward one.
	if (aiding_alt_data != nullptr) delta_r_n[2] -= c1 * (h0 - aiding_alt_data->aiding_alt) * dt;

	// Savage Vol 1 eq 7.3.1-11 (simple form, rho_zn = 0)
	Vector3 u_cross_r{-delta_r_n[1], delta_r_n[0], 0.0};
	zeta_n = dot(fc_n, u_cross_r);

	// Eqn 7.3.1-8
	auto C_n1_to_n0 = navutils::rot_vec_to_dcm(zeta_n);

	// Savage Vol 1 eq 7.3.1-6
	auto C_n_to_e_1 = dot(C_n_to_e_0, C_n1_to_n0);

	// Savage Vol 1 eq 7.3.1-1
	auto h1 = h0 + delta_r_n[2];

	return std::make_tuple(C_n_to_e_1, h1, v_n_1, C_s_to_l_1);
}

not_null<std::shared_ptr<InertialPosVelAtt>> mechanization_wander(
    const Vector3& meas_accel,
    const Vector3& meas_gyro,
    double dt,
    const not_null<std::shared_ptr<InertialPosVelAtt>> pva,
    const not_null<std::shared_ptr<InertialPosVelAtt>>,
    const MechanizationOptions& mech_options,
    AidingAltData* aiding_alt_data) {
	if (!pva->is_wander_capable())
		log_or_throw<std::invalid_argument>(
		    "Input pva must have a valid wander angle representation.");

	auto pos_tup = pva->get_C_n_to_e_h();
	auto tup     = mechanization_wander(meas_accel,
                                    meas_gyro,
                                    dt,
                                    std::get<0>(pos_tup),
                                    std::get<1>(pos_tup),
                                    pva->get_vn(),
                                    pva->get_C_s_to_l(),
                                    mech_options,
                                    aiding_alt_data);

	return std::make_shared<WanderPosVelAtt>(pva->time_validity + dt,
	                                         std::get<0>(tup),
	                                         std::get<1>(tup),
	                                         std::get<2>(tup),
	                                         std::get<3>(tup));
}
}  // namespace inertial
}  // namespace navtk
