#include <navtk/inertial/mechanization_standard.hpp>

#include <navtk/errors.hpp>
#include <navtk/inertial/AidingAltData.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/inertial/inertial_functions.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/wgs84.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/*
 * Calculates a DCM that represents the change in orientation of the sensor frame over a period
 * of time.
 *
 * @param C_s_to_n0 Current inertial sensor-to-nav DCM.
 * @param r_e @see navtk::navutils::transverse_radius.
 * @param r_n @see navtk::navutils::meridian_radius.
 * @param alt0 The inertial's current ellipsoidal altitude, m.
 * @param cos_l Cosine of inertial latitude.
 * @param dt Delta time that dth is to be integrated over.
 * @param dth Vector3 of delta rotations over dt in the inertial sensor
 * frame, rad.
 * @param sin_l Sine of inertial latitude.
 * @param tan_l Tangent of inertial latitude.
 * @param v_ned0 Vector3 of inertial velocity at the start of
 * integration in the NED frame, m/s.
 * @param dcm_integration_method Method to use to integrate the delta
 * rotation.
 * @param omega Earth rotation rate, rad/s.
 *
 * @return 3x3 DCM Css' that rotates from the sensor frame s at t + dt
 * to the sensor frame s' at t.
 */
Matrix3 calc_rot_mat(const Matrix3& C_s_to_n0,
                     double r_e,
                     double r_n,
                     double alt0,
                     double cos_l,
                     double dt,
                     const Vector3& dth,
                     double sin_l,
                     double tan_l,
                     const Vector3& v_ned0,
                     DcmIntegrationMethods dcm_integration_method,
                     double omega = navutils::ROTATION_RATE) {

	auto sigma =
	    calc_rot_rate(C_s_to_n0, r_e, r_n, alt0, cos_l, dt, dth, sin_l, tan_l, v_ned0, omega) * dt;

	switch (dcm_integration_method) {
	case DcmIntegrationMethods::SIXTH_ORDER:
		return navutils::rot_vec_to_dcm(sigma);
	case DcmIntegrationMethods::EXPONENTIAL:
		return expm(navutils::skew(sigma));
	default:
		if (dcm_integration_method != DcmIntegrationMethods::FIRST_ORDER)
			log_or_throw<std::invalid_argument>("Unrecognized dcm_integration_method");

		// First order approximation by default.
		return eye(3) + navutils::skew(sigma);
	}
}

/*
 * Calculate the NED frame change in position from NED frame velocity.
 *
 * @param dt Delta time to integrate over.
 * @param v_ned0 NED frame velocity at the start of mechanization, m/s.
 * @param v_ned1 NED frame velocity at the end of the mechanization
 * period (result of integrating accelerations starting from v_ned0), m/s.
 * @param integration_method Method to use to integrate velocities into
 * position.
 *
 * @return Vector3 of NED frame delta positions, m.
 */
Vector3 integrate_position(double dt,
                           const Vector3& v_ned0,
                           const Vector3& v_ned1,
                           const Vector3& v_ned_prev,
                           IntegrationMethods integration_method) {
	switch (integration_method) {
	case IntegrationMethods::TRAPEZOIDAL:
		return (v_ned1 + v_ned0) / 2 * dt;

	case IntegrationMethods::SIMPSONS_RULE:
		return (v_ned_prev + 4 * v_ned0 + v_ned1) / 6 * dt;

	default:
		if (integration_method != IntegrationMethods::RECTANGULAR)
			log_or_throw<std::invalid_argument>("Unrecognized integration_method");

		// Default to rectangular case
		return v_ned0 * dt;
	}
}

/*
 * Calculates the measured specific force and actual acceleration from
 * imu measurements and location-based parameters.
 *
 * @param C_s_to_n0 The inertial's current sensor-to-NED DCM.
 * @param r_e @see navtk::navutils::transverse_radius.
 * @param r_n @see navtk::navutils::meridian_radius.
 * @param alt0 The inertial's current ellipsoidal altitude, m.
 * @param cos_l Cosine of inertial latitude.
 * @param dt Delta time that acceleration is to be integrated over (sec).
 * @param dth Vector3 of measured delta rotations over dt in the
 * inertial sensor frame, rad.
 * @param dv Vector3 of measured delta velocities over dt in the
 * inertial sensor frame, m/s.
 * @param g Estimated local gravity along North, East and Down axes, m/s^2.
 * @param sec_l Secant of inertial latitude.
 * @param sin_l Sine of inertial latitude.
 * @param v_ned0 Vector3 of inertial velocity in the NED frame, m/s.
 * @param omega Earth rotation rate, rad/s.
 *
 * @return Pair of Vector3 of specific forces and 3x1 matrix of
 * accelerations, both in NED frame and m/s^2.
 */
std::pair<Vector3, Vector3> calc_acceleration(const Matrix3& C_s_to_n0,
                                              double r_e,
                                              double r_n,
                                              double alt0,
                                              double cos_l,
                                              double dt,
                                              const Vector3& dth,
                                              const Vector3& dv,
                                              Vector3 g,
                                              double sec_l,
                                              double sin_l,
                                              const Vector3& v_ned0,
                                              double omega = navutils::ROTATION_RATE) {
	auto f_ned = calc_force_ned(C_s_to_n0, dt, dth, dv);

	Vector3 offset =
	    calc_force_and_acceleration_offset(r_e, r_n, alt0, cos_l, g, sec_l, sin_l, v_ned0, omega);
	Vector3 accel_ned = f_ned + offset;

	return {f_ned, accel_ned};
}

std::tuple<Vector3, Vector3, Matrix3> mechanization_standard(
    const Vector3& dv_s,
    const Vector3& dth_s,
    double dt,
    const Vector3& llh0,
    const Matrix3& C_s_to_n0,
    const Vector3& v_ned0,
    const Vector3& v_ned_prev,
    const MechanizationOptions& mech_options,
    AidingAltData* aiding_alt_data) {

	// Earth rotation rate in rad/s, also in wgs84.hpp, where it has
	// a slightly different value. The one used here is hardcoded in the
	// Kotlin version of this mechanization function, and is currently
	// repeated here for consistency. See wgs84.cpp for sources of values.
	double omega = 7.292115e-5;

	auto lat0 = llh0[0];
	auto lon0 = llh0[1];
	auto alt0 = llh0[2];

	// Didn't implement time checks that were in original Scorpion (for now)

	auto sin_l = sin(lat0);
	// Commenting- unused var err
	// auto sin2L = sinL * sinL;
	auto cos_l = cos(lat0);
	auto sec_l = 1 / cos_l;
	auto tan_l = sin_l / cos_l;

	// Take into account earth is elliptical ... Equation 3.82, pg 54, Titterton text
	auto r_n = navutils::meridian_radius(lat0);
	auto r_e = navutils::transverse_radius(lat0);

	// Commenting- unused var err
	auto r_zero = sqrt(r_n * r_e);

	Vector3 g;

	switch (mech_options.grav_model) {
	case navutils::GravModels::SCHWARTZ: {
		g = navutils::calculate_gravity_schwartz(alt0, lat0);
		break;
	}
	case navutils::GravModels::SAVAGE: {
		auto C_n_to_e = navutils::lat_lon_wander_to_C_n_to_e(lat0, lon0, 0.0);
		g             = navutils::calculate_gravity_savage_ned(C_n_to_e, alt0);
		break;
	}
	default:
		if (mech_options.grav_model != navutils::GravModels::TITTERTON)
			log_or_throw<std::invalid_argument>("Invalid GravModels enum value supplied.");

		// Default to Titterton model
		g = navutils::calculate_gravity_titterton(alt0, lat0, r_zero);
	}

	// Propagate DCM forward ...
	Matrix3 A = calc_rot_mat(C_s_to_n0,
	                         r_e,
	                         r_n,
	                         alt0,
	                         cos_l,
	                         dt,
	                         dth_s,
	                         sin_l,
	                         tan_l,
	                         v_ned0,
	                         mech_options.dcm_method,
	                         omega);

	// Perform actual DCM integration (Eqn 10.4)
	Matrix3 C_s_to_n1 = navtk::dot(C_s_to_n0, A);

	// Calculate rates of change of Lat and Long... Equation 3.82, pg 54, Titterton text
	auto accel_pair = calc_acceleration(
	    C_s_to_n0, r_e, r_n, alt0, cos_l, dt, dth_s, dv_s, g, sec_l, sin_l, v_ned0, omega);
	auto& accel_ned = accel_pair.second;

	// Apply aiding altitude
	double c1 = 0;
	if (aiding_alt_data != nullptr)
		c1 = apply_aiding_alt_accel(r_zero, &accel_ned, aiding_alt_data, alt0, dt, g);

	// Integrate once to get the velocities
	Vector3 v_ned1 = xt::fma(accel_ned, dt, v_ned0);

	// Integrate again to get position
	auto delta_pos_ned =
	    integrate_position(dt, v_ned0, v_ned1, v_ned_prev, mech_options.int_method);

	// Adjust the vertical channel position by adding the aiding factor.
	if (aiding_alt_data != nullptr)
		delta_pos_ned[2] += c1 * (alt0 - aiding_alt_data->aiding_alt) * dt;

	// Update latitude
	auto lat1 = lat0 + delta_pos_ned[0] / (r_n + alt0);
	// Update longitude
	auto lon1 = lon0 + delta_pos_ned[1] * sec_l / (r_e + alt0);
	// Update height
	auto alt1 = alt0 - delta_pos_ned[2];

	Vector3 llh1{lat1, lon1, alt1};

	return std::make_tuple(llh1, v_ned1, C_s_to_n1);
}

not_null<std::shared_ptr<InertialPosVelAtt>> mechanization_standard(
    const Vector3& dv_s,
    const Vector3& dth_s,
    double dt,
    const not_null<std::shared_ptr<InertialPosVelAtt>> pva,
    const not_null<std::shared_ptr<InertialPosVelAtt>> old_pva,
    const MechanizationOptions& mech_options,
    AidingAltData* aiding_alt_data) {

	auto tup = mechanization_standard(dv_s,
	                                  dth_s,
	                                  dt,
	                                  pva->get_llh(),
	                                  pva->get_C_s_to_ned(),
	                                  pva->get_vned(),
	                                  old_pva->get_vned(),
	                                  mech_options,
	                                  aiding_alt_data);

	return std::make_shared<StandardPosVelAtt>(pva->time_validity + dt, tup);
}

}  // namespace inertial
}  // namespace navtk
