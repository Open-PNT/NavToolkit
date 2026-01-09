#include <gtest/gtest.h>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/stateblocks/EarthModel.hpp>
#include <navtk/filtering/stateblocks/Pinson21NedBlock.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using navtk::cross;
using navtk::dot;
using navtk::Matrix;
using navtk::Matrix3;
using navtk::Vector;
using navtk::Vector3;
using navtk::filtering::calc_numerical_jacobian;
using navtk::filtering::NavSolution;
using navtk::navutils::skew;
using std::cos;
using std::sin;
using std::tan;

/*
 * Corrupts a NavSolution by adding errors.
 *
 * @param ns Uncorrected/raw NavSolution.
 * @param xx Pinson15Ned error state vector.
 *
 * @return NavSolution with errors added.
 */
NavSolution corrupt(NavSolution ns, Vector xx) {
	auto cor_lat = ns.pos[0] + navtk::navutils::north_to_delta_lat(xx[0], ns.pos[0], ns.pos[2]);
	auto cor_lon = ns.pos[1] + navtk::navutils::east_to_delta_lon(xx[1], ns.pos[0], ns.pos[2]);
	auto cor_alt = ns.pos[2] - xx[2];
	auto cor_vel = ns.vel + xt::view(xx, xt::range(3, 6));
	auto cor_cnb =
	    dot((navtk::eye(3) - skew(xt::view(xx, xt::range(6, 9)))), xt::transpose(ns.rot_mat));
	auto cor_ns = NavSolution(Vector3{cor_lat, cor_lon, cor_alt},
	                          cor_vel,
	                          xt::transpose(cor_cnb),
	                          aspn_xtensor::TypeTimestamp((int64_t)0));
	return cor_ns;
}

/*
 * Generates the F matrix (Jacobian of the continuous time propagation function/differential
 * equations) from a Pinson15Ned StateBlock using the supplied aux data.
 *
 * @param aux AspnBaseVector to use as linearization point.
 *
 * @return 15x15 Jacobian of the continuous time propagation function.
 */
Matrix calc_pinson_f21(aspn_xtensor::MeasurementPositionVelocityAttitude pva_aux,
                       aspn_xtensor::MeasurementImu f_and_r_aux) {
	auto mod = navtk::filtering::hg1700_model();
	auto p21 = navtk::filtering::Pinson21NedBlock("p21", mod);
	return p21.generate_f_pinson(pva_aux, f_and_r_aux);
}

/*
 * Test case for evaluating correctness of various Pinson propagation elements, specifically the F
 * Jacobian Matrix (which is transcribed from a book) against other calculations of the same,
 * usually from numerical methods. Equation methods throughout file are from Titterton and Weston,
 * Strapdown Inertial Navigation Technology, 2nd ed unless otherwise noted.
 */
struct Pinson21NonlinearTests : public ::testing::Test {
	/* Pinson21 error state vector, subtractive error definition */
	Vector x21;
	/* Raw specific forces, NED frame, m/s^2 */
	Vector3 fned_true;
	/* Raw body rotation rates, in the body/ins frame, rad/s */
	Vector3 w_b_ib_true;
	/* Nominal/raw ins position, LLA, rad/rad/m */
	Vector3 pos_true;
	/* Nominal/raw ins velocity, NED frame, m/s */
	Vector3 vned_true;
	/* Nominal/raw ins roll, pitch, yaw, radians */
	Vector3 rpy_true;
	/* Nominal/raw ins body to nav DCM (from rpy) */
	Matrix3 cnb_true;
	/* Nominal/raw ins NavSolution, containing pos, vned, rpy, C_platform_to_nav */
	NavSolution ns_true;
	/* AspnBaseVector created by passing ns, fned and x21 through the 'correct' function to remove
	 * error.
	 */
	aspn_xtensor::MeasurementPositionVelocityAttitude pva_aux_true;
	aspn_xtensor::MeasurementImu f_and_r_aux_true;
	/* The 21x21 F matrix generate by the Pinson15Ned StateBlock when linearized about cor_aux */
	Matrix jac21;

	Pinson21NonlinearTests()
	    : ::testing::Test(),
	      x21({1.0,  3.0,  -14.0, 0.1,   -1.3, 1.1,  -3e-4, 4e-4,   2e-4,   3e-6,   -4e-5,
	           1e-6, 3e-9, -4e-8, -2e-7, 1e-1, 5e-2, 1e-3,  -40e-6, 375e-6, 1000e-6}),
	      fned_true({1e-5, 2e-4, -9.79}),
	      w_b_ib_true({-2e-6, 4e-4, 1e-5}),
	      pos_true({0.7, -1.4, 123.0}),
	      vned_true({1.0, -0.5, 6.7}),
	      rpy_true({1.0, 2.0, 2.2}),
	      cnb_true(navtk::navutils::rpy_to_dcm(rpy_true)),
	      ns_true(NavSolution(pos_true,
	                          vned_true,
	                          xt::transpose(cnb_true),
	                          aspn_xtensor::TypeTimestamp((int64_t)0))),
	      pva_aux_true(navtk::utils::to_positionvelocityattitude(ns_true)),
	      f_and_r_aux_true(
	          navtk::utils::to_imu(pva_aux_true.get_time_of_validity(), fned_true, w_b_ib_true)),
	      jac21(calc_pinson_f21(pva_aux_true, f_and_r_aux_true)) {}
};

/*
 * Combines the pos_prop, vel_prop and dcm_prop functions.
 *
 * @param ns PVA process to integrate.
 * @param fb Specific force input, in body frame m/s^2.
 * @param w_b_ib Body rotation rate input, in body frame rad/s.
 *
 * @return A NavSolution holding p_dot (NED frame), v_dot and cbn_dot (use of NavSolution container
 * is an abuse; result is not actually a valid NavSolution).
 */
NavSolution tstraight_prop(NavSolution ns, Vector3 fb, Vector3 w_b_ib) {
	Vector ned_dot = ns.vel;
	auto em        = navtk::filtering::EarthModel(ns.pos, ns.vel);
	Vector v_dot   = dot(xt::transpose(ns.rot_mat), fb) -
	               cross(2.0 * em.omega_ie_n + em.omega_en_n, ns.vel) + em.g_n;
	auto cnb        = xt::transpose(ns.rot_mat);
	Matrix3 cnb_dot = dot(cnb, skew(w_b_ib)) - dot(skew(em.omega_in_n), cnb);

	return NavSolution(
	    ned_dot, v_dot, xt::transpose(cnb_dot), aspn_xtensor::TypeTimestamp((int64_t)0));
}

TEST_F(Pinson21NonlinearTests, check_linearization_error_SLOW) {

	auto fx = [cnb_true    = cnb_true,
	           ns_true     = ns_true,
	           fned_true   = fned_true,
	           w_b_ib_true = w_b_ib_true](Vector xx) {
		// First, convert fned to body frame
		Vector fb_true = dot(xt::transpose(cnb_true), fned_true);

		// Generate corrupted force and rate measurements by scaling and adding biases
		Vector accel_bias = xt::view(xx, xt::range(9, 12));
		Vector gyro_bias  = xt::view(xx, xt::range(12, 15));
		Vector accel_sf   = 1.0 + xt::view(xx, xt::range(15, 18));
		Vector gyro_sf    = 1.0 + xt::view(xx, xt::range(18, 21));

		Vector fb_est     = fb_true * accel_sf + accel_bias;
		Vector w_b_ib_est = w_b_ib_true * gyro_sf + gyro_bias;

		// Generate estimated PVA
		NavSolution ns_est = corrupt(ns_true, xx);

		// Calculate 'pva_dot' for both true and estimated pva/measurements (See 12.12,
		// 12.13, 12.18, 12.19)
		NavSolution ns_prop_est  = tstraight_prop(ns_est, fb_est, w_b_ib_est);
		NavSolution ns_prop_true = tstraight_prop(ns_true, fb_true, w_b_ib_true);

		// Diff pos and vel results to get 'delta_dot' (12.20, 12.23)
		Vector pos_dot_diff = ns_prop_est.pos - ns_prop_true.pos;
		Vector vel_dot_diff = ns_prop_est.vel - ns_prop_true.vel;

		// DCMs are handled differently (eq. 12.11)
		Matrix cnb_dot_diff = -dot(xt::transpose(ns_prop_est.rot_mat), ns_true.rot_mat) -
		                      dot(xt::transpose(ns_est.rot_mat), ns_prop_true.rot_mat);

		// Pull out the skew elements to get tilt_dot in vector form
		Vector tilts{cnb_dot_diff(2, 1), cnb_dot_diff(0, 2), cnb_dot_diff(1, 0)};

		// Biases are FOGM, so they just prop as 1/tau; delta form does as well
		auto mod               = navtk::filtering::hg1700_model();
		Vector accel_bias_prop = -xt::view(xx, xt::range(9, 12)) / mod.accel_bias_tau;
		Vector gyro_bias_prop  = -xt::view(xx, xt::range(12, 15)) / mod.gyro_bias_tau;

		// Whole 'x_dot', with zeros at end for constant scale factors
		Vector complete = xt::concatenate(xt::xtuple(
		    pos_dot_diff, vel_dot_diff, tilts, accel_bias_prop, gyro_bias_prop, navtk::zeros(6)));
		return complete;
	};

	// Calculate delta_x_dot
	auto full = fx(x21);

	// Linearized; Pinson F * dx
	Vector linearized = dot(jac21, x21);
	Vector diff       = full - linearized;

	// Pull out error states for convenience
	Vector dv    = xt::view(x21, xt::range(3, 6));
	Vector tilts = xt::view(x21, xt::range(6, 9));
	Vector dfb   = xt::view(x21, xt::range(9, 12));
	Vector dgb   = xt::view(x21, xt::range(12, 15));
	Vector sfb   = xt::view(x21, xt::range(15, 18));
	Vector sgb   = xt::view(x21, xt::range(18, 21));


	Vector fb_true = dot(xt::transpose(cnb_true), fned_true);
	auto ns_est    = corrupt(ns_true, x21);
	auto em_est    = navtk::filtering::EarthModel(ns_est.pos, ns_est.vel);
	auto em_true   = navtk::filtering::EarthModel(ns_true.pos, ns_true.vel);

	Vector fb_est           = fb_true * (1.0 + sfb) + dfb;
	Vector w_b_ib_est       = w_b_ib_true * (1.0 + sgb) + dgb;
	Vector delta_omega_n_in = em_est.omega_in_n - em_true.omega_in_n;
	Vector d_w_n_ie         = em_est.omega_ie_n - em_true.omega_ie_n;
	Vector d_w_n_en         = em_est.omega_en_n - em_true.omega_en_n;

	// Determine what linearization error should be by calculating each of the terms that do not
	// appear in Pinson due to either being dropped as error products or simply not accounted for
	// yet. dv_dot = C_platform_to_nav * (diag(accel_scale) * f_b) + C_platform_to_nav * df_b +
	// (C_platform_to_nav * f_b) X tilts
	// + (C_platform_to_nav * diag(accel_scale) * f_b) X tilts - skew(tilts) * C_platform_to_nav *
	// df_b First term is new due to the addition of scale factor state and has been added to
	// Pinson21. Next 2 are already accounted for in the Pinson15 model; 4th term is new due to
	// scale and dropped as error product (scale * tilt), and last term was already present but also
	// dropped as error term.
	Vector expected_vel_err_t1 = -dot(dot(skew(tilts), cnb_true), sfb * fb_true);

	Vector expected_vel_err_t2 = -dot(skew(tilts), dot(cnb_true, dfb));

	Vector expected_vel_err_t3 = dot(-skew(2.0 * d_w_n_ie + d_w_n_en), dv);

	Vector errs = expected_vel_err_t1 + expected_vel_err_t2 + expected_vel_err_t3;

	Vector t1       = dot(cnb_true, sfb * fb_true);
	Vector t2       = dot(cnb_true, dfb);
	Vector t3       = -dot(skew(tilts), dot(cnb_true, fb_true));
	Vector t4       = -dot(skew(2 * em_true.omega_ie_n + em_true.omega_en_n), dv);
	Vector t5       = em_est.g_n - em_true.g_n;
	Vector t6       = dot(-skew(2.0 * d_w_n_ie + d_w_n_en), ns_true.vel);
	Vector inc_term = t1 + t2 + t3 + t4 + t5 + t6;

	Vector full_term = inc_term + errs;

	Vector vel_compact = dot(xt::transpose(ns_est.rot_mat), fb_est) -
	                     dot(xt::transpose(ns_true.rot_mat), fb_true) -
	                     dot(skew(2.0 * em_est.omega_ie_n + em_est.omega_en_n), ns_est.vel) +
	                     dot(skew(2.0 * em_true.omega_ie_n + em_true.omega_en_n), ns_true.vel) +
	                     em_est.g_n - em_true.g_n;

	ASSERT_ALLCLOSE(xt::view(diff, xt::range(3, 6)), errs);

	Vector expected_tilt_err_t1 = dot(dot(skew(tilts), cnb_true), sgb * w_b_ib_true);
	Vector expected_tilt_err_t2 = dot(dot(skew(tilts), cnb_true), dgb);
	Matrix tilt_err_t3_mat      = -dot(skew(delta_omega_n_in), skew(tilts));
	Vector expected_tilt_err_t3 = {
	    tilt_err_t3_mat(2, 1), tilt_err_t3_mat(0, 2), tilt_err_t3_mat(0, 1)};

	auto all_tilt_err = expected_tilt_err_t1 + expected_tilt_err_t2 + expected_tilt_err_t3;

	ASSERT_ALLCLOSE(xt::view(diff, xt::range(6, 9)), all_tilt_err);

	Vector lin_err =
	    xt::concatenate(xt::xtuple(navtk::zeros(3), errs, all_tilt_err, navtk::zeros(12)));

	auto num_jac = calc_numerical_jacobian(fx, x21);

	ASSERT_ALLCLOSE(linearized + lin_err, full);

	// PNTOS-331, should be +lin_err
	ASSERT_ALLCLOSE(dot(num_jac, x21) - lin_err, full);
}
