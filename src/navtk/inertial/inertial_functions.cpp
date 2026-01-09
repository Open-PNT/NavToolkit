#include <navtk/inertial/inertial_functions.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/stateblocks/EarthModel.hpp>
#include <navtk/filtering/stateblocks/GravityModelSchwartz.hpp>
#include <navtk/inertial/AidingAltData.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/utils/conversions.hpp>

namespace navtk {
namespace inertial {

Vector3 calc_force_ned(const Matrix3& C_s_to_n, double dt, const Vector3& dth, const Vector3& dv) {

	/*
	// 'Normal' implementation. Long chains of xtensor related operations can cause large slowdowns,
	// especially w/ ASAN testing, so actual implementation does math 'manually' to achieve speed.

	 auto rot_corr = 0.5 * Vector3{dth[1] * dv[2] - dv[1] * dth[2],
	                               dth[2] * dv[0] - dv[2] * dth[0],
	                               dth[0] * dv[1] - dv[0] * dth[1]};

	 return dot(C_s_to_n, dv + rot_corr) / dt;
	 */

	double dv0  = dv[0];
	double dv1  = dv[1];
	double dv2  = dv[2];
	double dth0 = dth[0];
	double dth1 = dth[1];
	double dth2 = dth[2];

	return {(C_s_to_n(0, 0) * (dv0 + 0.5 * (dth1 * dv2 - dv1 * dth2)) +
	         C_s_to_n(0, 1) * (dv1 + 0.5 * (dth2 * dv0 - dv2 * dth0)) +
	         C_s_to_n(0, 2) * (dv2 + 0.5 * (dth0 * dv1 - dv0 * dth1))) /
	            dt,
	        (C_s_to_n(1, 0) * (dv0 + 0.5 * (dth1 * dv2 - dv1 * dth2)) +
	         C_s_to_n(1, 1) * (dv1 + 0.5 * (dth2 * dv0 - dv2 * dth0)) +
	         C_s_to_n(1, 2) * (dv2 + 0.5 * (dth0 * dv1 - dv0 * dth1))) /
	            dt,
	        (C_s_to_n(2, 0) * (dv0 + 0.5 * (dth1 * dv2 - dv1 * dth2)) +
	         C_s_to_n(2, 1) * (dv1 + 0.5 * (dth2 * dv0 - dv2 * dth0)) +
	         C_s_to_n(2, 2) * (dv2 + 0.5 * (dth0 * dv1 - dv0 * dth1))) /
	            dt};
}

Vector3 calc_rot_rate(const Matrix3& C_s_to_n0,
                      double r_e,
                      double r_n,
                      double alt0,
                      double cos_l,
                      double dt,
                      const Vector3& dth,
                      double sin_l,
                      double tan_l,
                      const Vector3& v_ned0,
                      double omega) {
	/*
	// 'Normal' implementation. Long chains of xtensor related operations can cause large slowdowns,
	// especially w/ ASAN testing, so actual implementation does math 'manually' to achieve speed.
	Vector3 omega_en_n{
	    v_ned0[1] / (r_e + alt0), -v_ned0[0] / (r_n + alt0), -v_ned0[1] * tan_l / (r_e + alt0)};

	// Calculate earth turn rate experienced, omega_ie, in the navigation frame ... remember, this
	is a
	// rate Equation 3.72, pg 51, Titterton text
	Vector3 omega_ie_n{omega * cos_l, 0, -omega * sin_l};

	// Remove these effects so that we now have delta_theta as rotations of the sensor to nav
	frame
	// Titterton & Weston Eqn 3.29, p 36
	// Note: raw delta_thetas are actually omega_ib_b
	return dth / dt - dot(xt::transpose(C_s_to_n0), omega_ie_n + omega_en_n);
	*/


	double o1 = v_ned0[1] / (r_e + alt0) + omega * cos_l;
	double o2 = -v_ned0[0] / (r_n + alt0);
	double o3 = -v_ned0[1] * tan_l / (r_e + alt0) - omega * sin_l;

	return Vector3{
	    dth[0] / dt - C_s_to_n0(0, 0) * o1 - C_s_to_n0(1, 0) * o2 - C_s_to_n0(2, 0) * o3,
	    dth[1] / dt - C_s_to_n0(0, 1) * o1 - C_s_to_n0(1, 1) * o2 - C_s_to_n0(2, 1) * o3,
	    dth[2] / dt - C_s_to_n0(0, 2) * o1 - C_s_to_n0(1, 2) * o2 - C_s_to_n0(2, 2) * o3};
}

Vector3 calc_rot_rate(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva,
                      double dt,
                      const Vector3& dth) {

	return calc_rot_rate(navutils::quat_to_dcm(pva.get_quaternion()),
	                     navutils::transverse_radius(pva.get_p1()),
	                     navutils::meridian_radius(pva.get_p1()),
	                     pva.get_p3(),
	                     cos(pva.get_p1()),
	                     dt,
	                     dth,
	                     sin(pva.get_p1()),
	                     tan(pva.get_p1()),
	                     utils::extract_vel(pva));
}

Vector3 calc_rot_rate(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva1,
                      const aspn_xtensor::MeasurementPositionVelocityAttitude& pva2) {
	int64_t pva1_t     = pva1.get_aspn_c()->time_of_validity.elapsed_nsec;
	int64_t pva2_t     = pva2.get_aspn_c()->time_of_validity.elapsed_nsec;
	double delta_t     = (pva1_t - pva2_t) * 1e-9;
	Matrix3 C_n_to_s2  = transpose(navutils::quat_to_dcm(pva2.get_quaternion()));
	Matrix3 C_s1_to_n  = navutils::quat_to_dcm(pva1.get_quaternion());
	Matrix3 C_s1_to_s2 = dot(C_n_to_s2, C_s1_to_n);
	return navutils::dcm_to_rpy(C_s1_to_s2) / delta_t;
}

Vector3 calc_force_ned(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva1,
                       const aspn_xtensor::MeasurementPositionVelocityAttitude& pva2) {

	Vector3 velocity1    = utils::extract_vel(pva1);
	Vector3 velocity2    = utils::extract_vel(pva2);
	auto delta_v         = velocity2 - velocity1;
	int64_t pva1_t       = pva1.get_aspn_c()->time_of_validity.elapsed_nsec;
	int64_t pva2_t       = pva2.get_aspn_c()->time_of_validity.elapsed_nsec;
	double delta_t       = (pva2_t - pva1_t) * 1e-9;
	Vector3 acceleration = delta_v / delta_t;
	auto earth_model     = filtering::EarthModel(utils::extract_pos(pva1), velocity1);
	auto offset          = calc_force_and_acceleration_offset(earth_model.r_e,
                                                     earth_model.r_n,
                                                     pva1.get_p3(),
                                                     earth_model.cos_l,
                                                     earth_model.g_n,
                                                     earth_model.sec_l,
                                                     earth_model.sin_l,
                                                     velocity1);
	return acceleration - offset;
}

double apply_aiding_alt_accel(double r_zero,
                              Vector3* accel_vector,
                              AidingAltData* aiding_alt_data,
                              double alt0,
                              double dt,
                              const Vector3& g) {

	// Parameters for aiding altitude feedback constants (Rogers, p 90, eqn 5.83)
	auto lambda_ = aiding_alt_data->time_constant;
	auto c1      = 3 * lambda_;
	auto c2      = 4 * lambda_ * lambda_ + 2 * g[2] / r_zero;
	auto c3      = 2 * lambda_ * lambda_ * lambda_;

	aiding_alt_data->integrated_alt_error += (alt0 - aiding_alt_data->aiding_alt) * dt;

	// Aid the acceleration with C2 and C3; sign depending on coordinate frame
	auto integrated_alt_error = aiding_alt_data->integrated_alt_error;

	(*accel_vector)[2] =
	    (*accel_vector)[2] + c2 * (alt0 - aiding_alt_data->aiding_alt) + c3 * integrated_alt_error;

	return c1;
}

Vector3 calc_force_and_acceleration_offset(double r_e,
                                           double r_n,
                                           double alt0,
                                           double cos_l,
                                           const Vector3& g,
                                           double sec_l,
                                           double sin_l,
                                           const Vector3& v_ned0,
                                           double omega) {
	auto l_dot      = v_ned0[0] / (r_n + alt0);
	auto lambda_dot = v_ned0[1] * sec_l / (r_e + alt0);

	Vector3 offset = zeros(3);
	offset[0]      = -v_ned0[1] * (2 * omega + lambda_dot) * sin_l + v_ned0[2] * l_dot + g[0];
	// Equation 3.77, pp 52, Titterton text
	offset[1] = v_ned0[0] * (2 * omega + lambda_dot) * sin_l +
	            v_ned0[2] * (2 * omega + lambda_dot) * cos_l + g[1];
	// Equation 3.78, pp 52, Titterton text
	offset[2] = -v_ned0[1] * (2 * omega + lambda_dot) * cos_l - v_ned0[0] * l_dot + g[2];
	return offset;
}

}  // namespace inertial
}  // namespace navtk
