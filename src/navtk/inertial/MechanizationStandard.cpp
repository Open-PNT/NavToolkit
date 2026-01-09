#include <navtk/errors.hpp>
#include <navtk/inertial/AidingAltData.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/MechanizationStandard.hpp>
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

not_null<std::shared_ptr<InertialPosVelAtt>> MechanizationStandard::mechanize(
    const Vector3& dv_s,
    const Vector3& dth_s,
    double dt,
    const not_null<std::shared_ptr<InertialPosVelAtt>> pva,
    const not_null<std::shared_ptr<InertialPosVelAtt>> old_pva,
    const MechanizationOptions& mech_options,
    AidingAltData* aiding_alt_data) {
	options = mech_options;
	prep(pva, old_pva, dv_s, dth_s, dt);
	compute(aiding_alt_data);
	return std::make_shared<StandardPosVelAtt>(pva->time_validity + dt, llh1, vned1, C_s_to_n1);
}

void MechanizationStandard::prep(const not_null<std::shared_ptr<InertialPosVelAtt>> pva,
                                 const not_null<std::shared_ptr<InertialPosVelAtt>> oldpva,
                                 const Vector3& dv,
                                 const Vector3& dth,
                                 const double dt) {
	auto llh         = pva->get_llh();
	auto v           = pva->get_vned();
	lat0             = llh(0);
	lon0             = llh(1);
	alt0             = llh(2);
	vn0              = v(0);
	ve0              = v(1);
	vd0              = v(2);
	this->C_s_to_n0  = pva->get_C_s_to_ned();
	this->v_ned_prev = oldpva->get_vned();
	sin_l            = sin(lat0);
	cos_l            = cos(lat0);
	sec_l            = 1 / cos_l;
	tan_l            = sin_l / cos_l;
	rn               = navutils::meridian_radius(lat0);
	re               = navutils::transverse_radius(lat0);
	r_zero           = sqrt(rn * re);
	this->dt         = dt;
	dv0              = dv(0);
	dv1              = dv(1);
	dv2              = dv(2);
	dth0             = dth(0);
	dth1             = dth(1);
	dth2             = dth(2);
}

void MechanizationStandard::calc_grav() {
	switch (options.grav_model) {
	case navutils::GravModels::SCHWARTZ: {
		g = navutils::calculate_gravity_schwartz(alt0, lat0);
		break;
	}
	case navutils::GravModels::SAVAGE: {
		auto C_n_to_e = navutils::lat_lon_wander_to_C_n_to_e(lat0, lon0, 0.0);
		g             = navutils::calculate_gravity_savage_ned(C_n_to_e, alt0);
		break;
	}
	case navutils::GravModels::TITTERTON: {
		g = navutils::calculate_gravity_titterton(alt0, lat0, r_zero);
		break;
	}
	default: {
		log_or_throw<std::invalid_argument>("Invalid GravModels enum value supplied.");
	}
	}
}

void MechanizationStandard::calc_rot_rate() {
	double o1 = ve0 / (re + alt0) + omega * cos_l;
	double o2 = -vn0 / (rn + alt0);
	double o3 = -ve0 * tan_l / (re + alt0) - omega * sin_l;

	sigma[0] =
	    (dth0 / dt - C_s_to_n0(0, 0) * o1 - C_s_to_n0(1, 0) * o2 - C_s_to_n0(2, 0) * o3) * dt;
	sigma[1] =
	    (dth1 / dt - C_s_to_n0(0, 1) * o1 - C_s_to_n0(1, 1) * o2 - C_s_to_n0(2, 1) * o3) * dt;
	sigma[2] =
	    (dth2 / dt - C_s_to_n0(0, 2) * o1 - C_s_to_n0(1, 2) * o2 - C_s_to_n0(2, 2) * o3) * dt;
}

void MechanizationStandard::calc_dcm() {
	switch (options.dcm_method) {
	case DcmIntegrationMethods::SIXTH_ORDER: {
		A = navutils::rot_vec_to_dcm(sigma);
		break;
	}
	case DcmIntegrationMethods::EXPONENTIAL: {
		A = expm(navutils::skew(sigma));
		break;
	}
	case DcmIntegrationMethods::FIRST_ORDER: {
		A = I3 + navutils::skew(sigma);
		break;
	}
	default: {

		log_or_throw<std::invalid_argument>("Unrecognized dcm_integration_method");
	}
	}
	C_s_to_n1 = navtk::dot(C_s_to_n0, A);
}

void MechanizationStandard::calc_force_ned() {

	fn = (C_s_to_n0(0, 0) * (dv0 + 0.5 * (dth1 * dv2 - dv1 * dth2)) +
	      C_s_to_n0(0, 1) * (dv1 + 0.5 * (dth2 * dv0 - dv2 * dth0)) +
	      C_s_to_n0(0, 2) * (dv2 + 0.5 * (dth0 * dv1 - dv0 * dth1))) /
	     dt;

	fe = (C_s_to_n0(1, 0) * (dv0 + 0.5 * (dth1 * dv2 - dv1 * dth2)) +
	      C_s_to_n0(1, 1) * (dv1 + 0.5 * (dth2 * dv0 - dv2 * dth0)) +
	      C_s_to_n0(1, 2) * (dv2 + 0.5 * (dth0 * dv1 - dv0 * dth1))) /
	     dt;

	fd = (C_s_to_n0(2, 0) * (dv0 + 0.5 * (dth1 * dv2 - dv1 * dth2)) +
	      C_s_to_n0(2, 1) * (dv1 + 0.5 * (dth2 * dv0 - dv2 * dth0)) +
	      C_s_to_n0(2, 2) * (dv2 + 0.5 * (dth0 * dv1 - dv0 * dth1))) /
	     dt;
}

void MechanizationStandard::calc_acceleration() {
	auto l_dot      = vn0 / (rn + alt0);
	auto lambda_dot = ve0 * sec_l / (re + alt0);
	an              = fn - ve0 * (2 * omega + lambda_dot) * sin_l + vd0 * l_dot + g(0);
	ae =
	    fe + vn0 * (2 * omega + lambda_dot) * sin_l + vd0 * (2 * omega + lambda_dot) * cos_l + g(1);
	ad = fd - ve0 * (2 * omega + lambda_dot) * cos_l - vn0 * l_dot + g(2);
}

void MechanizationStandard::integrate_to_velocity() {
	vned1[0] = an * dt + vn0;
	vned1[1] = ae * dt + ve0;
	vned1[2] = ad * dt + vd0;
}

void MechanizationStandard::integrate_to_position() {
	// Unrolling these Vector adds results in numerical differences with
	// the mechanize_standard function so not doing that
	switch (options.int_method) {
	case IntegrationMethods::TRAPEZOIDAL: {
		dpn = (vned1[0] + vn0) / 2 * dt;
		dpe = (vned1[1] + ve0) / 2 * dt;
		dpd = (vned1[2] + vd0) / 2 * dt;
		break;
	}
	case IntegrationMethods::SIMPSONS_RULE: {
		dpn = (v_ned_prev[0] + 4 * vn0 + vned1[0]) / 6 * dt;
		dpe = (v_ned_prev[1] + 4 * ve0 + vned1[1]) / 6 * dt;
		dpd = (v_ned_prev[2] + 4 * vd0 + vned1[2]) / 6 * dt;
		break;
	}
	case IntegrationMethods::RECTANGULAR: {
		dpn = vn0 * dt;
		dpe = ve0 * dt;
		dpd = vd0 * dt;
		break;
	}
	default: {
		log_or_throw<std::invalid_argument>("Unrecognized integration_method");
	}
	}
}

void MechanizationStandard::compute(AidingAltData* aiding_alt_data) {
	calc_grav();
	calc_rot_rate();
	calc_dcm();
	calc_force_ned();
	calc_acceleration();

	if (aiding_alt_data != nullptr) {
		auto lambda_ = aiding_alt_data->time_constant;
		c1           = 3 * lambda_;
		auto c2      = 4 * lambda_ * lambda_ + 2 * g(2) / r_zero;
		auto c3      = 2 * lambda_ * lambda_ * lambda_;
		aiding_alt_data->integrated_alt_error += (alt0 - aiding_alt_data->aiding_alt) * dt;
		ad = ad + c2 * (alt0 - aiding_alt_data->aiding_alt) +
		     c3 * aiding_alt_data->integrated_alt_error;
	}

	integrate_to_velocity();
	integrate_to_position();

	if (aiding_alt_data != nullptr) dpd += c1 * (alt0 - aiding_alt_data->aiding_alt) * dt;

	llh1[0] = lat0 + dpn / (rn + alt0);
	llh1[1] = lon0 + dpe * sec_l / (re + alt0);
	llh1[2] = alt0 - dpd;
}

}  // namespace inertial
}  // namespace navtk
