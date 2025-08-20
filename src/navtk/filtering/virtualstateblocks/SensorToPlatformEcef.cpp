#include <navtk/filtering/virtualstateblocks/SensorToPlatformEcef.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/derivatives.hpp>
#include <navtk/navutils/leverarms.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

constexpr Size SensorToPlatformEcef::POS_START;
constexpr Size SensorToPlatformEcef::POS_END;
constexpr Size SensorToPlatformEcef::VEL_START;
constexpr Size SensorToPlatformEcef::VEL_END;
constexpr Size SensorToPlatformEcef::RPY_START;
constexpr Size SensorToPlatformEcef::RPY_END;

SensorToPlatformEcef::SensorToPlatformEcef(std::string current,
                                           std::string target,
                                           aspn_xtensor::TypeMounting sensor_mount,
                                           double scale_factor)
    : VirtualStateBlock(std::move(current), std::move(target)),
      sensor_mount(std::move(sensor_mount)),
      scale_factor(scale_factor) {}

not_null<std::shared_ptr<VirtualStateBlock>> SensorToPlatformEcef::clone() {
	return std::make_shared<SensorToPlatformEcef>(*this);
}

Vector SensorToPlatformEcef::convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto s_ecef = xt::view(x, xt::range(POS_START, POS_END));

	auto C_ecef_to_s = transpose(navutils::rpy_to_dcm(xt::view(x, xt::range(RPY_START, RPY_END))));

	auto p_ecef = navutils::sensor_to_platform(
	    std::pair<Vector3, Matrix3>(s_ecef, C_ecef_to_s),
	    sensor_mount.get_lever_arm() / scale_factor,
	    navtk::navutils::quat_to_dcm(sensor_mount.get_orientation_quaternion()));

	auto x_out = xt::concatenate(xt::xtuple(p_ecef.first,
	                                        xt::view(x, xt::range(VEL_START, VEL_END)),
	                                        navutils::dcm_to_rpy(xt::transpose(p_ecef.second)),
	                                        xt::view(x, xt::range(RPY_END, num_rows(x)))));
	return x_out;
}

Matrix SensorToPlatformEcef::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto rpy_range = xt::range(RPY_START, RPY_END);
	auto pos_range = xt::range(POS_START, POS_END);

	auto C_s_to_p =
	    transpose(navtk::navutils::quat_to_dcm(sensor_mount.get_orientation_quaternion()));
	Vector3 l_ps_p = sensor_mount.get_lever_arm() / scale_factor;
	auto base_rpy  = xt::view(x, rpy_range);
	auto C_k_to_s  = transpose(navutils::rpy_to_dcm(base_rpy));

	// As C_platform_to_sensor is constant (no reliance on rpy) it's derivative goes to 0,
	// so we just need derivative of rpy_to_dcm of original matrix
	auto C_k_to_s_der_r = transpose(navutils::d_cns_wrt_r(base_rpy));
	auto C_k_to_s_der_p = transpose(navutils::d_cns_wrt_p(base_rpy));
	auto C_k_to_s_der_y = transpose(navutils::d_cns_wrt_y(base_rpy));

	auto C_k_to_p_der_r = dot(C_s_to_p, C_k_to_s_der_r);
	auto C_k_to_p_der_p = dot(C_s_to_p, C_k_to_s_der_p);
	auto C_k_to_p_der_y = dot(C_s_to_p, C_k_to_s_der_y);

	auto out = eye(num_rows(x));

	// Usually premultiply each by C_k_to_j, but always eye(3) in this case
	xt::view(out, pos_range, RPY_START)     = dot(transpose(C_k_to_p_der_r), -l_ps_p);
	xt::view(out, pos_range, RPY_START + 1) = dot(transpose(C_k_to_p_der_p), -l_ps_p);
	xt::view(out, pos_range, RPY_START + 2) = dot(transpose(C_k_to_p_der_y), -l_ps_p);

	auto ab = dot(C_s_to_p, C_k_to_s);
	auto dx = dot(C_s_to_p, C_k_to_s_der_r);
	auto dy = dot(C_s_to_p, C_k_to_s_der_p);
	auto dz = dot(C_s_to_p, C_k_to_s_der_y);

	xt::view(out, rpy_range, rpy_range) = navutils::d_dcm_to_rpy(ab, dx, dy, dz);

	return out;
}
}  // namespace filtering
}  // namespace navtk
