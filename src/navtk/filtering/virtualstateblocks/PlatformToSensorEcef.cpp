#include <navtk/filtering/virtualstateblocks/PlatformToSensorEcef.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/derivatives.hpp>
#include <navtk/navutils/leverarms.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

constexpr Size PlatformToSensorEcef::POS_START;
constexpr Size PlatformToSensorEcef::POS_END;
constexpr Size PlatformToSensorEcef::VEL_START;
constexpr Size PlatformToSensorEcef::VEL_END;
constexpr Size PlatformToSensorEcef::RPY_START;
constexpr Size PlatformToSensorEcef::RPY_END;

PlatformToSensorEcef::PlatformToSensorEcef(std::string current,
                                           std::string target,
                                           aspn_xtensor::TypeMounting sensor_mount,
                                           double scale_factor)
    : VirtualStateBlock(std::move(current), std::move(target)),
      sensor_mount(std::move(sensor_mount)),
      scale_factor(scale_factor) {}

not_null<std::shared_ptr<VirtualStateBlock>> PlatformToSensorEcef::clone() {
	return std::make_shared<PlatformToSensorEcef>(*this);
}

Vector PlatformToSensorEcef::convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto p_ecef = xt::view(x, xt::range(POS_START, POS_END));
	auto C_ecef_to_p =
	    xt::transpose(navutils::rpy_to_dcm(xt::view(x, xt::range(RPY_START, RPY_END))));

	auto s_ecef = navutils::platform_to_sensor(
	    std::pair<Vector3, Matrix3>(p_ecef, C_ecef_to_p),
	    sensor_mount.get_lever_arm() / scale_factor,
	    navtk::navutils::quat_to_dcm(sensor_mount.get_orientation_quaternion()));

	auto x_out = xt::concatenate(xt::xtuple(s_ecef.first,
	                                        xt::view(x, xt::range(VEL_START, VEL_END)),
	                                        navutils::dcm_to_rpy(xt::transpose(s_ecef.second)),
	                                        xt::view(x, xt::range(RPY_END, num_rows(x)))));
	return x_out;
}

Matrix PlatformToSensorEcef::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto pos_range = xt::range(POS_START, POS_END);
	auto rpy_range = xt::range(RPY_START, RPY_END);

	Vector3 l_bs_b = sensor_mount.get_lever_arm() / scale_factor;
	auto base_rpy  = xt::view(x, rpy_range);
	auto C_k_to_p  = xt::transpose(navutils::rpy_to_dcm(base_rpy));

	auto C_p_to_k_der_r = navutils::d_cns_wrt_r(base_rpy);
	auto C_p_to_k_der_p = navutils::d_cns_wrt_p(base_rpy);
	auto C_p_to_k_der_y = navutils::d_cns_wrt_y(base_rpy);

	auto out = eye(num_rows(x));

	auto a  = navtk::navutils::quat_to_dcm(sensor_mount.get_orientation_quaternion());
	auto ab = dot(a, C_k_to_p);
	auto dx = dot(a, transpose(C_p_to_k_der_r));
	auto dy = dot(a, transpose(C_p_to_k_der_p));
	auto dz = dot(a, transpose(C_p_to_k_der_y));

	// This is equivalent to:
	// const auto Z3 = zeros(3, 3);
	// xt::view(out, rpy_range, rpy_range) = navutils::d_dcm_to_rpy(
	//     navtk::navutils::quat_to_dcm(sensor_mount.get_orientation_quaternion()),
	//     Z3,
	//     Z3,
	//     Z3,
	//     C_k_to_p,
	//     transpose(C_p_to_k_der_r),
	//     transpose(C_p_to_k_der_p),
	//     transpose(C_p_to_k_der_y));
	// But with the matrix multiplication simplified where the inputs are zeroes.
	xt::view(out, rpy_range, rpy_range) = navutils::d_dcm_to_rpy(ab, dx, dy, dz);

	// Normally pre-multiply by C_k_to_j, but it is always eye(3) in this case
	xt::view(out, pos_range, RPY_START)     = dot(C_p_to_k_der_r, l_bs_b);
	xt::view(out, pos_range, RPY_START + 1) = dot(C_p_to_k_der_p, l_bs_b);
	xt::view(out, pos_range, RPY_START + 2) = dot(C_p_to_k_der_y, l_bs_b);

	return out;
}
}  // namespace filtering
}  // namespace navtk
