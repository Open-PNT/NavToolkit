#include <navtk/filtering/virtualstateblocks/SensorToPlatformEcefQuat.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/inspect.hpp>
#include <navtk/navutils/derivatives.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/quaternions.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

using navtk::navutils::quat_mult;

constexpr Size SensorToPlatformEcefQuat::POS_START;
constexpr Size SensorToPlatformEcefQuat::POS_END;
constexpr Size SensorToPlatformEcefQuat::VEL_START;
constexpr Size SensorToPlatformEcefQuat::VEL_END;
constexpr Size SensorToPlatformEcefQuat::QUAT_START;
constexpr Size SensorToPlatformEcefQuat::QUAT_END;

SensorToPlatformEcefQuat::SensorToPlatformEcefQuat(std::string current,
                                                   std::string target,
                                                   aspn_xtensor::TypeMounting sensor_mount,
                                                   double scale_factor)
    : VirtualStateBlock(std::move(current), std::move(target)),
      sensor_mount(std::move(sensor_mount)),
      scale_factor(scale_factor) {}

not_null<std::shared_ptr<VirtualStateBlock>> SensorToPlatformEcefQuat::clone() {
	return std::make_shared<SensorToPlatformEcefQuat>(*this);
}

Vector SensorToPlatformEcefQuat::convert_estimate(const Vector& x,
                                                  const aspn_xtensor::TypeTimestamp&) {
	auto s_ecef = xt::view(x, xt::range(POS_START, POS_END));

	auto q_s_to_ecef = xt::view(x, xt::range(QUAT_START, QUAT_END));
	auto q_p_to_s    = sensor_mount.get_orientation_quaternion();
	auto q_p_to_ecef = quat_mult(q_s_to_ecef, q_p_to_s);
	auto p_ecef =
	    s_ecef - navutils::quat_rot(q_p_to_ecef, sensor_mount.get_lever_arm() / scale_factor);

	return xt::concatenate(xt::xtuple(p_ecef,
	                                  xt::view(x, xt::range(VEL_START, VEL_END)),
	                                  q_p_to_ecef,
	                                  xt::view(x, xt::range(QUAT_END, num_rows(x)))));
}

Matrix SensorToPlatformEcefQuat::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto quat_range = xt::range(QUAT_START, QUAT_END);

	auto out = eye(num_rows(x));

	xt::view(out, xt::range(POS_START, POS_END), quat_range) =
	    navutils::d_sensor_to_platform_pos_wrt_q(
	        xt::view(x, quat_range),
	        sensor_mount.get_lever_arm() / scale_factor,
	        navtk::navutils::quat_to_dcm(sensor_mount.get_orientation_quaternion()));

	auto q_p_to_s                             = sensor_mount.get_orientation_quaternion();
	xt::view(out, quat_range, QUAT_START)     = quat_mult(Vector{1, 0, 0, 0}, q_p_to_s);
	xt::view(out, quat_range, QUAT_START + 1) = quat_mult(Vector{0, 1, 0, 0}, q_p_to_s);
	xt::view(out, quat_range, QUAT_START + 2) = quat_mult(Vector{0, 0, 1, 0}, q_p_to_s);
	xt::view(out, quat_range, QUAT_START + 3) = quat_mult(Vector{0, 0, 0, 1}, q_p_to_s);
	return out;
}
}  // namespace filtering
}  // namespace navtk
