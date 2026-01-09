#include <navtk/filtering/virtualstateblocks/PlatformToSensorEcefQuat.hpp>

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

using navtk::navutils::quat_conj;
using navtk::navutils::quat_mult;

constexpr Size PlatformToSensorEcefQuat::POS_START;
constexpr Size PlatformToSensorEcefQuat::POS_END;
constexpr Size PlatformToSensorEcefQuat::VEL_START;
constexpr Size PlatformToSensorEcefQuat::VEL_END;
constexpr Size PlatformToSensorEcefQuat::QUAT_START;
constexpr Size PlatformToSensorEcefQuat::QUAT_END;

PlatformToSensorEcefQuat::PlatformToSensorEcefQuat(std::string current,
                                                   std::string target,
                                                   aspn_xtensor::TypeMounting sensor_mount,
                                                   double scale_factor)
    : VirtualStateBlock(std::move(current), std::move(target)),
      sensor_mount(std::move(sensor_mount)),
      scale_factor(scale_factor) {}

not_null<std::shared_ptr<VirtualStateBlock>> PlatformToSensorEcefQuat::clone() {
	return std::make_shared<PlatformToSensorEcefQuat>(*this);
}

Vector PlatformToSensorEcefQuat::convert_estimate(const Vector& x,
                                                  const aspn_xtensor::TypeTimestamp&) {
	auto p_ecef = xt::view(x, xt::range(POS_START, POS_END));

	auto q_p_to_ecef = xt::view(x, xt::range(QUAT_START, QUAT_END));
	auto q_p_to_s    = sensor_mount.get_orientation_quaternion();
	auto q_s_to_ecef = quat_mult(q_p_to_ecef, quat_conj(q_p_to_s));

	auto s_ecef =
	    p_ecef + navutils::quat_rot(q_p_to_ecef, sensor_mount.get_lever_arm() / scale_factor);

	auto x_out = xt::concatenate(xt::xtuple(s_ecef,
	                                        xt::view(x, xt::range(VEL_START, VEL_END)),
	                                        q_s_to_ecef,
	                                        xt::view(x, xt::range(QUAT_END, num_rows(x)))));
	return x_out;
}

Matrix PlatformToSensorEcefQuat::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto quat_range = xt::range(QUAT_START, QUAT_END);

	auto out = eye(num_rows(x));

	auto q_p_to_s = sensor_mount.get_orientation_quaternion();

	xt::view(out, xt::range(POS_START, POS_END), quat_range) =
	    navutils::d_platform_to_sensor_pos_wrt_q(xt::view(x, quat_range),
	                                             sensor_mount.get_lever_arm() / scale_factor);

	xt::view(out, quat_range, QUAT_START)     = quat_mult(Vector{1, 0, 0, 0}, quat_conj(q_p_to_s));
	xt::view(out, quat_range, QUAT_START + 1) = quat_mult(Vector{0, 1, 0, 0}, quat_conj(q_p_to_s));
	xt::view(out, quat_range, QUAT_START + 2) = quat_mult(Vector{0, 0, 1, 0}, quat_conj(q_p_to_s));
	xt::view(out, quat_range, QUAT_START + 3) = quat_mult(Vector{0, 0, 0, 1}, quat_conj(q_p_to_s));
	return out;
}
}  // namespace filtering
}  // namespace navtk
