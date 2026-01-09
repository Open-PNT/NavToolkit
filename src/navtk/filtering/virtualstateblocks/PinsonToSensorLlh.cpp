#include <navtk/filtering/virtualstateblocks/PinsonToSensorLlh.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/leverarms.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

constexpr Size PinsonToSensorLlh::POS_START;
constexpr Size PinsonToSensorLlh::TILT_START;
constexpr Size PinsonToSensorLlh::TILT_END;

PinsonToSensorLlh::PinsonToSensorLlh(
    std::string current,
    std::string target,
    std::function<NavSolution(const aspn_xtensor::TypeTimestamp& time)> ref_fun,
    aspn_xtensor::TypeMounting inertial_mount,
    aspn_xtensor::TypeMounting sensor_mount)
    : NumericalVirtualStateBlock(std::move(current), std::move(target)),
      ref_fun(std::move(ref_fun)),
      inertial_mount(std::move(inertial_mount)),
      sensor_mount(std::move(sensor_mount)) {}

not_null<std::shared_ptr<VirtualStateBlock>> PinsonToSensorLlh::clone() {
	return std::make_shared<PinsonToSensorLlh>(*this);
}

Vector PinsonToSensorLlh::fx(const Vector& x, const aspn_xtensor::TypeTimestamp& time) {
	auto sol = ref_fun(time);

	auto delta_lat  = navutils::north_to_delta_lat(x(POS_START), sol.pos(0), sol.pos(2));
	auto delta_lon  = navutils::east_to_delta_lon(x(POS_START + 1), sol.pos(0), sol.pos(2));
	auto delta_alt  = -x(POS_START + 2);
	auto corr_s_llh = Vector3{delta_lat, delta_lon, delta_alt} + sol.pos;

	auto corr_s_ecef = navutils::llh_to_ecef(corr_s_llh);
	auto C_nav_to_sensor =
	    dot(sol.rot_mat, eye(3) + navutils::skew(xt::view(x, xt::range(TILT_START, TILT_END))));
	auto C_nav_to_ecef = navutils::llh_to_cen(corr_s_llh);

	auto corr_p_ecef = navutils::sensor_to_platform(
	    std::pair<Vector3, Matrix3>(corr_s_ecef, C_nav_to_sensor),
	    inertial_mount.get_lever_arm(),
	    navtk::navutils::quat_to_dcm(inertial_mount.get_orientation_quaternion()),
	    C_nav_to_ecef);
	auto corr_s2_ecef = navutils::platform_to_sensor(
	    corr_p_ecef,
	    sensor_mount.get_lever_arm(),
	    navtk::navutils::quat_to_dcm(sensor_mount.get_orientation_quaternion()),
	    C_nav_to_ecef);
	return navutils::ecef_to_llh(corr_s2_ecef.first);
}
}  // namespace filtering
}  // namespace navtk
