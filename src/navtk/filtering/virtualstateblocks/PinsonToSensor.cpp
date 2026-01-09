#include <navtk/filtering/virtualstateblocks/PinsonToSensor.hpp>

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

constexpr Size PinsonToSensor::POS_START;
constexpr Size PinsonToSensor::VEL_START;
constexpr Size PinsonToSensor::VEL_END;
constexpr Size PinsonToSensor::TILT_START;
constexpr Size PinsonToSensor::TILT_END;

PinsonToSensor::PinsonToSensor(
    std::string current,
    std::string target,
    std::function<NavSolution(const aspn_xtensor::TypeTimestamp& time)> ref_fun,
    aspn_xtensor::TypeMounting inertial_mount,
    aspn_xtensor::TypeMounting sensor_mount)
    : NumericalVirtualStateBlock(std::move(current), std::move(target)),
      ref_fun(std::move(ref_fun)),
      inertial_mount(std::move(inertial_mount)),
      sensor_mount(std::move(sensor_mount)) {}

not_null<std::shared_ptr<VirtualStateBlock>> PinsonToSensor::clone() {
	return std::make_shared<PinsonToSensor>(*this);
}

Vector PinsonToSensor::fx(const Vector& x, const aspn_xtensor::TypeTimestamp& time) {
	auto sol = ref_fun(time);

	// Correct nominal PVA of 'sensor 1/ins' frame with error states and reference to ecef frame
	auto delta_lat  = navutils::north_to_delta_lat(x(POS_START), sol.pos(0), sol.pos(2));
	auto delta_lon  = navutils::east_to_delta_lon(x(POS_START + 1), sol.pos(0), sol.pos(2));
	auto delta_alt  = -x(POS_START + 2);
	auto corr_s_llh = Vector3{delta_lat, delta_lon, delta_alt} + sol.pos;

	auto corr_s_ecef = navutils::llh_to_ecef(corr_s_llh);

	auto corr_vel_ned = sol.vel + xt::view(x, xt::range(VEL_START, VEL_END));

	auto corr_C_ned_to_s =
	    dot(sol.rot_mat, eye(3) + navutils::skew(xt::view(x, xt::range(TILT_START, TILT_END))));

	auto C_ned_to_ecef = navutils::llh_to_cen(corr_s_llh);

	// Shift corrected 'sensor 1' frame PVA through platform frame to 'sensor 2' frame
	auto corr_p_ecef = navutils::sensor_to_platform(
	    std::pair<Vector3, Matrix3>(corr_s_ecef, corr_C_ned_to_s),
	    inertial_mount.get_lever_arm(),
	    navtk::navutils::quat_to_dcm(inertial_mount.get_orientation_quaternion()),
	    C_ned_to_ecef);

	auto corr_s2_ecef = navutils::platform_to_sensor(
	    corr_p_ecef,
	    sensor_mount.get_lever_arm(),
	    navtk::navutils::quat_to_dcm(sensor_mount.get_orientation_quaternion()),
	    C_ned_to_ecef);

	// Convert to output units
	auto corr_s2_llh = navutils::ecef_to_llh(corr_s2_ecef.first);
	auto rpy         = navutils::dcm_to_rpy(xt::transpose(corr_s2_ecef.second));

	auto x_out = xt::concatenate(
	    xt::xtuple(corr_s2_llh, corr_vel_ned, rpy, xt::view(x, xt::range(TILT_END, num_rows(x)))));
	return x_out;
}
}  // namespace filtering
}  // namespace navtk
