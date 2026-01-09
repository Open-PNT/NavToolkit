#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Shifts an ECEF representation (with quaternion attitude) of a sensor frame origin to the
 * platform frame. Velocity is not affected (any tangential velocity effects as a result of the
 * platform rotating are not accounted for in the shift). Output quaternion is platform to ECEF, in
 * contrast with the input sensor to ECEF attitude.
 *
 * Input/Output state units:
 *
 * 0 - ECEF X position coordinate (m).
 *
 * 1 - ECEF Y position coordinate (m).
 *
 * 2 - ECEF Z position coordinate (m).
 *
 * 3 - ECEF X-axis velocity (m/s).
 *
 * 4 - ECEF X-axis velocity (m/s).
 *
 * 5 - ECEF X-axis velocity (m/s).
 *
 * 6 - 'Own' to ECEF quaternion scalar element.
 *
 * 7 - 'Own' to ECEF quaternion 'i' vector element. (Own is sensor for input, platform for output).
 *
 * 8 - 'Own' to ECEF quaternion 'j' vector element.
 *
 * 9 - 'Own' to ECEF quaternion 'k' vector element.
 *
 * Additional trailing states are retained, unmodified.
 */
class SensorToPlatformEcefQuat : public VirtualStateBlock {
public:
	/**
	 * Constructor.
	 * @param current Label associated with the block to be converted.
	 * @param target Label to give to the virtual block.
	 * @param sensor_mount Lever arm and orientation relating the sensor frame to the platform
	 * frame.
	 * @param scale_factor Optional factor that relates the lever arm units to the ECEF position
	 * units. For example, if the ECEF position were in km then the scale factor would be 1000
	 * (as the lever arm must be meters). This parameter must be provided when scaling is applied to
	 * the ECEF position representations in order to alleviate additional truncation issues.
	 */
	SensorToPlatformEcefQuat(std::string current,
	                         std::string target,
	                         aspn_xtensor::TypeMounting sensor_mount,
	                         double scale_factor = 1.0);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

private:
	aspn_xtensor::TypeMounting sensor_mount;
	double scale_factor;
	static constexpr Size POS_START  = 0;
	static constexpr Size POS_END    = 3;
	static constexpr Size VEL_START  = 3;
	static constexpr Size VEL_END    = 6;
	static constexpr Size QUAT_START = 6;
	static constexpr Size QUAT_END   = 10;
};
}  // namespace filtering
}  // namespace navtk
