#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Shifts an ECEF representation of a platform frame origin to the sensor frame. Velocity is not
 * affected (any tangential velocity effects as a result of the platform rotating are not accounted
 * for in the shift). Output angles reflect the ECEF to sensor orientation, in contrast with the
 * input ECEF to platform attitude.
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
 * 4 - ECEF Y-axis velocity (m/s).
 *
 * 5 - ECEF Z-axis velocity (m/s).
 *
 * 6 - ECEF 'roll' (x Euler angle, rad).
 *
 * 7 - ECEF 'pitch' (y Euler angle, rad).
 *
 * 8 - ECEF 'yaw' (z Euler angle, rad).
 *
 * Additional trailing states are retained, unmodified.
 */
class PlatformToSensorEcef : public VirtualStateBlock {
public:
	/**
	 * Constructor.
	 *
	 * @param current Label associated with the block to be converted.
	 * @param target Label to give to the virtual block.
	 * @param sensor_mount Sensor mounting parameters.
	 * @param scale_factor Scale factor applied to ECEF positions to alleviate some numerical
	 * issues. A value of 1000 results in km position representation. Output position units are not
	 * altered.
	 */
	PlatformToSensorEcef(std::string current,
	                     std::string target,
	                     aspn_xtensor::TypeMounting sensor_mount,
	                     double scale_factor = 1.0);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

private:
	aspn_xtensor::TypeMounting sensor_mount;
	double scale_factor;
	static constexpr Size POS_START = 0;
	static constexpr Size POS_END   = 3;
	static constexpr Size VEL_START = 3;
	static constexpr Size VEL_END   = 6;
	static constexpr Size RPY_START = 6;
	static constexpr Size RPY_END   = 9;
};
}  // namespace filtering
}  // namespace navtk
