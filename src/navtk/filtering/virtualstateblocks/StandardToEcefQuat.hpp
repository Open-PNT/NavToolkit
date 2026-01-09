#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * VirtualStateBlock representing an N-state ECEF frame PVA (position, velocity, attitude),
 * constructed from a 'Standard' PVA. 'Standard' here refers to position represented as
 * latitude, longitude and altitude position in radians, radians, meters; North, East and Down
 * velocity in meters/sec; and NED referenced attitude, as a quaternion in this case.
 *
 * The first 10 virtual states are formatted as:
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
 * 6 - 'Own' to ECEF quaternion scalar element.
 *
 * 7 - 'Own' to ECEF quaternion 'i' vector element.
 *
 * 8 - 'Own' to ECEF quaternion 'j' vector element.
 *
 * 9 - 'Own' to ECEF quaternion 'k' vector element.
 *
 * The first 10 input states to be transformed should be in the following format:
 *
 * 0 - Latitude (rad).
 *
 * 1 - Longitude (rad).
 *
 * 2 - Altitude HAE (m).
 *
 * 3 - North velocity (m/s).
 *
 * 4 - East velocity (m/s).
 *
 * 5 - Down velocity (m/s).
 *
 * 6 - 'Own' to NED quaternion scalar element.
 *
 * 7 - 'Own' to NED quaternion 'i' vector element.
 *
 * 8 - 'Own' to NED quaternion 'j' vector element.
 *
 * 9 - 'Own' to NED quaternion 'k' vector element.
 *
 * Additional trailing states are retained, unmodified.
 */
class StandardToEcefQuat : public VirtualStateBlock {
public:
	/**
	 * Constructor.
	 * @param current Label associated with the block to be converted.
	 * @param target Label to give to the virtual block.
	 */
	StandardToEcefQuat(std::string current, std::string target);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

private:
	static constexpr Size POS_START  = 0;
	static constexpr Size POS_END    = 3;
	static constexpr Size VEL_START  = 3;
	static constexpr Size VEL_END    = 6;
	static constexpr Size QUAT_START = 6;
	static constexpr Size QUAT_END   = 10;
};
}  // namespace filtering
}  // namespace navtk
