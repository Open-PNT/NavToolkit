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
 * velocity in meters/sec; and NED referenced attitude, as Euler angles in this case. Inverse of
 * EcefToStandard.
 *
 * The first 9 virtual states are formatted as:
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
 * With possible additional trailing states if they are present in the input; their values will be
 * unchanged. The attitude states are the 3-2-1 successive rotation angles to go from the ECEF
 * frame the 'own' frame (platform, sensor, whatever the PVA represents); in other words the
 * return value of dcm_to_rpy(C_nav_to_platform, C_e_to_nav).
 *
 * The first 9 input states to be transformed should be in the following format:
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
 * 6 - Roll (rad).
 *
 * 7 - Pitch (rad).
 *
 * 8 - Yaw (rad).
 */
class StandardToEcef : public VirtualStateBlock {
public:
	/**
	 * Constructor.
	 *
	 * @param current The label associated with the standard PVA block.
	 * @param target Label to associate with the ECEF formatted block.
	 */
	StandardToEcef(std::string current, std::string target);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

private:
	static constexpr Size POS_START = 0;
	static constexpr Size POS_END   = 3;
	static constexpr Size VEL_START = 3;
	static constexpr Size VEL_END   = 6;
	static constexpr Size RPY_START = 6;
	static constexpr Size RPY_END   = 9;
};
}  // namespace filtering
}  // namespace navtk
