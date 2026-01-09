#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * VirtualStateBlock representing an N-state 'Standard PVA', constructed from an ECEF frame
 * referenced PVA. Inverse of StandardToEcef. 'Standard' here refers to position represented as
 * latitude, longitude and altitude position in radians, radians, meters; North, East and Down
 * velocity in meters/sec; and NED referenced attitude, as Euler angles in this case.
 *
 * The first 9 virtual states are formatted as:
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
 *
 * With possible additional trailing states if they are present in the input; their values will be
 * unchanged.
 *
 * The first 9 input states to be transformed should be in the following format:
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
 */
class EcefToStandard : public VirtualStateBlock {

public:
	/**
	 * Constructor.
	 *
	 * @param current The label associated with the ECEF PVA block.
	 * @param target Label to associate with the standard formatted block.
	 */
	EcefToStandard(std::string current, std::string target);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

private:
	// Constants describing ranges for certain state vector elements. Using const xt::ranges
	// would be better, but type resolution issues are preventing that
	static constexpr Size POS_START = 0;
	static constexpr Size POS_END   = 3;
	static constexpr Size VEL_START = 3;
	static constexpr Size VEL_END   = 6;
	static constexpr Size RPY_START = 6;
	static constexpr Size RPY_END   = 9;
};
}  // namespace filtering
}  // namespace navtk
