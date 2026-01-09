#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * VirtualStateBlock that maps Pinson-style error states into a 'Standard' whole-valued
 * representation by combining the error states with the uncorrected reference PVA. 'Standard' here
 * refers to position represented as latitude, longitude and altitude position in radians, radians,
 * meters; North, East and Down velocity in meters/sec; and NED to 'own' frame attitude, as Euler
 * angles in this case. Only the first 9 (position, velocity and attitude states) are transformed;
 * any additional 'trailing' states are carried over to the virtual block without modification.
 * See navtk::filtering::Pinson15NedBlock for a description of input states units/frames.
 *
 * The transformed state order and units are:
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
class PinsonErrorToStandard : public VirtualStateBlock {
public:
	/**
	 * Constructor.
	 *
	 * @param current Label associated with the Pinson block to be converted.
	 * @param target Label to give to the virtual block.
	 * @param ref_fun A function that accepts a time and returns a 'free inertial' NavSolution from
	 * the inertial sensor whose errors the Pinson block is estimating, valid at the argument time.
	 */
	PinsonErrorToStandard(
	    std::string current,
	    std::string target,
	    std::function<NavSolution(const aspn_xtensor::TypeTimestamp& time)> ref_fun);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	virtual Vector convert_estimate(const Vector& x,
	                                const aspn_xtensor::TypeTimestamp& time) override;

	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp& time) override;

private:
	/* Pointer to function that returns the uncorrected inertial solution given time. */
	std::function<NavSolution(const aspn_xtensor::TypeTimestamp& time)> ref_fun = 0;
	// Constants describing ranges for certain state vector elements. Using const xt::ranges
	// would be better, but type resolution issues are preventing that
	static constexpr Size POS_START = 0;
	static constexpr Size POS_END   = 3;
	static constexpr Size VEL_START = 3;
	static constexpr Size VEL_END   = 6;
	static constexpr Size ATT_START = 6;
	static constexpr Size ATT_END   = 9;
};
}  // namespace filtering
}  // namespace navtk
