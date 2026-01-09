#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/virtualstateblocks/NumericalVirtualStateBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Converts from Pinson error states to NavSolution type PVA. Uses
 * numerical calculation of Jacobian.
 */
class PinsonToSensor : public NumericalVirtualStateBlock {
public:
	/**
	 * Constructor.
	 *
	 * @param current Label associated with the Pinson block to be converted.
	 * @param target Label to give to the virtual block.
	 * @param ref_fun A function that accepts a time (in seconds) and returns a 'free inertial'
	 * NavSolution from the inertial sensor whose errors the Pinson block is estimating, valid at
	 * the argument time.
	 * @param inertial_mount Lever arm from the origin of the platform frame to the origin of the
	 * inertial sensor frame, coordinatized in the platform frame, in meters, and the orientation
	 * DCM that rotates from the platform frame to the inertial sensor frame.
	 * @param sensor_mount As `inertial_mount`, but swapping in 'other sensor' for inertial.
	 */
	PinsonToSensor(std::string current,
	               std::string target,
	               std::function<NavSolution(const aspn_xtensor::TypeTimestamp& time)> ref_fun,
	               aspn_xtensor::TypeMounting inertial_mount,
	               aspn_xtensor::TypeMounting sensor_mount);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

protected:
	/**
	 * Converts Pinson error-state vector in the inertial sensor frame to a whole state vector in
	 * some other sensor frame.
	 *
	 * @param x Pinson error state vector.
	 * @param time Time of validity for `x`.
	 *
	 * @return Whole state vector in LLH units [rad, rad, m] at sensor frame origin.
	 */
	Vector fx(const Vector& x, const aspn_xtensor::TypeTimestamp& time) override;

private:
	std::function<NavSolution(const aspn_xtensor::TypeTimestamp& time)> ref_fun = 0;
	aspn_xtensor::TypeMounting inertial_mount;
	aspn_xtensor::TypeMounting sensor_mount;
	static constexpr Size POS_START  = 0;
	static constexpr Size VEL_START  = 3;
	static constexpr Size VEL_END    = 6;
	static constexpr Size TILT_START = 6;
	static constexpr Size TILT_END   = 9;
};
}  // namespace filtering
}  // namespace navtk
