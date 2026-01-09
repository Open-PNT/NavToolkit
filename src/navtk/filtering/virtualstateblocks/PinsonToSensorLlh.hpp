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
 * Takes the Pinson15 error state block, combines with inertial reference solution, corrects
 * position/attitude for mounting, and extracts and returns the resulting position states
 * (lat, lon, alt in rad, rad, m). Due to the number of operations involved, the Jacobian is
 * numerically calculated, which may be slower/slightly more error-prone.
 */
class PinsonToSensorLlh : public NumericalVirtualStateBlock {
public:
	/**
	 * Constructor.
	 *
	 * @param current Label associated with the Pinson block to be converted.
	 * @param target Label to give to the virtual block.
	 * @param ref_fun A function that accepts a time and returns a 'free inertial'
	 * NavSolution from the inertial sensor whose errors the Pinson block is estimating, valid at
	 * the argument time.
	 * @param inertial_mount Lever arm from the origin of the platform frame to the origin of the
	 * inertial sensor frame, coordinatized in the platform frame, in meters, and the orientation
	 * DCM that rotates from the platform frame to the inertial sensor frame.
	 * @param sensor_mount As `inertial_mount`, but swapping in 'other sensor' for inertial.
	 */
	PinsonToSensorLlh(std::string current,
	                  std::string target,
	                  std::function<NavSolution(const aspn_xtensor::TypeTimestamp& time)> ref_fun,
	                  aspn_xtensor::TypeMounting inertial_mount,
	                  aspn_xtensor::TypeMounting sensor_mount);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

protected:
	/**
	 * Converts Pinson error-state vector in the inertial sensor frame to a whole state position
	 * vector in some other sensor frame.
	 *
	 * @param x Pinson error state vector.
	 * @param time Time of validity for `x`.
	 *
	 * @return Whole position state vector in LLH units at sensor frame origin.
	 */
	Vector fx(const Vector& x, const aspn_xtensor::TypeTimestamp& time) override;

private:
	std::function<NavSolution(const aspn_xtensor::TypeTimestamp& time)> ref_fun = 0;
	aspn_xtensor::TypeMounting inertial_mount;
	aspn_xtensor::TypeMounting sensor_mount;
	static constexpr Size POS_START  = 0;
	static constexpr Size TILT_START = 6;
	static constexpr Size TILT_END   = 9;
};
}  // namespace filtering
}  // namespace navtk
