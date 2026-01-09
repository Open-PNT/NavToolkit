#pragma once

#include <navtk/filtering/containers/Pose.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A navigation solution consisting of a position, velocity, orientation, and time.
 */
struct NavSolution : Pose {
	/**
	 * The current North-East-Down (NED) velocity in m/s.
	 */
	Vector3 vel;

	/**
	 * @param pose The value to store in #Pose.
	 * @param vel The value to store in #vel.
	 */
	NavSolution(Pose pose, Vector3 vel) : Pose(pose), vel(vel) {}

	/**
	 * @param pos A vector representing position as Latitude-Longitude-Height (LLH) in
	 * rad-rad-meters.
	 * @param vel The value to store in #vel.
	 * @param rot_mat A 3x3 DCM representing the rotation from the navigation frame to the sensor
	 * frame.
	 * @param time The time at which the position, velocity, and attitude are valid in seconds.
	 */
	NavSolution(Vector3 pos, Vector3 vel, Matrix3 rot_mat, aspn_xtensor::TypeTimestamp time)
	    : Pose(pos, rot_mat, time), vel(vel) {}
};

}  // namespace filtering
}  // namespace navtk
