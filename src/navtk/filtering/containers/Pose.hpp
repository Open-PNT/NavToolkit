#pragma once

#include <navtk/aspn.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A structure for describing Pose using position, attitude, and time.
 */
struct Pose {
	/**
	 * A vector representing position as Latitude-Longitude-Height (LLH) in rad-rad-meters.
	 */
	Vector3 pos;

	/**
	 * A 3x3 DCM representing the rotation from the navigation frame to the sensor frame.
	 */
	Matrix3 rot_mat;

	/**
	 * The time at which the attitude and position are valid, in seconds.
	 */
	aspn_xtensor::TypeTimestamp time;

	/**
	 * @param pos A vector representing position as Latitude-Longitude-Height (LLH) in
	 * rad-rad-meters.
	 * @param rot_mat A 3x3 DCM representing the rotation from the navigation frame to the sensor
	 * frame.
	 * @param time The time at which the attitude and position are valid, in seconds.
	 */
	Pose(Vector3 pos, Matrix3 rot_mat, aspn_xtensor::TypeTimestamp time)
	    : pos(pos), rot_mat(rot_mat), time(time) {}
};

}  // namespace filtering
}  // namespace navtk
