#pragma once

#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Performs a stationary alignment using quaternions to find the local solution that
 * minimizes the components of accel/gyro in the down and East directions respectively.
 * Data passed in must be contiguous from a strapdown inertial that was stationary during
 * the duration of the data.
 *
 * @param dv_avg Average dv collected from a time series of stationary data, m/s
 * @param dth_avg Average dth collected from a time series of stationary data, rad
 *
 * @return The 3x3 `C_sensor_to_nav` rotation matrix between sensor and navigation
 * (North-East-Down) frames.
 *
 * @throw std::runtime_error when either of the the residual horizontal acceleration components in
 * the NED frame are >= than 1e-10, or the residual rotation rate in the east direction is >= 1e-10,
 * but only if the error mode is ErrorMode::DIE for either case.
 */
Matrix3 quaternion_static_alignment(const Vector3& dv_avg, const Vector3& dth_avg);

}  // namespace inertial
}  // namespace navtk
