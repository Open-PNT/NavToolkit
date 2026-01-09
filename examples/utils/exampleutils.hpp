#pragma once

#include <navtk/factory.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace exampleutils {

/**
 * Generates a vector of NavSolutions that approximate a constant-velocity wings-level trajectory
 * over a static local level frame. Earth effects (curvature, rotation etc.) are not accounted for.
 *
 * @param start_pva The initial position, velocity and attitude (attitude is ignored).
 * @param dt Time step between solutions (sec).
 * @param stop_time End time for trajectory simulation (sec).
 *
 * @return A vector of NavSolutions. Positions are calculated by a dead-reckoned integration
 * of the velocities contained in \p start_pva . All rotation matrices are identical and estimated
 * from the NED velocity vector, assuming level flight (roll assumed 0, pitch in [-pi/2, pi/2],
 * and heading in (-pi, pi]). The first solution generated is equivalent to the starting PVA, and
 * the final will be generated with time ( \p stop_time - \p dt ).
 */
std::vector<filtering::NavSolution> constant_vel_pva(filtering::NavSolution start_pva,
                                                     double dt,
                                                     double stop_time);

/**
 * Generate a set of uncorrelated corrupted position measurements.
 *
 * @param truth vector of true trajectory points of size n.
 * @param sigma 3-length measurement sigma Vector, in meters NED.
 * @param err Optional 3 x n Matrix of errors to add directly to the measurements (such as biases).
 * Errors must be in meters, NED.
 *
 * @return 3 x n Matrix of position measurements. The first row will be latitude in radians, the
 * second row longitude in radians, and the third altitude, m HAE.
 */
Matrix noisy_pos_meas(const std::vector<filtering::NavSolution> &truth,
                      const Vector &sigma = zeros(3),
                      const Matrix &err   = zeros(0, 0));

/**
 * Generate a set of corrupted altitude measurements.
 *
 * @param truth vector of true trajectory points of size n.
 * @param sigma Altitude sigma, in meters.
 * @param err Optional n length Vector of errors to add directly to the measurements (such as
 * biases). Errors must be in meters, Down.
 *
 * @return n length Vector of altitude measurements, m HAE.
 */
Vector noisy_alt_meas(const std::vector<filtering::NavSolution> &truth,
                      double sigma      = 0.0,
                      const Vector &err = zeros(0));

/**
 * Generate a set of uncorrelated corrupted velocity measurements.
 *
 * @param truth vector of true trajectory points of size n.
 * @param sigma 3-length measurement sigma Vector, in meters/s NED.
 * @param err Optional 3 x n Matrix of errors to add directly to the measurements (such as biases).
 * Errors must be in meters/s, NED.
 *
 * @return 3 x n Matrix of NED velocity measurements, in m/s.
 */
Matrix noisy_vel_meas(const std::vector<filtering::NavSolution> &truth,
                      const Vector &sigma = zeros(3),
                      const Matrix &err   = zeros(0, 0));

/**
 * Generate a set of uncorrelated corrupted attitude measurements (DCMs, nav to platform).
 *
 * @param truth vector of true trajectory points of size n.
 * @param tilt_sigma 3-length measurement sigma Vector, in radians.
 * @param tilts Optional 3 x n Matrix of NED tilt errors, in radians to apply to the rotation
 * Matrix.
 *
 * @return n length vector of 3x3 nav-to-platform DCMs.
 */
std::vector<Matrix> noisy_att_meas(const std::vector<filtering::NavSolution> &truth,
                                   const Vector &tilt_sigma = zeros(3),
                                   const Matrix &tilts      = zeros(0, 0));

}  // namespace exampleutils
}  // namespace navtk
