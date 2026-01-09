#pragma once

#include <utility>

#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Function pointer for discretization functions that perform discretization on continuous-time
 * state-transition matrices.
 */
using DiscretizationStrategy = std::function<std::pair<Matrix, Matrix>(
    const Matrix& F, const Matrix& G, const Matrix& Q, double dt)>;

/**
 * Calculates approximations of the state-transition matrices using a first-order model.
 *
 * @param F Linearized propagation matrix.
 * @param G Maps the noise matrix Q to the states.
 * @param Q Continuous time process noise matrix.
 * @param dt Delta time in seconds from the current filter time to the target propagation time.
 *
 * @return A pair containing Phi (discrete-time state transition matrix) and Qd (discrete-time
 * process noise covariance matrix).
 */
std::pair<Matrix, Matrix> first_order_discretization_strategy(const Matrix& F,
                                                              const Matrix& G,
                                                              const Matrix& Q,
                                                              double dt);

/**
 * Calculates approximations of the state-transition matrices using a second-order model.
 *
 * @param F Linearized propagation matrix.
 * @param G Maps the noise matrix Q to the states.
 * @param Q Continuous time process noise matrix.
 * @param dt Delta time in seconds from the current filter time to the target propagation time.
 *
 * @return A pair containing Phi (discrete-time state transition matrix) and Qd (discrete-time
 * process noise covariance matrix).
 */
std::pair<Matrix, Matrix> second_order_discretization_strategy(const Matrix& F,
                                                               const Matrix& G,
                                                               const Matrix& Q,
                                                               double dt);

/**
 * Calculates approximations of the state-transition matrices using a full-order model via the
 * Van Loan calculation.
 *
 * @param F Linearized propagation matrix.
 * @param G Maps the noise matrix Q to the states.
 * @param Q Continuous time process noise matrix.
 * @param dt Delta time in seconds from the current filter time to the target propagation time.
 *
 * @return A pair containing Phi (discrete-time state transition matrix) and Qd (discrete-time
 * process noise covariance matrix).
 */
std::pair<Matrix, Matrix> full_order_discretization_strategy(const Matrix& F,
                                                             const Matrix& G,
                                                             const Matrix& Q,
                                                             double dt);

}  // namespace filtering
}  // namespace navtk
