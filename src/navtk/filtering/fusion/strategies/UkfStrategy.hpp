#pragma once

#include <functional>
#include <memory>

#include <xtensor/views/xview.hpp>

#include <navtk/filtering/containers/LinearizedStrategyBase.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/fusion/strategies/StandardModelStrategy.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * An implementation of Bayesian innovation using the Unscented Kalman Filter (UKF) equations.
 */
class UkfStrategy : public StandardModelStrategy, public LinearizedStrategyBase {
public:
	/**
	 * Unscented Kalman Filter Interface implementation of propagate.
	 *
	 * @see StandardModelStrategy#propagate.
	 *
	 * @param dynamics_model A description of the dynamics of the state vector.
	 */
	void propagate(const StandardDynamicsModel &dynamics_model) override;

	/**
	 * Unscented Kalman Filter Interface implementation of update.
	 *
	 * @see StandardModelStrategy#update.
	 *
	 * @param measurement_model The measurement model to use during innovation.
	 */
	void update(const StandardMeasurementModel &measurement_model) override;

	not_null<std::shared_ptr<FusionStrategy>> clone() const override;

protected:
	/**
	 * Returns the suggested kappa parameter (assumes Gaussian distribution).
	 *
	 * @param tuning: Number of states in vector sigma points are drawn from.
	 *
	 * @return kappa: 'Fine tuning' parameter for sigma point selection. A
	 * larger kappa value more strongly weights the central sigma point and
	 * de-weights the 'spread' points that incorporate the covariance. Kappa
	 * may not equal the negative of the number of states. Default is -N + 3,
	 * or 1.0 if there are 3 states (semi-randomly selected), where N represents
	 * the tuning parameter.
	 */
	int default_kappa(int tuning);

	/**
	 * Generates a set of sigma points drawn from a mean vector and covariance matrix.
	 *
	 * @param kappa: Scaling value that describes the spread of sigma points chosen.
	 * @param x: State mean vector.
	 * @param P: State covariance matrix.
	 *
	 * @return N x 2N+1 matrix of sigma points, where N represents the number of states in x.
	 */
	Matrix mean_sigma_points(int kappa, Vector x, Matrix P);

	/**
	 * Calculates the weighting parameter for all non-central sigma points.
	 *
	 * @param kappa: Scaling value that describes the spread of sigma points chosen.
	 * @param num_states: Number of states in vector sigma points are drawn from.
	 *
	 * @return The weighting parameter for all non-central sigma points.
	 */
	double weight_off(int kappa, Size num_states);

	/**
	 * Calculates the weighting parameter for the first UKF sigma point mean.
	 *
	 * @param kappa: Scaling value that describes the spread of sigma points chosen.
	 * @param num_states: Number of states in vector sigma points are drawn from.
	 *
	 * @return The weighting parameter for the first UKF sigma point mean.
	 */
	double mean_weight0(int kappa, Size num_states);

	/**
	 * Calculates the weights associated with a matrix of UKF sigma points, used to recalculate both
	 * the mean and covariance.
	 *
	 * @param kappa: Scaling value that describes the spread of sigma points chosen.
	 * @param num_states: Number of states in vector sigma points are drawn from.
	 *
	 * @return The Nx2+1 Vector of weights associated with a matrix of UKF sigma points, where N
	 * represents num_states.
	 */
	Vector mean_weights(int kappa, Size num_states);

	/**
	 * Calculates the mean from a set of sigma points.
	 *
	 * @param sigma_points: N x 2N+1 matrix of sigma points, where N represents the number of
	 * states.
	 * @param sigma_weights: 2N+1 vector of weighting parameters (must sum to 1), where N represents
	 * the number of states.
	 *
	 * @return The mean vector, Nx1, where N represents the number of states.
	 */
	Vector reconstruct_x_from_sigma_points(Matrix sigma_points, Vector sigma_weights);

	/**
	 * Calculates the covariance matrix from a set of sigma points.
	 *
	 * @param sigma_points: N x 2N+1 matrix of sigma points, where N represents the number of
	 * states.
	 * @param sigma_weights: 2N+1 x 1 vector of weighting parameters (must sum to 1, unless beta was
	 * included), where N represents the number of states.
	 * @param new_mean: N x 1 mean vector for sigma, where N represents the number of states.
	 *
	 * @return Covariance matrix, NxN, where N reprents the number of states.
	 */
	Matrix reconstruct_p_from_sigma_points(Matrix sigma_points,
	                                       Vector sigma_weights,
	                                       Vector new_mean);

	/**
	 * Calculates the cross correlation between 2 sets of observations/estimates given their means.
	 * N represents the number of states for `mean1`. K represents the number of states for `mean2`.
	 * M represents the number of weighting values.
	 *
	 * @param pred1: NxM observation matrix.
	 * @param mean1: Nx1 weighted mean vector for `pred1`.
	 * @param pred2: KxN observation matrix.
	 * @param mean2: Kx1 weighted mean vector for `pred2`.
	 * @param weights: Mx1 vector of weighting values.
	 *
	 * @return NxK covariance matrix with weighting applied such that the
	 * scale/units of the elements are as they are for `mean1` and `mean2`.
	 */
	Matrix calc_weighted_cov(
	    Matrix pred1, Vector mean1, Matrix pred2, Vector mean2, Vector weights);
};
}  // namespace filtering
}  // namespace navtk
