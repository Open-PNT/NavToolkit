#pragma once

#include <functional>
#include <iterator>
#include <memory>
#include <vector>

#include <xtensor/containers/xtensor.hpp>
#include <xtensor/core/xmath.hpp>
#include <xtensor/views/xview.hpp>

#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/experimental/containers/RbpfModel.hpp>
#include <navtk/filtering/experimental/resampling.hpp>
#include <navtk/filtering/fusion/strategies/StandardModelStrategy.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/transform.hpp>

namespace navtk {
namespace filtering {
namespace experimental {

/**
 * A function pointer that takes a Vector and pointer to number of desired samples (or `nullptr` for
 * all) by constant reference and returns a ResamplingResult
 */
using ResamplingFunction = std::function<ResamplingResult(const Vector &, const size_t *)>;

/**
 * An implementation of Bayesian innovation using the Rao-Blackwellized Particle Filter (RBPF)
 * equations.
 */
class RbpfStrategy : public StandardModelStrategy, public RbpfModel {
public:
	/**
	 * @param num_particles Number of particles for particle states.
	 * @param resampling_threshold Particle resampling threshold.
	 * @param calc_single_jacobian When true, simulates the effects of calculating a single jacobian
	 * for the propagation and update models. In this case, the covariance of the linear states is
	 * maintained as a single matrix, and only this value is modified during propagation and
	 * updates. When false, a separate covariance is maintained for each particle. Note that in the
	 * current implementation this setting does not actually change how jacobians are calculated; in
	 * each case jacobians are evaluated once at the 'mean estimate' of all particles rather than
	 * about each individually.
	 * @param _resampling_fun The name of the resampling approach.
	 */
	RbpfStrategy(Size num_particles                 = DEFAULT_PARTICLE_COUNT,
	             double resampling_threshold        = 0.75,
	             bool calc_single_jacobian          = true,
	             ResamplingFunction _resampling_fun = &residual_resample_with_replacement);

	/**
	 * Predicts future values of the estimate and covariance at the time given in the
	 * `dynamics_model` using a particle-filtering algorithm for marked states and an EKF algorithm
	 * for the rest.
	 *
	 * @see StandardModelStrategy#propagate.
	 *
	 * @param dynamics_model A description of the dynamics of the state vector.
	 */
	void propagate(const StandardDynamicsModel &dynamics_model) override;

	/**
	 * Updates the estimate estimate and covariance with the given measurement.
	 *
	 * @see StandardModelStrategy#update.
	 *
	 * @param measurement_model The measurement model to use during innovation.
	 */
	void update(const StandardMeasurementModel &measurement_model) override;

	/**
	 * Return which states are represented as particles.
	 *
	 * @return A vector of booleans corresponding to the state vector, such that `estimate[0]` is
	 * represented as a particle if and only if `get_particle_state_marks[0]` is true.
	 */
	std::vector<bool> get_particle_state_marks() const;

	/**
	 * @return RbpfStrategy::state_particles.
	 */
	Matrix get_state_particles() const;

	/**
	 * @return RbpfStrategy::covariance_particles.
	 */
	Tensor<3> get_state_particles_cov() const;

	/**
	 * @return RbpfStrategy::state_particles_weights.
	 */
	Vector get_state_particles_weights() const;

	/**
	 * Resampling threshold to prevent particle impoverishment. A higher threshold means particles
	 * are resampled more often, and a lower threshold means particles are resampled less often.
	 * After updates, the number of effective particles is compared to the threshold, and particles
	 * are resampled accordingly. This value should be between 0 and 1.
	 */
	double resampling_threshold;

	not_null<std::shared_ptr<FusionStrategy>> clone() const override;

protected:
	/**
	 * Predicts future values of the estimate and full Jacobian covariance at the time given in
	 * the `dynamics_model` using a particle-filtering algorithm for marked states and an EKF
	 * algorithm for the rest.
	 *
	 * @param dynamics_model A description of the dynamics of the state vector.
	 * @param chol_qd Cholesky decomposition of the process noise matrix.
	 *
	 */
	void propagate_rbpf(const StandardDynamicsModel &dynamics_model, const Matrix &chol_qd);

	/**
	 * Predicts future values of the estimate and full Jacobian covariance at the time given in
	 * the `dynamics_model` using an extended Kalman filter algorithm.
	 *
	 * @param dynamics_model A description of the dynamics of the state vector.
	 *
	 */
	void propagate_single_ekf(const StandardDynamicsModel &dynamics_model);

	/**
	 * Predicts future values of the estimate and full Jacobian covariance at the time given in
	 * the `dynamics_model` using a bank of extended Kalman filters algorithm.
	 *
	 * @param dynamics_model A description of the dynamics of the state vector.
	 *
	 */
	void propagate_bank_ekf(const StandardDynamicsModel &dynamics_model);

	/**
	 * Predicts future values of the estimate and Jacobian covariance at the time given in
	 * the `dynamics_model` using a particle-filtering algorithm.
	 *
	 * @param dynamics_model A description of the dynamics of the state vector.
	 * @param chol_qd Cholesky decomposition of the process noise matrix.
	 */
	void propagate_particle_filter(const StandardDynamicsModel &dynamics_model,
	                               const Matrix &chol_qd);

	/**
	 * Updates the estimate and Jacobian covariance using a single state EKF algorithm.
	 *
	 * @param measurement_model The filter measurement model.
	 *
	 */
	void update_as_single_ekf(const StandardMeasurementModel &measurement_model);

	/**
	 * Updates the estimate and Jacobian covariance using an EKF for each particle state.
	 *
	 * @param measurement_model The filter measurement model.
	 *
	 */
	void update_as_bank_ekf(const StandardMeasurementModel &measurement_model);

	/**
	 * Compute the weighted sum of particle states for a state estimate.
	 *
	 * @return The state estimate.
	 */
	Vector calc_weighted_estimate();

	/**
	 * Performs the particle state recovery to the previous state.
	 *
	 * @param recovery_states The recovery state vector to apply to the particle states.
	 */
	void reset_particles(Vector const &recovery_states);

	/**
	 * @param estimate_filtered The estimated state vector.
	 * @param weights The posterior sample weights.
	 * @param index The indexes to the weights after resampling.
	 *
	 * @return The covariance estimate.
	 */
	Matrix compute_covariance(const Vector &estimate_filtered,
	                          const Vector &weights,
	                          const std::vector<int> &index);

	/**
	 * Updates the estimate and Jacobian covariance using a particle-filtering algorithm.
	 *
	 * @param measurement_model The filter measurement model.
	 *
	 */
	void update_with_particle_method(const StandardMeasurementModel &measurement_model);

	/**
	 * @param x The prior state estimate.
	 * @param dynamics_model The filter dynamics model.
	 * @param cov_mat The prior state covariance estimate.
	 *
	 * @return The propagated covariance and state estimate.
	 */
	EstimateWithCovariance ekf_propagate(const Vector &x,
	                                     const StandardDynamicsModel &dynamics_model,
	                                     const Matrix &cov_mat);

	/**
	 * @param x The state estimate.
	 * @param measurement_model The filter measurement model.
	 * @param cov_mat The prior state covariance estimate.
	 *
	 * @return The covariance and state estimate.
	 */
	EstimateWithCovariance ekf_update(const Vector &x,
	                                  const StandardMeasurementModel &measurement_model,
	                                  const Matrix &cov_mat);

	/**
	 * Determine the weighted covariance values for the jitter states if applicable.
	 *
	 * @return Jitter application flag and state jitter scaling values.
	 */
	std::pair<bool, Vector> jitter_contribution();

	/**
	 * Compute the estimate of the number of effective particles.
	 *
	 * @param index The indexes to the weights after resampling.
	 */
	void copy_particles_by(const std::vector<size_t> &index);

private:
	/**
	 * Selected resampling function
	 */
	ResamplingFunction _resampling_fun;

};  // class RbpfStrategy

}  // namespace experimental
}  // namespace filtering
}  // namespace navtk
