#pragma once

#include <functional>

#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/fusion/strategies/FusionStrategy.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/ValidationResult.hpp>

namespace navtk {
namespace filtering {

/**
 * A strategy capable of Bayesian inference on a linearized discrete-time system with
 * Gaussian noise inputs. Assumes the system is described by discrete-time matrices and noise
 * inputs are zero-mean white Gaussian. Implementations of this interface may optionally provide a
 * version of each method without the Jacobian as a parameter. In this case, the Jacobian will be
 * calculated by the implementing class numerically from the function.
 */
class StandardModelStrategy : virtual public FusionStrategy {
public:
	virtual ~StandardModelStrategy() = default;

	/**
	 * Propagates the state estimate and covariance forward one time epoch.
	 *
	 * @param dynamics_model The model which describes to the filter how to propagate the states.
	 */
	virtual void propagate(const StandardDynamicsModel& dynamics_model) = 0;

	/**
	 * Updates the state estimate and covariance at the current time with the given measurement.
	 *
	 * @param measurement_model The model which describes to the filter how to update the states
	 * using a processed measurement. It contains a measurement vector `z` and a function that maps
	 * the estimate `xhat` to `z`.
	 */
	virtual void update(const StandardMeasurementModel& measurement_model) = 0;

protected:
	StandardModelStrategy() = default;

	/**
	 * Validates the dimensions of StandardMeasurementModel::H.
	 *
	 * @param measurement_model The StandardMeasurementModel to validate.
	 *
	 * @throw std::invalid_argument If ErrorMode::DIE and the number of columns of H in
	 * `measurement_model` does not equal the number of rows of `xhat` and the error mode is
	 * ErrorMode::DIE.
	 * @return utils::ValidationResult::BAD if validation failed, utils::ValidationResult::GOOD if
	 * succeeded, or utils::ValidationResult::NOT_CHECKED if ErrorMode::OFF
	 */
	utils::ValidationResult check_update_args(const StandardMeasurementModel& measurement_model);

	/**
	 * Validate the dynamics in a call to propagate on a StandardFusionEngine.
	 *
	 * @param dynamics_jacobian The derivative of a dynamics function passed to propagate.
	 * @param dynamics_noise_covariance The covariance matrix of the discrete-time noise inputs
	 * passed to propagate.
	 *
	 * @throw std::range_error If `dynamics_jacobian` or `dynamics_noise_covariance` are not NxN
	 * and the error mode is ErrorMode::DIE.
	 * @throw std::domain_error If `dynamics_noise_covariance` is not symmetric and the error mode
	 * is ErrorMode::DIE.
	 * @return utils::ValidationResult::BAD if validation failed, utils::ValidationResult::GOOD if
	 * succeeded, or utils::ValidationResult::NOT_CHECKED if ErrorMode::OFF
	 */
	utils::ValidationResult validate_linearized_propagate(const Matrix& dynamics_jacobian,
	                                                      const Matrix& dynamics_noise_covariance);

	/**
	 * Validate measurements in a call to update on a StandardFusionEngine.
	 *
	 * @param measurement_jacobian The derivative of the measurement function passed to update.
	 * @param measurement_noise_covariance The noise covariance matrix passed to update.
	 * @param measurement A numerical measurement passed to update.
	 * @param hx The output of `h(xhat)`, where `h` is the measurement function.
	 *
	 * @throw std::range_error If `measurement` or `hx` are not Mx1, if
	 * `measurement_noise_covariance` is not MxM or if `measurement_jacobian` is not MxN and the
	 * error mode is ErrorMode::DIE for either case.
	 * @throw std::domain_error If `measurement_noise_covariance `is not symmetric and the error
	 * mode is ErrorMode::DIE.
	 * @return utils::ValidationResult::BAD if validation failed, utils::ValidationResult::GOOD if
	 * succeeded, or utils::ValidationResult::NOT_CHECKED if ErrorMode::OFF
	 */
	utils::ValidationResult validate_linearized_update(const Matrix& measurement_jacobian,
	                                                   const Matrix& measurement_noise_covariance,
	                                                   const Vector& measurement,
	                                                   const Vector& hx);
};

}  // namespace filtering
}  // namespace navtk
