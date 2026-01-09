#pragma once

#include <functional>

#include <navtk/filtering/containers/SampledDynamicsModel.hpp>
#include <navtk/filtering/containers/SampledMeasurementModel.hpp>
#include <navtk/filtering/fusion/strategies/FusionStrategy.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A strategy capable of Bayesian inference on a sampled system model.
 */
class SampledModelStrategy : virtual public FusionStrategy {
public:
	/**
	 * Propagates the state estimate and covariance forward one time epoch.
	 *
	 * @param dynamics_model The model which describes to the filter how to propagate the states.
	 */
	virtual void propagate(const SampledDynamicsModel& dynamics_model) = 0;

	/**
	 * Updates the state estimate and covariance at the current time with the given measurement.
	 *
	 * @param measurement_model The model which describes to the filter how to update the states
	 * using a processed measurement. It contains a measurement vector `z` and a function that maps
	 * the estimate `xhat` to `z`.
	 */
	virtual void update(const SampledMeasurementModel& measurement_model) = 0;



protected:
	SampledModelStrategy() = default;
};

}  // namespace filtering
}  // namespace navtk
