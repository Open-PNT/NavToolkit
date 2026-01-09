#pragma once

#include <functional>
#include <memory>

#include <navtk/filtering/containers/LinearizedStrategyBase.hpp>
#include <navtk/filtering/fusion/strategies/StandardModelStrategy.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * An implementation of Bayesian innovation using the Extended Kalman Filter (EKF) equations.
 */
class EkfStrategy : public StandardModelStrategy, public LinearizedStrategyBase {
public:
	/**
	 * Propagate states using Extended Kalman Filter (EKF) equations.
	 *
	 * @see StandardModelStrategy#propagate.
	 *
	 * @param dynamics_model A description of the dynamics of the state vector.
	 */
	void propagate(const StandardDynamicsModel &dynamics_model) override;

	void update(const StandardMeasurementModel &measurement_model) override;


	not_null<std::shared_ptr<FusionStrategy>> clone() const override;
};

}  // namespace filtering
}  // namespace navtk
