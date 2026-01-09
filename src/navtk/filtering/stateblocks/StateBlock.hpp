#pragma once

#include <memory>
#include <string>

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/SampledDynamicsModel.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/stateblocks/discretization_strategy.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/ValidationContext.hpp>

namespace navtk {
namespace filtering {

/**
 * A description of a set of states estimates, associated covariances, and the states' dynamics.
 *
 * `StateBlock`s are used by a fusion engine, along with one or several `MeasurementProcessor`s and
 * a `FusionStrategy`. `StateBlock`s allow the fusion engine to predict how states will change over
 * time, via dynamics models. `MeasurementProcessor`s give the fusion engine the models needed to
 * relate measurements to the states. The `FusionStrategy` is responsible for providing the fusion
 * engine with the general algorithms used to combine the information from `StateBlock`s and
 * `MeasurementProcessor`s.
 *
 * A `StateBlock` provides the fusion engine with a dynamics model, the number of states
 * (`#num_states`), and a label (via `#get_label`). A dynamics model usually consists of the
 * non-linear discrete-time state-transition function, denoted as `g`, the Jacobian of `g`, denoted
 * as `Phi` (used in the linear equation), and the discrete-time process noise covariance matrix,
 * denoted as `Qd`. These three quantities are used by the fusion engine to propagate the states
 * and their covariances forward in time when `StandardFusionEngine::propagate` is called, using
 * the output of `#generate_dynamics`. See `ModelType` template documentation for additional
 * information about when the dynamics model contents may deviate from those described.
 *
 * @tparam ModelType The type of DynamicsModel returned by the `#generate_dynamics` function.
 * Currently two are available. The `StandardDynamicsModel` is meant for cases where Gaussian states
 * are simply propagated via the `StandardDynamicsModel.g` function with no additional perturbation;
 * the uncertainty increase due to propagation is handled via `StandardDynamicsModel.Qd`, a
 * discrete-time representation of average white Gaussian noise inputs in the state-space.
 * The `SampledDynamicsModel` on the other hand does not assume that the states can be modeled as
 * Gaussian; the `SampledDynamicsModel.g` function is free to treat each `navtk::Vector` input as a
 * sample from an arbitrary but known distribution and propagate it accordingly, possibly adding
 * random noise samples directly to the states.
 */
template <typename ModelType = StandardDynamicsModel>
class StateBlock {
public:
	/**
	 * Disabled default constructor.
	 */
	StateBlock() = delete;

	virtual ~StateBlock() = default;

	/**
	 * The StateBlock constructor sets up the StateBlock's properties.
	 * When StateBlock is instantiated directly, it functions as a StateBlock that helps estimate a
	 * constant value, and uses the Q matrix provided in the constructor to
	 * give dynamics when generate_dynamics is called. When the StateBlock class is inherited in
	 * another StateBlock, it is not guaranteed to function as a constant StateBlock or to use
	 * Q matrix or the discretization strategy provided through the constructor, and most often
	 * intentionally has different dynamics.
	 *
	 * @param num_states The number of states in this state block.
	 * @param label The name of this state block.
	 * @param Q The matrix to model error propagation when the StateBlock is used as a
	 * constant block. By default, Q is a matrix of zeros.
	 * @param discretization_strategy A discretization strategy that determines
	 * how Q will be discretized to produce a discrete time Qd when the StateBlock is used as a
	 * constant block.
	 * @throw Error if the shape of the Q matrix is not equal to {num_states, num_states}. This
	 * error is avoided if the number of states is greater than 1 and the Q matrix is the
	 * default (zeros(1, 1)). In this case, the Q matrix becomes zeros(num_states, num_states).
	 */
	StateBlock(size_t num_states,
	           const std::string &label,
	           DiscretizationStrategy discretization_strategy = &full_order_discretization_strategy,
	           Matrix Q                                       = {{0.0}})
	    : num_states(num_states),
	      discretization_strategy(std::move(discretization_strategy)),
	      label(label),
	      Q(std::move(Q)),
	      F(zeros(num_states, num_states)) {
		if (num_states > 1 && this->Q.shape(0) == 1 && this->Q.shape(1) == 1 && this->Q(0) == 0.0) {
			this->Q = zeros(num_states, num_states);
		}
		navtk::utils::ValidationContext{}
		    .add_matrix(this->Q, "Q")
		    .dim(num_states, num_states)
		    .validate();
	}

	/**
	 * Generate a complete description of how to propagate this state block forward in time, given a
	 * current estimate to linearize about. This function may call the fusion engine passed in the
	 * constructor as necessary. For simple models, this can simply return a set of static matrices
	 * that are pre-defined.
	 *
	 * @param gen_x_and_p_func A function that will generate `xhat` (a vector of estimates for the
	 * states corresponding to this state block's label) and `P` (the covariance matrix
	 * corresponding to xhat) when called.
	 * @param time_from Time propagating from.
	 * @param time_to Time propagating to.
	 *
	 * @return The Dynamics which describe the non-linear propagation of this state block.
	 */
	virtual ModelType generate_dynamics(GenXhatPFunction gen_x_and_p_func,
	                                    aspn_xtensor::TypeTimestamp time_from,
	                                    aspn_xtensor::TypeTimestamp time_to) {

		auto g = [](Vector x) { return x; };

		// The (void)`gen_x_and_p_func` is used here so that the `gen_x_and_p_func` parameter can be
		// documented (only named parameters can be documented)
		(void)gen_x_and_p_func;

		double dt        = (time_to.get_elapsed_nsec() - time_from.get_elapsed_nsec()) * 1e-9;
		auto discretized = discretization_strategy(F, eye(num_rows(Q)), Q, dt);
		auto Phi         = discretized.first;
		auto Qd          = discretized.second;

		return StandardDynamicsModel(g, Phi, Qd);
	}

	/**
	 * Receive and use arbitrary aux data sent from the sensor. This method will be called by the
	 * fusion engine when the fusion engine receives aux data from a give_state_block_aux_data call.
	 * The default implementation logs a warning that the state block does not use the given type of
	 * aux data.
	 *
	 * If your StateBlock needs to receive aux data, this function needs to be implemented to be
	 * able to ingest your desired type(s) of aspn_xtensor::AspnBase from the incoming vector. Your
	 * override should use `std::dynamic_pointer_cast` or similar to check the message type of each
	 * message in the incoming AspnBaseVector against the specific aspn_xtensor::AspnBase subclasses
	 * you can support.
	 */
	virtual void receive_aux_data(const AspnBaseVector &) {
		spdlog::warn("The state block labeled {} does not utilize this type of aux data.", label);
	};

	/**
	 * Create a copy of the StateBlock with the same properties.
	 *
	 * @return A shared pointer to a copy of the StateBlock. A unique pointer is not used here
	 * because of an issue with the Python bindings: https://github.com/pybind/pybind11/issues/673
	 */
	virtual not_null<std::shared_ptr<StateBlock<ModelType>>> clone() {
		return std::make_shared<StateBlock>(*this);
	}

	/**
	 * @return The unique identifier for this state block.
	 */
	std::string get_label() const { return label; }

	/**
	 * @return The number of states in this state block.
	 */
	size_t get_num_states() const { return num_states; }

protected:
	/**
	 * The number of states in this state block.
	 */
	size_t num_states;

	/**
	 * The discretization strategy to turn Q into Qd when using the StateBlock as a constant block.
	 */
	DiscretizationStrategy discretization_strategy;

private:
	/**
	 * A unique identifier for this state block.
	 */
	std::string label;
	/**
	 * The Q matrix when using the base StateBlock as a constant block.
	 */
	Matrix Q;
	/**
	 * The F matrix when using the base StateBlock as a constant block.
	 */
	Matrix F;
};

/**
 * A StateBlock which generates a StandardDynamicsModel.
 */
typedef StateBlock<StandardDynamicsModel> StandardStateBlock;

/**
 * A StateBlock which generates a SampledDynamicsModel.
 */
typedef StateBlock<SampledDynamicsModel> SampledStateBlock;


}  // namespace filtering
}  // namespace navtk
