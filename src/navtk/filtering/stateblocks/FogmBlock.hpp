#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/filtering/stateblocks/discretization_strategy.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * An implementation of StateBlock that represents a first-order Gauss-Markov stochastic process
 * in N dimensions.
 */
class FogmBlock : public StateBlock<> {
public:
	virtual ~FogmBlock() = default;

	/**
	 * @param label The label uniquely identifying this particular set of states.
	 * @param time_constants The FOGM time constants (decay term) for each state in seconds.
	 * @param state_sigmas The 1-sigma value of each state at steady-state. The units of this value
	 * should match the units of the states this StateBlock is estimating.
	 * @param num_states How many total states to add (i.e. length of `time_constants`
	 * and `state_sigmas`).
	 * @param discretization_strategy Determines how the matrices `F` and `Q` will be linearized to
	 * produce `Phi` and `Qd`. Options include a first-order, second-order, or full discretization.
	 */
	FogmBlock(const std::string& label,
	          Vector time_constants,
	          Vector state_sigmas,
	          Vector::shape_type::value_type num_states,
	          DiscretizationStrategy discretization_strategy = &full_order_discretization_strategy);

	/**
	 * @param label The label uniquely identifying this particular set of states.
	 * @param time_constant The FOGM time constant (decay term) applied to all states.
	 * @param state_sigma The 1-sigma value at steady-state applied to all states.
	 * @param num_states How many total states to add.
	 * @param discretization_strategy Determines how the matrices `F` and `Q` will be linearized to
	 * produce `Phi` and `Qd`. Options include a first-order, second-order, or full discretization.
	 */
	FogmBlock(const std::string& label,
	          double time_constant,
	          double state_sigma,
	          Vector::shape_type::value_type num_states,
	          DiscretizationStrategy discretization_strategy = &full_order_discretization_strategy);

	/**
	 * Custom copy constructor which creates a deep copy.
	 *
	 * @param block The FogmBlock to copy.
	 */
	FogmBlock(const FogmBlock& block);

	/**
	 * Create a copy of the StateBlock with the same properties.
	 *
	 * @return A shared pointer to a copy of the StateBlock.
	 */
	not_null<std::shared_ptr<StateBlock<>>> clone() override;

	DynamicsModel generate_dynamics(GenXhatPFunction,
	                                aspn_xtensor::TypeTimestamp time_from,
	                                aspn_xtensor::TypeTimestamp time_to) override;

protected:
	/**
	 * Calculates the `F` and `Q` matrices.
	 *
	 * @param F An NxN empty matrix, where N is the number of states declared in this block.
	 * @param Q An NxN empty matrix, where N is the number of states declared in this block.
	 */
	virtual void populate_f_and_q(Matrix& F, Matrix& Q);

	/**
	 * The time constants for each FOGM process.
	 */
	Vector time_constants;

	/**
	 * The 1-sigma value of each FOGM process at steady-state.
	 */
	Vector state_sigmas;
};

}  // namespace filtering
}  // namespace navtk
