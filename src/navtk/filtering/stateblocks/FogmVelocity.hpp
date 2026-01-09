#pragma once

#include <memory>
#include <string>

#include <navtk/filtering/stateblocks/FogmBlock.hpp>
#include <navtk/filtering/stateblocks/discretization_strategy.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Models a set of position and velocity states in 3 dimensions, where the dynamics of the velocity
 * states are first-order Gauss-Markov processes.
 */
class FogmVelocity : public FogmBlock {
public:
	/**
	 * @param label The label uniquely identifying this particular set of states.
	 * @param time_constants The FOGM time constants (decay term) for each velocity state in
	 * seconds.
	 * @param state_sigmas The 1-sigma value of each velocity state at steady-state where the first
	 * three values are in meters and the last three are in m/s.
	 * @param num_dimensions The number of dimensions of space needed. For example, if
	 * `num_dimensions=2`, then this block will have 4 states: X and Y position and X and Y
	 * velocity.
	 * @param discretization_strategy Determines how the matrices `F` and `Q` will be linearized to
	 * produce `Phi` and `Qd`. Options include a first-order, second-order, or full discretization.
	 */
	FogmVelocity(
	    const std::string& label,
	    Vector time_constants,
	    Vector state_sigmas,
	    size_t num_dimensions,
	    DiscretizationStrategy discretization_strategy = &full_order_discretization_strategy);

	/**
	 * @param label The label uniquely identifying this particular set of states.
	 * @param time_constant The FOGM time constant (decay term) applied to all states.
	 * @param state_sigma The 1-sigma value at steady-state applied to all states.
	 * @param num_dimensions The number of dimensions of space needed. For example, if
	 * `num_dimensions=2`, then this block will have 4 states: X and Y position and X and Y
	 * velocity.
	 * @param discretization_strategy Determines how the matrices `F` and `Q` will be linearized to
	 * produce `Phi` and `Qd`. Options include a first-order, second-order, or full discretization.
	 */
	FogmVelocity(
	    const std::string& label,
	    double time_constant,
	    double state_sigma,
	    size_t num_dimensions,
	    DiscretizationStrategy discretization_strategy = &full_order_discretization_strategy);

	/**
	 * Create a copy of the StateBlock with the same properties.
	 *
	 * @return A shared pointer to a copy of the StateBlock.
	 */
	not_null<std::shared_ptr<StateBlock<>>> clone() override {
		return std::make_shared<FogmVelocity>(*this);
	}

protected:
	/**
	 * Calculates the `F` and `Q` matrices.
	 *
	 * @param F An NxN empty matrix, where N is the number of states declared in this block.
	 * @param Q An NxN empty matrix, where N is the number of states declared in this block.
	 */
	virtual void populate_f_and_q(Matrix& F, Matrix& Q) override;
};

}  // namespace filtering
}  // namespace navtk
