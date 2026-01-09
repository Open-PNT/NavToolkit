#pragma once

#include <navtk/experimental/random.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {
namespace experimental {

/**
 * A state block that models one or more sampled FOGM (first-order Gauss-Markov) processes.
 */
class SampledFogmBlock : public StateBlock<> {
public:
	/**
	 * Constructor for the SampledFogmBlock.
	 * @param label The label of the state block.
	 * @param time_constants The FOGM time constants (decay term) for each state in seconds.
	 * @param state_sigmas The 1-sigma value of each state at steady-state. The units of this value
	 * should match the units of the states this StateBlock is estimating.
	 * @param num_states How many total states to add (i.e. length of \p time_constants
	 * and \p state_sigmas).
	 * @param rng A shared pointer to a random number generator object.
	 */
	SampledFogmBlock(
	    const std::string& label,
	    Vector time_constants,
	    Vector state_sigmas,
	    size_t num_states,
	    navtk::not_null<std::shared_ptr<navtk::experimental::RandomNumberGenerator>> rng);

	/**
	 * @param label The label uniquely identifying this particular set of states.
	 * @param time_constant The FOGM time constant (decay term) applied to all states.
	 * @param state_sigma The 1-sigma value at steady-state applied to all states.
	 * @param num_states How many total states to add.
	 * @param rng A shared pointer to a random number generator object.
	 */
	SampledFogmBlock(
	    const std::string& label,
	    double time_constant,
	    double state_sigma,
	    size_t num_states,
	    navtk::not_null<std::shared_ptr<navtk::experimental::RandomNumberGenerator>> rng);
	/**
	 * Clone the SampledFogmBlock.
	 * @return A shared pointer to a cloned copy of the SampledFogmBlock. Note: the random number
	 * stream of the cloned copy is not guaranteed to match the random number stream of the
	 * original, so the cloned block may produce different results.
	 */
	not_null<std::shared_ptr<StateBlock<>>> clone() override;
	/**
	 * Generate the propagation dynamics of the FOGM process state.
	 * @param time_from The time to start the propagation.
	 * @param time_to The time to end the propagation.
	 * @return A standard dynamics model containing g(x) which can propagate the fogm process
	 * state vector over the time interval. This block is designed to work in a particle filter,
	 * and so the propagation occurs entirely in g(x), while Phi and Qd are not used and likely
	 * invalid.
	 */
	StandardDynamicsModel generate_dynamics(GenXhatPFunction,
	                                        aspn_xtensor::TypeTimestamp time_from,
	                                        aspn_xtensor::TypeTimestamp time_to) override;

protected:
	/**
	 * The FOGM time constants (decay term) for each state in seconds.
	 */
	Vector time_constants;
	/**
	 * The 1-sigma value of each state at steady-state. The units of these values
	 * should match the units of the states this StateBlock is estimating.
	 */
	Vector state_sigmas;

private:
	/**
	 * Shared pointer to random number generator object.
	 */
	std::shared_ptr<navtk::experimental::RandomNumberGenerator> rng;
};
}  // namespace experimental
}  // namespace filtering
}  // namespace navtk
