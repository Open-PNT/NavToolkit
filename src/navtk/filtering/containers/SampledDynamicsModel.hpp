#pragma once

#include <functional>

#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A container for the description of a sampled system's dynamics model. This object is not
 * ordinarily used directly. Instead a SampledDynamicsModel is produced by a StateBlock for the
 * filter.
 */
class SampledDynamicsModel {
public:
	/**
	 * Function type used as the state-transition function.
	 */
	typedef std::function<Vector(Vector)> SampledPropagationFunction;

	/**
	 * Set fields to the given values.
	 *
	 * @param g The value to store in #g, which should be an instance of
	 * #SampledPropagationFunction. Accepts the state vector (`xhat`) and returns a Vector.
	 */
	SampledDynamicsModel(SampledPropagationFunction g) : g(std::move(g)) {}

	/**
	 * The state-transition function used to propagate the state of a sampled model system. Accepts
	 * the state vector and returns the propagated state vector.
	 */
	SampledPropagationFunction g;
};
}  // namespace filtering
}  // namespace navtk
