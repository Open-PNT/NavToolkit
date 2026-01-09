#pragma once

#include <functional>

#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A measurement model suitable for use with a sampled model.
 */
class SampledMeasurementModel {
public:
	/**
	 * Function type used as the state-update function.
	 */
	typedef std::function<Vector(Vector)> SampledUpdateFunction;

	/// The measurement update vector.
	Vector z;

	/**
	 * The measurement prediction function used to update the state of a sampled model system.
	 * Accepts the state vector (`xhat`) and returns the expected measurement vector.
	 */
	SampledUpdateFunction h;

	/**
	 * Set fields to the given values.
	 *
	 * @param z The value to store in #z.
	 * @param h The value to store in #h.
	 */
	SampledMeasurementModel(Vector z, SampledUpdateFunction h) : z(std::move(z)), h(std::move(h)) {}
};

}  // namespace filtering
}  // namespace navtk
