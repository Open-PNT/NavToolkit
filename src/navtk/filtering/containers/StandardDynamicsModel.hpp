#pragma once

#include <functional>

#include <navtk/filtering/utils.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A container for the description of a non-linear system's dynamics model. This object is not
 * ordinarily used directly. Instead a navtk::filtering::StandardStateBlock is produced by a
 * StateBlock for the filter.
 */
class StandardDynamicsModel {
public:
	/**
	 * Function type used as the state-transition function \f$ g(x) \f$.
	 */
	typedef std::function<Vector(const Vector&)> StateTransitionFunction;

	/**
	 * Set fields to the given values using `std::move`. Also validates matrix sizes.
	 *
	 * @param g The value to store in #g.
	 * @param Phi The value to store in #Phi.
	 * @param Qd The value to store in #Qd.
	 *
	 * @throw std::range_error If Phi or Qd are not NxN and the error mode is ErrorMode::DIE.
	 */
	StandardDynamicsModel(StateTransitionFunction g, Matrix Phi, Matrix Qd);

	/**
	 * Set fields to the given values using `std::move` and sets #g to \f$ g(x)
	 * = \Phi x \f$. Also validates matrix sizes.
	 *
	 * @param Phi The value to store in #Phi.
	 * @param Qd The value to store in #Qd.
	 *
	 * @throw std::range_error If Phi or Qd are not NxN and the error mode is ErrorMode::DIE.
	 */
	StandardDynamicsModel(Matrix Phi, Matrix Qd);

	/**
	 * The non-linear discrete-time state-transition function (i.e. \f$ g(x) \f$ in \f$ x_k =
	 * g(x_{k-1}) + w_k \f$). Accepts the state vector and returns the propagated state vector.
	 */
	StateTransitionFunction g;

	/**
	 * Jacobian of #g (i.e. Phi in the linearized equation \f$ x_k = \Phi x_{k-1} \f$).
	 */
	Matrix Phi;

	/**
	 * Discrete-time process noise covariance matrix.
	 */
	Matrix Qd;
};

/**
 * The StandardDynamicsModel will be used most frequently, so this alias is used to for convenience.
 */
typedef StandardDynamicsModel DynamicsModel;

}  // namespace filtering
}  // namespace navtk
