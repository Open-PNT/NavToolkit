#pragma once

#include <memory>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/linear_algebra.hpp>

namespace navtk {
namespace filtering {

/**
 * Abstract class for VirtualStateBlock implementations whose Jacobians are calculated numerically.
 * Child classes implement a single protected function, #fx, which is a function that transforms a
 * state estimate navtk::Vector from one representation to another. This is wrapped in a lambda
 * function with a `this` capture (`[&]`) and supplied to first-order approximation functions.
 *
 * VirtualStateBlocks that have similar behavior but do not require information beyond the state
 * Vector `x` and Time inputs to convert `x` or calculate a Jacobian may alternatively use
 * FirstOrderVirtualStateBlock.
 */
class NumericalVirtualStateBlock : public VirtualStateBlock {

public:
	/**
	 * Constructor.
	 *
	 * @param current The label associated with base state.
	 * @param target Label to associate with the transformed state.
	 */
	NumericalVirtualStateBlock(std::string current, std::string target);

	/**
	 * Converts \p ec by wrapping #fx in a `[&]` lambda and passing it to
	 * navtk::filtering::first_order_approx.
	 *
	 * @param ec EstimateWithCovariance from the StateBlock to be transformed; format depends on
	 * implementing class.
	 * @param time Time of validity for \p ec.
	 *
	 * @return An EstimateWithCovariance transformed by #fx, valid to first-order.
	 */
	EstimateWithCovariance convert(const EstimateWithCovariance& ec,
	                               const aspn_xtensor::TypeTimestamp& time) override;

	/**
	 * Evaluates #fx with the function inputs.
	 *
	 * @param x State vector to convert.
	 * @param time Time of validity for state \p x.
	 *
	 * @return Converted state.
	 */
	Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp& time) override;

	/**
	 * Calculate the Jacobian of #fx using navtk::filtering::calc_numerical_jacobian with default
	 * args.
	 *
	 * @param x State vector to linearize about; format depends on implementing class.
	 * @param time Time of validity for \p x.
	 *
	 * @return Jacobian matrix that maps \p x into the transformed state, valid to first-order.
	 */
	Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp& time) override;

protected:
	/**
	 * Function that converts the StateBlock estimate \p x in some manner and returns the result.
	 *
	 * @param x State vector to transform.
	 * @param time Time of validity for \p x.
	 *
	 * @return The transformed state vector.
	 */
	virtual Vector fx(const Vector& x, const aspn_xtensor::TypeTimestamp& time) = 0;
};

}  // namespace filtering
}  // namespace navtk
