#pragma once

#include <memory>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {

/**
 * VirtualStateBlock that applies an arbitrary mapping to an
 * EstimateWithCovariance valid to first-order.
 */
class FirstOrderVirtualStateBlock : public VirtualStateBlock {
public:
	/**
	 * Constructor.
	 *
	 * @param current Represents the initial format of the
	 * EstimateWithCovariance prior to transformation.
	 * @param target Represents the final format of the
	 * EstimateWithCovariance after it has been transformed.
	 * @param fx Function that takes a state vector in the the `current`
	 * format and converts it to the `target` format.
	 * @param jx Function that returns the Jacobian of `fx`. If NULL the
	 * the Jacobian will be calculated numerically (slowly).
	 */
	FirstOrderVirtualStateBlock(std::string current,
	                            std::string target,
	                            std::function<Vector(const Vector&)> fx,
	                            std::function<Matrix(const Vector&)> jx = 0);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	/**
	 * Produces `fx(x)`, using the `fx` function passed to the constructor.
	 *
	 * @param x State estimate to convert, in the `current` representation.
	 *
	 * @return State estimate in the `target` representation.
	 */
	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

	/**
	 * Produces `jx(x)`, using the `jx` function passed to the constructor or
	 * the numerical derivative of fx if `jx` was not supplied.
	 *
	 * @param x State estimate used in evaluating the jacobian, in the `current` representation.
	 *
	 * @return Jacobian, given x.
	 */
	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;


private:
	std::function<Vector(const Vector&)> fx = 0;
	std::function<Matrix(const Vector&)> jx = 0;
};

}  // namespace filtering
}  // namespace navtk
