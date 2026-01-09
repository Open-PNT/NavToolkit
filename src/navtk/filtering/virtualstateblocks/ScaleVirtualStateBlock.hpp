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
 * Transformer that is used to scale/unscale an EstimateWithCovariance.
 */
class ScaleVirtualStateBlock : public VirtualStateBlock {
public:
	/**
	 * Constructor.
	 *
	 * @param current Represents the initial format of the states prior
	 * to scaling.
	 * @param target Represents the final format of the states after
	 * scaling.
	 * @param scale An Nx1 vector of scale factors that when multiplied
	 * element wise with the N elements of an
	 * EstimateWithCovariance.estimate in the `current` format, produces
	 * an EstimateWithCovariance.estimate in the `target` format.
	 */
	ScaleVirtualStateBlock(std::string current, std::string target, const Vector& scale);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	/**
	 * Converts a state estimate by scaling, where the resulting
	 * estimate is Ax, where A is an NxN matrix with scaling elements along
	 * the diagonal and 0 elsewhere.
	 *
	 * @param x State estimate in the `current` reperesentation.
	 *
	 * @return State estimate in the `target` representation.
	 */
	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

	/**
	 * Calculates the Jacobian of the transform.
	 *
	 * @return The Jacobian of the transform, which is just a NxN matrix
	 * with scaling elements along the diagonal and 0 elsewhere.
	 */
	virtual Matrix jacobian(const Vector&, const aspn_xtensor::TypeTimestamp&) override;

private:
	Matrix jac;
};

}  // namespace filtering
}  // namespace navtk
