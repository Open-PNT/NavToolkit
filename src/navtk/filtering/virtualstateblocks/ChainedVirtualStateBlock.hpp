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
 * Class that represents a composite of other VirtualStateBlock
 * classes.
 */
class ChainedVirtualStateBlock : public VirtualStateBlock {
public:
	/**
	 * Constructor.
	 *
	 * @param to_chain Vector of pointers to VirtualStateBlocks that this
	 * instance represents. Must be in order (the first element performs
	 * the initial transform from some real representation to an
	 * intermediate value, and so on, while last element transforms to
	 * the final expected format). The `current` field is extracted from
	 * the first element, and the `target` field from the last.
	 */
	ChainedVirtualStateBlock(std::vector<not_null<std::shared_ptr<VirtualStateBlock>>> to_chain);

	/**
	 * Default destructor.
	 */
	virtual ~ChainedVirtualStateBlock() = default;

	/**
	 * Copy constructor.
	 *
	 * @param other Another instance of this class whose fields are copied.
	 */
	ChainedVirtualStateBlock(const ChainedVirtualStateBlock& other);

	/**
	 * Copy assignment operator.
	 *
	 * @param other Another instance of this class whose fields are copied.
	 *
	 * @return A copy of \p other .
	 */
	ChainedVirtualStateBlock& operator=(ChainedVirtualStateBlock const& other);

	/**
	 * Default move constructor.
	 *
	 * @param other Another instance of this class.
	 */
	ChainedVirtualStateBlock(ChainedVirtualStateBlock&& other) = default;

	/**
	 * Default move assignment operator.
	 *
	 * @param other Another instance of this class.
	 *
	 * @return An instance of ChainedVirtualStateBlock with the data from \p other .
	 */
	ChainedVirtualStateBlock& operator=(ChainedVirtualStateBlock&& other) = default;

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	/**
	 * Converts an EstimateWithCovariance through each individual
	 * transform provided to the constructor, in sequence.
	 *
	 * @param ec EstimateWithCovariance in the `current` representation.
	 * @param time Time of validity for ec.
	 *
	 * @return EstimateWithCovariance in the `target` representation. Results
	 * (including Jacobian calculated as a side-effect) are cached such that
	 * a subsequent call to convert(), convert_estimate() or jacobian() with the
	 * same state vector and time inputs will return the cached values. Any changes
	 * made to the state of constituent VirtualStateBlocks will not be reflected
	 * in such cases.
	 */
	virtual EstimateWithCovariance convert(const EstimateWithCovariance& ec,
	                                       const aspn_xtensor::TypeTimestamp& time) override;

	/**
	 * Converts an estimate through each individual
	 * transform provided to the constructor, in sequence.
	 *
	 * @param x Estimate in the `current` representation.
	 * @param time Time of validity for state.
	 *
	 * @return State estimate in the `target` representation. Warnings regarding
	 * caching outlined in the convert() documentation apply.
	 */
	virtual Vector convert_estimate(const Vector& x,
	                                const aspn_xtensor::TypeTimestamp& time) override;

	/**
	 * Calculate the composite Jacobian of the underlying chained
	 * transform functions `f(x)`, evaluated about the input `x`.
	 * It is the product of the Jacobians of each individual transform.
	 *
	 * @param x State vector about which to evaluate the Jacobian.
	 * @param time Time of validity for `x`.
	 *
	 * @return An MxN Matrix representing the first-order derivative of
	 * the mapping function `f(x)` with respect to the M states of `x`. Warnings
	 * regarding caching outlined in the convert() documentation apply.
	 */
	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp& time) override;

private:
	std::vector<not_null<std::shared_ptr<VirtualStateBlock>>> chained;

	aspn_xtensor::TypeTimestamp last_time = aspn_xtensor::to_type_timestamp();
	Vector last_x;
	Matrix last_jac;
	Vector last_x_conv;
	bool conv_synced;

	bool cache_safe(const Vector& x, const aspn_xtensor::TypeTimestamp& time);
};

}  // namespace filtering
}  // namespace navtk
