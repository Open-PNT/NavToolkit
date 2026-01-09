#pragma once

#include <memory>

#include <navtk/factory.hpp>
#include <navtk/filtering/fusion/strategies/FusionStrategy.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A simple container for holding an estimate and covariance.
 *
 * Implements the filtering::FusionStrategy interface for convenience.
 */
struct LinearizedStrategyBase : virtual public FusionStrategy {

	/**
	 * Initializes #estimate and #covariance using FusionStrategy::get_estimate and
	 * FusionStrategy::get_covariance.
	 *
	 * @param src A fusion strategy model which produces an estimate and covariance.
	 */
	LinearizedStrategyBase(const FusionStrategy& src);
	LinearizedStrategyBase() = default;

	Vector get_estimate() const override;
	Matrix get_covariance() const override;
	not_null<std::shared_ptr<FusionStrategy>> clone() const override;

protected:
	/**
	 * Adds states to the estimate and covariance.
	 *
	 * @param initial_estimate The estimate to append to #estimate.
	 * @param initial_covariance The covariance to append to #covariance.
	 */
	void on_fusion_engine_state_block_added_impl(Vector const& initial_estimate,
	                                             Matrix const& initial_covariance) override;

	/**
	 * Set a slice of #estimate.
	 *
	 * @param new_estimate The new estimate slice.
	 * @param first_index The first index of #estimate that should be replaced by \p new_estimate .
	 */
	void set_estimate_slice_impl(Vector const& new_estimate, Size first_index) override;
	/**
	 * Change the value of a slice of the covariance matrix, starting at the
	 * given \p first_row and \p first_col.
	 *
	 * @param new_covariance The new covariance slice to replace all or a portion of the covariance
	 * matrix P.
	 * @param first_row The first row of P to be replaced.
	 * @param first_col The first column of P to be replaced.
	 */
	void set_covariance_slice_impl(Matrix const& new_covariance,
	                               Size first_row,
	                               Size first_col) override;
	/**
	 * Called by `on_fusion_engine_state_block_removed` with sanitized inputs.
	 * @param first_index first state to remove, guaranteed to be >= 0 and < get_num_states().
	 * @param count number of states to remove. `first_index + count` is guaranteed to be <=
	 * get_num_states().
	 */
	void on_fusion_engine_state_block_removed_impl(Size first_index, Size count) override;

	/**
	 * An Nx1 estimate.
	 */
	Vector estimate = zeros(0);

	/**
	 * An NxN covariance matrix for the estimate.
	 */
	Matrix covariance = zeros(0, 0);
};
}  // namespace filtering
}  // namespace navtk
