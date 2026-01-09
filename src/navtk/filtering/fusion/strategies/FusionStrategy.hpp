#pragma once

#include <memory>

#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Interface for an internal representation of states used by a fusion engine that can be
 * represented as a state vector (#get_estimate) and a covariance matrix (#get_covariance).
 *
 * An implementation of FusionStrategy is required by a fusion engine. The fusion engine uses a
 * fusion strategy to combine the information from state blocks and measurement processors to
 * propagate and update the estimate and covariance of the states in a fusion engine. The fusion
 * strategy provides the algorithm backing the `StandardFusionEngine::propagate` and
 * `StandardFusionEngine::update` methods, which predict future values of states and adjust states
 * based on new measurements, respectively.
 *
 * If a fusion strategy natively deals in the estimate and covariance (as in
 * navtk::filtering::EkfStrategy, for example), the navtk::filtering::LinearizedStrategyBase
 * struct implements this interface with the estimate and covariance backed by simple fields.
 *
 * Subclasses should verify their behavior conforms to expectations by adding themselves to the
 * typed test case in FusionStrategyTests.cpp.
 */
class FusionStrategy {
public:
	virtual ~FusionStrategy() = default;

	/**
	 * @return The number of states represented by this filter. This value
	 * (`engine.get_num_states()`) is equal to `num_rows(engine.get_estimate())`, but may be
	 * faster to compute depending on the underlying fusion engine implementation.
	 */
	virtual Size get_num_states() const;

	/**
	 * A method that is called by the fusion engine when a state block has been added. It is not
	 * intended to be called directly. It increases the size of the estimate and covariance by the
	 * given number of states. The newly-added states are initialized to zero. The newly added
	 * covariances are initialized to `eye(how_many)`.
	 *
	 * @param how_many Number of states to add
	 *
	 * @return The index of the first state added (let this be N), such that
	 * `xt::view(get_estimate(), xt::range(N, N + how_many))` will be the new states.
	 */
	Size on_fusion_engine_state_block_added(Size how_many);
	/**
	 * A method that is called by the fusion engine when a state block has been added. It is not
	 * intended to be called directly. It increases the size of the estimate and covariance. The
	 * newly-added states are initialized to \p initial_estimate. The newly added covariances are
	 * initialized to \p initial_covariance.
	 *
	 * @param initial_estimate The initial estimate of the new states.
	 * @param initial_covariance The initial covariance of the new states.
	 *
	 * @return The index of the first state added (let this be N), such that
	 * `xt::view(get_estimate(), xt::range(N, N + how_many))` will be the new states.
	 */
	Size on_fusion_engine_state_block_added(Vector const &initial_estimate,
	                                        Matrix const &initial_covariance);
	/**
	 * A method that is called by the fusion engine when a state block has been added. It is not
	 * intended to be called directly. It increases the size of the estimate and covariance. The
	 * newly-added states are initialized to \p initial_estimate. The newly added covariances are
	 * initialized to \p initial_covariance. The off-diagonal covariance between the existing states
	 * and new states is initialized to \p cross_covariance.
	 *
	 * @param initial_estimate The initial estimate of the new states.
	 * @param initial_covariance The initial covariance of the new states.
	 * @param cross_covariance The correlation between the existing states and the new states. If
	 * the fusion strategy stores off a covariance matrix then \p cross_covariance is applied the
	 * upper right of that matrix and its tranpose is applied to the lower left.
	 *
	 * @return The index of the first state added (let this be N), such that
	 * `xt::view(get_estimate(), xt::range(N, N + how_many))` will be the new states.
	 */
	Size on_fusion_engine_state_block_added(Vector const &initial_estimate,
	                                        Matrix const &initial_covariance,
	                                        Matrix const &cross_covariance);

	/**
	 * A method that is called by the fusion engine when a state block has been removed. It is not
	 * intended to be called directly. It removes \p count states from the strategy starting at \p
	 * first_index.
	 *
	 * @param first_index The index of the first state to remove.
	 * @param count The number of states to remove.
	 */
	void on_fusion_engine_state_block_removed(Size first_index, Size count);

	/**
	 * @return The current state estimate.
	 */
	virtual Vector get_estimate() const = 0;

	/**
	 * Set a new state vector.
	 *
	 * @param new_estimate The new estimate slice.
	 * @param first_index The first index of the estimate that should be replaced by \p
	 * new_estimate .
	 */
	void set_estimate_slice(Vector const &new_estimate, Size first_index = 0);

	/**
	 * @return The current state covariance.
	 */
	virtual Matrix get_covariance() const = 0;

	/**
	 * Change the value of a slice of the covariance matrix, starting at the given \p first_row and
	 * \p first_col.
	 *
	 * @param new_covariance The new covariance slice to replace all or a portion of the covariance
	 * matrix P.
	 * @param first_row The first row of P to be replaced.
	 * @param first_col The first column of P to be replaced.
	 */
	void set_covariance_slice(Matrix const &new_covariance, Size first_row, Size first_col);

	/**
	 * Change a subset of the covariance matrix, along its diagonal, starting with the given state
	 * index.
	 *
	 * `set_covariance_slice(eye(3), 2)` is equivalent to `set_covariance_slice(eye(3), 2, 2)`.
	 *
	 * Subclasses should override #set_covariance_slice_impl to customize this behavior.
	 *
	 * @param new_covariance The new covariance slice to replace all or a portion of the covariance
	 * matrix P.
	 * @param first_state The first state of the covariance to be replaced by \p new_covariance .
	 */
	void set_covariance_slice(Matrix const &new_covariance, Size first_state = 0);

	// TODO (PNTOS-266) Find a workaround in Pybind11 for returning std::`unique_ptr` and refactor
	//      clone() to return std::`unique_ptr` instead of `std::shared_ptr`.
	/**
	 * @return A deep copy of the state.
	 */
	virtual not_null<std::shared_ptr<FusionStrategy>> clone() const = 0;

	/**
	 * Adds P to its transpose and divides by 2 if covariance tests as asymmetric. Useful for
	 * keeping P symmetric from small numerical errors.
	 *
	 * @param rtol Relative tolerance allowed between 2 elements; unitless.
	 * @param atol Absolute tolerance allowed between 2 elements; units determined by elements
	 * being compared.
	 */
	void symmetricize_covariance(double rtol = 1e-5, double atol = 1e-8);

protected:
	/**
	 * Add the given new states to the end of the estimate and covariance. Called by
	 * #on_fusion_engine_state_block_added with pre-validated inputs.
	 *
	 * @param initial_estimate The state estimate used to initialize the new states.
	 * @param initial_covariance The state covariance used to initialize the new states.
	 */
	virtual void on_fusion_engine_state_block_added_impl(Vector const &initial_estimate,
	                                                     Matrix const &initial_covariance) = 0;

	/**
	 * Called by #set_covariance_slice and some implementations of
	 * #on_fusion_engine_state_block_added to change the covariance matrix.
	 * @param new_covariance validated matrix of values to replace on the covariance matrix.
	 * @param first_row row of the top-left corner of the coefficients to overwrite
	 * @param first_col column of the top-left corner of the coefficients to overwrite
	 */
	virtual void set_covariance_slice_impl(Matrix const &new_covariance,
	                                       Size first_row,
	                                       Size first_col) = 0;

	/**
	 * Called by set_estimate_slice with validated inputs. Subclasses must implement this
	 * function by replacing the `[first_index, first_index + num_rows(new_estimate))` range
	 * of states with the values in `new_estimate`.
	 * @param new_estimate new state values
	 * @param first_index where to start writing
	 */
	virtual void set_estimate_slice_impl(Vector const &new_estimate, Size first_index) = 0;

	/**
	 * Called by `on_fusion_engine_state_block_removed` with sanitized inputs.
	 * @param first_index first state to remove, guaranteed to be >= 0 and < get_num_states().
	 * @param count number of states to remove. `first_index + count` is guaranteed to be <=
	 * get_num_states().
	 */
	virtual void on_fusion_engine_state_block_removed_impl(Size first_index, Size count) = 0;

	/**
	 * Called after an operation such as
	 * `on_fusion_engine_state_block_added`/`on_fusion_engine_state_block_removed` changes the
	 * number of states.
	 */
	virtual void on_state_count_changed();
};

}  // namespace filtering
}  // namespace navtk
