#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <xtensor/views/xindex_view.hpp>
#include <xtensor/views/xstrided_view.hpp>
#include <xtensor/views/xview.hpp>

#include <navtk/filtering/containers/LinearizedStrategyBase.hpp>
#include <navtk/filtering/fusion/strategies/FusionStrategy.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {
namespace experimental {

/**
 * An intermediary class which implements custom functions from the defaults provided by
 * FusionStrategy and stores the state of the RbpfStrategy.
 */
class RbpfModel : virtual public FusionStrategy {
public:
	/**
	 * A convenience type for storing state indices.
	 */
	typedef std::vector<Size> StateIndices;
	/**
	 * If the target number of particles is not set by the constructor then it is set by this
	 * constant.
	 */
	static constexpr Size DEFAULT_PARTICLE_COUNT = 100;

	/**
	 * @param particle_count Determines how many particles will be used for each state.
	 * @param calc_single_jacobian Setting to `true` saves computation at a potential (usually
	 * small) loss of accuracy. In this mode, any portions of the covariance returned by
	 * #get_covariance that is associated with linear states is taken from the first covariance
	 * particle, this being the only linear covariance propagated and updated in the RbpfStrategy
	 * when its `calc_single_jacobian` argument is true. When `false`, the linear covariance is
	 * calculated as the weighted sum of covariance of each particle's linearized states.
	 *
	 * @throw std::invalid_argument If particle count is less than one and the error mode is
	 * ErrorMode::DIE.
	 */
	RbpfModel(Size particle_count = DEFAULT_PARTICLE_COUNT, bool calc_single_jacobian = true);
	/**
	 * @return The current number of particles for each state.
	 */
	Size count_particles() const;
	/**
	 * @return The target number of particles for each state.
	 */
	Size get_particle_count_target() const;
	/**
	 * Sets the target number of particles for each state.
	 *
	 * @param particle_count_target The new target number of particles for each state.
	 * @throw std::runtime_error If the particle count changed after filter initialization and when
	 * the error mode is ErrorMode::DIE.
	 */
	void set_particle_count_target(Size particle_count_target);

	/**
	 * Marks states as particle states.
	 *
	 * @param marked_states A vector of indices which marks the corresponding indices of the state
	 * estimate to be filtered as particle states rather than linear states. New states are linear
	 * until marked.
	 * @param jitter_scales A vector of values that provides the jitter scale factor for each
	 * associated marked (particle) state. If only one jitter scale factor is provided then that
	 * value applies to all particle states. If no jitter scale values are provided then no jitter
	 * will be applied to the marked (particle) states.
	 * @throw std::invalid_argument If `jitter_scales.size() > marked_states.size()`; or
	 * if `jitter_scales.size() != 1` and `jitter_scales.size() != marked_states.size()`. The error
	 * mode is ErrorMode::DIE.
	 */
	void set_marked_states(const StateIndices& marked_states,
	                       const std::vector<double>& jitter_scales = {0.});
	/**
	 * @return A vector of indices of the state estimate which are marked as particle states.
	 */
	StateIndices get_marked_states() const;
	/**
	 * @return A vector, the size of the state, where marked as particle jitter states have values
	 * greater to or equal to zero and linear states have values of zero.
	 */
	std::vector<double> get_jitter_scaling() const;

	/**
	 * Set the jitter values for the particle states.
	 *
	 * @param jitter_scales A vector of values that provides the jitter scale factor for each
	 * associated particle state or if only one scale factor is provided it applies to all particle
	 * states.
	 * @throw std::invalid_argument If `jitter_scales` is empty or has a length that is greater
	 * than the number of marked states and the error mode is ErrorMode::DIE.
	 */
	void set_jitter_scaling(const std::vector<double>& jitter_scales);
	/**
	 * @return The number of states.
	 */
	Size get_num_states() const override;
	/**
	 * @return The state estimate.
	 */
	Vector get_estimate() const override;
	/**
	 * @return The state covariance.
	 */
	Matrix get_covariance() const override;
	/**
	 * @return A deep copy of the state.
	 */
	not_null<std::shared_ptr<FusionStrategy>> clone() const override;

	/**
	 * @return `true` if there are any marked states in the model.
	 */
	bool any_nonlinear() const;

	/**
	 * @return `true` if there are any unmarked states in the model.
	 */
	bool any_linear() const;

	/**
	 * Adds `P` to its transpose and divides by 2. Useful for keeping `P` symmetric from small
	 * numerical errors.
	 * @param P A square matrix that must be symmetric.
	 *
	 * @return The symmetric version of `P`.
	 */
	Matrix symmetricize_covariance(Matrix& P) const;

	/**
	 * Adds `P` to its transpose and divides by 2. Useful for keeping `P` symmetric from small
	 * numerical errors.
	 */
	void symmetricize_covariance();

	/**
	 * A flag to use a single Jacobian matrix to save processing
	 */
	bool calc_single_jacobian;

protected:
	/**
	 * Verify the user has marked the states and jitter appropriately
	 *
	 * @param marked_states The user-determined nonlinear states
	 * @param jitter_scales The jitter level assigned to each nonlinear state - can have single
	 * scale to apply to all states.
	 *
	 * @return `true` if marking is allowed, `false` if there are incorrect markings.
	 */
	bool validate_jitter_values(const RbpfModel::StateIndices& marked_states,
	                            const std::vector<double>& jitter_scales);

	/**
	 * Adds states to the estimate and covariance.
	 *
	 * @param initial_estimate The estimate used to initialize the new states.
	 * @param initial_covariance The covariance used to initialize the new states.
	 */
	void on_fusion_engine_state_block_added_impl(Vector const& initial_estimate,
	                                             Matrix const& initial_covariance) override;
	/**
	 * Set a slice of the state estimate.
	 *
	 * @param new_estimate The new estimate slice.
	 * @param first_index The first index of the state estimate that should be replaced by \p
	 * new_estimate .
	 */
	void set_estimate_slice_impl(Vector const& new_estimate, Size first_index) override;
	/**
	 * Change the value of a slice of the covariance matrix, starting at the given \p first_row and
	 * \p first_col.
	 *
	 * @param new_covariance The new covariance slice to replace all or a portion of the covariance.
	 * @param first_row The first row of the covariance to be replaced.
	 * @param first_col The first column of the covariance to be replaced.
	 */
	void set_covariance_slice_impl(Matrix const& new_covariance,
	                               Size first_row,
	                               Size first_col) override;
	/**
	 * Called by `on_fusion_engine_state_block_removed` with sanitized inputs.
	 * @param first_index first state to remove, guaranteed to be greater than or equal to 0 and l
	 * ess than get_num_states().
	 * @param count number of states to remove. `first_index + count` is guaranteed to be less than
	 * or equal to get_num_states().
	 */
	void on_fusion_engine_state_block_removed_impl(Size first_index, Size count) override;
	/**
	 * Invokes FusionStrategy::on_state_count_changed and RbpfModel::on_state_marks_changed.
	 */
	void on_state_count_changed() override;
	/**
	 * Invokes FusionStrategy::on_state_marks_changed and RbpfModel::update_particle_count.
	 */
	virtual void on_state_marks_changed();
	/**
	 * Updates #nonlinear_states, #linear_states, #drop_linear, #keep_linear.
	 */
	void update_state_mark_helper_variables();
	/**
	 * Resamples #state_particles and #covariance_particles and resets #state_particle_weights.
	 */
	void update_particle_count();
	/**
	 * A vector that indicates the jitter scaling of the main diagonal covariances of the all
	 * states. Entries set to zero for all linear states.
	 */
	std::vector<double> jitter_levels;
	/**
	 * A vector that indicates which states are nonlinear. If `is_nonlinear[idx]` is `true` then
	 * `estimate[idx]` has been marked as a particle state.
	 */
	std::vector<bool> is_nonlinear;
	/**
	 * An NxM matrix where N is the number of states and M is the number of particles.
	 */
	Matrix state_particles;
	/**
	 * A vector of length N where N is the number of particles.
	 */
	Vector state_particle_weights;
	/**
	 * An MxNxN matrix where N is the number of states and M is the number of particles.
	 */
	Tensor<3> covariance_particles;
	/**
	 * The current state estimate.
	 */
	Vector estimate;
	/**
	 * The last covariance calculated or set by the user.
	 */
	mutable Matrix covariance;
	/**
	 * A flag indicating whether #covariance is valid. It is set to `true` after propagation and
	 * updates and set `false` after it is recalculated.
	 */
	mutable bool covariance_stale = false;

	/**
	 * An array of indices corresponding to the linear states.
	 */
	StateIndices linear_states;
	/**
	 * An array of indices corresponding to the particle states.
	 */
	StateIndices nonlinear_states;
	/**
	 * A convenience variable which can be used with `xt::view`to drop the linear states, resulting
	 * in just the nonlinear states.
	 */
	decltype(xt::drop(StateIndices())) drop_linear = xt::drop(StateIndices());
	/**
	 * A convenience variable which can be used with `xt::view`to keep the linear states, resulting
	 * in just the linear states.
	 */
	decltype(xt::keep(StateIndices())) keep_linear = xt::keep(StateIndices());


private:
	Vector compute_state_noise_weights() const;
	void apply_state_particle_noise(Size first_index, Size stop_index);
	// Given the absolute range of indices between first_index and first_index+count, return (if
	// any) the equivalent indices relative to a block compromised of only the linear states.
	StateIndices get_relative_linear_indices(Size first_index, Size stop_index) const;
	Size particle_count_target = DEFAULT_PARTICLE_COUNT;
};

}  // namespace experimental
}  // namespace filtering
}  // namespace navtk
