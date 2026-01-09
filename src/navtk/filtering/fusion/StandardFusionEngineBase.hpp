#pragma once

#include <string>
#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {

/**
 * An interface for a container of state blocks with methods to modify the container as well as
 * modifying individual state blocks (using state block labels).
 */
class StandardFusionEngineBase {

public:
	virtual ~StandardFusionEngineBase() = default;

	/**
	 * Set the current time of the filter
	 *
	 * @param time The time to set the filter to
	 */
	virtual void set_time(const aspn_xtensor::TypeTimestamp &time) = 0;

	/**
	 * Get the current time of the filter
	 *
	 * @return The current time of the filter
	 */
	virtual aspn_xtensor::TypeTimestamp get_time() const = 0;

	/**
	 * Gets a list of the labels of real state blocks that have been added to this fusion engine.
	 * Note that this function does not work for virtual state blocks. See
	 * #get_virtual_state_block_target_labels instead.
	 *
	 * @return The list of real state block labels.
	 */
	virtual std::vector<std::string> get_state_block_names_list() const = 0;

	/**
	 * See if the fusion engine has a real state block with matching label.  Note that this function
	 * does not work for virtual state blocks.  See #has_virtual_state_block instead.
	 *
	 * @param label StateBlock label to look for.
	 *
	 * @return `true` if the fusion engine has corresponding StateBlock, `false` otherwise.
	 */
	virtual bool has_block(std::string const &label) const = 0;

	/**
	 * See if the fusion engine has a VirtualStateBlock capable of generating an estimate.
	 *
	 * @param target_label VirtualStateBlock to look for (via target label).
	 *
	 * @return `true` if the fusion engine has corresponding VirtualStateBlock and it can be linked
	 * to a StateBlock, `false` otherwise.
	 */
	virtual bool has_virtual_state_block(std::string const &target_label) const = 0;

	/**
	 * Get the real state block object if the fusion engine currently has it.  Note that this
	 * function does not work for virtual state blocks.
	 *
	 * @param label The unique label of the real state block.
	 *
	 * @return The reference to the StateBlock object.
	 *
	 * @throw std::invalid_argument If the label does not match any StateBlock.
	 */
	virtual not_null<std::shared_ptr<StateBlock<>>> get_state_block(std::string const &label) = 0;

	/**
	 * Get the real state block object if the fusion engine currently has it.  Note that this
	 * function does not work for virtual state blocks.
	 *
	 * @param label The unique label of the real state block.
	 *
	 * @return The reference to the StateBlock object.
	 *
	 * @throw std::invalid_argument If the label does not match any StateBlock.
	 */
	virtual not_null<std::shared_ptr<const StateBlock<>>> get_state_block(
	    std::string const &label) const = 0;

	/**
	 * Get the current covariance matrix for a StateBlock or VirtualStateBlock represented by
	 * \p label.
	 *
	 * @param label StateBlock label or VirtualStateBlock target label.
	 *
	 * @return The covariance matrix.  If \p label represents a virtual state block, the matrix will
	 * be in the virtual representation.
	 *
	 * @throw std::invalid_argument If the label does not match any StateBlocks or
	 * VirtualStateBlocks.
	 */
	virtual Matrix get_state_block_covariance(std::string const &label) const = 0;

	/**
	 * Sets the current covariance matrix for a real state block directly.  Note that this function
	 * does not work for virtual state blocks.
	 *
	 * @param label The unique label of the real state block.
	 * @param covariance The covariance matrix to set the real state block's covariance to.
	 *
	 * @throw std::range_error If the covariance is not NxN, where N is the number of states.
	 * @throw std::invalid_argument If the label does not match any StateBlock.
	 */
	virtual void set_state_block_covariance(std::string const &label, Matrix const &covariance) = 0;

	/**
	 * Get the current estimate vector for a StateBlock or VirtualStateBlock represented
	 * by \p label.
	 *
	 * @param label StateBlock label or VirtualStateBlock target label.
	 *
	 * @return The estimate vector.  If \p label represents a virtual state block, the vector will
	 * be in the virtual representation.
	 *
	 * @throw std::invalid_argument If the label does not match any StateBlocks or
	 * VirtualStateBlocks.
	 */
	virtual Vector get_state_block_estimate(std::string const &label) const = 0;

	/**
	 * Sets the current estimate vector for a real state block directly.  Note that this function
	 * does not work for virtual state blocks.
	 *
	 * @param label The unique label of the real state block.
	 * @param estimate The estimate vector to set the real state block's estimate to.
	 *
	 * @throw std::range_error If the estimate is not Nx1, where N is the number of states.
	 * @throw std::invalid_argument If the label does not match any StateBlock.
	 */
	virtual void set_state_block_estimate(std::string const &label, Vector const &estimate) = 0;

	/**
	 * Add a real state block to the fusion engine. A state block provides the fusion engine with a
	 * dynamics model, the number of states (StateBlock::num_states), and a label
	 * (StateBlock::get_label()). Let N represent StateBlock::num_states. When a state block is
	 * added to the fusion engine, the fusion strategy expands `xhat`(the concatenated state vector)
	 * by N. The added state estimates are set to 0 in `xhat` by default. The filter P (concatenated
	 * state covariance matrix) corresponding to `xhat` expands to (M+N)x(M+N), where M is the
	 * filter's current number of states. The corresponding state covariance matrix is set to eye(N)
	 * by default. `xhat` can be set using #set_state_block_estimate. P can be set using
	 * #set_state_block_covariance. The covariance between two state blocks can be set using
	 * #set_cross_term_covariance.
	 *
	 * The `shared_ptr` to this block will be retained until a corresponding call to
	 * #remove_state_block or until the the fusion engine is destroyed.
	 *
	 * Note that this function does not work for virtual state blocks.  See #add_virtual_state_block
	 * instead.
	 *
	 * @param block A description of dynamics and number of states uniquely identified by
	 * StateBlock::get_label().
	 */
	virtual void add_state_block(not_null<std::shared_ptr<StateBlock<>>> block) = 0;

	/**
	 * Remove a real state block from the fusion engine.  Note that this function does not work for
	 * virtual state blocks.  See #remove_virtual_state_block instead.
	 *
	 * Let N represent (StateBlock::num_states). The filter `xhat`(concatenated state vector)
	 * decreases from M to M-N, where M is the filter's current number of states. The filter P
	 * (concatenated state covariance matrix) corresponding to `xhat` decreases from MxM to
	 * (M-N)x(M-N).
	 *
	 * @param label The unique label of the StateBlock.
	 */
	virtual void remove_state_block(std::string const &label) = 0;

	/**
	 * Sets the cross term block matrix between two real state blocks in the internal discrete-time
	 * process noise covariance matrix.  Note that this function does not work for virtual state
	 * blocks.
	 *
	 * @param label1 The first real state block's unique label.
	 * @param label2 The second real state block's unique label.
	 * @param block The cross correlation terms for the discrete-time process noise covariance
	 * matrix between the states corresponding to \p label1 and the states corresponding to
	 * \p label2. The rows correspond to \p label1 and the columns correspond to \p label2. If
	 * \p label1 has M states and \p label2 has N states, then block will be an MxN matrix.
	 *
	 * @throw std::range_error If the block is not NxM, where N and M are the number of states in
	 *        the StateBlocks corresponding to \p label1 and \p label2 respectively.
	 * @throw std::invalid_argument If the labels do not match StateBlocks.
	 */
	virtual void set_cross_term_process_covariance(std::string const &label1,
	                                               std::string const &label2,
	                                               Matrix const &block) = 0;

	/**
	 * Gets the cross term block matrix between two real or virtual state blocks in the estimate
	 * covariance.
	 *
	 * @param label1 The first StateBlock label or VirtualStateBlock target label.
	 * @param label2 The second StateBlock label or VirtualStateBlock target label.
	 *
	 * @return  The cross terms between the states corresponding to \p label1 and the states
	 * corresponding to \p label2. The rows correspond to \p label1 and the columns correspond to
	 * \p label2. If \p label1 has M states and \p label2 has N states, then block will be an MxN
	 * matrix.
	 * @throw std::invalid_argument If the labels do not match StateBlocks or VirtualStateBlocks.
	 */
	virtual Matrix get_cross_term_covariance(std::string const &label1,
	                                         std::string const &label2) const = 0;

	/**
	 * Sets the cross term block matrix between two real state blocks in the estimate covariance.
	 * Note that this function does not support Virtual State Blocks.
	 *
	 * @param label1 The first StateBlock label.
	 * @param label2 The second StateBlock label.
	 * @param block The cross correlation terms for the covariance between the states corresponding
	 * to \p label1 and the states corresponding to \p label2. The rows correspond to \p label1 and
	 * the columns correspond to \p label2. If \p label1 has M states and \p label2 has N states,
	 * then block will be an MxN matrix.
	 *
	 * @throw std::invalid_argument If the labels do not match StateBlocks.
	 */
	virtual void set_cross_term_covariance(std::string const &label1,
	                                       std::string const &label2,
	                                       Matrix const &block) = 0;

	/**
	 * Give an AspnBaseVector to the specified real state block (for real state blocks that need
	 * additional data passed in).  Note that this function does not work for virtual state blocks.
	 * See #give_virtual_state_block_aux_data instead.
	 *
	 * @param label The unique label of the real state block.
	 * @param data The aux data.
	 *
	 * @throw std::invalid_argument If the label does not match any StateBlock.
	 */
	virtual void give_state_block_aux_data(std::string const &label,
	                                       AspnBaseVector const &data) = 0;

	/**
	 * Resets the specified state estimates to zeros, and returns the current estimate and
	 * covariance of those states.  The states to reset are specified by a real state block label.
	 * Note that this function does not work for virtual state blocks.
	 *
	 * @param time The time to perform the reset at.
	 * @param label The label of the real state block to reset.
	 * @param indices The indices of the state estimates to reset within the state block.
	 *
	 * @return The estimate and covariance of the state read synchronously at the time the filter
	 * was reset (for feedback).
	 *
	 * @throw std::invalid_argument If the label does not match any StateBlock.
	 * @throw std::invalid_argument If \p time is less than the current filter time.
	 * @throw std::invalid_argument If \p indices is empty or contains an index that exceeds the
	 * number of states in the fusion engine.
	 */
	virtual EstimateWithCovariance reset_state_estimate(aspn_xtensor::TypeTimestamp time,
	                                                    std::string const &label,
	                                                    std::vector<size_t> const &indices) = 0;

	/**
	 * Get the total number of states currently in the filter from all real state blocks.
	 *
	 * @return The total number of states currently in the filter from all real state blocks.
	 */
	virtual size_t get_num_states() const = 0;

	/**
	 * Add a measurement processor which can be used to process future measurements that specify
	 * themselves to be with respect to this measurement processor.
	 *
	 * @param processor The measurement processor.
	 */
	virtual void add_measurement_processor(
	    not_null<std::shared_ptr<MeasurementProcessor<>>> processor) = 0;

	/**
	 * Remove a measurement processor by name.
	 * @param label The unique label of the measurement processor.
	 */
	virtual void remove_measurement_processor(std::string const &label) = 0;

	/**
	 * Gets a list of the labels of measurement processors that have been added to this fusion
	 * engine.
	 * @return The list of labels.
	 */
	virtual std::vector<std::string> get_measurement_processor_names_list() const = 0;

	/**
	 * See if the fusion engine has a processor with matching label.
	 *
	 * @param label MeasurementProcessor label to look for.
	 *
	 * @return `true` if the fusion engine has corresponding MeasurementProcessor, `false`
	 * otherwise.
	 */
	virtual bool has_processor(std::string const &label) const = 0;

	/**
	 * Get a measurement processor object by name.
	 *
	 * @param label The unique label of the measurement processor.
	 *
	 * @return The measurement processor.
	 *
	 * @throw std::invalid_argument If the label does not match any MeasurementProcessor.
	 */
	virtual not_null<std::shared_ptr<MeasurementProcessor<>>> get_measurement_processor(
	    std::string const &label) = 0;

	/**
	 * Get a measurement processor object by name.
	 *
	 * @param label The unique label of the measurement processor.
	 *
	 * @return The measurement processor.
	 *
	 * @throw std::invalid_argument If the label does not match any MeasurementProcessor.
	 */
	virtual not_null<std::shared_ptr<const MeasurementProcessor<>>> get_measurement_processor(
	    std::string const &label) const = 0;

	/**
	 * Give an AspnBaseVector to the specified measurement processor.
	 * @param label The unique label of the measurement processor.
	 * @param data The aux data.
	 *
	 * @throw std::invalid_argument If the label does not match any MeasurementProcessor.
	 */
	virtual void give_measurement_processor_aux_data(std::string const &label,
	                                                 AspnBaseVector const &data) = 0;

	/**
	 * Propagate the filter estimate forward in time. May be evaluated lazily (not evaluated
	 * immediately).
	 *
	 * @param time Time to propagate to.
	 *
	 * @throw std::range_error If the Jacobian of the discrete-time state-transition function or
	 * discrete-time process noise covariance matrix returned by the StateBlocks is not NxN, where N
	 * is the number of states.
	 */
	virtual void propagate(aspn_xtensor::TypeTimestamp time) = 0;

	/**
	 * Update the filter with the given measurement. Will propagate first if needed to reach the
	 * time encoded inside the measurement.
	 *
	 * @param processor_label The unique label of the measurement processor used to process \p
	 * measurement
	 * @param measurement The raw measurement.
	 * @param timestamp A pointer containing the timestamp extracted from \p measurement prior to
	 * calling update. If \p timestamp is nullptr, then the fusion engine will attempt to extract
	 * a timestamp from \p measurement itself. Default value is nullptr.
	 */
	virtual void update(std::string const &processor_label,
	                    std::shared_ptr<aspn_xtensor::AspnBase> measurement,
	                    std::shared_ptr<aspn_xtensor::TypeTimestamp> timestamp = nullptr) = 0;

	/**
	 * Calculates the estimate and covariance at a requested time and for requested states using all
	 * currently available information, without permanently changing the filter state.  The states
	 * are requested using real state block labels or virtual state block target labels.
	 *
	 * @param time time of estimate request.
	 * @param mixed_block_labels List of StateBlock labels and/or VirtualStateBlock target labels.
	 *
	 * @returns Filter estimate and covariance at the requested time. Will be `nullptr` if
	 * \p mixed_block_labels empty.  If a label in \p mixed_block_labels represents a virtual state
	 * block, then the estimate and covariance will be in the virtual representation.
	 */
	virtual std::shared_ptr<EstimateWithCovariance> peek_ahead(
	    aspn_xtensor::TypeTimestamp time,
	    std::vector<std::string> const &mixed_block_labels) const = 0;

	/**
	 * Returns a list of the target labels of virtual state blocks that have been added to this
	 * fusion engine.
	 *
	 * A label being returned by this list is not a guarantee that the virtual state block has a
	 * valid source.  Call has_virtual_state_block() on a label to see if the source is valid.
	 *
	 * @returns A list of the target labels of virtual state blocks that have been added to this
	 * fusion engine.
	 */
	virtual std::vector<std::string> get_virtual_state_block_target_labels() const = 0;

	/**
	 * Register a VirtualStateBlock with the fusion engine to allow generation of alternative
	 * representations of real StateBlocks.
	 *
	 * @param v VirtualStateBlock to add.
	 */
	virtual void add_virtual_state_block(not_null<std::shared_ptr<VirtualStateBlock>> v) = 0;

	/**
	 * Give an AspnBaseVector struct to the specified virtual state block (for VSBs that need
	 * additional data passed in).
	 *
	 * @param target_label The unique target label of the virtual state block.
	 * @param data The aux data.
	 *
	 * @throw std::invalid_argument If the label does not match any VirtualStateBlock.
	 */
	virtual void give_virtual_state_block_aux_data(std::string const &target_label,
	                                               AspnBaseVector const &data) = 0;

	/**
	 * Remove a VirtualStateBlock that is registered with the fusion engine
	 *
	 * @param target target value of the virtual state block instance that is to be removed (point
	 * B).
	 */
	virtual void remove_virtual_state_block(std::string const &target) = 0;

	/**
	 * Generates the state and covariance matrices built corresponding to a list of StateBlock
	 * and/or VirtualStateBlock labels. Blocks are assembled in the order that the labels are passed
	 * in.
	 *
	 * @param mixed_block_labels List of StateBlock labels and/or VirtualStateBlock target labels.
	 *
	 * @return The state estimate vector and the covariance matrix. Will be `nullptr` if
	 * \p mixed_block_labels empty.  If a label in \p mixed_block_labels represents a virtual state
	 * block, then the estimate and covariance will be in the virtual representation.
	 */
	virtual std::shared_ptr<EstimateWithCovariance> generate_x_and_p(
	    std::vector<std::string> const &mixed_block_labels) const = 0;
};

}  // namespace filtering
}  // namespace navtk
