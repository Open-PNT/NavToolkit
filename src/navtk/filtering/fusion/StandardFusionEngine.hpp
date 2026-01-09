#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/fusion/StandardFusionEngineBase.hpp>
#include <navtk/filtering/fusion/strategies/EkfStrategy.hpp>
#include <navtk/filtering/fusion/strategies/StandardModelStrategy.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlockManager.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {

/**
 * An implementation of the StandardFusionEngineBase interface.
 *
 * The `StandardFusionEngine` requires a #strategy that specifies the type of fusion strategy (EKF,
 * UKF, RBPF, etc). The #strategy provides the algorithm backing the
 * `StandardFusionEngine::propagate` and `StandardFusionEngine::update` methods, which predict
 * future values of states and adjust states based on new measurements, respectively.
 *
 * The filter requires a dynamics model in order to propagate. A `StateBlock` block provides a
 * dynamics model for a group of states, such as position, velocity, and acceleration of a vehicle
 * being estimated by the `StandardFusionEngine`. The `StateBlock` also provides the number of
 * states (`StateBlock::num_states`), and a unique identifier (StateBlock::get_label()). At least
 * one `StateBlock` block is required by `StandardFusionEngine` in order to propagate, though
 * additional `StateBlock` blocks may be added to the fusion engine.
 *
 * The fusion strategy constructs a state estimate vector (xhat) and covariance matrix (P) large
 * enough to allocate `StateBlock::num_states` coefficients for each state block. These are grouped
 * according to their `StateBlock::get_label()`. An incoming measurement may relate to any number of
 * StateBlocks. The filter propagates the corresponding `xhat` states to a specified time using the
 * dynamics model. `StateBlock::generate_dynamics` provides the dynamics model,
 * `StandardDynamicsModel`. The complete list of `StateBlock` blocks in the `StandardFusionEngine`
 * can be accessed by using #get_state_block_names_list.
 *
 * The filter requires a measurement model in order to update. The `MeasurementProcessor` provides
 * the measurement model. The purpose of a measurement model is to map states to measurements, such
 * as mapping GPS pseudorange measurements to position, velocity, and acceleration state estimates.
 * When the raw measurements are received, the `MeasurementProcessor::generate_model` is used to
 * provide a `StandardMeasurementModel`. The `StandardMeasurementModel` provides all parameters
 * necessary for the #strategy update: the measurement vector (`z`), the corresponding measurement
 * covariance matrix (`R`), measurement prediction function that returns expected measurement vector
 * (`h`), and matrix providing approximate mapping of states to measurements (`H`).
 *
 */
class StandardFusionEngine : public StandardFusionEngineBase {
public:
	virtual ~StandardFusionEngine() = default;

	/**
	 * Maps the parameters to the fields.
	 *
	 * @param cur_time The initial time for the filter.
	 * @param strategy The type of filter (EKF, UKF, etc).
	 */
	StandardFusionEngine(const aspn_xtensor::TypeTimestamp &cur_time,
	                     not_null<std::shared_ptr<StandardModelStrategy>> strategy);

	/**
	 * Maps the parameters to the fields. Sets #strategy to EkfStrategy with zero-length matrices.
	 *
	 * @param cur_time The initial time for the filter.
	 */
	StandardFusionEngine(
	    const aspn_xtensor::TypeTimestamp &cur_time = aspn_xtensor::TypeTimestamp((int64_t)0));

	/**
	 * Copy constructor for StandardFusionEngine.  Performs a deep copy of other.
	 * @param other
	 */
	StandardFusionEngine(const StandardFusionEngine &other);

	/**
	 * Copy assignment for StandardFusionEngine.  Performs a deep copy of other.
	 * @param other
	 * @return Copy of StandardFusionEngine
	 */
	StandardFusionEngine &operator=(const StandardFusionEngine &other);

	/**
	 * Default move constructor.
	 */
	StandardFusionEngine(StandardFusionEngine &&) = default;

	/**
	 * Default move assignment operator.
	 *
	 * @return A new StandardFusionEngine containing the data from the original.
	 */
	StandardFusionEngine &operator=(StandardFusionEngine &&) = default;

	/**
	 * Set the current time of the filter
	 *
	 * @param time The time to set the filter to
	 */
	void set_time(const aspn_xtensor::TypeTimestamp &time) override;

	/**
	 * Get the current time of the filter
	 *
	 * @return The current time of the filter
	 */
	aspn_xtensor::TypeTimestamp get_time() const override;

	/**
	 * Gets a list of the labels of real state blocks that have been added to this fusion engine.
	 * Note that this function does not work for virtual state blocks. See
	 * #get_virtual_state_block_target_labels instead.
	 *
	 * @return The list of real state block labels.
	 */
	std::vector<std::string> get_state_block_names_list() const override;

	/**
	 * See if the fusion engine has a real state block with matching label.  Note that this function
	 * does not work for virtual state blocks.  See #has_virtual_state_block instead.
	 *
	 * @param label StateBlock label to look for.
	 *
	 * @return `true` if the fusion engine has corresponding StateBlock, `false` otherwise.
	 */
	bool has_block(std::string const &label) const override;

	/**
	 * See if the fusion engine has a VirtualStateBlock capable of generating an estimate.
	 *
	 * @param target_label VirtualStateBlock to look for (via target label).
	 *
	 * @return `true` if the fusion engine has corresponding VirtualStateBlock and it can be linked
	 * to a StateBlock, `false` otherwise.
	 */
	virtual bool has_virtual_state_block(std::string const &target_label) const override;

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
	not_null<std::shared_ptr<StateBlock<>>> get_state_block(std::string const &label) override;

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
	not_null<std::shared_ptr<const StateBlock<>>> get_state_block(
	    std::string const &label) const override;

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
	Matrix get_state_block_covariance(std::string const &label) const override;

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
	void set_state_block_covariance(std::string const &label, Matrix const &covariance) override;

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
	Vector get_state_block_estimate(std::string const &label) const override;

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
	void set_state_block_estimate(std::string const &label, Vector const &estimate) override;

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
	void add_state_block(not_null<std::shared_ptr<StateBlock<>>> block) override;

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
	void remove_state_block(std::string const &label) override;

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
	void set_cross_term_process_covariance(std::string const &label1,
	                                       std::string const &label2,
	                                       Matrix const &block) override;

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
	Matrix get_cross_term_covariance(std::string const &label1,
	                                 std::string const &label2) const override;

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
	void set_cross_term_covariance(std::string const &label1,
	                               std::string const &label2,
	                               Matrix const &block) override;

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
	void give_state_block_aux_data(std::string const &label, AspnBaseVector const &data) override;

	/**
	 * Give an AspnBaseVector to the specified measurement processor.
	 * @param label The unique label of the measurement processor.
	 * @param data The aux data.
	 *
	 * @throw std::invalid_argument If the label does not match any MeasurementProcessor.
	 */
	void give_measurement_processor_aux_data(std::string const &label,
	                                         AspnBaseVector const &data) override;

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
	EstimateWithCovariance reset_state_estimate(aspn_xtensor::TypeTimestamp time,
	                                            std::string const &label,
	                                            std::vector<size_t> const &indices) override;

	/**
	 * Get the total number of states currently in the filter from all real state blocks.
	 *
	 * @return The total number of states currently in the filter from all real state blocks.
	 */
	size_t get_num_states() const override;

	/**
	 * Add a measurement processor which can be used to process future measurements that specify
	 * themselves to be with respect to this measurement processor.
	 *
	 * @param processor The measurement processor.
	 */
	void add_measurement_processor(
	    not_null<std::shared_ptr<MeasurementProcessor<>>> processor) override;

	/**
	 * Remove a measurement processor by name.
	 * @param label The unique label of the measurement processor.
	 */
	void remove_measurement_processor(std::string const &label) override;

	/**
	 * Gets a list of the labels of measurement processors that have been added to this fusion
	 * engine.
	 * @return The list of labels.
	 */
	std::vector<std::string> get_measurement_processor_names_list() const override;

	/**
	 * See if the fusion engine has a processor with matching label.
	 *
	 * @param label MeasurementProcessor label to look for.
	 *
	 * @return `true` if the fusion engine has corresponding MeasurementProcessor, `false`
	 * otherwise.
	 */
	bool has_processor(std::string const &label) const override;

	/**
	 * Get a measurement processor object by name.
	 *
	 * @param label The unique label of the measurement processor.
	 *
	 * @return The measurement processor.
	 *
	 * @throw std::invalid_argument If the label does not match any MeasurementProcessor.
	 */
	not_null<std::shared_ptr<MeasurementProcessor<>>> get_measurement_processor(
	    std::string const &label) override;

	/**
	 * Get a measurement processor object by name.
	 *
	 * @param label The unique label of the measurement processor.
	 *
	 * @return The measurement processor.
	 *
	 * @throw std::invalid_argument If the label does not match any MeasurementProcessor.
	 */
	not_null<std::shared_ptr<const MeasurementProcessor<>>> get_measurement_processor(
	    std::string const &label) const override;

	/**
	 * Propagate the filter estimate forward in time. May be evaluated lazily (not evaluated
	 * immediately).
	 *
	 * @param time time to propagate to.
	 *
	 * @throw std::range_error If the Jacobian of the discrete-time state-transition function or
	 * discrete-time process noise covariance matrix returned by the StateBlocks is not NxN, where N
	 * is the number of states.
	 */
	void propagate(aspn_xtensor::TypeTimestamp time) override;

	/**
	 * Update the filter with the given measurement. Will propagate first if needed to reach the
	 * time encoded inside the measurement.
	 *
	 * @param processor_label The unique label of the measurement processor used to process \p
	 * measurement
	 * @param measurement The raw measurement.
	 * @param timestamp A pointer containing the timestamp extracted from \p measurement prior to
	 * calling update. The fusion engine will propagate up to this time. If \p timestamp is nullptr,
	 * then the fusion engine will attempt to extract a timestamp from \p measurement itself and
	 * propagate to that time.
	 */
	void update(std::string const &processor_label,
	            std::shared_ptr<aspn_xtensor::AspnBase> measurement,
	            std::shared_ptr<aspn_xtensor::TypeTimestamp> timestamp = nullptr) override;

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
	std::shared_ptr<EstimateWithCovariance> peek_ahead(
	    aspn_xtensor::TypeTimestamp time,
	    std::vector<std::string> const &mixed_block_labels) const override;

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
	std::vector<std::string> get_virtual_state_block_target_labels() const override;

	/**
	 * Register a VirtualStateBlock with the fusion engine to allow generation of alternative
	 * representations of real StateBlocks.
	 *
	 * @param v VirtualStateBlock to add.
	 */
	void add_virtual_state_block(not_null<std::shared_ptr<VirtualStateBlock>> v) override;

	/**
	 * Give an AspnBaseVector struct to the specified virtual state block (for VSBs that need
	 * additional data passed in).
	 *
	 * @param target_label The unique target label of the virtual state block.
	 * @param data The aux data.
	 *
	 * @throw std::invalid_argument If the label does not match any VirtualStateBlock.
	 */
	void give_virtual_state_block_aux_data(std::string const &target_label,
	                                       AspnBaseVector const &data) override;

	/**
	 * Remove a VirtualStateBlock registered with the fusion engine
	 *
	 * @param target target value of the virtual state block instance that is to be removed (point
	 * B).
	 */
	void remove_virtual_state_block(std::string const &target) override;

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
	std::shared_ptr<EstimateWithCovariance> generate_x_and_p(
	    std::vector<std::string> const &mixed_block_labels) const override;

protected:
	/**
	 * The time the filter initializes to, used in propagate() and update() to determine delta-t.
	 */
	aspn_xtensor::TypeTimestamp cur_time;

	/**
	 * Manager for registered VirtualStateBlocks.
	 */
	VirtualStateBlockManager vsb_man;

	/**
	 * A list of StateBlocks which have been added to the fusion engine.
	 */
	std::vector<not_null<std::shared_ptr<StateBlock<>>>> blocks;

	/**
	 * A list of MeasurementProcessors which have been added to the fusion engine.
	 */
	std::vector<not_null<std::shared_ptr<MeasurementProcessor<>>>> processors;

	/**
	 * The underlying fusion strategy used by this fusion engine.
	 */
	not_null<std::shared_ptr<StandardModelStrategy>> strategy;

	/**
	 * Given the names of a number of real state blocks, return all corresponding indices into the
	 * internal master matrices.  Similar to get_mat_indices but returns all indices, not just the
	 * start/stop indices.
	 *
	 * Note that this function does not work for virtual state blocks.
	 *
	 * @param block_labels Labels of real state blocks to extract.
	 *
	 * @return Indices of all state blocks represented by \p block_labels, in the same order as the
	 * labels passed in.
	 */
	std::vector<Size> get_all_state_indices(const std::vector<std::string> &block_labels) const;

	/**
	 * Element in the `process_covariance_cross_terms list` to be used by propagation functions.
	 */
	struct ProcessCovarianceCrossTerm {
		/**
		 * Label of first StateBlock.
		 */
		std::string label1;

		/**
		 * Label of second StateBlock.
		 */
		std::string label2;

		/**
		 * The discrete-time process noise covariance matrix cross term.
		 */
		Matrix term;
	};

	/**
	 * A list of discrete-time process noise covariance matrix cross terms to be used by propagate
	 * functions.
	 */
	std::vector<ProcessCovarianceCrossTerm> process_covariance_cross_terms;

	/**
	 * Given the name of a real state block, produce the start/stop indices into the internal master
	 * matrices that correspond to the block.
	 *
	 * Note that this function does not work for virtual state blocks.
	 *
	 * @param label The StateBlock label.
	 *
	 * @return A pair of indices representing the start index (first) and stop index (second) into
	 * the internal master matrix that correspond to the states represented by the real state block
	 * labeled \p label.  The start index is inclusive and the stop index is exclusive.
	 *
	 * @throw std::invalid_argument if \p label does not correspond to any StateBlock.
	 */
	virtual std::pair<size_t, size_t> get_mat_indices(std::string const &label) const;

	/**
	 * Given the index of a real state block into the internal list of real state blocks, produce
	 * the start/stop indices into the internal master matrices that correspond to the block.
	 *
	 * @param idx The index of the real state block.
	 *
	 * @return A pair of indices representing the start index (first) and stop index (second) into
	 * the internal master matrix that correspond to the states represented by the state block
	 * stored in the internal state block list at index \p idx.  The start index is inclusive and
	 * the stop index is exclusive.
	 *
	 * @throw std::invalid_argument if \p idx does not correspond to any StateBlock.
	 */
	virtual std::pair<size_t, size_t> get_mat_indices(size_t idx) const;

	/**
	 * Produce the start/stop indices into the master estimate and covariance matrices for each
	 * real state block.
	 *
	 * @return A vector of pairs of indices representing the start index (first) and stop index
	 * (second) into the internal master matrix.  Each entry in the vector represents a different
	 * state block in the internal master matrix.  The start indices are inclusive and the stop
	 * indices are exclusive.
	 */
	virtual std::vector<std::pair<size_t, size_t>> get_mat_indices_list() const;

	/**
	 * Get the index into internal storage for a given StateBlock.  Note that this function does not
	 * work for virtual state blocks.
	 *
	 * @param label StateBlock label.
	 *
	 * @return Index into processors that will return the requested StateBlock.
	 * @throw std::invalid_argument If no StateBlock with the given label is found.
	 */
	virtual size_t find_block_idx_or_bail(std::string const &label) const;

	/**
	 * Get the index into internal storage for a given MeasurementProcessor.
	 *
	 * @param label MeasurementProcessor label.
	 *
	 * @return Index into processors that will return the requested MeasurementProcessor.
	 * @throw std::invalid_argument If no MeasurementProcessor with the given label is found.
	 */
	virtual size_t find_processor_idx_or_bail(std::string const &label) const;

	/**
	 * Expands filtering::StandardMeasurementModel::h and filtering::StandardMeasurementModel::H so
	 * that they can be evaluated against the filters' full state vector. Note that it is assumed
	 * that the Jacobian (H) is constructed in the same order as the labels contained
	 * filtering::MeasurementProcessor::state_block_labels.
	 *
	 * @param model The StandardMeasurementModel generated by the measurement processor using a
	 * subset of the states.
	 * @param proc The MeasurementProcessor used to generate the model.
	 *
	 * @return A modified copy of the input model.
	 */
	virtual not_null<std::shared_ptr<StandardMeasurementModel>> expand_update_model(
	    StandardMeasurementModel const &model, StandardMeasurementProcessor const &proc);

	/**
	 * Expands StandardMeasurementModel.h and StandardMeasurementModel.H so that they can be
	 * evaluated against the filters' full state vector. Note that it is assumed that the Jacobian
	 * (H) is constructed in the same order as the labels contained
	 * MeasurementProcessor.StandardStateBlockLabels.  Note that this function does not work for
	 * virtual state blocks.
	 *
	 * @param model The StandardMeasurementModel generated by the measurement processor using a
	 * subset of the states.
	 * @param state_block_labels The ordered list of StateBlock labels for the states that model is
	 * to be evaluated against.
	 *
	 * @return A modified copy of the input model.
	 */
	virtual not_null<std::shared_ptr<StandardMeasurementModel>> expand_update_model(
	    StandardMeasurementModel const &model, std::vector<std::string> const &state_block_labels);

	/**
	 * @param label StateBlock label or VirtualStateBlock target label.
	 *
	 * @return EstimateWithCovariance.
	 *
	 * @throw std::invalid_argument if \p label matches neither a real StateBlock.label nor a
	 * VirtualStateBlock.target.
	 * @throw std::invalid_argument if \p label matches a VirtualStateBlock.target, but no real
	 * StateBlock is available to be passed into the VirtualStateBlock.
	 */
	EstimateWithCovariance get_state_block_est_and_cov(std::string const &label) const;

	/**
	 * Finds the real state block labels that correspond to a mixed set of real and virtual state
	 * block labels.
	 *
	 * @param mixed_block_labels A vector of StateBlock labels and/or VirtualStateBlock target
	 * labels.
	 *
	 * @return A vector of the same length as \p mixed_block_labels, where the `i`th element is
	 * `mixed_block_labels[i]` if that label corresponds to an existing StateBlock, or the label of
	 * a StateBlock that can be transformed to a virtual state block of the `i`th type via one or
	 * more VirtualStateBlock.convert calls.
	 *
	 * @throw std::invalid_argument If any element in \p mixed_block_labels neither matches the
	 * label of an existing StateBlock nor references an available VirtualStateBlock.
	 */
	std::vector<std::string> get_real_block_labels(
	    std::vector<std::string> const &mixed_block_labels) const;

	/**
	 * Generate a function that that accepts a concatenated vector of real StateBlock estimates
	 * and returns another vector containing the virtual versions of those estimates, as
	 * appropriate.
	 *
	 * @param mixed_block_labels A vector of StateBlock labels and/or VirtualStateBlock
	 * target labels.
	 *
	 * @return A function, that when provided concatenated estimates of the corresponding real
	 * StateBlocks returns a Vector of estimates. The real StateBlocks are determined by the
	 * return value of `get_real_block_labels(mixed_block_labels)`. The returned Vector of
	 * estimates are formatted to match \p mixed_block_labels format.
	 *
	 * @throw std::invalid_argument If any element in \p mixed_block_labels neither matches the
	 * label of an existing StateBlock nor references an available VirtualStateBlock.
	 */
	std::function<Vector(Vector)> calc_full_transform(
	    std::vector<std::string> const &mixed_block_labels) const;

	/**
	 * Returns the Jacobian of the function returned by calc_full_transform(), when
	 * calc_full_transform() is called with the same \p mixed_block_labels argument. This assumes
	 * the return value of the function is evaluated using the same state values currently
	 * available. Accuracy of the Jacobian is dependent upon the accuracy of the underlying
	 * VirtualStateBlock.jacobian() implementations used when converting.
	 *
	 * @param mixed_block_labels A vector of StateBlock labels and/or VirtualStateBlock
	 * target labels.
	 *
	 * @return A PxQ Matrix, where Q is the sum of all states in the real StateBlocks referenced by
	 * \p mixed_block_labels (see calc_full_transform(), get_real_block_labels()). P is the sum of
	 * all states for the real or virtual state blocks in \p mixed_block_labels. In other words,
	 * pre-multiplying a real state estimate vector by this matrix would provide a first-order
	 * approximation of the virtual version of the state estimate vector, assuming that the state
	 * estimate vector contains the correct states.
	 */
	virtual Matrix calc_transform_jacobian(
	    std::vector<std::string> const &mixed_block_labels) const;

	/**
	 * Clear any cache variables used within the class.
	 */
	void clear_cache() const;

	/** Cached last set of mixed_block_labels provided to generate_x_and_p */
	mutable std::vector<std::string> last_gen_xp_args;

	/** Cached last x/p evaluation given last_gen_xp_args */
	mutable std::shared_ptr<EstimateWithCovariance> last_gen_xp_results;

	/**
	 * Last jacobian calculated by gen_x_and_p_impl if any element of last_gen_xp_args referenced
	 * a virtual state block.
	 */
	mutable Matrix last_jac;

	/**
	 * Last state transform function calculated by gen_x_and_p_impl if any element of
	 * last_gen_xp_args referenced a virtual state block.
	 */
	mutable std::function<Vector(Vector)> last_tx;

	/**
	 * Provides a GenXhatPFunction by wrapping an internal function. If a measurement processor
	 * evaluates this function, generate_x_and_p caches args (for the last evaluation only) and
	 * results so they can be reused during this update cycle
	 */
	GenXhatPFunction gen_x_and_p_func =
	    [&](const std::vector<std::string> &labels) -> std::shared_ptr<EstimateWithCovariance> {
		return generate_x_and_p(labels);
	};
};


}  // namespace filtering
}  // namespace navtk
