#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {

/**
 * A measurement processor that accepts biased range updates of a direct-position.
 * Assumes you have a 1x1 bias StateBlock and 3x1 position StateBlock in your filter. It then
 * calculates:
 *
 * `z = norm(position-beacon)+bias`
 */
class BiasedRangeProcessor : public MeasurementProcessor<> {

public:
	/**
	 * Constructor for processor that updates only one state block.
	 *
	 * @param label Name of this processor.
	 * @param position_label The label identifying the direct 3-position StateBlock this processor
	 * will be updating.
	 * @param bias_label The label identifying the bias StateBlock this processor will be updating.
	 */
	BiasedRangeProcessor(std::string label,
	                     const std::string &position_label,
	                     const std::string &bias_label);

	/**
	 * Constructor for processor that updates multiple state blocks.
	 *
	 * @param label Name of this processor.
	 * @param state_block_labels vector of two labels (bias StateBlock label, direct 3-position
	 * StateBlock label) assigned to the StateBlocks this processor will be updating.
	 *
	 * @throw std::invalid_argument if \p state_block_labels does not contain exactly two labels
	 * and the error mode is ErrorMode::DIE.
	 */
	BiasedRangeProcessor(std::string label, std::vector<std::string> state_block_labels);

	/**
	 * Generates a StandardMeasurementModel instance that maps the input data to the estimated
	 * states.
	 *
	 * @param measurement Measurement of type MeasurementRangeToPoint.
	 *
	 * @param gen_x_and_p_func A function that will generate `xhat` (a Vector of estimated states,
	 * constructed from state blocks referenced by `state_block_labels`) and `P` (covariance Matrix
	 * for `xhat`) when called.
	 *
	 * @return A constructed StandardMeasurementModel if \p measurement contains
	 * MeasurementRangeToPoint, otherwise `nullptr`.
	 */
	std::shared_ptr<StandardMeasurementModel> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase> measurement,
	    GenXhatPFunction gen_x_and_p_func) override;

	/**
	 * Create a copy of the MeasurementProcessor with the same properties.
	 *
	 * @return A shared pointer to a copy of the MeasurementProcessor.
	 */
	not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override;
};


}  // namespace filtering
}  // namespace navtk
