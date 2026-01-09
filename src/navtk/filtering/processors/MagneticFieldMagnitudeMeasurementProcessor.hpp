#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/not_null.hpp>
#include <navtk/utils/GriddedInterpolant.hpp>

namespace navtk {
namespace filtering {

/**
 * Processes Measurements containing an MeasurementMagneticFieldMagnitude and NavSolution to update
 * North and East position error states.
 */
class MagneticFieldMagnitudeMeasurementProcessor : public MeasurementProcessor<> {

public:
	/**
	 * Constructor for processor that updates only one state block.
	 *
	 * @param label Name of this processor.
	 * @param state_block_label A label referring to a Pinson-style StateBlock this processor
	 * updates. It is required that the block have an estimate vector of at least length 2 and
	 * elements[0..1] contain North and East position error states in m.
	 * @param x_vec 1 x N Vector representing x-axis values, where N > 2.
	 * @param y_vec 1 x M Vector representing y-axis values, where M > 2.
	 * @param map N x M grid of map values at grid points (x,y).
	 */
	MagneticFieldMagnitudeMeasurementProcessor(std::string label,
	                                           const std::string &state_block_label,
	                                           Vector x_vec,
	                                           Vector y_vec,
	                                           Matrix map);

	/**
	 * Constructor for processor that updates multiple state blocks.
	 *
	 * @param label Name of this processor.
	 * @param state_block_labels A vector of labels referring to StateBlocks this processor updates.
	 * It is required that the first state block have an estimate vector of at least length 2 and
	 * elements[0..1] contain North and East position error states in m.
	 * @param x_vec 1 x N Vector representing x-axis values, where N > 2.
	 * @param y_vec 1 x M Vector representing y-axis values, where M > 2.
	 * @param map N x M grid of map values at grid points (x,y).
	 */
	MagneticFieldMagnitudeMeasurementProcessor(std::string label,
	                                           std::vector<std::string> state_block_labels,
	                                           Vector x_vec,
	                                           Vector y_vec,
	                                           Matrix map);

	/**
	 * Generates a StandardMeasurementModel that relates a filtering::PairedPva (wrapping a
	 * MeasurementMagneticFieldMagnitude) to a state vector with NE position error states.
	 *
	 * @param measurement Measurement of type filtering::PairedPva.
	 *
	 * @param gen_x_and_p_func A function that will generate `xhat` (a Vector of estimated states,
	 * constructed from state blocks referenced by `state_block_labels`) and `P` (covariance Matrix
	 * for `xhat`) when called.
	 *
	 * @return If \p measurement is a filtering::PairedPva containing a
	 * MeasurementMagneticFieldMagnitude, a StandardMeasurementModel constructed from the contents
	 * of \p measurement and using the h_map class member. Otherwise, a `nullptr`.
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

private:
	/** 2D interpolator for use inside h function */
	utils::GriddedInterpolant h_map;
};


}  // namespace filtering
}  // namespace navtk
