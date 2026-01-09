#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {

/**
 * Processes Measurements containing a MeasurementAltitude and NavSolution to update a
 * Down position error state.
 */
class AltitudeMeasurementProcessorWithBias : public MeasurementProcessor<> {

public:
	/**
	 * Constructor for processor that updates multiple state blocks.
	 *
	 * @param label Name of this processor.
	 * @param pinson_label A label referring to a Pinson-style StateBlock this processor updates. It
	 * is required that the block have an estimate vector of at least length 3 and the third element
	 * be a Down position error state in meters.
	 * @param altitude_bias_label Label referring to a single state altitude bias in meters.
	 * @param expected_frame Logs a warning when the ASPN measurement's reference frame differs from
	 * \p expected_frame
	 */
	AltitudeMeasurementProcessorWithBias(
	    std::string label,
	    const std::string &pinson_label,
	    const std::string &altitude_bias_label,
	    AspnMeasurementAltitudeReference expected_frame = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL);

	/**
	 * Constructor for processor that updates multiple state blocks.
	 *
	 * @param label Name of this processor.
	 * @param state_block_labels A vector of 2 labels referring to StateBlocks this processor
	 * updates. It is required that the first state block have an estimate vector of at least length
	 * 3 and it's third element be a Down position error state in meters. The second block should be
	 * a single state altitude bias in meters.
	 * @param expected_frame Logs a warning when the ASPN measurement's reference frame differs from
	 * \p expected_frame
	 *
	 * @throw std::invalid_argument if \p state_block_labels does not contain exactly two labels
	 * and the error mode is ErrorMode::DIE.
	 */
	AltitudeMeasurementProcessorWithBias(
	    std::string label,
	    std::vector<std::string> state_block_labels,
	    AspnMeasurementAltitudeReference expected_frame = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL);

	/**
	 * Generates a StandardMeasurementModel that relates a `filtering::PairedPva` containing a
	 * `MeasurementAltitude` to a state vector with a Down Position error state in meters and an
	 * altitude bias state in meters.
	 *
	 * @param measurement Measurement of type `filtering::PairedPva` with a `MeasurementAltitude` on
	 * it's meas_data field. It is assumed that the measured altitude is with respect to the same
	 * reference as that contained in the reference navigation solution (such as HAE).
	 *
	 * @param gen_x_and_p_func A function that will generate `xhat` (a Vector of estimated states,
	 * constructed from state blocks referenced by `state_block_labels`) and `P` (covariance Matrix
	 * for `xhat`) when called.
	 *
	 * @return A constructed StandardMeasurementModel if \p measurement is a `filtering::PairedPva`
	 * containing a `MeasurementAltitude`, otherwise `nullptr`.
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
	const AspnMeasurementAltitudeReference expected_frame;
};


}  // namespace filtering
}  // namespace navtk
