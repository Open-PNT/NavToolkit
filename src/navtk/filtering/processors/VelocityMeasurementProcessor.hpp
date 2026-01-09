#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {

/**
 * Processes Measurements containing a MeasurementVelocity and NavSolution to update a velocity
 * error state.
 */
class VelocityMeasurementProcessor : public MeasurementProcessor<> {

public:
	/**
	 * Constructor for processor that updates only one state block.
	 *
	 * @param label Name of this processor.
	 * @param state_block_label A label referring to a Pinson-style StateBlock this processor
	 * updates. It is required that the block have an estimate vector of at least length 6 and
	 * elements[3..5] contain NED velocity error state in m/s.
	 * @param measurement_matrix NxM Matrix, mapping the M states to a N-element Vector containing
	 * the velocity measurement, where N is the number of use_* parameters that are true.
	 * @param use_x When true, attempts to use the X component of the velocity measurement. Throws
	 * an error if this component is null (NaN).
	 * @param use_y When true, attempts to use the Y component of the velocity measurement. Throws
	 * an error if this component is null (NaN).
	 * @param use_z When true, attempts to use the Z component of the velocity measurement. Throws
	 * an error if this component is null (NaN).
	 * @param expected_frame Logs a warning when the ASPN measurement's reference frame differs from
	 * \p expected_frame
	 */
	VelocityMeasurementProcessor(std::string label,
	                             const std::string &state_block_label,
	                             Matrix measurement_matrix,
	                             bool use_x,
	                             bool use_y,
	                             bool use_z,
	                             AspnMeasurementVelocityReferenceFrame expected_frame =
	                                 ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED);

	/**
	 * Constructor for processor that updates multiple state blocks.
	 *
	 * @param label Name of this processor.
	 * @param state_block_labels A vector of labels referring to StateBlocks this processor updates.
	 * It is required that the first state block have an estimate vector of at least length 6 and
	 * elements[3..5] contain NED velocity error state in m/s.
	 * @param measurement_matrix NxM Matrix, mapping the M states to a N-element Vector containing
	 * the velocity measurement, where N is the number of use_* parameters that are true.
	 * @param use_x When true, attempts to use the X component of the velocity measurement. Throws
	 * an error if this component is null (NaN).
	 * @param use_y When true, attempts to use the Y component of the velocity measurement. Throws
	 * an error if this component is null (NaN).
	 * @param use_z When true, attempts to use the Z component of the velocity measurement. Throws
	 * an error if this component is null (NaN).
	 * @param expected_frame Logs a warning when the ASPN measurement's reference frame differs from
	 * \p expected_frame
	 */
	VelocityMeasurementProcessor(std::string label,
	                             std::vector<std::string> state_block_labels,
	                             Matrix measurement_matrix,
	                             bool use_x,
	                             bool use_y,
	                             bool use_z,
	                             AspnMeasurementVelocityReferenceFrame expected_frame =
	                                 ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED);

	/**
	 * Generates a StandardMeasurementModel that relates a `filtering::PairedPva` wrapping a
	 * `MeasurementVelocity` to a state vector with NED velocity error states. The state updated is
	 * determined by the use_* parameters provided to the processor during initialization.
	 *
	 * @param measurement Measurement of type MeasurementVelocity.
	 *
	 * @param gen_x_and_p_func A function that will generate `xhat` (a Vector of estimated states,
	 * constructed from state blocks referenced by `state_block_labels`) and `P` (covariance Matrix
	 * for `xhat`) when called.
	 *
	 * @return A constructed StandardMeasurementModel if \p measurement is a PairedPva containing a
	 * MeasurementVelocity, otherwise `nullptr`.
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
	const Matrix measurement_matrix;
	Vector meas_vector;
	Size index1;
	Size index2;
	Size index3;
	const bool use_x;
	const bool use_y;
	const bool use_z;
	const AspnMeasurementVelocityReferenceFrame expected_frame;

	void setup();
};


}  // namespace filtering
}  // namespace navtk
