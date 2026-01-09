#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {

/**
 * Processes Measurements containing a MeasurementPositionVelocityAttitude and NavSolution to update
 * NED position, velocity, and attitude error states.
 */
class PositionVelocityAttitudeMeasurementProcessor : public MeasurementProcessor<> {

public:
	/**
	 * Constructor for processor that updates only one state block.
	 *
	 * @param label Name of this processor.
	 * @param state_block_label A label referring to a Pinson-style StateBlock this processor
	 * updates. It is required that the block have an estimate vector of at least length 6 where
	 * elements[0..2] contain North, East and Down position error states in m, and elements [3..5]
	 * contain North, East and Down velocity error states in m/s.
	 * @param measurement_matrix NxM Matrix, mapping the M states to a N-element Vector containing
	 * the velocity measurement, where N is the number of use_* parameters that are true.
	 * @param use_p1 When true, attempts to use the p1 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_p2 When true, attempts to use the p2 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_p3 When true, attempts to use the p3 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_v1 When true, attempts to use the v1 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_v2 When true, attempts to use the v2 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_v3 When true, attempts to use the v3 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_quaternion When true, attempts to use the quaternion component of the measurement.
	 * Throws an error if this component is null (NaN).
	 * @param expected_frame Logs a warning when the ASPN measurement's reference frame differs from
	 * \p expected_frame
	 */
	PositionVelocityAttitudeMeasurementProcessor(
	    std::string label,
	    const std::string &state_block_label,
	    Matrix measurement_matrix,
	    bool use_p1,
	    bool use_p2,
	    bool use_p3,
	    bool use_v1,
	    bool use_v2,
	    bool use_v3,
	    bool use_quaternion,
	    AspnMeasurementPositionVelocityAttitudeReferenceFrame expected_frame =
	        ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC);

	/**
	 * Constructor for processor that updates multiple state blocks.
	 *
	 * @param label Name of this processor.
	 * @param state_block_labels A vector of labels referring to StateBlocks this processor updates.
	 * It is required that the first state block have an estimate vector of at least length 6 where
	 * elements[0..2] contain North, East and Down position error states in m, and elements [3..5]
	 * contain North, East and Down velocity error states in m/s.
	 * @param measurement_matrix NxM Matrix, mapping the M states to a N-element Vector containing
	 * the velocity measurement, where N is the number of use_* parameters that are true.
	 * @param use_p1 When true, attempts to use the p1 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_p2 When true, attempts to use the p2 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_p3 When true, attempts to use the p3 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_v1 When true, attempts to use the v1 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_v2 When true, attempts to use the v2 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_v3 When true, attempts to use the v3 component of the measurement. Throws an error
	 * if this component is null (NaN).
	 * @param use_quaternion When true, attempts to use the quaternion component of the measurement.
	 * Throws an error if this component is null (NaN).
	 * @param expected_frame Logs a warning when the ASPN measurement's reference frame differs from
	 * \p expected_frame
	 */
	PositionVelocityAttitudeMeasurementProcessor(
	    std::string label,
	    std::vector<std::string> state_block_labels,
	    Matrix measurement_matrix,
	    bool use_p1,
	    bool use_p2,
	    bool use_p3,
	    bool use_v1,
	    bool use_v2,
	    bool use_v3,
	    bool use_quaternion,
	    AspnMeasurementPositionVelocityAttitudeReferenceFrame expected_frame =
	        ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC);

	/**
	 * Generates a StandardMeasurementModel that relates a `filtering::PairedPva` wrapping a
	 * `MeasurementPositionVelocityAttitude` to a state vector with contiguous NED position and
	 * velocity error states.
	 *
	 * @param measurement Measurement of type `filtering::PairedPva`. The latitude and longitude
	 * fields of the MeasurementPositionVelocityAttitude should be in units of radians, and altitude
	 * should be in meters. It is assumed that the measured altitude is with respect to the same
	 * reference as that contained in the reference navigation solution (such as HAE). The velocity
	 * field should contain North, East and Down velocity measurements in m/s.
	 *
	 * @param gen_x_and_p_func A function that will generate `xhat` (a Vector of estimated states,
	 * constructed from state blocks referenced by `state_block_labels`) and `P` (covariance Matrix
	 * for `xhat`) when called.
	 *
	 * @return A constructed StandardMeasurementModel if \p measurement is a `filtering::PairedPva`
	 * containing a `MeasurementPositionVelocityAttitude`, otherwise `nullptr`.
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
	Vector z;
	Size index_p1;
	Size index_p2;
	Size index_p3;
	Size index_v1;
	Size index_v2;
	Size index_v3;
	std::vector<Size> index_tilts;
	const bool use_p1;
	const bool use_p2;
	const bool use_p3;
	const bool use_v1;
	const bool use_v2;
	const bool use_v3;
	const bool use_quaternion;
	const AspnMeasurementPositionVelocityAttitudeReferenceFrame expected_frame;

	void setup();
};


}  // namespace filtering
}  // namespace navtk
