#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {

/**
 * Processes Measurements containing MeasurementDirection3DToPoints and NavSolution to update
 * a Pinson-style block. This processor projects pointing vectors from
 * MeasurementDirection3DToPoints onto a 2D plane at a unit-1 distance from the sensor center along
 * the sensor Z axis as the actual measurement
 *
 * Equations derived from:
 *      - Section 4.4.1 of:
 *        Veth, Michael J. Fusion of imaging and inertial sensors for navigation.
 *        No. AFIT/DS/ENG/06-09.
 *        AIR FORCE INST OF TECH WRIGHT-PATTERSON AFB OH SCHOOL OF ENGINEERING AND MANAGEMENT, 2006.
 *
 *      - Section 5.2 of:
 *        Venable, Donald T. Improving Real World Performance of Vision Aided Navigation in a Flight
 *        Environment.
 *        No. AFIT-ENG-DS-16-S-017. Air Force Institute of Technology WPAFB, 2016.
 *
 */
class DirectionToPoints3dMeasurementProcessor : public MeasurementProcessor<> {

public:
	/**
	 * Constructor for processor that updates only one state block.
	 *
	 * @param label Name of this processor.
	 * @param state_block_label Label assigned to the Pinson15 StateBlock this processor will be
	 * updating.
	 */
	DirectionToPoints3dMeasurementProcessor(std::string label,
	                                        const std::string &state_block_label);

	/**
	 * Generates a StandardMeasurementModel instance that maps the input data to the estimated
	 * states.
	 *
	 * @param measurement Measurement of type filtering::PairedPva (wrapping a
	 * MeasurementDirection3DToPoints).
	 *
	 * @param gen_x_and_p_func A function that will generate `xhat` (a Vector of estimated states,
	 * constructed from state blocks referenced by `state_block_labels`) and `P` (covariance Matrix
	 * for `xhat`) when called.
	 *
	 * @return A constructed StandardMeasurementModel if \p measurement is a filtering::PairedPva
	 * containing a MeasurementDirection3DToPoints, otherwise `nullptr`.
	 */
	std::shared_ptr<StandardMeasurementModel> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase> measurement,
	    GenXhatPFunction gen_x_and_p_func) override;

	/**
	 * Accepts aux data for body to sensor mounting; ignores all other types.
	 *
	 * @param aux_data `shared_ptr` to aspn_xtensor::MetadataGeneric, which contains an
	 * aspn_xtensor::TypeMounting. This mounting is assumed to contain a quaternion to rotate
	 * measurements from the body to sensor frame.
	 */
	void receive_aux_data(const AspnBaseVector &aux_data) override;

	/**
	 * Create a copy of the MeasurementProcessor with the same properties.
	 *
	 * @return A shared pointer to a copy of the MeasurementProcessor.
	 */
	not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override;

private:
	Matrix3 ccb_mat = eye(3);
};


}  // namespace filtering
}  // namespace navtk
