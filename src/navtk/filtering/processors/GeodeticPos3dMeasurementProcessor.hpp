#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/DirectMeasurementProcessor.hpp>
#include <navtk/filtering/utils.hpp>

namespace navtk {
namespace filtering {

/**
 * Measurement processor that accepts either a MeasurementPosition or PairedPva containing a
 * MeasurementPosition. When the measurement is a PairedPva, this processor generates a delta update
 * using the difference between the measurement and the position in the reference PVA. Otherwise, it
 * generates an absolute update directly from the values in the measurement.
 */
class GeodeticPos3dMeasurementProcessor : public DirectMeasurementProcessor {

public:
	/**
	 * Constructor for processor that updates only one state block.
	 *
	 * @param label Name of this processor.
	 * @param state_block_label Label assigned to the StateBlock this processor will be updating.
	 * @param measurement_matrix NxM Matrix, mapping the M states to a N-element Vector containing
	 * the delta position measurement, where N is the number of use_* parameters that are true.
	 */
	GeodeticPos3dMeasurementProcessor(std::string label,
	                                  const std::string &state_block_label,
	                                  Matrix measurement_matrix);

	/**
	 * Constructor for processor that updates multiple state blocks.
	 *
	 * @param label Name of this processor.
	 * @param state_block_labels Labels assigned to the StateBlocks this processor will be updating.
	 * @param measurement_matrix NxM Matrix, mapping the M states to a N-element Vector containing
	 * the delta position measurement, where N is the number of use_* parameters that are true.
	 */
	GeodeticPos3dMeasurementProcessor(std::string label,
	                                  std::vector<std::string> state_block_labels,
	                                  Matrix measurement_matrix);

	/**
	 * Generates a StandardMeasurementModel instance that maps the input data to the estimated
	 * states.
	 *
	 * @param measurement Measurement of type MeasurementPosition or PairedPva.
	 *
	 * @param gen_x_and_p_func A function that will generate `xhat` (a Vector of estimated states,
	 * constructed from state blocks referenced by `state_block_labels`) and `P` (covariance Matrix
	 * for `xhat`) when called.
	 *
	 * @return If \p measurement contains MeasurementPosition, a StandardMeasurementModel
	 * constructed from the contents of \p measurement and this processor's measurement matrix.
	 * Otherwise, a `nullptr`. Specifically, when not null the model assumes that the nonlinear
	 * measurement function \f$\hat{z} = h(x)\f$ is equivalent to \f$Hx\f$, where \f$\hat{z}\f$ is a
	 * 3-element vector containing whole-valued latitude, longitude and altitude measurements of the
	 * predicted sensor position in radians, radians and meters.
	 *
	 * @throw std::range_error If the estimate or covariance in measurement are not Nx1 and NxN,
	 * respectively, and the error mode is ErrorMode::DIE.
	 */
	std::shared_ptr<StandardMeasurementModel> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase> measurement,
	    GenXhatPFunction gen_x_and_p_func) override;
};

}  // namespace filtering
}  // namespace navtk
