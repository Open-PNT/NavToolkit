#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {



/**
 * A processor for measurements that have a simple, direct relationship to one or more states being
 * estimated. Appropriate for instances when no external (AspnBaseVector) is required for processing
 * and the measurement function `h(x)` can be defined by a time-invariant matrix `H`. Measurement
 * must be provided as GaussianVectorData.
 */
class DirectMeasurementProcessor : public MeasurementProcessor<> {

public:
	/**
	 * Constructor for processor that updates only one state block.
	 *
	 * @param label Name of this processor.
	 * @param state_block_label Label assigned to the StateBlock this processor will be updating.
	 *
	 * @param measurement_matrix MxN matrix that maps a Vector of states to a measurement
	 * vector, where M is the number of elements in the measurement vector and N is the total number
	 * of states referenced by `state_block_labels`..
	 */
	DirectMeasurementProcessor(std::string label,
	                           const std::string &state_block_label,
	                           Matrix measurement_matrix);

	/**
	 * Constructor for processor that updates multiple state blocks.
	 *
	 * @param label Name of this processor.
	 *
	 * @param state_block_labels Labels assigned to the StateBlocks this processor will be updating.
	 *
	 * @param measurement_matrix MxN matrix that maps a Vector of states to a measurement
	 * vector, where M is the number of elements in the measurement vector and N is the total number
	 * of states referenced by `state_block_labels`..
	 */
	DirectMeasurementProcessor(std::string label,
	                           std::vector<std::string> state_block_labels,
	                           Matrix measurement_matrix);

	/**
	 * Custom copy constructor which creates a deep copy.
	 *
	 * @param processor The DirectMeasurementProcessor to copy.
	 */
	DirectMeasurementProcessor(const DirectMeasurementProcessor &processor);

	/**
	 * Generates a StandardMeasurementModel instance that maps the input data to the estimated
	 * states.
	 *
	 * @param measurement Measurement of type GaussianVectorData. When it does,
	 * GaussianVectorData::estimate is used to populate StandardMeasurementModel::z, and
	 * GaussianVectorData::covariance populates StandardMeasurementModel::R.
	 *
	 * @return If \p measurement contains GaussianVectorData, a StandardMeasurementModel constructed
	 * from the contents of \p measurement and this processor's measurement matrix. Otherwise, a
	 * `nullptr`.
	 *
	 * @throw std::range_error If the estimate or covariance in measurement are not Nx1 and NxN,
	 * respectively, and the error mode is ErrorMode::DIE.
	 */
	std::shared_ptr<StandardMeasurementModel> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction) override;

	/**
	 * Create a copy of the MeasurementProcessor with the same properties.
	 *
	 * @return A shared pointer to a copy of the MeasurementProcessor.
	 */
	not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override;

	/**
	 * @return The measurement matrix used by this processor.
	 */
	Matrix get_measurement_matrix() const;

private:
	const Matrix measurement_matrix;
};


}  // namespace filtering
}  // namespace navtk
