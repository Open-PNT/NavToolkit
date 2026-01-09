#include <navtk/filtering/processors/DirectMeasurementProcessor.hpp>

#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/utils/ValidationContext.hpp>

using navtk::utils::ValidationContext;

namespace navtk {
namespace filtering {

DirectMeasurementProcessor::DirectMeasurementProcessor(std::string label,
                                                       const std::string &state_block_label,
                                                       Matrix measurement_matrix)
    : MeasurementProcessor(std::move(label), std::vector<std::string>(1, state_block_label)),
      measurement_matrix(std::move(measurement_matrix)) {}

DirectMeasurementProcessor::DirectMeasurementProcessor(std::string label,
                                                       std::vector<std::string> state_block_labels,
                                                       Matrix measurement_matrix)
    : MeasurementProcessor(std::move(label), std::move(state_block_labels)),
      measurement_matrix(std::move(measurement_matrix)) {}

DirectMeasurementProcessor::DirectMeasurementProcessor(const DirectMeasurementProcessor &processor)
    : MeasurementProcessor(processor), measurement_matrix(processor.measurement_matrix) {}

std::shared_ptr<StandardMeasurementModel> DirectMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction) {

	std::shared_ptr<GaussianVectorData> gvd =
	    std::dynamic_pointer_cast<GaussianVectorData>(measurement);
	if (gvd == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (GaussianVectorData). Unable to perform "
		    "update.");
		return nullptr;
	}

	ValidationContext{}
	    .add_matrix(gvd->estimate)
	    .dim('N', 1)
	    .add_matrix(gvd->covariance)
	    .dim('N', 'N')
	    .validate();

	return std::make_shared<StandardMeasurementModel>(
	    StandardMeasurementModel(gvd->estimate, measurement_matrix, gvd->covariance));
}

not_null<std::shared_ptr<MeasurementProcessor<>>> DirectMeasurementProcessor::clone() {
	return std::make_shared<DirectMeasurementProcessor>(*this);
}

Matrix DirectMeasurementProcessor::get_measurement_matrix() const { return measurement_matrix; }

}  // namespace filtering
}  // namespace navtk
