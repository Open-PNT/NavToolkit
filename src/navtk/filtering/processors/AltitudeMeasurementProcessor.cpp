#include <navtk/filtering/processors/AltitudeMeasurementProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>

using aspn_xtensor::MeasurementAltitude;

namespace navtk {
namespace filtering {

AltitudeMeasurementProcessor::AltitudeMeasurementProcessor(
    std::string label,
    const std::string &state_block_label,
    AspnMeasurementAltitudeReference expected_frame)
    : MeasurementProcessor(std::move(label), std::vector<std::string>(1, state_block_label)),
      expected_frame(expected_frame) {}

AltitudeMeasurementProcessor::AltitudeMeasurementProcessor(
    std::string label,
    std::vector<std::string> state_block_labels,
    AspnMeasurementAltitudeReference expected_frame)
    : MeasurementProcessor(std::move(label), std::move(state_block_labels)),
      expected_frame(expected_frame) {}

std::shared_ptr<StandardMeasurementModel> AltitudeMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	std::shared_ptr<PairedPva> input_meas = std::dynamic_pointer_cast<PairedPva>(measurement);
	if (input_meas == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (PairedPva). Unable to perform update.");
		return nullptr;
	}
	auto data = std::dynamic_pointer_cast<MeasurementAltitude>(input_meas->meas_data);
	if (data == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "PairedPva measurement data is not of correct type (MeasurementAltitude). Unable to "
		    "perform update.");
		return nullptr;
	}

	// Calculate z from input measurement data
	if (data->get_reference() != expected_frame)
		spdlog::warn("Received an altitude measurement in an unexpected frame.");
	Vector z{data->get_altitude() - input_meas->ref_pva.pos[2]};

	auto xhat_p = gen_x_and_p_func(get_state_block_labels());
	if (xhat_p == nullptr) {
		return nullptr;
	}
	Matrix linear_meas = zeros(1, num_rows(xhat_p->estimate));
	linear_meas(0, 2)  = -1.0;
	auto h = [linear_meas = linear_meas](const Vector &xhat) { return dot(linear_meas, xhat); };

	return std::make_shared<StandardMeasurementModel>(
	    StandardMeasurementModel(z, h, linear_meas, Matrix{{data->get_variance()}}));
}

not_null<std::shared_ptr<MeasurementProcessor<>>> AltitudeMeasurementProcessor::clone() {
	return std::make_shared<AltitudeMeasurementProcessor>(*this);
}

}  // namespace filtering
}  // namespace navtk
