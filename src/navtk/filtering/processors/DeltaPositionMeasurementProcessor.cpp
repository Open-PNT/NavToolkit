#include <navtk/filtering/processors/DeltaPositionMeasurementProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>

using aspn_xtensor::MeasurementDeltaPosition;

namespace navtk {
namespace filtering {

DeltaPositionMeasurementProcessor::DeltaPositionMeasurementProcessor(
    std::string label,
    const std::string &state_block_label,
    Matrix measurement_matrix,
    bool use_term1,
    bool use_term2,
    bool use_term3,
    AspnMeasurementDeltaPositionReferenceFrame expected_frame)
    : MeasurementProcessor(std::move(label), std::vector<std::string>(1, state_block_label)),
      measurement_matrix(std::move(measurement_matrix)),
      use_term1(use_term1),
      use_term2(use_term2),
      use_term3(use_term3),
      expected_frame(expected_frame) {
	setup();
}

DeltaPositionMeasurementProcessor::DeltaPositionMeasurementProcessor(
    std::string label,
    std::vector<std::string> state_block_labels,
    Matrix measurement_matrix,
    bool use_term1,
    bool use_term2,
    bool use_term3,
    AspnMeasurementDeltaPositionReferenceFrame expected_frame)
    : MeasurementProcessor(std::move(label), std::move(state_block_labels)),
      measurement_matrix(std::move(measurement_matrix)),
      use_term1(use_term1),
      use_term2(use_term2),
      use_term3(use_term3),
      expected_frame(expected_frame) {
	setup();
}

std::shared_ptr<StandardMeasurementModel> DeltaPositionMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	std::shared_ptr<PairedPva> input_meas = std::dynamic_pointer_cast<PairedPva>(measurement);
	if (input_meas == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (PairedPva). Unable to perform update.");
		return nullptr;
	}
	auto data = std::dynamic_pointer_cast<MeasurementDeltaPosition>(input_meas->meas_data);
	if (data == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "PairedPva measurement data is not of correct type (MeasurementDeltaPosition). Unable "
		    "to perform update.");
		return nullptr;
	}

	Vector3 ref_vel = input_meas->ref_pva.vel;

	// Calculate z from input measurement data
	if (data->get_reference_frame() != expected_frame)
		spdlog::warn("Received a delta position measurement in an unexpected reference frame.");

	auto term1 = data->get_term1();
	auto term2 = data->get_term2();
	auto term3 = data->get_term3();

	if (use_term1) {
		if (isnan(term1))
			log_or_throw(
			    "Expected non-null (non-NaN) value for term1 field in ASPN MeasurementVelocity "
			    "message");
		else
			z(index1) = term1 / data->get_delta_t() - ref_vel[0];
	}
	if (use_term2) {
		if (isnan(term2))
			log_or_throw(
			    "Expected non-null (non-NaN) value for term2 field in ASPN MeasurementVelocity "
			    "message");
		else
			z(index2) = term2 / data->get_delta_t() - ref_vel[1];
	}
	if (use_term3) {
		if (isnan(term3))
			log_or_throw(
			    "Expected non-null (non-NaN) value for term3 field in ASPN MeasurementVelocity "
			    "message");
		else
			z(index3) = term3 / data->get_delta_t() - ref_vel[2];
	}

	auto xhat_p = gen_x_and_p_func(get_state_block_labels());
	if (xhat_p == nullptr) {
		return nullptr;
	}
	auto h = [measurement_matrix = measurement_matrix](const Vector &xhat) {
		return dot(measurement_matrix, xhat);
	};

	return std::make_shared<StandardMeasurementModel>(
	    StandardMeasurementModel(z, h, measurement_matrix, data->get_covariance()));
}

not_null<std::shared_ptr<MeasurementProcessor<>>> DeltaPositionMeasurementProcessor::clone() {
	return std::make_shared<DeltaPositionMeasurementProcessor>(*this);
}

void DeltaPositionMeasurementProcessor::setup() {

	size_t num_expected = 0;
	if (use_term1) {
		index1 = num_expected;
		num_expected += 1;
	}
	if (use_term2) {
		index2 = num_expected;
		num_expected += 1;
	}
	if (use_term3) {
		index3 = num_expected;
		num_expected += 1;
	}

	if (utils::ValidationContext validation{}) {
		validation.add_matrix(measurement_matrix, "measurement_matrix")
		    .dim(num_expected, 'N')
		    .validate();
	}

	z = zeros(num_expected);
}

}  // namespace filtering
}  // namespace navtk
