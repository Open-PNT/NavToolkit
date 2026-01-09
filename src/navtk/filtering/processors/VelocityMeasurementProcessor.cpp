#include <navtk/filtering/processors/VelocityMeasurementProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>

using aspn_xtensor::MeasurementVelocity;

using navtk::utils::ValidationContext;

namespace navtk {
namespace filtering {

VelocityMeasurementProcessor::VelocityMeasurementProcessor(
    std::string label,
    const std::string &state_block_label,
    Matrix measurement_matrix,
    bool use_x,
    bool use_y,
    bool use_z,
    AspnMeasurementVelocityReferenceFrame expected_frame)
    : MeasurementProcessor(std::move(label), std::vector<std::string>(1, state_block_label)),
      measurement_matrix(std::move(measurement_matrix)),
      use_x(use_x),
      use_y(use_y),
      use_z(use_z),
      expected_frame(expected_frame) {

	setup();
}

VelocityMeasurementProcessor::VelocityMeasurementProcessor(
    std::string label,
    std::vector<std::string> state_block_labels,
    Matrix measurement_matrix,
    bool use_x,
    bool use_y,
    bool use_z,
    AspnMeasurementVelocityReferenceFrame expected_frame)
    : MeasurementProcessor(std::move(label), std::move(state_block_labels)),
      measurement_matrix(std::move(measurement_matrix)),
      use_x(use_x),
      use_y(use_y),
      use_z(use_z),
      expected_frame(expected_frame) {

	setup();
}

std::shared_ptr<StandardMeasurementModel> VelocityMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	std::shared_ptr<PairedPva> input_meas = std::dynamic_pointer_cast<PairedPva>(measurement);
	if (input_meas == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (PairedPva). Unable to "
		    "perform update.");
		return nullptr;
	}
	auto data = std::dynamic_pointer_cast<MeasurementVelocity>(input_meas->meas_data);
	if (data == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "PairedPva measurement data is not of correct type (MeasurementVelocity). Unable to "
		    "perform update.");
		return nullptr;
	}

	auto ref_vel = input_meas->ref_pva.vel;

	// Calculate measurement vector from input measurement data
	if (data->get_reference_frame() != expected_frame)
		spdlog::warn("Received a velocity measurement in an unexpected reference frame.");

	auto x = data->get_x();
	auto y = data->get_y();
	auto z = data->get_z();

	if (use_x) {
		if (isnan(x))
			log_or_throw(
			    "Expected non-null (non-NaN) value for x field in ASPN MeasurementVelocity "
			    "message");
		else
			meas_vector(index1) = x - ref_vel[0];
	}
	if (use_y) {
		if (isnan(y))
			log_or_throw(
			    "Expected non-null (non-NaN) value for y field in ASPN MeasurementVelocity "
			    "message");
		else
			meas_vector(index2) = y - ref_vel[1];
	}
	if (use_z) {
		if (isnan(z))
			log_or_throw(
			    "Expected non-null (non-NaN) value for z field in ASPN MeasurementVelocity "
			    "message");
		else
			meas_vector(index3) = z - ref_vel[2];
	}

	auto xhat_p = gen_x_and_p_func(get_state_block_labels());
	if (xhat_p == nullptr) {
		return nullptr;
	}
	auto h = [measurement_matrix = measurement_matrix](const Vector &xhat) {
		return dot(measurement_matrix, xhat);
	};

	return std::make_shared<StandardMeasurementModel>(
	    StandardMeasurementModel(meas_vector, h, measurement_matrix, data->get_covariance()));
}

not_null<std::shared_ptr<MeasurementProcessor<>>> VelocityMeasurementProcessor::clone() {
	return std::make_shared<VelocityMeasurementProcessor>(*this);
}

void VelocityMeasurementProcessor::setup() {

	size_t num_expected = 0;
	if (use_x) {
		index1 = num_expected;
		num_expected += 1;
	}
	if (use_y) {
		index2 = num_expected;
		num_expected += 1;
	}
	if (use_z) {
		index3 = num_expected;
		num_expected += 1;
	}

	ValidationContext{}
	    .add_matrix(measurement_matrix, "measurement_matrix")
	    .dim(num_expected, 'N')
	    .validate();

	meas_vector = zeros(num_expected);
}

}  // namespace filtering
}  // namespace navtk
