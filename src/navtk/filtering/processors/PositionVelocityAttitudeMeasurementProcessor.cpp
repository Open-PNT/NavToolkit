#include <navtk/filtering/processors/PositionVelocityAttitudeMeasurementProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/navutils/navigation.hpp>

using aspn_xtensor::MeasurementPositionVelocityAttitude;

namespace navtk {
namespace filtering {

PositionVelocityAttitudeMeasurementProcessor::PositionVelocityAttitudeMeasurementProcessor(
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
    AspnMeasurementPositionVelocityAttitudeReferenceFrame expected_frame)
    : MeasurementProcessor(std::move(label), std::vector<std::string>(1, state_block_label)),
      measurement_matrix(std::move(measurement_matrix)),
      use_p1(use_p1),
      use_p2(use_p2),
      use_p3(use_p3),
      use_v1(use_v1),
      use_v2(use_v2),
      use_v3(use_v3),
      use_quaternion(use_quaternion),
      expected_frame(expected_frame) {
	setup();
}

PositionVelocityAttitudeMeasurementProcessor::PositionVelocityAttitudeMeasurementProcessor(
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
    AspnMeasurementPositionVelocityAttitudeReferenceFrame expected_frame)
    : MeasurementProcessor(std::move(label), std::move(state_block_labels)),
      measurement_matrix(std::move(measurement_matrix)),
      use_p1(use_p1),
      use_p2(use_p2),
      use_p3(use_p3),
      use_v1(use_v1),
      use_v2(use_v2),
      use_v3(use_v3),
      use_quaternion(use_quaternion),
      expected_frame(expected_frame) {
	setup();
}

std::shared_ptr<StandardMeasurementModel>
PositionVelocityAttitudeMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	std::shared_ptr<PairedPva> input_meas = std::dynamic_pointer_cast<PairedPva>(measurement);
	if (input_meas == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (PairedPva). Unable to perform update.");
		return nullptr;
	}
	auto data =
	    std::dynamic_pointer_cast<MeasurementPositionVelocityAttitude>(input_meas->meas_data);
	if (data == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "PairedPva measurement data is not of correct type "
		    "(MeasurementPositionVelocityAttitude). Unable to perform update.");
		return nullptr;
	}

	Vector3 ref_pos = input_meas->ref_pva.pos;
	Vector3 ref_vel = input_meas->ref_pva.vel;

	// Calculate z from input measurement data
	if (data->get_reference_frame() != expected_frame)
		spdlog::warn("Received a PVA measurement in an unexpected reference frame.");

	auto p1            = data->get_p1();
	auto p2            = data->get_p2();
	auto p3            = data->get_p3();
	auto v1            = data->get_v1();
	auto v2            = data->get_v2();
	auto v3            = data->get_v3();
	Vector4 quaternion = data->get_quaternion();

	if (use_p1) {
		if (isnan(p1))
			log_or_throw(
			    "Received NaN value for p1 field in ASPN MeasurementPositionVelocityAttitude.");
		else {
			z(index_p1) = navutils::delta_lat_to_north(p1 - ref_pos(0), ref_pos[0], ref_pos[2]);
		}
	}
	if (use_p2) {
		if (isnan(p2))
			log_or_throw(
			    "Received NaN value for p2 field in ASPN MeasurementPositionVelocityAttitude.");
		else {
			z(index_p2) = navutils::delta_lat_to_north(p2 - ref_pos(1), ref_pos[0], ref_pos[2]);
		}
	}
	if (use_p3) {
		if (isnan(p3))
			log_or_throw(
			    "Received NaN value for p3 field in ASPN MeasurementPositionVelocityAttitude.");
		else {
			z(index_p3) = -(p3 - ref_pos(2));
		}
	}

	if (use_v1) {
		if (isnan(v1))
			log_or_throw(
			    "Received NaN value for v1 field in ASPN MeasurementPositionVelocityAttitude.");
		else {
			z(index_v1) = v1 - ref_vel(0);
		}
	}
	if (use_v2) {
		if (isnan(v2))
			log_or_throw(
			    "Received NaN value for v2 field in ASPN MeasurementPositionVelocityAttitude.");
		else {
			z(index_v2) = v2 - ref_vel(1);
		}
	}
	if (use_v3) {
		if (isnan(v3))
			log_or_throw(
			    "Received NaN value for v3 field in ASPN MeasurementPositionVelocityAttitude.");
		else {
			z(index_v3) = v3 - ref_vel(2);
		}
	}

	if (use_quaternion) {
		if (xt::isnan(quaternion)())
			log_or_throw(
			    "Received NaN value for quaternion field in ASPN "
			    "MeasurementPositionVelocityAttitude.");
		else {
			auto cns_meas  = navutils::quat_to_dcm(quaternion);
			auto csn_ref   = input_meas->ref_pva.rot_mat;
			auto c_nest_n  = dot(cns_meas, csn_ref);
			auto tilt_meas = -Vector3{-c_nest_n(1, 2) + c_nest_n(2, 1),
			                          c_nest_n(0, 2) - c_nest_n(2, 0),
			                          -c_nest_n(0, 1) + c_nest_n(1, 0)} /
			                 2.0;
			xt::view(z, xt::keep(index_tilts)) = tilt_meas;
		}
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

not_null<std::shared_ptr<MeasurementProcessor<>>>
PositionVelocityAttitudeMeasurementProcessor::clone() {
	return std::make_shared<PositionVelocityAttitudeMeasurementProcessor>(*this);
}

void PositionVelocityAttitudeMeasurementProcessor::setup() {

	size_t num_expected = 0;
	if (use_p1) {
		index_p1 = num_expected;
		num_expected += 1;
	}
	if (use_p2) {
		index_p2 = num_expected;
		num_expected += 1;
	}
	if (use_p3) {
		index_p3 = num_expected;
		num_expected += 1;
	}
	if (use_v1) {
		index_v1 = num_expected;
		num_expected += 1;
	}
	if (use_v2) {
		index_v2 = num_expected;
		num_expected += 1;
	}
	if (use_v3) {
		index_v3 = num_expected;
		num_expected += 1;
	}
	if (use_quaternion) {
		index_tilts = {num_expected, num_expected + 1, num_expected + 2};
		num_expected += 3;
	}

	utils::ValidationContext{}
	    .add_matrix(measurement_matrix, "measurement_matrix")
	    .dim(num_expected, 'N')
	    .validate();

	z = zeros(num_expected);
}

}  // namespace filtering
}  // namespace navtk
