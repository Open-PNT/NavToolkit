#include <navtk/filtering/processors/Attitude3dMeasurementProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/navutils/navigation.hpp>

using aspn_xtensor::MeasurementAttitude3D;

namespace navtk {
namespace filtering {

Attitude3dMeasurementProcessor::Attitude3dMeasurementProcessor(
    std::string label,
    const std::string &state_block_label,
    AspnMeasurementAttitude3DReferenceFrame expected_frame)
    : MeasurementProcessor(std::move(label), std::vector<std::string>(1, state_block_label)),
      expected_frame(expected_frame) {}

Attitude3dMeasurementProcessor::Attitude3dMeasurementProcessor(
    std::string label,
    std::vector<std::string> state_block_labels,
    AspnMeasurementAttitude3DReferenceFrame expected_frame)
    : MeasurementProcessor(std::move(label), std::move(state_block_labels)),
      expected_frame(expected_frame) {}


EstimateWithCovariance extract_z_r(const MeasurementAttitude3D &d,
                                   AspnMeasurementAttitude3DReferenceFrame expected_frame) {
	if (d.get_reference_frame() != expected_frame)
		spdlog::warn("Received an altitude measurement in an unexpected frame.");
	auto rpy = navutils::quat_to_rpy(d.get_quaternion());
	return {rpy, d.get_tilt_error_covariance()};
}

EstimateWithCovariance extract_z_r(std::shared_ptr<MeasurementAttitude3D> meas_data,
                                   const NavSolution &ref_pva,
                                   AspnMeasurementAttitude3DReferenceFrame expected_frame) {
	if (meas_data->get_reference_frame() != expected_frame)
		spdlog::warn("Received an altitude measurement in an unexpected frame.");
	auto cns_meas  = navutils::quat_to_dcm(Vector4{meas_data->get_quaternion()});
	auto csn_ref   = ref_pva.rot_mat;
	auto c_nest_n  = dot(cns_meas, csn_ref);
	auto tilt_meas = -Vector3{-c_nest_n(1, 2) + c_nest_n(2, 1),
	                          c_nest_n(0, 2) - c_nest_n(2, 0),
	                          -c_nest_n(0, 1) + c_nest_n(1, 0)} /
	                 2.0;
	return {tilt_meas, meas_data->get_tilt_error_covariance()};
}

EstimateWithCovariance extract_z_r(std::shared_ptr<aspn_xtensor::AspnBase> d,
                                   AspnMeasurementAttitude3DReferenceFrame expected_frame) {
	auto m1 = std::dynamic_pointer_cast<PairedPva>(d);
	if (m1 != nullptr) {
		auto att = std::dynamic_pointer_cast<MeasurementAttitude3D>(m1->meas_data);
		if (att != nullptr) return extract_z_r(att, m1->ref_pva, expected_frame);
	}
	auto m2 = std::dynamic_pointer_cast<MeasurementAttitude3D>(d);
	if (m2 != nullptr) {
		return extract_z_r(*m2, expected_frame);
	}
	// Size 0 will cause matrix validation to error and short-circuit our more useful message if DIE
	return EstimateWithCovariance{zeros(1), zeros(1, 1)};
}


std::shared_ptr<StandardMeasurementModel> Attitude3dMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	auto xhat_p = gen_x_and_p_func(get_state_block_labels());
	if (xhat_p == nullptr) {
		// StandardFusionEngine will log/throw in this case as well (empty state_block_labels)
		log_or_throw<std::runtime_error>(
		    "gen_x_and_p_func returned null. No model can be generated.");
		return nullptr;
	}

	auto num_states = num_rows(xhat_p->estimate);
	if (num_states < 3) {
		log_or_throw<std::runtime_error>(
		    "No model can be generated as there are fewer than 3 states available. Do "
		    "state_block_labels refer to the correct StateBlocks?");
		return nullptr;
	}

	auto zr = extract_z_r(measurement, expected_frame);
	if (num_rows(zr.estimate) != 3) {
		log_or_throw<std::runtime_error>(
		    "No model can be generated as data payload in measurement is unsupported. Check "
		    "documentation for supported types.");
		return nullptr;
	}

	Matrix linear_meas = zeros(3, num_states);
	if (num_states == 3) {
		xt::view(linear_meas, xt::all(), xt::range(0, 3)) = eye(3);
	} else if (num_states > 3 && num_states < 9) {
		xt::view(linear_meas, xt::all(), xt::range(num_states - 3, num_states)) = eye(3);
	} else {
		xt::view(linear_meas, xt::all(), xt::range(6, 9)) = eye(3);
	}

	auto h = [linear_meas = linear_meas](const Vector &xhat) { return dot(linear_meas, xhat); };

	return std::make_shared<StandardMeasurementModel>(
	    StandardMeasurementModel(zr.estimate, h, linear_meas, zr.covariance));
}

not_null<std::shared_ptr<MeasurementProcessor<>>> Attitude3dMeasurementProcessor::clone() {
	return std::make_shared<Attitude3dMeasurementProcessor>(*this);
}

}  // namespace filtering
}  // namespace navtk
