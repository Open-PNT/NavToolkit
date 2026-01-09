#include <navtk/filtering/processors/MagneticFieldMagnitudeMeasurementProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/navutils/navigation.hpp>

using aspn_xtensor::MeasurementMagneticFieldMagnitude;
using navtk::navutils::RAD2DEG;

namespace navtk {
namespace filtering {

MagneticFieldMagnitudeMeasurementProcessor::MagneticFieldMagnitudeMeasurementProcessor(
    std::string label, const std::string &state_block_label, Vector x_vec, Vector y_vec, Matrix map)
    : MeasurementProcessor(std::move(label), std::vector<std::string>(1, state_block_label)),
      h_map(std::move(x_vec), std::move(y_vec), std::move(map)) {}

MagneticFieldMagnitudeMeasurementProcessor::MagneticFieldMagnitudeMeasurementProcessor(
    std::string label,
    std::vector<std::string> state_block_labels,
    Vector x_vec,
    Vector y_vec,
    Matrix map)
    : MeasurementProcessor(std::move(label), std::move(state_block_labels)),
      h_map(std::move(x_vec), std::move(y_vec), std::move(map)) {}

std::shared_ptr<StandardMeasurementModel>
MagneticFieldMagnitudeMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	std::shared_ptr<PairedPva> input_meas = std::dynamic_pointer_cast<PairedPva>(measurement);
	if (input_meas == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (PairedPva). Unable to perform update.");
		return nullptr;
	}
	auto data = std::dynamic_pointer_cast<MeasurementMagneticFieldMagnitude>(input_meas->meas_data);
	if (data == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "PairedPva measurement data is not of correct type "
		    "(MeasurementMagneticFieldMagnitude). Unable to perform update.");
		return nullptr;
	}

	// Calculate z from input measurement data
	Vector z{data->get_field_strength()};

	auto xhat_p = gen_x_and_p_func(get_state_block_labels());
	if (xhat_p == nullptr) {
		return nullptr;
	}
	Vector est      = xhat_p->estimate;
	Vector3 ins_lla = input_meas->ref_pva.pos;
	auto h          = [ins_lla = ins_lla, this](double delta_north, double delta_east) {
        auto filter_corr_lat = ins_lla[0] + navutils::north_to_delta_lat(delta_north, ins_lla[0]);
        auto filter_corr_lon = ins_lla[1] + navutils::east_to_delta_lon(delta_east, ins_lla[0]);
        auto value = h_map.interpolate(filter_corr_lat * RAD2DEG, filter_corr_lon * RAD2DEG);
        return Vector{value};
	};
	auto model_h = [h = h](const Vector &xhat) { return h(xhat[0], xhat[1]); };

	// Calculate H
	auto epsilon    = 50.0;
	auto dh_d_north = (h(est[0] + epsilon / 2, est[1]) - h(est[0] - epsilon / 2, est[1])) / epsilon;
	auto dh_d_east  = (h(est[0], est[1] + epsilon / 2) - h(est[0], est[1] - epsilon / 2)) / epsilon;
	Matrix H{{dh_d_north[0], dh_d_east[0], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

	return std::make_shared<StandardMeasurementModel>(
	    StandardMeasurementModel(z, model_h, H, Matrix{{data->get_variance()}}));
}

not_null<std::shared_ptr<MeasurementProcessor<>>>
MagneticFieldMagnitudeMeasurementProcessor::clone() {
	return std::make_shared<MagneticFieldMagnitudeMeasurementProcessor>(*this);
}

}  // namespace filtering
}  // namespace navtk
