#include <navtk/filtering/processors/GeodeticPos2dMeasurementProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/processors/DirectMeasurementProcessor.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>

using aspn_xtensor::MeasurementPosition;

namespace navtk {
namespace filtering {

GeodeticPos2dMeasurementProcessor::GeodeticPos2dMeasurementProcessor(
    std::string label, const std::string& state_block_label, Matrix measurement_matrix)
    : DirectMeasurementProcessor(
          std::move(label), std::move(state_block_label), std::move(measurement_matrix)) {}

GeodeticPos2dMeasurementProcessor::GeodeticPos2dMeasurementProcessor(
    std::string label, std::vector<std::string> state_block_labels, Matrix measurement_matrix)
    : DirectMeasurementProcessor(
          std::move(label), std::move(state_block_labels), std::move(measurement_matrix)) {}

std::shared_ptr<StandardMeasurementModel> GeodeticPos2dMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	std::shared_ptr<NavSolution> pva;
	auto data = std::dynamic_pointer_cast<MeasurementPosition>(measurement);
	if (data == nullptr) {
		auto paired_pva = std::dynamic_pointer_cast<PairedPva>(measurement);
		if (paired_pva == nullptr) {
			log_or_throw<std::invalid_argument>(
			    "Measurement is not of correct type (PairedPva or MeasurementPosition). Unable to "
			    "perform update.");
			return nullptr;
		}
		data = std::dynamic_pointer_cast<MeasurementPosition>(paired_pva->meas_data);
		if (data == nullptr) {
			log_or_throw<std::invalid_argument>(
			    "PairedPva measurement data is not of correct type (MeasurementPosition). Unable "
			    "to perform update.");
			return nullptr;
		}
		pva = std::make_shared<NavSolution>(paired_pva->ref_pva);
	}

	auto time = data->get_time_of_validity();

	if (data->get_reference_frame() != ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC)
		spdlog::warn("Received a position measurement in an unexpected reference frame.");

	auto latitude  = data->get_term1();
	auto longitude = data->get_term2();

	Vector z{latitude, longitude};

	if (xt::isnan(z)()) log_or_throw("Some position components are NaN");

	if (pva != nullptr) z -= xt::view(pva->pos, xt::range(0, 2));

	z(0) = navutils::north_to_delta_lat(z(0), pva->pos[0], pva->pos[2]);
	z(1) = navutils::east_to_delta_lon(z(1), pva->pos[0], pva->pos[2]);

	auto new_meas = std::make_shared<GaussianVectorData>(time, z, data->get_covariance());
	return DirectMeasurementProcessor::generate_model(new_meas, gen_x_and_p_func);
}

}  // namespace filtering
}  // namespace navtk
