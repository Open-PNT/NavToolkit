#include <navtk/filtering/experimental/processors/NonlinearAltitudeProcessor.hpp>

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/factory.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>

#include <atomic>

using aspn_xtensor::MeasurementAltitude;
using navtk::geospatial::SimpleElevationProvider;

namespace navtk {
namespace filtering {
namespace experimental {

NonlinearAltitudeProcessor::NonlinearAltitudeProcessor(
    std::string label,
    std::vector<std::string> state_block_labels,
    std::vector<unsigned long> marked_state_indices,
    not_null<std::shared_ptr<SimpleElevationProvider>> elevation_provider,
    size_t state_vector_length,
    int warning_threshold)
    : MeasurementProcessor(std::move(label), std::move(state_block_labels)),
      elevation_provider(elevation_provider),
      state_indices(std::move(marked_state_indices)),
      state_vector_length(state_vector_length),
      elevation_warning_threshold(warning_threshold) {
	if (state_indices.size() < 2) {
		log_or_throw<std::invalid_argument>(
		    "NonlinearAltitudeProcessor constructor's marked_state_indices argument needs at least "
		    "two elements (latitude and longitude indices).");
	}
}


std::shared_ptr<StandardMeasurementModel> NonlinearAltitudeProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction) {

	std::shared_ptr<MeasurementAltitude> data =
	    std::dynamic_pointer_cast<MeasurementAltitude>(measurement);
	if (data == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "NonlinearAltitudeProcessor::generate_model() received a non-altitude "
		    "measurement, returning nullptr, unable to perform update.");
		return nullptr;
	}
	auto h = [this, data](const Vector& x) {
		double latitude  = x(state_indices.at(0));
		double longitude = x(state_indices.at(1));

		longitude = navtk::navutils::wrap_to_2_pi(longitude);

		std::pair<bool, double> elevation_lookup =
		    this->elevation_provider->lookup_datum(latitude, longitude);

		if (!elevation_lookup.first) {
			static std::atomic<int> failure_count(0);

			if (failure_count >= 0) {
				failure_count++;

				if (failure_count > elevation_warning_threshold) {
					spdlog::warn(
					    "lookup_datum has failed {} times, latest for {}/{}. This warning will not "
					    "be displayed again. Is elevation data available?",
					    fmt::streamed(failure_count),
					    latitude,
					    longitude);
					failure_count = INT32_MIN;
				}
			}
		}
		double bias = state_indices.size() == 3 ? x(state_indices.at(2)) : 0.0;
		// Return double max if particle is off the map, to trigger that particle to resample
		// TODO #618: Consider not updating a particle if it is off the map (once that is possible)
		return Vector{elevation_lookup.first ? elevation_lookup.second + bias
		                                     : std::numeric_limits<double>::max()};
	};

	Vector z = {data->get_altitude()};
	// TODO #600: Consider using sampled model support when that is implemented.
	Matrix H = navtk::zeros(1, state_vector_length);
	Matrix R = {{data->get_variance()}};

	return std::make_shared<StandardMeasurementModel>(StandardMeasurementModel(z, h, H, R));
}

not_null<std::shared_ptr<MeasurementProcessor<>>> NonlinearAltitudeProcessor::clone() {
	return std::make_shared<NonlinearAltitudeProcessor>(*this);
}
}  // namespace experimental
}  // namespace filtering
}  // namespace navtk
