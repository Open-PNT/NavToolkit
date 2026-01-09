#pragma once

#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/geospatial/providers/SimpleElevationProvider.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {
namespace experimental {

/**
 * A measurement processor that generates a non-linear map-based altitude measurement model.
 */
class NonlinearAltitudeProcessor : public MeasurementProcessor<> {

public:
	/**
	 * Constructor for NonlinearAltitudeProcessor.
	 * @param label The label for the measurement processor.
	 * @param state_block_labels The labels of state blocks associated with this measurement
	 * processor.
	 * @param marked_state_indices The indices of the latitude, longitude and possibly bias states
	 * in the combined state vector of whatever state blocks are associated with this measurement
	 * processor. For example, if your measurement model is z = Elevation at (latitude, longitude)
	 * + bias, then marked_state_indices will have {latitude_index, longitude_index,
	 * bias_state_index} in that order, while if your measurement model does not include a bias
	 * state, marked_state_indices will only have the latitude and longitude indices.
	 * @param elevation_provider A geospatial::SimpleElevationProvider object for looking up
	 * elevations from an elevation map source or collection of sources. The measurement processor
	 * assumes the elevation_provider has already been given the necessary sources.
	 * @param state_vector_length The length of the combined state vector from all associated state
	 * blocks.
	 * @param warning_threshold The number of elevation lookup failures in generate_model that must
	 * occur before the user is warned that there may be a problem.  Set to INT32_MIN to disable the
	 * warning completely.
	 */
	NonlinearAltitudeProcessor(
	    std::string label,
	    std::vector<std::string> state_block_labels,
	    std::vector<unsigned long> marked_state_indices,
	    not_null<std::shared_ptr<geospatial::SimpleElevationProvider>> elevation_provider,
	    size_t state_vector_length,
	    int warning_threshold = 10000);
	/**
	 * Generates the measurement model.
	 * @param measurement The altitude measurement.
	 * @return A measurement model containing `z`, `h`, `H`, and `R`. `z` is passed through without
	 * modification. The `R` returned is the same as the measurement covariance passed in. The `H`
	 * returned does not actually function as `H`, since the measurement model is not able to be
	 * linearized.
	 * Note: h will return a highly unlikely expected measurement value if the state vector's
	 * location is off the provided elevation map.
	 */
	std::shared_ptr<StandardMeasurementModel> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction) override;
	/**
	 * Clones the measurement processor.
	 * @return A shared pointer to a clone of the measurement processor.
	 */
	not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override;

private:
	/**
	 * Object for looking up elevations from a map source.
	 */
	not_null<std::shared_ptr<geospatial::SimpleElevationProvider>> elevation_provider;
	/** The indices of the latitude, longitude and possibly bias states in the
	 * combined state vector of whatever state blocks are associated with this measurement
	 * processor.
	 */
	std::vector<unsigned long> state_indices;
	/**
	 * The length of the combined state vector from all associated state blocks.
	 */
	size_t state_vector_length;
	/**
	 * The number of elevation lookup failures in generate_model that must
	 * occur before the user is warned that there may be a problem.  Set to
	 * INT32_MIN to disable the warning completely.
	 */
	int elevation_warning_threshold;
};
}  // namespace experimental
}  // namespace filtering
}  // namespace navtk
