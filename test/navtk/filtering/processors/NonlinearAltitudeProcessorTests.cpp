#include <memory>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/experimental/processors/NonlinearAltitudeProcessor.hpp>
#include <navtk/geospatial/providers/SimpleElevationProvider.hpp>
#include <navtk/geospatial/sources/ElevationSource.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::MeasurementAltitude;
using aspn_xtensor::MeasurementBarometer;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::dot;
using navtk::eye;
using navtk::Matrix;
using navtk::Matrix3;
using navtk::Vector;
using navtk::Vector3;
using navtk::zeros;
using navtk::filtering::experimental::NonlinearAltitudeProcessor;
using navtk::geospatial::ElevationSource;
using navtk::geospatial::SimpleElevationProvider;
using navtk::geospatial::SpatialMapDataProvider;
using navtk::navutils::hae_to_msl;
using navtk::navutils::msl_to_hae;

class FakeMapDataSource : public ElevationSource {
public:
	FakeMapDataSource(
	    AspnMeasurementAltitudeReference in_ref  = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE,
	    AspnMeasurementAltitudeReference out_ref = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE) {
		latitude_max     = 0.0;
		latitude_min     = -0.1;
		longitude_max    = -1.36;
		longitude_min    = -1.38;
		input_reference  = in_ref;
		output_reference = out_ref;
	}
	std::pair<bool, double> lookup_datum(double latitude, double longitude) const override {
		if (longitude >= navtk::navutils::PI) {
			longitude = longitude - 2.0 * navtk::navutils::PI;
		}
		if (latitude >= latitude_min && latitude <= latitude_max && longitude >= longitude_min &&
		    longitude <= longitude_max) {
			double elevation = -500.0 * latitude + 4000.0 * (1.37 + longitude);
			if (input_reference == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE &&
			    output_reference == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL) {
				return hae_to_msl(elevation, latitude, longitude);
			} else if (input_reference == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL &&
			           output_reference == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE) {
				return msl_to_hae(elevation, latitude, longitude);
			}
			return {true, elevation};
		} else {
			return {false, 0.0};
		}
	}

	void set_output_vertical_reference_frame(AspnMeasurementAltitudeReference out_ref) override {
		output_reference = out_ref;
	}

private:
	double latitude_max;
	double latitude_min;
	double longitude_max;
	double longitude_min;
	AspnMeasurementAltitudeReference input_reference;
	AspnMeasurementAltitudeReference output_reference;
};

class NonlinearAltitudeProcessorTests : public ::testing::Test {
public:
	NonlinearAltitudeProcessorTests() {
		elevation_provider = std::make_shared<SimpleElevationProvider>(
		    std::make_shared<FakeMapDataSource>(ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE));
		msl_elevation_provider = std::make_shared<SimpleElevationProvider>(
		    std::make_shared<FakeMapDataSource>(ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE),
		    ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL);
		lat                   = -0.03278667;
		auto alternate_lat    = -0.06455983;
		lon                   = -1.36505483;
		auto alternate_lon    = -1.37722395;
		auto height           = elevation_provider->lookup_datum(lat, lon);
		auto alternate_height = elevation_provider->lookup_datum(alternate_lat, alternate_lon);
		if (!height.first || !alternate_height.first) {
			std::runtime_error("Expected elevation points not found in example GEOTIFF file.");
		}
		llh                = {lat, lon, height.second};
		alternate_llh      = {alternate_lat, alternate_lon, alternate_height.second};
		state_block_labels = {"label1", "label2"};
		state_indices      = {0, 1, 2};
		processor_label    = "processor";
		processor          = std::make_shared<NonlinearAltitudeProcessor>(
            processor_label, state_block_labels, state_indices, elevation_provider, 3);

		auto header = TypeHeader(ASPN_MEASUREMENT_ALTITUDE, 0, 0, 0, 0);
		auto time   = TypeTimestamp(int64_t(0));
		altitude    = 85.0;
		variance    = 1.0;
		altitude_message =
		    std::make_shared<MeasurementAltitude>(header,
		                                          time,
		                                          ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL,
		                                          altitude,
		                                          variance,
		                                          ASPN_MEASUREMENT_ALTITUDE_ERROR_MODEL_NONE,
		                                          Vector(),
		                                          std::vector<aspn_xtensor::TypeIntegrity>{});
	}
	double lat;
	double lon;
	std::shared_ptr<SimpleElevationProvider> elevation_provider;
	std::shared_ptr<SimpleElevationProvider> msl_elevation_provider;
	std::shared_ptr<NonlinearAltitudeProcessor> processor;
	Vector3 llh;
	Vector3 alternate_llh;
	std::string undulation_path;
	std::vector<std::string> state_block_labels;
	std::vector<unsigned long> state_indices;
	std::string processor_label;
	double altitude;
	double variance;
	std::shared_ptr<MeasurementAltitude> altitude_message;
};

ERROR_MODE_SENSITIVE_TEST(TEST_F, NonlinearAltitudeProcessorTests, test_constructor) {
	std::shared_ptr<NonlinearAltitudeProcessor> another_processor = nullptr;
	std::string label                                             = "label";
	std::vector<std::string> state_block_labels                   = {"sb1", "sb2"};
	std::vector<unsigned long> state_indices                      = {0, 1};
	EXPECT_NO_THROW(another_processor = std::make_shared<NonlinearAltitudeProcessor>(
	                    label, state_block_labels, state_indices, test.elevation_provider, 3));
	state_indices = {0};
	EXPECT_HONORS_MODE_EX(another_processor = std::make_shared<NonlinearAltitudeProcessor>(
	                          label, state_block_labels, state_indices, test.elevation_provider, 3),
	                      "needs at least two",
	                      std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, NonlinearAltitudeProcessorTests, test_wrong_measurement) {
	auto header     = TypeHeader(ASPN_MEASUREMENT_BAROMETER, 0, 0, 0, 0);
	auto time       = TypeTimestamp(int64_t(0));
	double pressure = 101325.0;
	auto pressure_message =
	    std::make_shared<MeasurementBarometer>(header,
	                                           time,
	                                           pressure,
	                                           0,
	                                           ASPN_MEASUREMENT_BAROMETER_ERROR_MODEL_NONE,
	                                           Vector(),
	                                           std::vector<aspn_xtensor::TypeIntegrity>{});

	decltype(test.processor->generate_model(pressure_message, nullptr)) model = nullptr;
	EXPECT_HONORS_MODE_EX(model = test.processor->generate_model(pressure_message, nullptr),
	                      "received a non-altitude measurement",
	                      std::invalid_argument);
	EXPECT_EQ(model, nullptr);
}

TEST_F(NonlinearAltitudeProcessorTests, test_generate_model) {
	auto model = processor->generate_model(altitude_message, nullptr);

	ASSERT_ALLCLOSE(Vector{altitude}, model->z);
	ASSERT_ALLCLOSE(Matrix{{variance}}, model->R);

	// Test under normal conditions
	Vector x    = {lat, lon, 5.0};
	auto h_of_x = model->h(x);

	ASSERT_ALLCLOSE(Vector{llh(2) + x(2)}, h_of_x);
	ASSERT_ALLCLOSE(navtk::zeros(1, 3), model->H);

	// Test out of map bounds. Bounds are:
	// latitude: [-0.1, 0.0]
	// longitude: [-1.38, -1.36]
	Vector out_of_bounds_x = {0.5, -1.2, 5.0};
	h_of_x                 = model->h(out_of_bounds_x);

	ASSERT_ALLCLOSE(Vector{std::numeric_limits<double>::max()}, h_of_x);

	// Test msl conversion
	auto msl_processor = std::make_shared<NonlinearAltitudeProcessor>(
	    processor_label, state_block_labels, state_indices, msl_elevation_provider, 3);

	auto msl_model         = msl_processor->generate_model(altitude_message, nullptr);
	auto elevation         = msl_elevation_provider->lookup_datum(llh(0), llh(1));
	Vector expected_h_of_x = {elevation.second + x(2)};
	ASSERT_ALLCLOSE(expected_h_of_x, msl_model->h(x));
}

TEST_F(NonlinearAltitudeProcessorTests, test_clone) {
	auto processor_clone =
	    std::dynamic_pointer_cast<NonlinearAltitudeProcessor>(processor->clone());

	ASSERT_EQ(processor->get_state_block_labels()[0], processor_clone->get_state_block_labels()[0]);
	ASSERT_EQ(processor->get_label(), processor_clone->get_label());

	auto model            = processor->generate_model(altitude_message, nullptr);
	auto model_from_clone = processor_clone->generate_model(altitude_message, nullptr);

	Vector x = {lat, lon, 5.0};
	ASSERT_ALLCLOSE(model->h(x), model_from_clone->h(x));
}
