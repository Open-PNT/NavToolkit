#include <memory>

#include <gtest/gtest.h>
#include <math.h>
#include <error_mode_assert.hpp>
#include <scalar_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/experimental/random.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/experimental/processors/BaronavMeasurementProcessor.hpp>
#include <navtk/filtering/experimental/stateblocks/BaronavMotionBlock.hpp>
#include <navtk/filtering/experimental/stateblocks/SampledFogmBlock.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/geospatial/providers/SimpleElevationProvider.hpp>
#include <navtk/geospatial/providers/SpatialMapDataProvider.hpp>
#include <navtk/geospatial/sources/SpatialMapDataSource.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::eye;
using navtk::Matrix;
using navtk::Vector;
using navtk::Vector3;
using navtk::filtering::EkfStrategy;
using navtk::filtering::EstimateWithCovariance;
using navtk::filtering::StandardFusionEngine;
using navtk::filtering::experimental::BaronavMeasurementProcessor;
using navtk::filtering::experimental::BaronavMeasurementProcessorStatus;
using navtk::geospatial::ElevationSource;
using navtk::geospatial::SimpleElevationProvider;
using navtk::geospatial::SpatialMapDataProvider;
using GenXhatPFunction =
    std::function<std::shared_ptr<EstimateWithCovariance>(const std::vector<std::string> &)>;
using aspn_xtensor::MeasurementBarometer;
using aspn_xtensor::MeasurementDeltaPosition;
using aspn_xtensor::MeasurementHeading;
using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::num_cols;
using navtk::num_rows;
using navtk::Size;
using navtk::filtering::experimental::BaronavMotionBlock;
using navtk::filtering::experimental::SampledFogmBlock;
using navtk::navutils::hae_to_msl;
using navtk::navutils::llh_to_ecef;
using navtk::navutils::msl_to_hae;

std::shared_ptr<MeasurementBarometer> create_baro_measurement(
    const aspn_xtensor::TypeTimestamp &time, double pressure) {
	auto header = TypeHeader(ASPN_MEASUREMENT_BAROMETER, 0, 0, 0, 0);
	return std::make_shared<MeasurementBarometer>(header,
	                                              time,
	                                              pressure,
	                                              0,
	                                              ASPN_MEASUREMENT_BAROMETER_ERROR_MODEL_NONE,
	                                              Vector(),
	                                              std::vector<aspn_xtensor::TypeIntegrity>{});
}

std::shared_ptr<MeasurementHeading> create_heading_measurement(
    const aspn_xtensor::TypeTimestamp &time, double obs, double variance) {
	TypeHeader header(ASPN_MEASUREMENT_HEADING, 0, 0, 0, 0);
	return std::make_shared<MeasurementHeading>(header,
	                                            time,
	                                            ASPN_MEASUREMENT_HEADING_REFERENCE_TRUE_HEADING,
	                                            obs,
	                                            variance,
	                                            ASPN_MEASUREMENT_HEADING_ERROR_MODEL_NONE,
	                                            Vector{},
	                                            std::vector<aspn_xtensor::TypeIntegrity>{});
}

std::shared_ptr<MeasurementDeltaPosition> create_delta_pos_1d_measurement(
    const aspn_xtensor::TypeTimestamp &time, double delta_t, double x, double variance) {
	TypeHeader header(ASPN_MEASUREMENT_DELTA_POSITION, 0, 0, 0, 0);
	return std::make_shared<MeasurementDeltaPosition>(
	    header,
	    time,
	    ASPN_MEASUREMENT_DELTA_POSITION_REFERENCE_FRAME_NED,
	    delta_t,
	    x,
	    NAN,
	    NAN,
	    Matrix{{variance}},
	    ASPN_MEASUREMENT_DELTA_POSITION_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
}

std::shared_ptr<MeasurementPosition> create_pos_measurement(const aspn_xtensor::TypeTimestamp &time,
                                                            Vector3 pos,
                                                            Matrix covariance) {
	TypeHeader header(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0);
	return std::make_shared<MeasurementPosition>(header,
	                                             time,
	                                             ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
	                                             pos[0],
	                                             pos[1],
	                                             pos[2],
	                                             covariance,
	                                             ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
	                                             Vector{},
	                                             std::vector<aspn_xtensor::TypeIntegrity>{});
}

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


TEST(BaronavProcessorTests, wrongMeasType) {

	std::shared_ptr<FakeMapDataSource> source = std::make_shared<FakeMapDataSource>();
	std::shared_ptr<SimpleElevationProvider> elevation_provider =
	    std::make_shared<SimpleElevationProvider>(source);
	std::shared_ptr<navtk::experimental::RandomNumberGenerator> rng =
	    std::make_shared<navtk::experimental::LocalEngineWrapper>();

	std::vector<std::string> state_block_labels = {"baronav_motion_block"};
	auto baronav_mp = BaronavMeasurementProcessor("a", state_block_labels, elevation_provider, rng);

	int num_states                = 4;
	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};

	auto meas = std::make_shared<navtk::filtering::GaussianVectorData>(
	    to_type_timestamp(1.0), Vector{1.0}, Matrix{{2.0}});
	auto measurement_model = baronav_mp.generate_model(meas, dummy_xhat_p);

	// Measurement isn't a MeasurementBarometer
	EXPECT_EQ(measurement_model, nullptr);
}

TEST(BaronavProcessorTests, nullChecks_SLOW) {
	std::vector<std::string> state_block_labels = {"baronav_motion_block"};

	std::shared_ptr<FakeMapDataSource> source = std::make_shared<FakeMapDataSource>();
	std::shared_ptr<SimpleElevationProvider> elevation_provider =
	    std::make_shared<SimpleElevationProvider>(source);

	std::shared_ptr<navtk::experimental::RandomNumberGenerator> rng =
	    std::make_shared<navtk::experimental::LocalEngineWrapper>();

	double barometer_tau                     = 1.0;
	double barometer_bias_process_sigma      = 0.0;
	double baro_elevation_measurement_sigma  = 0.0;
	double heading_tau                       = 1.0;
	double heading_bias_process_sigma        = 0.0;
	double heading_meas_sigma                = 0.0;
	double delta_pos_meas_sigma              = 0.0;
	double propagate_heading_when_stationary = false;
	size_t num_particles                     = 10;
	double initial_barometer_bias_variance   = 0.0;
	Vector initialization_variance_scale_factors{1.0, 1.0, 1.0, 1.0};
	double min_initial_position_variance     = 0.0;
	double resampling_threshold              = 1.0;
	double min_update_time                   = 0.0;
	double max_propagate_time                = 5.0;
	bool calc_single_jacobian                = true;
	double delta_t_to_estimate_current_state = 1.0;
	bool enable_debug_printouts              = false;
	std::string debug_filename               = "";

	auto baronav_mp =
	    std::make_shared<BaronavMeasurementProcessor>("a",
	                                                  state_block_labels,
	                                                  elevation_provider,
	                                                  rng,
	                                                  barometer_tau,
	                                                  barometer_bias_process_sigma,
	                                                  baro_elevation_measurement_sigma,
	                                                  heading_tau,
	                                                  heading_bias_process_sigma,
	                                                  heading_meas_sigma,
	                                                  delta_pos_meas_sigma,
	                                                  propagate_heading_when_stationary,
	                                                  num_particles,
	                                                  initial_barometer_bias_variance,
	                                                  initialization_variance_scale_factors,
	                                                  min_initial_position_variance,
	                                                  resampling_threshold,
	                                                  min_update_time,
	                                                  max_propagate_time,
	                                                  calc_single_jacobian,
	                                                  delta_t_to_estimate_current_state,
	                                                  enable_debug_printouts,
	                                                  debug_filename);

	int num_states                = 4;
	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};

	// Measurement isn't a MeasurementBarometer, expect null.
	auto meas = std::make_shared<navtk::filtering::GaussianVectorData>(
	    to_type_timestamp(1.0), Vector{1.0}, Matrix{{2.0}});
	auto measurement_model = baronav_mp->generate_model(meas, dummy_xhat_p);
	EXPECT_EQ(measurement_model, nullptr);

	// Filter is not initialized, expect null.
	double pressure       = 1344;
	auto pressure_message = create_baro_measurement(to_type_timestamp(1.0), pressure);
	measurement_model     = baronav_mp->generate_model(pressure_message, dummy_xhat_p);
	EXPECT_EQ(measurement_model, nullptr);
	EXPECT_EQ(baronav_mp->get_status(), BaronavMeasurementProcessorStatus::WAITING_FOR_INIT);

	Vector geo3d{-0.01, -1.37, 3.0};
	for (double ii = 0.0; ii < 6.0; ii += 2.0) {
		auto heading_aux_data = create_heading_measurement(to_type_timestamp(ii), 2 * ii, 1.0);
		auto delta_pos_aux =
		    create_delta_pos_1d_measurement(to_type_timestamp(ii), 1.0, 2 * ii, 1.0);
		auto position_aux = create_pos_measurement(to_type_timestamp(ii), geo3d, eye(3));
		baronav_mp->receive_aux_data({heading_aux_data, delta_pos_aux, position_aux});
	}
	EXPECT_EQ(baronav_mp->get_status(), BaronavMeasurementProcessorStatus::WAITING_FOR_INIT);

	// Add a measurement, get a measurement model
	pressure_message  = create_baro_measurement(to_type_timestamp(2), pressure);
	measurement_model = baronav_mp->generate_model(pressure_message, dummy_xhat_p);
	EXPECT_EQ(baronav_mp->get_status(), BaronavMeasurementProcessorStatus::RUNNING);

	// Measurement is at the same time that filter already is. Expect null.
	pressure_message  = create_baro_measurement(to_type_timestamp(2), pressure);
	measurement_model = baronav_mp->generate_model(pressure_message, dummy_xhat_p);
	EXPECT_EQ(measurement_model, nullptr);

	// Measurement is more than 1.0 s into the future.
	pressure_message  = create_baro_measurement(to_type_timestamp(5.5), pressure);
	measurement_model = baronav_mp->generate_model(pressure_message, dummy_xhat_p);

	for (double ii = 6.0; ii < 8.0; ++ii) {
		auto heading_aux_data = create_heading_measurement(to_type_timestamp(ii), 2.0, 1.0);
		auto delta_pos_aux = create_delta_pos_1d_measurement(to_type_timestamp(ii), 1.0, 0.0, 1.0);
		baronav_mp->receive_aux_data({heading_aux_data, delta_pos_aux});
		pressure_message  = create_baro_measurement(to_type_timestamp(ii + 1), pressure);
		measurement_model = baronav_mp->generate_model(pressure_message, dummy_xhat_p);
	}
	EXPECT_EQ(baronav_mp->get_status(), BaronavMeasurementProcessorStatus::STATIONARY);

	auto heading_aux_data = create_heading_measurement(to_type_timestamp(16.0), 2.0, 1.0);
	auto delta_pos_aux    = create_delta_pos_1d_measurement(to_type_timestamp(16.0), 1.0, 2.0, 1.0);
	baronav_mp->receive_aux_data({heading_aux_data, delta_pos_aux});

	// Measurement is more than max_propagate_time into the future. Expect a log.
	pressure_message  = create_baro_measurement(to_type_timestamp(17), pressure);
	measurement_model = baronav_mp->generate_model(pressure_message, dummy_xhat_p);
	EXPECT_EQ(baronav_mp->get_status(), BaronavMeasurementProcessorStatus::WAITING_FOR_INIT);
}

TEST(BaronavProcessorTests, positionGetters_SLOW) {
	// TODO: figure out why this is so slow
	std::shared_ptr<FakeMapDataSource> source = std::make_shared<FakeMapDataSource>();
	std::shared_ptr<SimpleElevationProvider> elevation_provider =
	    std::make_shared<SimpleElevationProvider>(source);
	std::shared_ptr<navtk::experimental::RandomNumberGenerator> rng =
	    std::make_shared<navtk::experimental::LocalEngineWrapper>();

	std::vector<std::string> state_block_labels = {"baronav_motion_block"};
	auto baronav_mp = BaronavMeasurementProcessor("a", state_block_labels, elevation_provider, rng);
	int num_states  = 4;
	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};
	double pressure        = 1344;
	auto pressure_message  = create_baro_measurement(to_type_timestamp(1), pressure);
	auto measurement_model = baronav_mp.generate_model(pressure_message, dummy_xhat_p);

	Vector start_pos{-0.01, -1.37, 3.0};
	for (double ii = 0.0; ii < 2.0; ++ii) {
		auto heading_aux_data = create_heading_measurement(to_type_timestamp(ii), 2 * ii, 1.0);
		auto delta_pos_aux =
		    create_delta_pos_1d_measurement(to_type_timestamp(ii), 1.0, 2 * ii, 1.0);
		auto position_aux = create_pos_measurement(to_type_timestamp(ii), start_pos, eye(3));
		baronav_mp.receive_aux_data({heading_aux_data, delta_pos_aux, position_aux});
	}

	// Add a measurement, get a measurement model
	pressure_message  = create_baro_measurement(to_type_timestamp(2), pressure);
	measurement_model = baronav_mp.generate_model(pressure_message, dummy_xhat_p);

	std::pair<bool, double> start_elevation =
	    elevation_provider->lookup_datum(start_pos[0], start_pos[1]);

	Vector ecef_reference_expected =
	    llh_to_ecef({start_pos[0], start_pos[1], start_elevation.second});
	Vector ecef_reference_actual = baronav_mp.get_ecef_reference();

	std::pair<bool, Vector> init_llh =
	    baronav_mp.get_initialization_buffer_llh(to_type_timestamp(1.0));

	double atol = 1e-5;
	double rtol = 1e-5;
	EXPECT_ALLCLOSE_EX(start_pos, init_llh.second, rtol, atol);
	EXPECT_ALLCLOSE_EX(ecef_reference_expected, ecef_reference_actual, rtol, atol);
	EXPECT_NEAR_EX(start_elevation.second, baronav_mp.get_altitude_reference(), rtol, atol);
}

TEST(BaronavProcessorTests, baroSigma) {
	std::vector<std::string> state_block_labels = {"baronav_motion_block"};

	std::shared_ptr<FakeMapDataSource> source = std::make_shared<FakeMapDataSource>();
	std::shared_ptr<SimpleElevationProvider> elevation_provider =
	    std::make_shared<SimpleElevationProvider>(source);

	std::shared_ptr<navtk::experimental::RandomNumberGenerator> rng =
	    std::make_shared<navtk::experimental::LocalEngineWrapper>();

	double barometer_tau                     = 1.0;
	double barometer_bias_process_sigma      = 0.0;
	double baro_elevation_measurement_sigma  = 3.0;
	double heading_tau                       = 1.0;
	double heading_bias_process_sigma        = 0.0;
	double heading_meas_sigma                = 0.0;
	double delta_pos_meas_sigma              = 0.0;
	double propagate_heading_when_stationary = false;
	size_t num_particles                     = 10;
	double initial_barometer_bias_variance   = 0.0;
	Vector initialization_variance_scale_factors{1.0, 1.0, 1.0, 1.0};
	double min_initial_position_variance     = 0.0;
	double resampling_threshold              = 1.0;
	double min_update_time                   = 0.0;
	double max_propagate_time                = 1000;
	bool calc_single_jacobian                = true;
	double delta_t_to_estimate_current_state = 1.0;
	bool enable_debug_printouts              = false;
	std::string debug_filename               = "";

	auto baronav_mp =
	    std::make_shared<BaronavMeasurementProcessor>("a",
	                                                  state_block_labels,
	                                                  elevation_provider,
	                                                  rng,
	                                                  barometer_tau,
	                                                  barometer_bias_process_sigma,
	                                                  baro_elevation_measurement_sigma,
	                                                  heading_tau,
	                                                  heading_bias_process_sigma,
	                                                  heading_meas_sigma,
	                                                  delta_pos_meas_sigma,
	                                                  propagate_heading_when_stationary,
	                                                  num_particles,
	                                                  initial_barometer_bias_variance,
	                                                  initialization_variance_scale_factors,
	                                                  min_initial_position_variance,
	                                                  resampling_threshold,
	                                                  min_update_time,
	                                                  max_propagate_time,
	                                                  calc_single_jacobian,
	                                                  delta_t_to_estimate_current_state,
	                                                  enable_debug_printouts,
	                                                  debug_filename);

	double sigma = baronav_mp->get_baro_elevation_measurement_sigma();
	EXPECT_DOUBLE_EQ(sigma, baro_elevation_measurement_sigma);
	double new_baro_elevation_measurement_sigma = 4.0;
	baronav_mp->set_baro_elevation_measurement_sigma(new_baro_elevation_measurement_sigma);
	double new_sigma = baronav_mp->get_baro_elevation_measurement_sigma();
	EXPECT_DOUBLE_EQ(new_sigma, new_baro_elevation_measurement_sigma);
}

TEST(BaronavProcessorTests, getters_SLOW) {
	// TODO: figure out why this is so slow
	std::shared_ptr<FakeMapDataSource> source = std::make_shared<FakeMapDataSource>();
	std::shared_ptr<SimpleElevationProvider> elevation_provider =
	    std::make_shared<SimpleElevationProvider>(source);
	std::shared_ptr<navtk::experimental::RandomNumberGenerator> rng =
	    std::make_shared<navtk::experimental::LocalEngineWrapper>();

	std::vector<std::string> state_block_labels = {"baronav_motion_block"};
	auto baronav_mp = BaronavMeasurementProcessor("a", state_block_labels, elevation_provider, rng);

	Vector weights = baronav_mp.get_weights();
	double atol    = 1e-12;
	double rtol    = 1e-12;

	EXPECT_ALLCLOSE_EX(weights, 0.001 * navtk::ones(1000), rtol, atol);

	int num_states                = 4;
	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};
	double pressure        = 1344;
	auto pressure_message  = create_baro_measurement(to_type_timestamp(1), pressure);
	auto measurement_model = baronav_mp.generate_model(pressure_message, dummy_xhat_p);

	Vector start_pos{-0.01, -1.37, 3.0};
	for (double ii = 0.0; ii < 2.0; ++ii) {
		auto heading_aux_data = create_heading_measurement(to_type_timestamp(ii), 2 * ii, 1.0);
		auto delta_pos_aux =
		    create_delta_pos_1d_measurement(to_type_timestamp(ii), 1.0, 2 * ii, 1.0);
		auto position_aux = create_pos_measurement(to_type_timestamp(ii), start_pos, eye(3));
		baronav_mp.receive_aux_data({heading_aux_data, delta_pos_aux, position_aux});
	}

	pressure_message  = create_baro_measurement(to_type_timestamp(1), pressure);
	measurement_model = baronav_mp.generate_model(pressure_message, dummy_xhat_p);

	// TODO: check that the particles make sense, statistically, rather than just checking the size
	Matrix particles = baronav_mp.get_particles();
	EXPECT_EQ(Size(4), num_rows(particles));
	EXPECT_EQ(Size(1000), num_cols(particles));

	auto filter_time = baronav_mp.get_filter_time();
	EXPECT_EQ(filter_time, 1);
}


TEST(BaronavProcessorTests, getEstimateCovariance_SLOW) {

	std::shared_ptr<FakeMapDataSource> source = std::make_shared<FakeMapDataSource>();
	std::shared_ptr<SimpleElevationProvider> elevation_provider =
	    std::make_shared<SimpleElevationProvider>(source);

	std::vector<std::string> state_block_labels = {"a"};
	std::shared_ptr<navtk::experimental::RandomNumberGenerator> rng =
	    std::make_shared<navtk::experimental::LocalEngineWrapper>();

	auto baronav_mp = std::make_shared<BaronavMeasurementProcessor>(
	    "a", state_block_labels, elevation_provider, rng);


	int num_states                = 4;
	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};
	double pressure        = 1344;
	auto pressure_message  = create_baro_measurement(to_type_timestamp(1), pressure);
	auto measurement_model = baronav_mp->generate_model(pressure_message, dummy_xhat_p);

	Vector start_pos{-0.01, -1.37, 3.0};
	for (double ii = 0.0; ii < 2.0; ++ii) {
		auto heading_aux_data = create_heading_measurement(to_type_timestamp(ii), 2 * ii, 1.0);
		auto delta_pos_aux =
		    create_delta_pos_1d_measurement(to_type_timestamp(ii), 1.0, 2 * ii, 1.0);
		auto position_aux = create_pos_measurement(to_type_timestamp(ii), start_pos, eye(3));
		baronav_mp->receive_aux_data({heading_aux_data, delta_pos_aux, position_aux});
	}

	pressure_message  = create_baro_measurement(to_type_timestamp(1), pressure);
	measurement_model = baronav_mp->generate_model(pressure_message, dummy_xhat_p);

	Matrix actual_motion_cov = baronav_mp->get_motion_block_cov();
	Matrix actual_fogm_cov   = baronav_mp->get_baro_bias_block_cov();
	Vector actual_motion_est = baronav_mp->get_motion_block_estimate();
	Vector actual_fogm_est   = baronav_mp->get_baro_bias_block_estimate();
	// TODO: Actually test these values in some way

	EXPECT_EQ(Size(3), num_rows(baronav_mp->get_motion_block_cov()));
	EXPECT_EQ(Size(3), num_cols(baronav_mp->get_motion_block_cov()));
	EXPECT_EQ(Size(1), num_rows(baronav_mp->get_baro_bias_block_cov()));
	EXPECT_EQ(Size(1), num_cols(baronav_mp->get_baro_bias_block_cov()));
	EXPECT_EQ(Size(3), num_rows(baronav_mp->get_motion_block_estimate()));
	EXPECT_EQ(Size(1), num_rows(baronav_mp->get_baro_bias_block_estimate()));
}

TEST(BaronavProcessorTests, getInternalFilterSolution_SLOW) {
	// TODO: figure out why this is so slow

	std::shared_ptr<FakeMapDataSource> source = std::make_shared<FakeMapDataSource>();
	std::shared_ptr<SimpleElevationProvider> elevation_provider =
	    std::make_shared<SimpleElevationProvider>(source);

	std::vector<std::string> state_block_labels = {"a"};
	std::shared_ptr<navtk::experimental::RandomNumberGenerator> rng =
	    std::make_shared<navtk::experimental::LocalEngineWrapper>();

	auto baronav_mp = std::make_shared<BaronavMeasurementProcessor>(
	    "a", state_block_labels, elevation_provider, rng);

	int num_states                = 4;
	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};
	double pressure        = 1344;
	auto pressure_message  = create_baro_measurement(to_type_timestamp(1), pressure);
	auto measurement_model = baronav_mp->generate_model(pressure_message, dummy_xhat_p);

	// Measurement processor not initialized, filter solution should be null.
	auto null_filter_solution = baronav_mp->generate_internal_filter_model(pressure_message);
	EXPECT_EQ(null_filter_solution, nullptr);

	pressure_message  = create_baro_measurement(to_type_timestamp(1), pressure);
	measurement_model = baronav_mp->generate_model(pressure_message, dummy_xhat_p);

	Vector start_pos{-0.01, -1.37, 3.0};
	for (double ii = 0.0; ii < 2.0; ++ii) {
		auto heading_aux_data = create_heading_measurement(to_type_timestamp(ii), 2 * ii, 1.0);
		auto delta_pos_aux =
		    create_delta_pos_1d_measurement(to_type_timestamp(ii), 1.0, 2 * ii, 1.0);
		auto position_aux = create_pos_measurement(to_type_timestamp(ii), start_pos, eye(3));
		baronav_mp->receive_aux_data({heading_aux_data, delta_pos_aux, position_aux});
	}

	pressure_message  = create_baro_measurement(to_type_timestamp(1), pressure);
	measurement_model = baronav_mp->generate_model(pressure_message, dummy_xhat_p);

	// Measurement processor is initialized, filter solution returns a StandardMeasurementModel.
	auto filter_solution = baronav_mp->generate_internal_filter_model(pressure_message);
	EXPECT_EQ(num_rows(filter_solution->z), Size(3));
	EXPECT_EQ(num_rows(filter_solution->H), Size(3));
	EXPECT_EQ(num_cols(filter_solution->H), Size(3));
	EXPECT_EQ(num_rows(filter_solution->R), Size(3));
	EXPECT_EQ(num_cols(filter_solution->R), Size(3));
}

TEST(BaronavProcessorTests, convertPressureAltitude) {

	std::shared_ptr<FakeMapDataSource> source = std::make_shared<FakeMapDataSource>();
	std::shared_ptr<SimpleElevationProvider> elevation_provider =
	    std::make_shared<SimpleElevationProvider>(source);

	std::vector<std::string> state_block_labels = {"a"};
	std::shared_ptr<navtk::experimental::RandomNumberGenerator> rng =
	    std::make_shared<navtk::experimental::LocalEngineWrapper>();

	auto baronav_mp = std::make_shared<BaronavMeasurementProcessor>(
	    "a", state_block_labels, elevation_provider, rng);

	double pressure     = 1344;
	double deg_k        = 288.15;
	double expected_alt = -(deg_k / 0.0065) *
	                      (pow(pressure / 101325.0, 8314.32 * 0.0065 / (9.80665 * 28.9644)) - 1.0);

	double actual_alt = baronav_mp->convert_pressure_to_altitude(pressure);
	EXPECT_DOUBLE_EQ(expected_alt, actual_alt);
}
