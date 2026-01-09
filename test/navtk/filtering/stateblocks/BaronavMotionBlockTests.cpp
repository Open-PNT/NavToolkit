#include <memory>

#include <gtest/gtest.h>
#include <math.h>
#include <error_mode_assert.hpp>
#include <scalar_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/errors.hpp>
#include <navtk/experimental/random.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/experimental/stateblocks/BaronavMotionBlock.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::expm;
using navtk::eye;
using navtk::Matrix;
using navtk::Vector;
using navtk::zeros;
using navtk::experimental::LocalEngineWrapper;
using navtk::experimental::RandomNumberGenerator;
using navtk::filtering::DiscretizationStrategy;
using navtk::filtering::NULL_GEN_XHAT_AND_P_FUNCTION;
using navtk::filtering::experimental::BaronavMotionBlock;
using xt::all;
using xt::view;

TEST(BaronavMotionBlockTests, ZeroInput) {

	// Tests that with initial position at 0, 0 values in the aux data and 0 noise,
	// the estimate is still 0.

	double heading_measurement_sigma           = 0.0;
	double heading_bias_sigma                  = 0.0;
	double heading_tau                         = 1.0;
	double delta_position_sigma_per_sec        = 0.0;
	bool propagate_heading_when_stationary     = false;
	std::shared_ptr<RandomNumberGenerator> rng = std::make_shared<LocalEngineWrapper>();
	double dt                                  = 1.0;

	auto block = BaronavMotionBlock("a",
	                                heading_measurement_sigma,
	                                heading_tau,
	                                heading_bias_sigma,
	                                rng,
	                                delta_position_sigma_per_sec,
	                                propagate_heading_when_stationary);

	aspn_xtensor::TypeHeader header(ASPN_MEASUREMENT_DELTA_POSITION, 0, 0, 0, 0);
	aspn_xtensor::TypeTimestamp ts(int64_t(1e9));
	auto delta_p = std::make_shared<aspn_xtensor::MeasurementDeltaPosition>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_DELTA_POSITION_REFERENCE_FRAME_NED,
	    1.0,
	    0.0,
	    NAN,
	    NAN,
	    eye(1),
	    ASPN_MEASUREMENT_DELTA_POSITION_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	header.set_message_type(ASPN_MEASUREMENT_HEADING);
	auto heading = std::make_shared<aspn_xtensor::MeasurementHeading>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_HEADING_REFERENCE_TRUE_HEADING,
	    0,
	    0,
	    ASPN_MEASUREMENT_HEADING_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	block.receive_aux_data({delta_p, heading});

	// Build expected g(x).
	Vector x{0.0, 0.0, 0.0};
	Vector g_x_expected{0.0, 0.0, 0.0};
	auto dyn = block.generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));

	// Build actual g(x).
	auto g_x = dyn.g(x);

	// Check is abs(actual - expected)) <= absolute_tolerance + expected*relative_tolerance
	// So these values result in a check of the percent difference
	auto absolute_tolerance = 0.0;
	auto relative_tolerance = 1e-14;

	EXPECT_ALLCLOSE_EX(g_x_expected, g_x, relative_tolerance, absolute_tolerance);
}

TEST(BaronavMotionBlockTests, XPropagate) {

	// Tests that with initial position at 0, 0 noise, and a delta position in the x
	// direction, the estimate moves only in the x direction.

	double heading_measurement_sigma           = 0.0;
	double heading_bias_sigma                  = 0.0;
	double heading_tau                         = 1.0;
	double delta_position_sigma_per_sec        = 0.0;
	bool propagate_heading_when_stationary     = false;
	std::shared_ptr<RandomNumberGenerator> rng = std::make_shared<LocalEngineWrapper>();
	double dt                                  = 1.0;

	auto block = BaronavMotionBlock("a",
	                                heading_measurement_sigma,
	                                heading_tau,
	                                heading_bias_sigma,
	                                rng,
	                                delta_position_sigma_per_sec,
	                                propagate_heading_when_stationary);

	aspn_xtensor::TypeHeader header(ASPN_MEASUREMENT_DELTA_POSITION, 0, 0, 0, 0);
	aspn_xtensor::TypeTimestamp ts(int64_t(1e9));
	auto delta_p = std::make_shared<aspn_xtensor::MeasurementDeltaPosition>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_DELTA_POSITION_REFERENCE_FRAME_NED,
	    1.0,
	    2.0,
	    NAN,
	    NAN,
	    eye(1),
	    ASPN_MEASUREMENT_DELTA_POSITION_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	header.set_message_type(ASPN_MEASUREMENT_HEADING);
	auto heading = std::make_shared<aspn_xtensor::MeasurementHeading>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_HEADING_REFERENCE_TRUE_HEADING,
	    M_PI / 2,
	    0,
	    ASPN_MEASUREMENT_HEADING_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	block.receive_aux_data({delta_p, heading});

	auto dyn = block.generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(1.0), to_type_timestamp(1.0 + dt));

	// Build expected g(x).
	Vector g_x_expected{2.0, 0.0, 0.0};

	// Build actual g(x).
	Vector x{0.0, 0.0, 0.0};
	auto g_x = dyn.g(x);

	// Check is abs(actual - expected)) <= absolute_tolerance + expected*relative_tolerance
	auto absolute_tolerance = 1e-14;
	auto relative_tolerance = 1e-4;

	EXPECT_ALLCLOSE_EX(g_x_expected, g_x, relative_tolerance, absolute_tolerance);
}

TEST(BaronavMotionBlockTests, YPropagate) {

	// Tests that with initial position at 0, 0 noise, and a delta position in the y
	// direction, the estimate moves only in the y direction.

	double heading_measurement_sigma           = 0.0;
	double heading_bias_sigma                  = 0.0;
	double heading_tau                         = 1.0;
	double delta_position_sigma_per_sec        = 0.0;
	bool propagate_heading_when_stationary     = false;
	std::shared_ptr<RandomNumberGenerator> rng = std::make_shared<LocalEngineWrapper>();
	double dt                                  = 1.0;

	auto block = BaronavMotionBlock("a",
	                                heading_measurement_sigma,
	                                heading_tau,
	                                heading_bias_sigma,
	                                rng,
	                                delta_position_sigma_per_sec,
	                                propagate_heading_when_stationary);

	aspn_xtensor::TypeHeader header(ASPN_MEASUREMENT_DELTA_POSITION, 0, 0, 0, 0);
	aspn_xtensor::TypeTimestamp ts(int64_t(1e9));
	auto delta_p = std::make_shared<aspn_xtensor::MeasurementDeltaPosition>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_DELTA_POSITION_REFERENCE_FRAME_NED,
	    1.0,
	    2.0,
	    NAN,
	    NAN,
	    eye(1),
	    ASPN_MEASUREMENT_DELTA_POSITION_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	header.set_message_type(ASPN_MEASUREMENT_HEADING);
	auto heading = std::make_shared<aspn_xtensor::MeasurementHeading>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_HEADING_REFERENCE_TRUE_HEADING,
	    0,
	    0,
	    ASPN_MEASUREMENT_HEADING_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	block.receive_aux_data({delta_p, heading});
	auto dyn = block.generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(1.0), to_type_timestamp(1.0 + dt));

	// Build expected g(x).
	Vector g_x_expected{0.0, 2.0, 0.0};

	// Build actual g(x).
	Vector x{0.0, 0.0, 0.0};
	auto g_x = dyn.g(x);

	// Check is abs(actual - expected)) <= absolute_tolerance + expected*relative_tolerance
	auto absolute_tolerance = 1e-14;
	auto relative_tolerance = 1e-4;

	EXPECT_ALLCLOSE_EX(g_x_expected, g_x, relative_tolerance, absolute_tolerance);
}

TEST(BaronavMotionBlockTests, ZeroNoisePropagate) {

	// Tests that with initial position at 0, 0 noise, and a known propagation value,
	// the estimate matches value expected by the mathematical formula.

	double heading_measurement_sigma           = 0.0;
	double heading_bias_sigma                  = 0.0;
	double heading_tau                         = 1.0;
	double delta_position_sigma_per_sec        = 0.0;
	bool propagate_heading_when_stationary     = false;
	std::shared_ptr<RandomNumberGenerator> rng = std::make_shared<LocalEngineWrapper>();
	double dt                                  = 1.0;
	double decay                               = exp(-dt / heading_tau);

	auto block = BaronavMotionBlock("a",
	                                heading_measurement_sigma,
	                                heading_tau,
	                                heading_bias_sigma,
	                                rng,
	                                delta_position_sigma_per_sec,
	                                propagate_heading_when_stationary);

	double x_aux       = 1.0;
	double heading_aux = M_PI / 3;
	aspn_xtensor::TypeHeader header(ASPN_MEASUREMENT_DELTA_POSITION, 0, 0, 0, 0);
	aspn_xtensor::TypeTimestamp ts(int64_t(1e9));
	auto delta_p = std::make_shared<aspn_xtensor::MeasurementDeltaPosition>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_DELTA_POSITION_REFERENCE_FRAME_NED,
	    dt,
	    x_aux,
	    NAN,
	    NAN,
	    eye(1),
	    ASPN_MEASUREMENT_DELTA_POSITION_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	header.set_message_type(ASPN_MEASUREMENT_HEADING);
	auto heading = std::make_shared<aspn_xtensor::MeasurementHeading>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_HEADING_REFERENCE_TRUE_HEADING,
	    heading_aux,
	    0,
	    ASPN_MEASUREMENT_HEADING_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	block.receive_aux_data({delta_p, heading});
	auto dyn = block.generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));

	// Build expected g(x).
	double heading_bias     = M_PI / 6;
	double heading_expected = heading_bias * decay;
	double x_val            = 0.0;
	double y_val            = 0.0;
	double x_expected       = x_val + sin(heading_aux - heading_bias);
	double y_expected       = y_val + cos(heading_aux - heading_bias);
	Vector g_x_expected{x_expected, y_expected, heading_expected};

	// Build actual g(x).
	Vector x{x_val, y_val, heading_bias};
	auto g_x = dyn.g(x);

	auto absolute_tolerance = 1e-14;
	auto relative_tolerance = 1e-4;

	EXPECT_ALLCLOSE_EX(g_x_expected, g_x, relative_tolerance, absolute_tolerance);
}

TEST(BaronavMotionBlockTests, MeanNoisyPropagate_SLOW) {

	// Tests that the mean of an ensemble of propagations matches the value
	// expected by the mathematical formula.

	double heading_measurement_sigma           = 0.05;
	double heading_bias_sigma                  = 0.05;
	double heading_tau                         = 5.0;
	double delta_position_sigma_per_sec        = 0.1;
	bool propagate_heading_when_stationary     = false;
	std::shared_ptr<RandomNumberGenerator> rng = std::make_shared<LocalEngineWrapper>();
	double dt                                  = 1.0;
	double decay                               = exp(-dt / heading_tau);
	double x_aux                               = 1.0;
	double heading_aux                         = M_PI / 3;
	double heading_bias                        = M_PI / 6;
	double x_val                               = 0.0;
	double y_val                               = 0.0;

	aspn_xtensor::TypeHeader header(ASPN_MEASUREMENT_DELTA_POSITION, 0, 0, 0, 0);
	aspn_xtensor::TypeTimestamp ts(int64_t(1e9));
	auto delta_p = std::make_shared<aspn_xtensor::MeasurementDeltaPosition>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_DELTA_POSITION_REFERENCE_FRAME_NED,
	    dt,
	    x_aux,
	    NAN,
	    NAN,
	    eye(1),
	    ASPN_MEASUREMENT_DELTA_POSITION_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	header.set_message_type(ASPN_MEASUREMENT_HEADING);
	auto heading = std::make_shared<aspn_xtensor::MeasurementHeading>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_HEADING_REFERENCE_TRUE_HEADING,
	    heading_aux,
	    0,
	    ASPN_MEASUREMENT_HEADING_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});

	size_t num_samples   = 1000;
	Matrix g_x_actual_mx = zeros(num_samples, 3);
	for (size_t ii = 0; ii < num_samples; ++ii) {

		auto block = BaronavMotionBlock("a",
		                                heading_measurement_sigma,
		                                heading_tau,
		                                heading_bias_sigma,
		                                rng,
		                                delta_position_sigma_per_sec,
		                                propagate_heading_when_stationary);

		block.receive_aux_data({delta_p, heading});

		auto dyn = block.generate_dynamics(
		    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));

		// Build actual g(x).
		Vector x{x_val, y_val, heading_bias};
		auto g_x                           = dyn.g(x);
		view(g_x_actual_mx, ii, xt::all()) = g_x;
	};

	Vector g_x_actual_mean = xt::mean(g_x_actual_mx, 0);
	// Build expected g(x).
	double heading_expected = heading_bias * decay;

	double x_expected = x_val + sin(heading_aux - heading_bias);
	double y_expected = y_val + cos(heading_aux - heading_bias);
	Vector g_x_expected{x_expected, y_expected, heading_expected};
	Vector g_x_actual_std = xt::stddev(g_x_actual_mx, {0});

	// Check is abs(actual - expected)) <= absolute_tolerance + expected*relative_tolerance
	auto absolute_tolerance = 1e-2;
	auto relative_tolerance = 1e-4;

	EXPECT_ALLCLOSE_EX(g_x_expected, g_x_actual_mean, relative_tolerance, absolute_tolerance);
}

TEST(BaronavMotionBlockTests, HeadingOnlyPropagate_SLOW) {

	// Tests that the heading bias (specifically the noise) propagates even when the x aux data is
	// 0.

	double heading_measurement_sigma           = 0.05;
	double heading_bias_sigma                  = 0.05;
	double heading_tau                         = 5.0;
	double delta_position_sigma_per_sec        = 0.1;
	bool propagate_heading_when_stationary     = true;
	std::shared_ptr<RandomNumberGenerator> rng = std::make_shared<LocalEngineWrapper>();
	double dt                                  = 1.0;
	double decay                               = exp(-dt / heading_tau);
	double x_aux                               = 0.0;
	double heading_aux                         = M_PI / 3;
	double heading_bias                        = M_PI / 6;
	double x_val                               = 0.0;
	double y_val                               = 0.0;

	aspn_xtensor::TypeHeader header(ASPN_MEASUREMENT_DELTA_POSITION, 0, 0, 0, 0);
	aspn_xtensor::TypeTimestamp ts(int64_t(1e9));
	auto delta_p = std::make_shared<aspn_xtensor::MeasurementDeltaPosition>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_DELTA_POSITION_REFERENCE_FRAME_NED,
	    dt,
	    x_aux,
	    NAN,
	    NAN,
	    eye(1),
	    ASPN_MEASUREMENT_DELTA_POSITION_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	header.set_message_type(ASPN_MEASUREMENT_HEADING);
	auto heading = std::make_shared<aspn_xtensor::MeasurementHeading>(
	    header,
	    ts,
	    ASPN_MEASUREMENT_HEADING_REFERENCE_TRUE_HEADING,
	    heading_aux,
	    0,
	    ASPN_MEASUREMENT_HEADING_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});

	size_t num_samples   = 1000;
	Matrix g_x_actual_mx = zeros(num_samples, 3);
	for (size_t ii = 0; ii < num_samples; ++ii) {

		auto block = BaronavMotionBlock("a",
		                                heading_measurement_sigma,
		                                heading_tau,
		                                heading_bias_sigma,
		                                rng,
		                                delta_position_sigma_per_sec,
		                                propagate_heading_when_stationary);

		block.receive_aux_data({heading, delta_p});

		auto dyn = block.generate_dynamics(
		    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));

		// Build actual g(x).
		Vector x{x_val, y_val, heading_bias};
		auto g_x                           = dyn.g(x);
		view(g_x_actual_mx, ii, xt::all()) = g_x;

		// Check that the x-y data has not been propagated (and that no noise has been added).
		auto g_x_pos            = view(g_x, xt::range(0, 2));
		auto absolute_tolerance = 1e-14;
		auto relative_tolerance = 1e-14;
		EXPECT_ALLCLOSE_EX(g_x_pos, zeros(2), relative_tolerance, absolute_tolerance);
	};

	Vector g_x_actual_std   = xt::stddev(g_x_actual_mx, {0});
	Vector g_x_actual_mean  = xt::mean(g_x_actual_mx, 0);
	double heading_expected = heading_bias * decay;

	// Check that the standard deviation of just the heading noise is what is expected, and that the
	// mean is 0
	auto absolute_tolerance         = 1e-2;
	auto relative_tolerance         = 1e-4;
	double heading_bias_input_sigma = heading_bias_sigma * sqrt((1 - exp(-2.0 * dt / heading_tau)));
	EXPECT_NEAR_EX(
	    g_x_actual_std(2), heading_bias_input_sigma, relative_tolerance, absolute_tolerance);
	EXPECT_NEAR_EX(g_x_actual_mean(2), heading_expected, relative_tolerance, absolute_tolerance);
}

ERROR_MODE_SENSITIVE_TEST(TEST, BaronavMotionBlockTests, NoAspnBaseVector) {

	// Tests that with with no aux data, generate_dynamics throws an error.

	double heading_measurement_sigma           = 0.0;
	double heading_bias_sigma                  = 0.0;
	double heading_tau                         = 1.0;
	double delta_position_sigma_per_sec        = 0.0;
	bool propagate_heading_when_stationary     = false;
	std::shared_ptr<RandomNumberGenerator> rng = std::make_shared<LocalEngineWrapper>();
	double dt                                  = 1.0;

	auto block = BaronavMotionBlock("a",
	                                heading_measurement_sigma,
	                                heading_tau,
	                                heading_bias_sigma,
	                                rng,
	                                delta_position_sigma_per_sec,
	                                propagate_heading_when_stationary);

	{
		EXPECT_THROW(EXPECT_ERROR(block.generate_dynamics(NULL_GEN_XHAT_AND_P_FUNCTION,
		                                                  to_type_timestamp(1.0),
		                                                  to_type_timestamp(1.0 + dt)),
		                          "cannot propagate"),
		             std::runtime_error);
	}
}
