#include <memory>

#include <gtest/gtest.h>
#include <scalar_assert.hpp>
#include <tensor_assert.hpp>
#include <xtensor/core/xmath.hpp>
#include <xtensor/views/xview.hpp>

#include <navtk/experimental/random.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/experimental/stateblocks/SampledFogmBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::ones;
using navtk::Vector;
using navtk::zeros;
using navtk::experimental::LocalEngineWrapper;
using navtk::filtering::NULL_GEN_XHAT_AND_P_FUNCTION;
using navtk::filtering::experimental::SampledFogmBlock;


TEST(SampledFogmBlockTests, testMultiState) {
	double tau   = 3.0;
	double sigma = 4.0;
	double dt    = 0.5;
	auto rng     = std::make_shared<LocalEngineWrapper>();
	rng->seed(1234);
	auto block = SampledFogmBlock("block", tau, sigma, 1, rng);
	Vector x   = {1.0};
	auto dyn   = block.generate_dynamics(
        NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));

	auto expected_g_of_x = Vector{-0.7129604714470631};
	auto actual_g_of_x   = dyn.g(x);

	auto absolute_tolerance = 0.0;
	auto relative_tolerance = 1e-6;
	EXPECT_ALLCLOSE_EX(expected_g_of_x, actual_g_of_x, relative_tolerance, absolute_tolerance);

	x     = ones(3);
	block = SampledFogmBlock("block", tau, sigma, 3, rng);
	dyn   = block.generate_dynamics(
        NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));
	expected_g_of_x = Vector{0.23909401865345348, 1.2838859230072144, 1.3712177546096282};
	actual_g_of_x   = dyn.g(x);

	EXPECT_ALLCLOSE_EX(expected_g_of_x, actual_g_of_x, relative_tolerance, absolute_tolerance);
}

TEST(SampledFogmBlockTests, testDifferentFOGMParameters) {
	Vector tau{3.0, 4.0, 5.0, 6.0};
	Vector sigma{4.0, 5.0, 6.0, 7.0};
	double dt = 0.3;
	auto rng  = std::make_shared<LocalEngineWrapper>();
	rng->seed(1234);
	Vector x   = ones(4);
	auto block = SampledFogmBlock("block", tau, sigma, 4, rng);
	auto dyn   = block.generate_dynamics(
        NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));
	auto expected_g_of_x =
	    Vector{-0.342198029831123, 0.395529128802927, 1.3561587824085248, 1.483288057381439};
	auto actual_g_of_x      = dyn.g(x);
	auto absolute_tolerance = 0.0;
	auto relative_tolerance = 1e-6;
	EXPECT_ALLCLOSE_EX(expected_g_of_x, actual_g_of_x, relative_tolerance, absolute_tolerance);
}

class TestableSampledFOGM : public SampledFogmBlock {
public:
	TestableSampledFOGM(const std::string& label,
	                    Vector time_constants,
	                    Vector state_sigmas,
	                    Vector::shape_type::value_type num_states,
	                    std::shared_ptr<navtk::experimental::RandomNumberGenerator> rng)
	    : SampledFogmBlock(label, time_constants, state_sigmas, num_states, rng) {}

	TestableSampledFOGM(const TestableSampledFOGM& block) : SampledFogmBlock(block) {}

	navtk::not_null<std::shared_ptr<StateBlock<>>> clone() {
		return std::make_shared<TestableSampledFOGM>(*this);
	}
	using SampledFogmBlock::state_sigmas;
	using SampledFogmBlock::time_constants;
};

TEST(SampledFogmBlockTests, test_clone_FOGM) {
	auto rng = std::make_shared<LocalEngineWrapper>();
	rng->seed(1234);
	auto block           = TestableSampledFOGM("block", Vector{2.0}, Vector{2.0}, 1, rng);
	auto block_copy_cast = std::dynamic_pointer_cast<TestableSampledFOGM>(block.clone());
	auto& block_copy     = *block_copy_cast;

	ASSERT_EQ(block.get_label(), block_copy.get_label());

	ASSERT_EQ(block.get_num_states(), block_copy.get_num_states());

	ASSERT_EQ(block.state_sigmas, block_copy.state_sigmas);
	block_copy.state_sigmas += 1;
	ASSERT_NE(block.state_sigmas, block_copy.state_sigmas);

	ASSERT_EQ(block.time_constants, block_copy.time_constants);
	block_copy.time_constants += 1;
	ASSERT_NE(block.time_constants, block_copy.time_constants);
}

// TODO: Re-instate test (and re-evaluate tolerances) once rng issues are resolved (PNTOS-613)
TEST(SampledFogmBlockTests, DISABLED_testSteadyState_SLOW) {
	size_t states     = 4;
	size_t iterations = 10000;
	Vector tau{3.0, 4.0, 5.0, 6.0};
	Vector sigma{0.01, 0.1, 1.0, 10.0};
	double dt = 0.3;
	auto rng  = std::make_shared<LocalEngineWrapper>();
	rng->seed(7363609467218478);
	auto block   = SampledFogmBlock("block", tau, sigma, states, rng);
	auto results = zeros(states, iterations);
	auto xhat    = zeros(states);
	// Since dt doesn't change, we can call generate_dynamics just once though we're using g for
	// each iteration.
	auto dyn = block.generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));
	for (decltype(iterations) it = 0; it < iterations; ++it) {
		xhat                             = dyn.g(xhat);
		xt::view(results, xt::all(), it) = xhat;
	}
	Vector sample_mean  = xt::eval(xt::mean(results, {1}));
	Vector sample_sigma = xt::eval(xt::stddev(results, {1}));

	// TODO: Re-evaluate the tolerances once PNTOS-613 is resolved
	// Test sample mean relative to sigma
	for (int ii = 0; ii < 4; ++ii) {
		double relative_tolerance = 0;
		double absolute_tolerance = 3.0 * sigma(ii) / sqrt(iterations);
		EXPECT_NEAR_EX(sample_mean[ii], 0, relative_tolerance, absolute_tolerance);
	}
	auto relative_tolerance = 1.5e-2;
	auto absolute_tolerance = 1e-2;
	EXPECT_ALLCLOSE_EX(sample_sigma, sigma, relative_tolerance, absolute_tolerance);
}

// TODO: Re-instate decay test and re-evaluate tolerances once rng issues are resolved (PNTOS-613)
TEST(SampledFogmBlockTests, DISABLED_testDecay_SLOW) {
	auto dt      = 1.0;
	auto tau     = 100;
	auto sigma   = 5.0;
	auto samples = 10000;
	auto rng     = std::make_shared<LocalEngineWrapper>();
	rng->seed(176583378221306121);
	auto block      = SampledFogmBlock("block", tau, sigma, 1, rng);
	auto iterations = 5 * (1 / dt) * tau;
	auto initial    = zeros(samples);
	auto results    = zeros(samples);
	// We can just call generate_dynamics once since it doesn't change based on x_hat, and the dt
	// will be the same for all the tests
	auto dyn = block.generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));
	for (int sample = 0; sample < samples; ++sample) {
		Vector xhat     = {3 * sigma * rng->rand_n()};
		initial(sample) = xhat(0);
		for (int it = 0; it < iterations; ++it) {
			xhat = dyn.g(xhat);
		}
		results(sample) = xhat(0);
	}
	Vector initial_sample_mean = {xt::mean(initial, {0})(0)};
	Vector sample_sigma        = {xt::stddev(results, {0})(0)};
	Vector sample_mean         = {xt::mean(results, {0})(0)};
	ASSERT_ALLCLOSE_EX(Vector{sigma}, sample_sigma, 0.1, 1e-2);
	ASSERT_ALLCLOSE_EX(zeros(1), initial_sample_mean, 0, 0.1);
	ASSERT_ALLCLOSE_EX(zeros(1), sample_mean, 0, 0.1);
}
