#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>
#include <xtensor/generators/xrandom.hpp>

#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/experimental/fusion/strategies/RbpfStrategy.hpp>
#include <navtk/filtering/fusion/strategies/EkfStrategy.hpp>
#include <xtensor/core/xmath.hpp>

#include <navtk/experimental/random.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/tensors.hpp>

using namespace navtk::filtering;
using namespace navtk::filtering::experimental;
using navtk::dot;
using navtk::eye;
using navtk::Matrix;
using navtk::num_cols;
using navtk::num_rows;
using navtk::ones;
using navtk::to_matrix;
using navtk::to_vec;
using navtk::Vector;
using navtk::zeros;
using navtk::experimental::get_global_rng;
using navtk::experimental::rand_n;
using navtk::experimental::set_global_rng;
using xt::sum;
using xt::transpose;

struct RbpfStrategyTest : public ::testing::Test {
	Vector x0, x1;
	Matrix P0, P1, P2, Q0, F0, H0, initial_p;
	size_t small_particle_count, medium_particle_count;
	size_t propagate_iterations;  // number of propagations before covariance calculation
	double estimate_threshold, covariance_threshold;
	uint64_t random_seed;

	RbpfStrategyTest()
	    : x0({1, 2, 0.3, -0.4}),
	      x1({1, -2, 4}),
	      P0({{4e-3, 1e-5, 2e-4, 3e-5},
	          {1e-5, 4e-3, 1e-5, 2e-4},
	          {2e-4, 1e-5, 5e-3, 5e-6},
	          {3e-5, 2e-4, 5e-6, 5e-3}}),
	      P1({{2.4, .4, .1}, {0.4, 1.2, 0.2}, {0.1, 0.2, 0.8}}),
	      P2({{4e-2, 0, 0, 0}, {0, 7e-2, 0, 0}, {0, 0, 31.69e-4, 0}, {0, 0, 0, 20e-3}}),

	      Q0({{1.1e-7, 1e-9, 2.5e-9, 1e-9},
	          {1e-9, 1.1e-7, 1e-9, 2.5e-9},
	          {2.5e-9, 1e-9, 1.1e-8, 1e-9},
	          {1e-9, 2.5e-9, 1e-9, 1.1e-8}}),
	      F0({{0.0001, 0.00005, 0.0005, 0.},
	          {0, 0.0001, 0.00005, 0.0005},
	          {0, 0, 0.0001, 0},
	          {0, 0, 0, 0.0001}}),
	      H0({{1, 0, 1, 0}, {0, 1, 0, 1}}),
	      initial_p(std::move(dot(dot(F0, P0), transpose(F0)) + Q0)),
	      small_particle_count(50),
	      medium_particle_count(125),
	      propagate_iterations(1),
	      estimate_threshold(1e-8 * propagate_iterations),
	      covariance_threshold(5e-8 * propagate_iterations),
	      random_seed(9159326332917817176) {}
	void SetUp() override { navtk::experimental::s_rand(random_seed); }
};

TEST_F(RbpfStrategyTest, RbpfStrategy_set_jitter_states_SLOW) {
	auto filter =
	    RbpfStrategy(small_particle_count, 0.8, false, residual_resample_with_replacement);
	filter.on_fusion_engine_state_block_added(num_rows(x0));
	filter.set_estimate_slice(x0);
	filter.set_covariance_slice(P2);

	// Mark first two states without jitter
	std::vector<double> expect = {0., 0., 0., 0.};
	filter.set_marked_states({0, 1});

	ASSERT_EQ(filter.get_jitter_scaling(), expect);

	// Mark all states as particle states with unique jitter scaling
	expect = {0.075, 0.025, 0.0125, 0.};
	filter.set_marked_states({0, 1, 2, 3}, expect);

	ASSERT_EQ(filter.get_jitter_scaling(), expect);

	// Mark two states as particle states with same jitter scaling
	filter.set_marked_states({0, 1}, {0.075});

	expect = {0.075, 0.075, 0., 0.};
	ASSERT_EQ(filter.get_jitter_scaling(), expect);

	// change marked state jitter values
	expect = {0.1, 0.1, 0., 0.};
	filter.set_jitter_scaling({0.1});

	ASSERT_EQ(filter.get_jitter_scaling(), expect);

	expect = {0.2, 0.3, 0., 0.};
	filter.set_jitter_scaling({0.2, 0.3});

	ASSERT_EQ(filter.get_jitter_scaling(), expect);

	// Too many marked or jitter states
	EXPECT_HONORS_MODE_EX(
	    filter.set_marked_states({0, 1, 2, 3}, {0.075, 0.025, 0.0125, 0., 0.}),
	    "Exception Occurred: There must either be only one jitter value, or as many jitter values "
	    "as marked states.",
	    std::invalid_argument);

	// Too few jitter states
	EXPECT_HONORS_MODE_EX(filter.set_marked_states({0, 1, 2, 3}, {}),
	                      "Exception Occurred: There must either be only one jitter value, or as "
	                      "many jitter values as marked states.",
	                      std::invalid_argument);

	// Empty marked states
	EXPECT_HONORS_MODE_EX(filter.set_marked_states({}, {0.75}),
	                      "Exception Occurred: There must either be only one jitter value, or as "
	                      "many jitter values as marked states.",
	                      std::invalid_argument);

	// Empty states
	EXPECT_HONORS_MODE_EX(filter.set_marked_states({}, {}),
	                      "Exception Occurred: Must have at least one marked state.",
	                      std::invalid_argument);
}

TEST_F(RbpfStrategyTest, RbpfStrategy_jitter_performance_SLOW) {
	Vector x{0.};
	Matrix p{{0.5}};

	auto h = [&](Vector it) -> Vector { return 2 * it(0) + zeros(1); };
	Matrix H{{2}};
	Matrix noise{{0.001}};
	Vector meas{3.4};

	{
		auto filter =
		    RbpfStrategy(medium_particle_count, 0.9, false, residual_resample_with_replacement);
		filter.on_fusion_engine_state_block_added(1);
		filter.set_estimate_slice(x);
		filter.set_covariance_slice(p);

		// test resampling without jitter
		filter.set_marked_states({0});

		filter.update(StandardMeasurementModel(meas, h, H, noise));

		Matrix cov_no_jitter = filter.get_covariance();
		ASSERT_ALLCLOSE_EX(zeros(1, 1), cov_no_jitter, 1e-3, 1e-3);
	}

	{
		auto filter =
		    RbpfStrategy(medium_particle_count, 0.9, false, residual_resample_with_replacement);
		filter.on_fusion_engine_state_block_added(1);
		filter.set_estimate_slice(x);
		filter.set_covariance_slice(p);

		// test resampling with jitter
		filter.set_marked_states({0}, {1.});

		filter.update(StandardMeasurementModel(meas, h, H, noise));

		Matrix cov_with_jitter = filter.get_covariance();

		ASSERT_ALLCLOSE_EX(p, cov_with_jitter, 1e-1, 1e-1);
	}
}

TEST_F(RbpfStrategyTest, RbpfStrategy_set_marked_states) {
	auto strategy = RbpfStrategy(small_particle_count);
	strategy.on_fusion_engine_state_block_added(num_rows(x0));
	strategy.set_estimate_slice(x0);
	strategy.set_covariance_slice(P2);

	// Too many marked states
	EXPECT_THROW(strategy.set_marked_states({1, 2, 3, 4}), std::exception);

	// Correct number of states
	strategy.set_marked_states({0, 1});

	std::vector<bool> expect = {true, true, false, false};
	auto output              = strategy.get_particle_state_marks();
	ASSERT_EQ(output, expect);
}

TEST_F(RbpfStrategyTest, RbpfStrategy_duplicate_mark_particle_states) {
	auto strategy = RbpfStrategy(small_particle_count);

	// Adding first stateblock
	strategy.on_fusion_engine_state_block_added(4);
	strategy.set_estimate_slice(x0);
	strategy.set_covariance_slice(eye(4));

	// Mark the states as particles - include duplicate
	strategy.set_marked_states({0, 1, 1, 3});

	std::vector<bool> expect = {true, true, false, true};
	auto output              = strategy.get_particle_state_marks();
	ASSERT_EQ(output, expect);
}

TEST_F(RbpfStrategyTest, PfStrategy_reset_particle_states_SLOW) {
	auto strategy = RbpfStrategy(small_particle_count);

	// Adding first stateblock
	strategy.on_fusion_engine_state_block_added(4);
	strategy.set_estimate_slice(x0);
	strategy.set_covariance_slice(initial_p);

	// Mark all the states as particles
	strategy.set_marked_states({0, 1, 2, 3});

	// Verify particles have correct distribution
	ASSERT_ALLCLOSE_EX(x0, strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(
	    initial_p, strategy.get_covariance(), covariance_threshold, covariance_threshold);

	// Reset particle states to new estimate
	strategy.set_estimate_slice(zeros(4));

	// Verify new state has covariance
	ASSERT_ALLCLOSE_EX(zeros(4), strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(
	    initial_p, strategy.get_covariance(), covariance_threshold, covariance_threshold);

	// Set new state to new covariance
	strategy.set_covariance_slice(2 * initial_p);

	// Verify new state has new covariance
	ASSERT_ALLCLOSE_EX(zeros(4), strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(
	    2 * initial_p, strategy.get_covariance(), covariance_threshold, covariance_threshold);
}

TEST_F(RbpfStrategyTest, RbpfStrategy_reset_particle_states_SLOW) {
	auto strategy = RbpfStrategy(small_particle_count);

	// Adding first stateblock
	strategy.on_fusion_engine_state_block_added(4);
	strategy.set_estimate_slice(x0);
	strategy.set_covariance_slice(initial_p);

	// Mark two of the states as particles
	strategy.set_marked_states({0, 1});

	// Verify particles have correct distribution
	ASSERT_ALLCLOSE_EX(x0, strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(
	    initial_p, strategy.get_covariance(), covariance_threshold, covariance_threshold);

	// Reset particle states to new estimate
	strategy.set_estimate_slice(zeros(4));

	// Verify new state has covariance
	ASSERT_ALLCLOSE_EX(zeros(4), strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(
	    initial_p, strategy.get_covariance(), covariance_threshold, covariance_threshold);

	// Set new state to new covariance
	strategy.set_covariance_slice(2 * initial_p);

	// Verify new state has new covariance
	ASSERT_ALLCLOSE_EX(zeros(4), strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(
	    2 * initial_p, strategy.get_covariance(), covariance_threshold, covariance_threshold);
}
TEST_F(RbpfStrategyTest, PF_update_recovery_SLOW) {
	auto g = [&](Vector x) -> Vector { return to_vec(dot(F0, x)); };

	auto h = [&](Vector x) -> Vector { return to_vec(dot(H0, to_matrix(x, 1))); };

	Matrix R = 2e-3 * eye(2);

	auto strategy = RbpfStrategy(medium_particle_count, 0.75, false, systematic_resampling);

	// Adding first stateblock
	strategy.on_fusion_engine_state_block_added(num_rows(F0));
	strategy.set_estimate_slice(x0);

	strategy.set_covariance_slice(initial_p);

	// Mark all states as particles
	strategy.set_marked_states({0, 1, 2, 3});

	strategy.propagate(StandardDynamicsModel(g, F0, Q0));

	Vector measurement{1e9, 1e9};

	Vector prior_state = strategy.get_estimate();
	Matrix prior_cov   = strategy.get_covariance();

	strategy.update(StandardMeasurementModel(measurement, h, H0, R));

	ASSERT_ALLCLOSE_EX(
	    prior_state, strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(
	    prior_cov, strategy.get_covariance(), covariance_threshold, covariance_threshold);
}

TEST_F(RbpfStrategyTest, RBPF_update_recovery_SLOW) {
	auto g = [&](Vector x) -> Vector { return to_vec(dot(F0, x)); };

	auto h = [&](Vector x) -> Vector { return to_vec(dot(H0, to_matrix(x, 1))); };

	Matrix R = 2e-3 * eye(2);

	auto strategy = RbpfStrategy(medium_particle_count, 0.75, false, systematic_resampling);

	// Adding first stateblock
	strategy.on_fusion_engine_state_block_added(num_rows(F0));
	strategy.set_estimate_slice(x0);

	strategy.set_covariance_slice(initial_p);

	// Mark the first two states as particles
	strategy.set_marked_states({0, 1});

	strategy.propagate(StandardDynamicsModel(g, F0, Q0));

	Vector measurement{1e9, 1e9};

	Vector prior_state = strategy.get_estimate();
	Matrix prior_cov   = strategy.get_covariance();

	strategy.update(StandardMeasurementModel(measurement, h, H0, R));

	ASSERT_ALLCLOSE_EX(
	    prior_state, strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(
	    prior_cov, strategy.get_covariance(), covariance_threshold, covariance_threshold);
}

TEST_F(RbpfStrategyTest, RBPF_propagate_without_mark) {

	auto strategy = RbpfStrategy(small_particle_count);

	// Adding first stateblock
	strategy.on_fusion_engine_state_block_added(4);
	strategy.set_estimate_slice(x0);
	strategy.set_covariance_slice(eye(4));

	// Try to propagate without marking states one way or the other
	strategy.propagate(
	    StandardDynamicsModel([](Vector x) -> Vector { return x * 2; }, 2 * eye(4, 4), eye(4, 4)));

	ASSERT_ALLCLOSE_EX(x0 * 2, strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(
	    eye(4) * 5, strategy.get_covariance(), covariance_threshold, covariance_threshold);
}

TEST_F(RbpfStrategyTest, RbpfStrategy_propagate_after_setting_estimate_and_covariance) {

	auto strategy                 = RbpfStrategy(small_particle_count);
	strategy.calc_single_jacobian = false;

	// Adding first stateblock
	strategy.on_fusion_engine_state_block_added(4);
	strategy.set_estimate_slice(x0);
	strategy.set_covariance_slice(zeros(4, 4));

	// Try to propagate
	strategy.propagate(
	    StandardDynamicsModel([](Vector x) -> Vector { return x + 2; }, eye(4, 4), eye(4, 4)));

	ASSERT_ALLCLOSE_EX(x0 + 2, strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(
	    eye(4), strategy.get_covariance(), covariance_threshold, covariance_threshold);
}
TEST_F(RbpfStrategyTest, RBPF_random_synthesis_SLOW) {

	auto strategy = RbpfStrategy(small_particle_count, 0.2, true);
	strategy.on_fusion_engine_state_block_added(num_rows(x1));
	strategy.set_estimate_slice(x1);
	strategy.set_covariance_slice(P1);

	strategy.set_marked_states({0, 1, 2});

	ASSERT_ALLCLOSE_EX(x1, strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(P1, strategy.get_covariance(), covariance_threshold, covariance_threshold);

	strategy = RbpfStrategy(small_particle_count, 0.2, false);
	strategy.on_fusion_engine_state_block_added(num_rows(x1));
	strategy.set_estimate_slice(x1);
	strategy.set_covariance_slice(P1);

	strategy.set_marked_states({0, 1, 2});

	ASSERT_ALLCLOSE_EX(x1, strategy.get_estimate(), estimate_threshold, estimate_threshold);
	ASSERT_ALLCLOSE_EX(P1, strategy.get_covariance(), covariance_threshold, covariance_threshold);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F,
                          RbpfStrategyTest,
                          RBPF_reset_particle_count_target_after_init_done) {
	auto strategy = RbpfStrategy(test.small_particle_count, 0.2, false);
	strategy.on_fusion_engine_state_block_added(num_rows(test.x1));
	strategy.set_estimate_slice(test.x1);
	strategy.set_covariance_slice(test.P1);

	strategy.set_marked_states({0, 1, 2});

	EXPECT_HONORS_MODE_EX(strategy.set_particle_count_target(2 * test.small_particle_count),
	                      "Exception Occurred: Particle count changed after initialization.",
	                      std::runtime_error);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, RbpfStrategyTest, RBPF_zero_particles) {
	EXPECT_HONORS_MODE_EX(RbpfStrategy(0, 0.2, false),
	                      "Number of particles must be greater than zero",
	                      std::invalid_argument);
}

TEST_F(RbpfStrategyTest, RBPF_vs_EKF_Example_SLOW) {
	auto prev_rng = get_global_rng();
	set_global_rng(std::make_shared<navtk::experimental::LocalEngineWrapper>());

	navtk::experimental::s_rand(9159326332917817176);

	size_t N          = 40;  // measurement evolutions
	size_t num_states = 2;
	size_t num_obs    = 1;

	Matrix H{{0., 0.}};
	Matrix h_ekf{{0., 0.}};
	Matrix R{{.001}};  // observation noise
	Matrix Phi{{1, 0}, {0, -1.05}};
	Matrix Qd{{0.005, 0}, {0, .01}};  // state noise
	Matrix p_init{{1e-2, 0}, {0, 1e-2}};
	Vector x_init{1., 0.53};

	auto g = [&](Vector it) -> Vector {  // propagation
		Vector out = zeros(2);
		out(0)     = it(0) - it(1) * cos(it(1));
		out(1)     = -1.05 * it(1);
		return out;
	};

	auto h = [&](Vector it) -> Vector {  // observation
		Vector out = zeros(1);
		out(0)     = it(0) * cos(it(1));
		return out;
	};

	Matrix state_truth = zeros(num_states, N);
	Matrix observation = zeros(num_obs, N);

	Matrix observation_noise = rand_n(num_obs, N);

	Vector x_true = x_init;
	for (size_t i = 0; i < num_states; i++) {
		x_true(i) += std::sqrt(p_init(i, i)) * rand_n();
	}

	// run the experiment
	for (size_t ii = 0; ii < N; ii++) {
		x_true = g(x_true);

		view(state_truth, xt::all(), ii) = x_true;

		Vector obs = h(x_true);
		for (size_t jj = 0; jj < num_obs; jj++) {
			obs(jj) += std::sqrt(R(jj, jj)) * observation_noise(jj, ii);
		}
		view(observation, xt::all(), ii) = obs;
	}
	auto strategy =
	    RbpfStrategy(medium_particle_count, 0.5, false, residual_resample_with_replacement);
	auto ekf = EkfStrategy();

	strategy.on_fusion_engine_state_block_added(num_rows(x_init));
	ekf.on_fusion_engine_state_block_added(num_rows(x_init));

	strategy.set_marked_states({0}, {0.175});

	strategy.set_estimate_slice(x_init);
	strategy.set_covariance_slice(p_init);
	ekf.set_estimate_slice(x_init);
	ekf.set_covariance_slice(p_init);

	Vector rmse     = zeros(num_states);
	Vector rmse_ekf = zeros(num_states);
	for (size_t ii = 0; ii < N; ii++) {

		strategy.propagate(StandardDynamicsModel(g, Phi, Qd));
		ekf.propagate(StandardDynamicsModel(g, Phi, Qd));

		Vector measurement = view(observation, xt::all(), ii);

		Vector state = strategy.get_estimate();
		H(0, 0)      = state(0) * cos(state(1)) - sin(state(1));

		state       = ekf.get_estimate();
		h_ekf(0, 0) = state(0) * cos(state(1)) - sin(state(1));

		strategy.update(StandardMeasurementModel(measurement, h, H, R));
		ekf.update(StandardMeasurementModel(measurement, h, h_ekf, R));

		Vector est     = strategy.get_estimate();
		Vector ekf_est = ekf.get_estimate();

		Vector err     = est - view(state_truth, xt::all(), ii);
		Vector err_ekf = ekf_est - view(state_truth, xt::all(), ii);

		Matrix c     = navtk::calc_cov(strategy.get_state_particles());
		Matrix ekf_c = strategy.get_covariance();

		for (size_t jj = 0; jj < num_states; jj++) {
			rmse(jj) += pow(est(jj) - state_truth(jj, ii), 2);
			rmse_ekf(jj) += pow(ekf_est(jj) - state_truth(jj, ii), 2);
		}
	}
	double norm_rmse     = std::sqrt(sum(rmse / static_cast<double>(N))[0]);
	double norm_rmse_ekf = std::sqrt(sum(rmse_ekf / static_cast<double>(N))[0]);
	EXPECT_GT(norm_rmse_ekf, 2. * norm_rmse);

	set_global_rng(prev_rng);
}
