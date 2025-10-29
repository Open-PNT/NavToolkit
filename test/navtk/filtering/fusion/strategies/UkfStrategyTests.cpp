#include <cmath>

#include <gtest/gtest.h>
#include <tensor_assert.hpp>
#include <xtensor/misc/xmanipulation.hpp>

#include <navtk/filtering/fusion/strategies/UkfStrategy.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/tensors.hpp>

using navtk::Matrix;
using navtk::Vector;
using navtk::filtering::UkfStrategy;

class TestableUKF : public UkfStrategy {
public:
	// Bring protected parent members into public
	using UkfStrategy::calc_weighted_cov;
	using UkfStrategy::default_kappa;
	using UkfStrategy::mean_sigma_points;
	using UkfStrategy::mean_weight0;
	using UkfStrategy::mean_weights;
	using UkfStrategy::reconstruct_p_from_sigma_points;
	using UkfStrategy::reconstruct_x_from_sigma_points;
	using UkfStrategy::weight_off;
};

struct UKFTest : public ::testing::Test {
	Vector x0;
	Vector x1;
	Vector x2;
	Matrix P0;
	Matrix P1;
	Matrix P2;
	navtk::Size num_states;
	TestableUKF ukf;

	UKFTest()
	    : x0({0.0, 0.0, 0.0}),
	      x1({1.0, 2.0, 3.0}),
	      x2({1, 2, 3, -4}),
	      P0(navtk::eye(3, 3)),
	      P1({{4, 0, 0}, {0, 7, 0}, {0, 0, 31.69}}),
	      P2({{4, 0, 0, 0}, {0, 7, 0, 0}, {0, 0, 31.69, 0}, {0, 0, 0, 20}}),
	      num_states{3},
	      ukf{} {
		ukf.on_fusion_engine_state_block_added(num_states);
		ukf.set_estimate_slice(x0);
		ukf.set_covariance_slice(P0);
	}
};

TEST_F(UKFTest, UKF_meanWeights) {
	int kappa     = 0;
	double center = kappa / (num_states + kappa);
	double off    = 1 / ((double)(2.0 * (num_states + kappa)));

	Vector expected_data = off * navtk::ones(2 * num_states + 1);
	expected_data(0)     = center;

	auto mat = ukf.mean_weights(kappa, num_states);

	ASSERT_ALLCLOSE(expected_data, mat);
}

TEST_F(UKFTest, UKF_meanSigmaPoints) {
	int kappa = 0;

	Matrix expected_data{{0, 1.732051, 0, 0, -1.732051, 0, 0},
	                     {0, 0, 1.732051, 0, 0, -1.732051, 0},
	                     {0, 0, 0, 1.732051, 0, 0, -1.732051}};

	auto mat = ukf.mean_sigma_points(kappa, x0, P0);

	ASSERT_ALLCLOSE(expected_data, mat);
}

TEST_F(UKFTest, UKF_reconstructXFromSigmaPoints) {
	Matrix sigma_points{{1, 2, 3, 4, 5}, {5, 4, 3, 2, 1}};
	Vector sigma_weights{0.2, 0.2, 0.2, 0.2, 0.2};

	Vector expected_data{3, 3};

	auto mat = ukf.reconstruct_x_from_sigma_points(sigma_points, sigma_weights);

	ASSERT_ALLCLOSE(expected_data, mat);
}

TEST_F(UKFTest, UKF_reconstructPFromSigmaPoints) {
	Matrix sigma_points{{1, 2, 3, 4, 5}, {5, 4, 3, 2, 1}};
	Vector sigma_weights{0.2, 0.2, 0.2, 0.2, 0.2};
	Vector new_mean{3, 3};
	Matrix expected_data{{2, -2}, {-2, 2}};

	auto mat = ukf.reconstruct_p_from_sigma_points(sigma_points, sigma_weights, new_mean);

	ASSERT_ALLCLOSE(expected_data, mat);
}

TEST_F(UKFTest, UKF_calcWeightedCov) {
	Matrix pred1{{1, 0}, {0, 1}};
	Matrix pred2{{1.5, 0.5}, {0.5, 1.5}};
	Vector mean1{0, 0};
	Vector mean2{.5, .5};
	Vector weights{0.5, 0.5};
	Matrix expected_data{{.5, 0}, {0, .5}};

	auto mat = ukf.calc_weighted_cov(pred1, mean1, pred2, mean2, weights);

	ASSERT_ALLCLOSE(expected_data, mat);
}
