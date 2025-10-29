#include <gtest/gtest.h>
#include <tensor_assert.hpp>
#include <xtensor/generators/xrandom.hpp>
#include <xtensor/io/xio.hpp>

#include <navtk/experimental/random.hpp>
#include <navtk/filtering/experimental/resampling.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/tensors.hpp>

using namespace navtk::filtering;
using namespace navtk::filtering::experimental;
using namespace navtk;
using navtk::experimental::rand_n;
using navtk::filtering::EstimateWithCovariance;

TEST(resampling_test, systematic_resampling) {
	ResamplingResult res;

	Vector weights{5., 0., 0.9, 0., 0.8, 0., 0.1, 0., 2., 2.4, 3.6, 4.7, 5.8, 5.9, 4.4};
	weights = weights / xt::sum(weights)[0];

	navtk::experimental::s_rand(0xabcdefULL);
	std::vector<size_t> const exp_index = {0, 0, 0, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14};
	std::vector<size_t> const exp_index_count = {3, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 1};

	res = systematic_resampling(weights, nullptr);


	ASSERT_EQ(exp_index, res.index);

	// Save a pointer to the current global random number generator so it can be restored after
	// this test.
	auto prev_rng = navtk::experimental::get_global_rng();

	// Test once with the xtensor rng and once with pcg64, making sure systematic_resampling honors
	// the global RNG setting.
	{
		ResamplingResult res;
		navtk::experimental::set_global_rng<xt::random::default_engine_type>();

		navtk::experimental::s_rand(0xabcdefULL);
		std::vector<size_t> const exp_index_short       = {0, 2, 9, 11, 12, 13, 13};
		std::vector<size_t> const exp_index_count_short = {
		    1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 2, 0};
		size_t num = 7;
		res        = systematic_resampling(weights, &num);
		EXPECT_EQ(exp_index_short, res.index);
		EXPECT_EQ(exp_index_count_short, res.index_count);
	}

	{
		navtk::experimental::set_global_rng(
		    std::make_shared<navtk::experimental::LocalEngineWrapper>());

		navtk::experimental::s_rand(0xabcdefULL);
		std::vector<size_t> const exp_index_short       = {0, 2, 9, 11, 12, 13, 13};
		std::vector<size_t> const exp_index_count_short = {
		    1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 2, 0};
		size_t num = 7;
		res        = systematic_resampling(weights, &num);
		EXPECT_EQ(exp_index_short, res.index);
		EXPECT_EQ(exp_index_count_short, res.index_count);
	}

	// Restore the original global rng setting.
	navtk::experimental::set_global_rng(prev_rng);
}

TEST(resampling_test, get_residual_resample_with_replacement) {
	ResamplingResult res;
	Vector weights{0.4, 5., 0., 0.9, 0., 0.8, 0., 0.1, 0., 2., 2.4, 3.6, 4.7, 5.8, 5.9, 4.4};
	weights = weights / xt::sum(weights)[0];

	// Save a pointer to the current global random number generator so it can be restored after this
	// test.
	auto prev_rng = navtk::experimental::get_global_rng();

	// Result variables
	std::vector<size_t> index, index_count;

	// Test with two different RNGs to make sure residual_resample_with_replacement honors the
	// global RNG setting.
	{
		navtk::experimental::set_global_rng<xt::random::default_engine_type>();
		navtk::experimental::s_rand(0xabcdefULL);
		std::vector<size_t> const exp_index = {
		    0, 1, 1, 9, 10, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15};
		std::vector<size_t> const exp_index_count = {
		    1, 2, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 4, 2};

		size_t M = num_rows(weights);

		res = residual_resample_with_replacement(weights, &M);
		EXPECT_EQ(exp_index, res.index);
		EXPECT_EQ(exp_index_count, res.index_count);
	}

	{
		navtk::experimental::set_global_rng(
		    std::make_shared<navtk::experimental::LocalEngineWrapper>());
		navtk::experimental::s_rand(0xabcdefULL);
		std::vector<size_t> const exp_index = {
		    1, 1, 9, 10, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15};
		std::vector<size_t> const exp_index_count = {
		    0, 2, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 3, 3, 3, 1};
		size_t M = num_rows(weights);

		res = residual_resample_with_replacement(weights, &M);
		EXPECT_EQ(exp_index, res.index);
		EXPECT_EQ(exp_index_count, res.index_count);
	}

	{
		// uniform weights so no residuals should be exercised.
		weights = ones(15);
		weights = weights / xt::sum(weights)[0];

		navtk::experimental::set_global_rng(
		    std::make_shared<navtk::experimental::LocalEngineWrapper>());
		navtk::experimental::s_rand(0xabcdefULL);
		std::vector<size_t> const exp_index = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
		std::vector<size_t> const exp_index_count = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
		size_t M                                  = num_rows(weights);

		res = residual_resample_with_replacement(weights, &M);
		EXPECT_EQ(exp_index, res.index);
		EXPECT_EQ(exp_index_count, res.index_count);
	}

	// Restore the original global rng setting.
	navtk::experimental::set_global_rng(prev_rng);
}
