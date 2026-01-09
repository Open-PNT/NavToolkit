#include <memory>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>

#include <navtk/factory.hpp>
#include <navtk/filtering/fusion/strategies/FusionStrategy.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using navtk::Matrix;
using navtk::Size;
using navtk::Vector;

// Child class which overrides pure virtual methods with dummy implementations so the class can be
// concrete.
class TestableFusionStrategy : public navtk::filtering::FusionStrategy {
public:
	using FusionStrategy::FusionStrategy;  // Inherit constructors

	Vector get_estimate() const override { return navtk::zeros(3); }

	Matrix get_covariance() const override { return navtk::eye(3); }

	navtk::not_null<std::shared_ptr<FusionStrategy>> clone() const override {
		return std::make_shared<TestableFusionStrategy>(*this);
	}

	void on_fusion_engine_state_block_added_impl(Vector const &, Matrix const &) override {}

	void set_covariance_slice_impl(Matrix const &, Size, Size) override {}

	void set_estimate_slice_impl(Vector const &, Size) override {}

	void on_fusion_engine_state_block_removed_impl(Size, Size) override {}
};

TEST(FusionStrategyTests, bad_set_covariance_slice) {
	auto strategy = TestableFusionStrategy();
	EXPECT_UB_OR_DIE(strategy.set_covariance_slice(navtk::eye(2), 0, 1),
	                 "overlap its own transpose.",
	                 std::invalid_argument);
	EXPECT_UB_OR_DIE(strategy.set_covariance_slice(navtk::eye(2), 0, 2),
	                 "Trying to assign covariance beyond the end of the covariance matrix.",
	                 std::runtime_error);
}

ERROR_MODE_SENSITIVE_TEST(TEST, FusionStrategyTests, bad_on_fusion_engine_state_block_removed) {
	auto strategy = TestableFusionStrategy();
	EXPECT_HONORS_MODE_EX(strategy.on_fusion_engine_state_block_removed(0, 4),
	                      "Trying to remove more states than exist.",
	                      std::invalid_argument);
	EXPECT_HONORS_MODE_EX(strategy.on_fusion_engine_state_block_removed(1, 3),
	                      "Invalid state indices passed to remove_states",
	                      std::invalid_argument);
}
