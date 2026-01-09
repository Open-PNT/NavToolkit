#include <filtering/fusion/strategies/standard_model_strategy_tests.hpp>

#include <type_traits>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>

#include <navtk/experimental/random.hpp>
#include <navtk/filtering/experimental/fusion/strategies/RbpfStrategy.hpp>
#include <navtk/filtering/fusion/strategies/EkfStrategy.hpp>
#include <navtk/filtering/fusion/strategies/UkfStrategy.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/tensors.hpp>

using namespace navtk::filtering;
using namespace navtk::filtering::experimental;
using navtk::Size;

template <std::unique_ptr<StandardModelStrategy> (*factory)(Vector, Matrix)>
class DynamicStrategy {
public:
	static std::unique_ptr<StandardModelStrategy> strategy_factory(Vector x, Matrix P) {
		return factory(std::move(x), std::move(P));
	}
};

template <typename T>
struct StandardModelStrategyTests : ::testing::Test {
	template <typename>
	struct IsDynamicStrategy : public std::false_type {};

	template <std::unique_ptr<StandardModelStrategy> (*factory)(Vector, Matrix)>
	struct IsDynamicStrategy<DynamicStrategy<factory>> : public std::true_type {};

	static_assert(IsDynamicStrategy<T>::value, "Tested types must use DynamicStrategy");

	static std::unique_ptr<StandardModelStrategy> strategy_factory(Vector x, Matrix P) {
		return T::strategy_factory(std::move(x), std::move(P));
	}
};
TYPED_TEST_SUITE_P(StandardModelStrategyTests);

TYPED_TEST_P(StandardModelStrategyTests, basicPropagation_SLOW) {
	auto x        = zeros(3) + 1.5;
	auto p        = zeros(3, 3);
	auto strategy = this->strategy_factory(x, p);
	test_basic_propagation(*strategy);
}
TYPED_TEST_P(StandardModelStrategyTests, basicUpdate_SLOW) {
	auto x        = zeros(2) + 1.5;
	auto p        = eye(2) * 0.04;
	auto strategy = this->strategy_factory(x, p);
	test_basic_update(*strategy);
}
TYPED_TEST_P(StandardModelStrategyTests, changeableDynamics_SLOW) {
	auto x        = zeros(2) + 1.5;
	auto p        = eye(2) * 0.0001;
	auto strategy = this->strategy_factory(x, p);
	test_changeable_standard_dynamics_model(*strategy);
}
TYPED_TEST_P(StandardModelStrategyTests, demonstrateChangeableUpdate_SLOW) {
	auto x        = zeros(2) + 1.5;
	auto p        = eye(2) * 0.001;
	auto strategy = this->strategy_factory(x, p);
	demonstrate_changeable_update(*strategy);
}
TYPED_TEST_P(StandardModelStrategyTests, propagateAffine_SLOW) {
	auto x        = zeros(3) + 1.5;
	auto p        = zeros(3, 3);
	auto strategy = this->strategy_factory(x, p);
	test_propagate_affine(*strategy);
}
TYPED_TEST_P(StandardModelStrategyTests, testStateResetNotChangingCovariance_SLOW) {
	auto x        = zeros(3) + 1.5;
	auto p        = ones(3, 3);
	auto strategy = this->strategy_factory(x, p);
	test_state_reset_not_changing_covariance(*strategy);
}
TYPED_TEST_P(StandardModelStrategyTests, updateNonLinear_SLOW) {
	auto x        = zeros(3) + 1.5;
	auto p        = 0.004 * eye(3, 3);
	auto strategy = this->strategy_factory(x, p);
	test_update_non_linear(*strategy);
}
TYPED_TEST_P(StandardModelStrategyTests, updateConvergesZeroStandardDynamicsModelNoise_SLOW) {
	auto x        = zeros(2) + 1.5;
	auto p        = eye(2);
	auto strategy = this->strategy_factory(x, p);
	test_update_converges_zero_standard_dynamics_model_noise(*strategy);
}
TYPED_TEST_P(StandardModelStrategyTests, testUpdate_SLOW) {
	Vector x{2.5, 2.0};
	Matrix p = 1e-5 * eye(2, 2);
	p(0, 0)  = 1.25e-4;

	auto strategy = this->strategy_factory(x, p);
	test_update(*strategy);
}

REGISTER_TYPED_TEST_SUITE_P(StandardModelStrategyTests,
                            testStateResetNotChangingCovariance_SLOW,
                            basicPropagation_SLOW,
                            basicUpdate_SLOW,
                            changeableDynamics_SLOW,
                            demonstrateChangeableUpdate_SLOW,
                            propagateAffine_SLOW,
                            updateNonLinear_SLOW,
                            updateConvergesZeroStandardDynamicsModelNoise_SLOW,
                            testUpdate_SLOW);

std::unique_ptr<StandardModelStrategy> ukf_strategy_factory(Vector x, Matrix p) {
	auto strategy = std::make_unique<UkfStrategy>();
	strategy->on_fusion_engine_state_block_added(num_rows(x));
	strategy->set_estimate_slice(std::move(x));
	strategy->set_covariance_slice(std::move(p));
	return strategy;
}
std::unique_ptr<StandardModelStrategy> ekf_strategy_factory(Vector x, Matrix p) {
	auto strategy = std::make_unique<EkfStrategy>();
	strategy->on_fusion_engine_state_block_added(num_rows(x));
	strategy->set_estimate_slice(std::move(x));
	strategy->set_covariance_slice(std::move(p));
	return strategy;
}
std::unique_ptr<StandardModelStrategy> rbpf_strategy_nostate_factory(Vector x, Matrix p) {
	auto strategy = std::make_unique<RbpfStrategy>(5, 0.02, false);
	strategy->on_fusion_engine_state_block_added(num_rows(x));
	strategy->set_estimate_slice(std::move(x));
	strategy->set_covariance_slice(std::move(p));
	strategy->set_marked_states(std::vector<size_t>());
	return strategy;
}
std::unique_ptr<StandardModelStrategy> rbpf_strategy_halfstate_factory(Vector x, Matrix p) {
	navtk::experimental::s_rand(9159329240996);
	auto strategy =
	    std::make_unique<RbpfStrategy>(200, 0.02, false, residual_resample_with_replacement);
	auto rows = num_rows(x);
	strategy->on_fusion_engine_state_block_added(rows);
	strategy->set_estimate_slice(std::move(x));
	strategy->set_covariance_slice(std::move(p));
	std::vector<size_t> marks(rows, 0);
	for (navtk::Size ii = 0; ii < rows; ii += 2) marks[ii] = ii;
	strategy->set_marked_states(marks, {0.075});
	return strategy;
}
std::unique_ptr<StandardModelStrategy> rbpf_strategy_allstate_factory(Vector x, Matrix p) {
	navtk::experimental::s_rand(9159329240996);
	auto strategy =
	    std::make_unique<RbpfStrategy>(200, 0.02, false, residual_resample_with_replacement);
	auto rows = num_rows(x);
	strategy->on_fusion_engine_state_block_added(rows);
	strategy->set_estimate_slice(std::move(x));
	strategy->set_covariance_slice(std::move(p));
	std::vector<size_t> marks(rows, 0);
	for (auto ii = rows; ii--;) marks[ii] = ii;
	strategy->set_marked_states(marks, {0.075});
	return strategy;
}
std::unique_ptr<StandardModelStrategy> rbpf_strategy_nostate_singlejacobian_factory(Vector x,
                                                                                    Matrix p) {
	auto strategy = std::make_unique<RbpfStrategy>(5, 0.02, true);
	strategy->on_fusion_engine_state_block_added(num_rows(x));
	strategy->set_estimate_slice(std::move(x));
	strategy->set_covariance_slice(std::move(p));
	strategy->set_marked_states(std::vector<size_t>());
	return strategy;
}
std::unique_ptr<StandardModelStrategy> rbpf_strategy_halfstate_singlejacobian_factory(Vector x,
                                                                                      Matrix p) {
	navtk::experimental::s_rand(9159329240996);
	auto strategy =
	    std::make_unique<RbpfStrategy>(200, 0.02, true, residual_resample_with_replacement);
	auto rows = num_rows(x);
	strategy->on_fusion_engine_state_block_added(rows);
	strategy->set_estimate_slice(std::move(x));
	strategy->set_covariance_slice(std::move(p));
	std::vector<size_t> marks(rows, 0);
	for (navtk::Size ii = 0; ii < rows; ii += 2) marks[ii] = ii;
	strategy->set_marked_states(marks, {0.075});
	return strategy;
}
std::unique_ptr<StandardModelStrategy> rbpf_strategy_allstate_singlejacobian_factory(Vector x,
                                                                                     Matrix p) {
	navtk::experimental::s_rand(9159329240996);
	auto strategy =
	    std::make_unique<RbpfStrategy>(200, 0.02, true, residual_resample_with_replacement);
	auto rows = num_rows(x);
	strategy->on_fusion_engine_state_block_added(rows);
	strategy->set_estimate_slice(std::move(x));
	strategy->set_covariance_slice(std::move(p));
	std::vector<size_t> marks(rows, 0);
	for (auto ii = rows; ii--;) marks[ii] = ii;
	strategy->set_marked_states(marks, {0.075});
	return strategy;
}

// Add subclasses of StandardModelStrategy that you wish to test to this template parameter list.
typedef ::testing::Types<DynamicStrategy<ukf_strategy_factory>,
                         DynamicStrategy<ekf_strategy_factory>,
                         DynamicStrategy<rbpf_strategy_nostate_factory>,
                         DynamicStrategy<rbpf_strategy_halfstate_factory>,
                         DynamicStrategy<rbpf_strategy_allstate_factory>,
                         DynamicStrategy<rbpf_strategy_nostate_singlejacobian_factory>,
                         DynamicStrategy<rbpf_strategy_halfstate_singlejacobian_factory>,
                         DynamicStrategy<rbpf_strategy_allstate_singlejacobian_factory>>
    StandardModelStrategyTestsTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(StandardModel,
                               StandardModelStrategyTests,
                               StandardModelStrategyTestsTypes, );

// Since BadUpdateModel is only intended for specific classes, these tests are separate.
ERROR_MODE_SENSITIVE_TEST(TEST, StandardModelStrategyTests, UkfStrategyBadUpdateModel) {
	auto x        = zeros(3);
	auto p        = eye(3, 3);
	auto strategy = std::make_unique<UkfStrategy>();
	strategy->on_fusion_engine_state_block_added(num_rows(x));
	strategy->set_estimate_slice(std::move(x));
	strategy->set_covariance_slice(std::move(p));
	EXPECT_HONORS_MODE_EX(test_bad_update_model_error(*strategy), "dimension", std::range_error);
}
ERROR_MODE_SENSITIVE_TEST(TEST, StandardModelStrategyTests, EkfStrategyBadUpdateModel) {
	auto x        = zeros(3);
	auto p        = eye(3, 3);
	auto strategy = std::make_unique<EkfStrategy>();
	strategy->on_fusion_engine_state_block_added(num_rows(x));
	strategy->set_estimate_slice(std::move(x));
	strategy->set_covariance_slice(std::move(p));
	EXPECT_HONORS_MODE_EX(test_bad_update_model_error(*strategy), "dimension", std::range_error);
}
