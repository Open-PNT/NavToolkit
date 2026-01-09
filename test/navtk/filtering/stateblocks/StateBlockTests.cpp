#include <gtest/gtest.h>

#include <error_mode_assert.hpp>
#include <memory>
#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/filtering/stateblocks/discretization_strategy.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::Matrix;
using navtk::Vector;
using navtk::filtering::DiscretizationStrategy;
using navtk::filtering::first_order_discretization_strategy;
using navtk::filtering::full_order_discretization_strategy;
using navtk::filtering::NULL_GEN_XHAT_AND_P_FUNCTION;
using navtk::filtering::StateBlock;

TEST(StateBlockTests, receive_aux_data_warning) {
	auto state_block_label = "my_label";
	auto state_block       = StateBlock<navtk::filtering::DynamicsModel>(1, state_block_label);
	auto aux_data          = AspnBaseVector();
	EXPECT_WARN(state_block.receive_aux_data(aux_data), state_block_label);
}
TEST(StateBlockTests, constructor) {
	Matrix Q                     = {{1.0}};
	Matrix expected_phi          = {{1.0}};
	Matrix expected_qd           = {{1.0}};
	auto atol                    = 0.0;
	auto rtol                    = 0.0;
	auto discretization_strategy = DiscretizationStrategy{&full_order_discretization_strategy};
	auto custom_constant_stateblock =
	    StateBlock<navtk::filtering::DynamicsModel>(1, "custom", discretization_strategy, Q);
	auto custom_dyn = custom_constant_stateblock.generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(1, 0));
	ASSERT_ALLCLOSE_EX(expected_phi, custom_dyn.Phi, rtol, atol);
	ASSERT_ALLCLOSE_EX(expected_qd, custom_dyn.Qd, rtol, atol);
}
TEST(StateBlockTests, default_noise_to_zeros) {
	size_t num_states  = 2;
	Matrix expected_qd = navtk::zeros(num_states, num_states);
	auto test_q_zeros_state_block =
	    StateBlock<navtk::filtering::DynamicsModel>(num_states, "default_noise_to_zeros");
	auto dyn = test_q_zeros_state_block.generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(1, 0));
	ASSERT_ALLCLOSE_EX(expected_qd, dyn.Qd, 0.0, 0.0);
}
ERROR_MODE_SENSITIVE_TEST(TEST, StateBlockTests, q_matrix_shape_error) {
	EXPECT_HONORS_MODE(StateBlock<navtk::filtering::DynamicsModel>(
	                       1,
	                       "constructor_error",
	                       DiscretizationStrategy{&full_order_discretization_strategy},
	                       navtk::zeros(2, 2)),
	                   "Invalid matrix dimensions");
}
TEST(StateBlockTests, clone) {
	auto state_block_label       = "my_label";
	auto discretization_strategy = DiscretizationStrategy{&full_order_discretization_strategy};
	auto state_block             = StateBlock<navtk::filtering::DynamicsModel>(
        1, state_block_label, discretization_strategy, {{1.0}});
	auto state_block_clone = state_block.clone();
	auto atol              = 0.0;
	auto rtol              = 0.0;
	EXPECT_EQ(state_block.get_num_states(), state_block_clone->get_num_states());
	EXPECT_TRUE(state_block.get_label() == state_block_clone->get_label());
	auto original_dynamics = state_block.generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(1, 0));
	auto cloned_dynamics = state_block_clone->generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(1, 0));
	ASSERT_ALLCLOSE_EX(original_dynamics.Phi, cloned_dynamics.Phi, rtol, atol);
	ASSERT_ALLCLOSE_EX(original_dynamics.Qd, cloned_dynamics.Qd, rtol, atol);
}
TEST(StateBlockTests, generate_dynamics) {
	auto state_block_label = "my_label";
	auto state_block       = StateBlock<navtk::filtering::DynamicsModel>(1, state_block_label);
	auto atol              = 0.0;
	auto rtol              = 0.0;
	auto dyn               = state_block.generate_dynamics(
        NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(1, 0));
	Matrix expected_phi = {{1.0}};
	Matrix expected_qd  = {{0.0}};
	ASSERT_ALLCLOSE_EX(expected_phi, dyn.Phi, rtol, atol);
	ASSERT_ALLCLOSE_EX(expected_qd, dyn.Qd, rtol, atol);
}
