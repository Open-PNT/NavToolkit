#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/stateblocks/ClockBiasesStateBlock.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::filtering::ClockBiasesStateBlock;
using navtk::filtering::ClockChoice;
using navtk::filtering::ClockModel;
using navtk::filtering::NULL_GEN_XHAT_AND_P_FUNCTION;
using navtk::filtering::RUBIDIUM_CLOCK;
using navtk::navutils::discretize_van_loan;
using navtk::navutils::PI;
using navtk::utils::NANO_PER_SEC;

ERROR_MODE_SENSITIVE_TEST(TEST, ClockBiasesTest, testBadInput) {
	auto block1 = ClockBiasesStateBlock("block1", RUBIDIUM_CLOCK, ClockChoice::QD, false);
	EXPECT_HONORS_MODE_EX(
	    (void)ClockBiasesStateBlock("block1", {1, -2, 3, 0}, ClockChoice::QD, false),
	    "Clock model coefficients cannot be negative",
	    std::invalid_argument);

	auto block1_dynamics1 = block1.generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(2, 0), to_type_timestamp(10, 0));
	EXPECT_HONORS_MODE_EX(
	    (void)block1.generate_dynamics(
	        NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(2, 0), to_type_timestamp()),
	    "dt should be >= 0 but it is.*-2",
	    std::invalid_argument);
}

// Check the re-derived 'Allan parameter Qd' matrix against the 'spectral density'
// noise matrix given in Brown and Hwang (only valid for no flicker noise)
TEST(ClockBiasesTest, verify_zero_h_neg1_model) {
	double h0      = 1.234e-18;
	double h_neg_2 = 8.7654e-17;
	ClockModel model{h0, 0.0, h_neg_2, 0};

	auto S_f = h0 / 2.0;
	auto S_g = 2.0 * pow(PI, 2.0) * h_neg_2;

	auto t_start = to_type_timestamp(0.376);
	auto t_stop  = to_type_timestamp(1.2423);
	double dt    = aspn_xtensor::to_seconds(t_stop - t_start);

	navtk::Matrix Q{{S_f, 0.0}, {0.0, S_g}};
	navtk::Matrix F{{0.0, 1.0}, {0.0, 0.0}};
	auto disc   = discretize_van_loan(F, Q, dt);
	auto block1 = ClockBiasesStateBlock("block1", model, ClockChoice::QD, false);
	auto dyn    = block1.generate_dynamics(NULL_GEN_XHAT_AND_P_FUNCTION, t_start, t_stop);
	ASSERT_ALLCLOSE_EX(disc.second, dyn.Qd, 1e-25, 1e-25);
}

class TestableCBSB : public ClockBiasesStateBlock {
public:
	TestableCBSB(std::string label, ClockModel clock_model)
	    : ClockBiasesStateBlock(label, clock_model, ClockChoice::QD, true) {}

	TestableCBSB(const TestableCBSB& block) : ClockBiasesStateBlock(block) {}

	navtk::not_null<std::shared_ptr<StateBlock<>>> clone() {
		return std::make_shared<TestableCBSB>(*this);
	}

	using ClockBiasesStateBlock::model;
};

// Validate clone() as implemented returns a deep copy by modifying all of the clone's properties
// and asserting they do not equal the original's.
TEST(ClockBiasesTest, test_clone) {
	auto block           = TestableCBSB("block", {0.0, 0.0, 0.0, 0.0});
	auto block_copy_cast = std::dynamic_pointer_cast<TestableCBSB>(block.clone());
	auto& block_copy     = *block_copy_cast;

	ASSERT_EQ(block.get_label(), block_copy.get_label());
	ASSERT_EQ(block.get_num_states(), block_copy.get_num_states());

	ASSERT_EQ(block.model.h_0, block_copy.model.h_0);
	block_copy.model.h_0 += 1;
	ASSERT_NE(block.model.h_0, block_copy.model.h_0);
}

// Test that validates the accuracy of the Q values and functionality of the switch statement
TEST(ClockBiasesTest, testQ) {

	/*do math in test case for Qd here (our expected)*/

	// sample starting and ending times
	auto t_start = to_type_timestamp(0.376);
	auto t_stop  = to_type_timestamp(1.2423);

	// setting up value of dt and dt^2
	auto dt    = (t_stop.get_elapsed_nsec() - t_start.get_elapsed_nsec()) * 1e-9;
	double dt2 = dt * dt;

	// setting up the other values, rubidium clock values
	double h0    = 2e-20;
	double hm1   = 7e-24;
	double pi_sq = PI * PI;
	double hm2   = 4e-29;

	/**Here, I am creating the comparison matrices for the expect_allclose blocks.**/
	// setting up the values of Qd matrix indices (only for Qd option)
	double index_one        = h0 / 2 * dt + 2 * hm1 * dt2 + 2.0 / 3 * pi_sq * hm2 * dt2 * dt;
	double diagonal_indices = pi_sq * hm2 * dt2;
	double index_four       = 4 * hm1 + 2 * pi_sq * hm2 * dt;
	navtk::Matrix comparison_qd{{index_one, diagonal_indices}, {diagonal_indices, index_four}};

	// setting up the values of Qd1 matrix indices (only for Qd1 option)
	double index_one1        = h0 / 2 * dt + 2.0 / 3 * pi_sq * hm2 * dt2 * dt;
	double diagonal_indices1 = pi_sq * hm2 * dt2;
	double index_four1       = h0 / (2 * dt) + (8.0 / 3) * pi_sq * hm2 * dt;
	navtk::Matrix comparison_qd1{{index_one1, diagonal_indices1}, {diagonal_indices1, index_four1}};

	// setting up the values of Qd2 matrix indices (only for Qd2 option)
	double index_one2        = 0.5 * h0 * dt + (2.0 / 3) * pi_sq * hm2 * dt * dt2;
	double diagonal_indices2 = hm1 * dt + pi_sq * hm2 * dt2;
	double index_four2       = 2 * pi_sq * hm2 * dt;
	navtk::Matrix comparison_qd2{{index_one2, diagonal_indices2}, {diagonal_indices2, index_four2}};

	// setting up the values of Qd3 matrix indices (only for Qd3 option)
	double index_one3        = 0.5 * h0 * dt + (2.0 / 3) * pi_sq * hm2 * dt * dt2;
	double diagonal_indices3 = pi_sq * hm2 * dt2;
	double index_four3       = 2 * pi_sq * hm2 * dt;
	navtk::Matrix comparison_qd3{{index_one3, diagonal_indices3}, {diagonal_indices3, index_four3}};

	/*auto value from the method generate_dynamics where the switch statement we want to test is
	This is our actual value that we compare against expected.*/

	// initialize choice for instance, in this case Qd
	ClockChoice choice  = ClockChoice::QD;
	ClockChoice choice1 = ClockChoice::QD1;
	ClockChoice choice2 = ClockChoice::QD2;
	ClockChoice choice3 = ClockChoice::QD3;

	auto block_q = ClockBiasesStateBlock("blockQ", RUBIDIUM_CLOCK, choice, false);
	auto method  = block_q.generate_dynamics(NULL_GEN_XHAT_AND_P_FUNCTION, t_start, t_stop);

	auto block_q1 = ClockBiasesStateBlock("blockQ1", RUBIDIUM_CLOCK, choice1, false);
	auto method1  = block_q1.generate_dynamics(NULL_GEN_XHAT_AND_P_FUNCTION, t_start, t_stop);

	auto block_q2 = ClockBiasesStateBlock("blockQ2", RUBIDIUM_CLOCK, choice2, false);
	auto method2  = block_q2.generate_dynamics(NULL_GEN_XHAT_AND_P_FUNCTION, t_start, t_stop);

	auto block_q3 = ClockBiasesStateBlock("blockQ3", RUBIDIUM_CLOCK, choice3, false);
	auto method3  = block_q3.generate_dynamics(NULL_GEN_XHAT_AND_P_FUNCTION, t_start, t_stop);


	EXPECT_ALLCLOSE_EX(comparison_qd, method.Qd, 1e-25, 0.0);

	EXPECT_ALLCLOSE_EX(comparison_qd1, method1.Qd, 1e-25, 0.0);

	EXPECT_ALLCLOSE_EX(comparison_qd2, method2.Qd, 1e-25, 0.0);

	EXPECT_ALLCLOSE_EX(comparison_qd3, method3.Qd, 1e-25, 0.0);
}

TEST(ClockBiasesTest, third_state_functionality) {

	/*do math in test case for Qd here (our expected)*/

	// sample starting and ending times
	auto t_start = to_type_timestamp(0.376);
	auto t_stop  = to_type_timestamp(1.2423);

	// setting up value of dt and dt^2
	auto dt    = (t_stop.get_elapsed_nsec() - t_start.get_elapsed_nsec()) * 1e-9;
	double dt2 = dt * dt;

	// setting up the other values, rubidium clock values
	double pi_sq = PI * PI;
	double hm2   = 4e-29;
	double q3    = 0.0;
	double h0    = 2e-20;
	double hm1   = 7e-24;
	double ct    = pi_sq * hm2 * dt2;
	double ct1   = hm1 * dt + pi_sq * hm2 * dt2;

	navtk::Matrix third_state{
	    {{(1.0 / 20) * q3 * dt2 * dt2 * dt, (1.0 / 8) * q3 * dt2 * dt2, (1.0 / 6) * q3 * dt2 * dt},
	     {(1.0 / 8) * q3 * dt2 * dt2, (1.0 / 3) * q3 * dt * dt2, (1.0 / 2) * q3 * dt2},
	     {(1.0 / 6) * q3 * dt2 * dt, (1.0 / 2) * q3 * dt2, q3 * dt}}};

	// Qd
	navtk::Matrix qd_comparison =
	    navtk::Matrix{{h0 / 2 * dt + 2 * hm1 * dt2 + 2.0 / 3 * pi_sq * hm2 * dt2 * dt, ct, 0},
	                  {ct, 4 * hm1 + 2 * pi_sq * hm2 * dt, 0},
	                  {0, 0, 0}} +
	    third_state;

	// Qd1
	navtk::Matrix qd1_comparison =
	    navtk::Matrix{{h0 / 2 * dt + 2.0 / 3 * pi_sq * hm2 * dt2 * dt, ct, 0},
	                  {ct, h0 / (2 * dt) + (8.0 / 3) * pi_sq * hm2 * dt, 0},
	                  {0, 0, 0}} +
	    third_state;

	// Qd2
	navtk::Matrix qd2_comparison =
	    navtk::Matrix{{0.5 * h0 * dt + (2.0 / 3) * pi_sq * hm2 * dt * dt2, ct1, 0},
	                  {ct1, 2 * pi_sq * hm2 * dt, 0},
	                  {0, 0, 0}} +
	    third_state;

	// Qd3
	navtk::Matrix qd3_comparison =
	    navtk::Matrix{{0.5 * h0 * dt + (2.0 / 3) * pi_sq * hm2 * dt * dt2, ct, 0},
	                  {ct, 2 * pi_sq * hm2 * dt, 0},
	                  {0, 0, 0}} +
	    third_state;

	// initialize choice for instance, in this case Qd
	ClockChoice choice  = ClockChoice::QD;
	ClockChoice choice1 = ClockChoice::QD1;
	ClockChoice choice2 = ClockChoice::QD2;
	ClockChoice choice3 = ClockChoice::QD3;

	auto block_q = ClockBiasesStateBlock("blockQ", RUBIDIUM_CLOCK, choice, true);
	auto method  = block_q.generate_dynamics(NULL_GEN_XHAT_AND_P_FUNCTION, t_start, t_stop);

	auto block_q1 = ClockBiasesStateBlock("blockQ1", RUBIDIUM_CLOCK, choice1, true);
	auto method1  = block_q1.generate_dynamics(NULL_GEN_XHAT_AND_P_FUNCTION, t_start, t_stop);

	auto block_q2 = ClockBiasesStateBlock("blockQ2", RUBIDIUM_CLOCK, choice2, true);
	auto method2  = block_q2.generate_dynamics(NULL_GEN_XHAT_AND_P_FUNCTION, t_start, t_stop);

	auto block_q3 = ClockBiasesStateBlock("blockQ3", RUBIDIUM_CLOCK, choice3, true);
	auto method3  = block_q3.generate_dynamics(NULL_GEN_XHAT_AND_P_FUNCTION, t_start, t_stop);


	EXPECT_ALLCLOSE_EX(qd_comparison, method.Qd, 1e-25, 0.0);

	EXPECT_ALLCLOSE_EX(qd1_comparison, method1.Qd, 1e-25, 0.0);

	EXPECT_ALLCLOSE_EX(qd2_comparison, method2.Qd, 1e-25, 0.0);

	EXPECT_ALLCLOSE_EX(qd3_comparison, method3.Qd, 1e-25, 0.0);
}
