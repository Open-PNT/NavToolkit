#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/stateblocks/DeadReckoningStateBlock.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::eye;
using navtk::Matrix;
using navtk::Vector;
using navtk::filtering::DeadReckoningStateBlock;
using navtk::filtering::DiscretizationStrategy;
using navtk::navutils::DEG2RAD;
using navtk::navutils::discretize_first_order;
using navtk::navutils::discretize_second_order;
using navtk::navutils::discretize_van_loan;
using navtk::navutils::east_to_delta_lon;
using navtk::navutils::north_to_delta_lat;

struct DeadReckoningStateBlockTests : public ::testing::Test {
	double lat_sig                             = 5.0;
	double lon_sig                             = 7.0;
	Vector time_const                          = {200.0, 100.0};
	Vector process_sigmas                      = {3.0, 2.0};
	double init_alt                            = 1000.0;
	Vector xhat                                = {39.0 * DEG2RAD, -82.0 * DEG2RAD, 2.0, 3.0};
	double dt                                  = 2.0;
	aspn_xtensor::TypeTimestamp t_from         = to_type_timestamp();
	aspn_xtensor::TypeTimestamp t_to           = to_type_timestamp(2, 0);
	navtk::filtering::GenXhatPFunction gen_x_p = [=, this](const std::vector<std::string>&) {
		return std::make_shared<navtk::filtering::EstimateWithCovariance>(xhat, navtk::zeros(4, 4));
	};

	void test_strategy(DiscretizationStrategy strat, size_t order) {
		// Lat, lon, vel north, vel east
		Matrix Q = navtk::zeros(4, 4);
		Q(0, 0)  = lat_sig * lat_sig;
		Q(1, 1)  = lon_sig * lon_sig;
		Q(2, 2)  = (2 * process_sigmas(0) * process_sigmas(0)) / time_const(0);
		Q(3, 3)  = (2 * process_sigmas(1) * process_sigmas(1)) / time_const(1);
		Matrix F = navtk::zeros(4, 4);
		F(0, 2)  = navtk::navutils::north_to_delta_lat(1.0, xhat(0), init_alt);
		F(1, 3)  = navtk::navutils::east_to_delta_lon(1.0, xhat(0), init_alt);
		F(2, 2)  = -1.0 / time_const(0);
		F(3, 3)  = -1.0 / time_const(1);

		auto block = DeadReckoningStateBlock(
		    "block", lat_sig, lon_sig, time_const, process_sigmas, init_alt, strat);
		auto dyn_model = block.generate_dynamics(gen_x_p, t_from, t_to);
		Matrix expected_qd;
		Matrix expected_phi;
		if (order == 1) {
			auto disc    = discretize_first_order(F, Q, dt);
			expected_phi = disc.first;
			expected_qd  = disc.second;
		} else if (order == 2) {
			auto disc    = discretize_second_order(F, Q, dt);
			expected_phi = disc.first;
			expected_qd  = disc.second;
		} else {
			auto disc    = discretize_van_loan(F, Q, dt);
			expected_phi = disc.first;
			expected_qd  = disc.second;
		}
		auto exp_g_of_x = navtk::dot(expected_phi, xhat);
		ASSERT_ALLCLOSE_EX(expected_qd, dyn_model.Qd, 1e-14, 1e-14);
		ASSERT_ALLCLOSE_EX(expected_phi, dyn_model.Phi, 1e-14, 1e-14);
		ASSERT_ALLCLOSE_EX(exp_g_of_x, dyn_model.g(xhat), 1e-14, 1e-14);
	}
};


class TestableDRSB : public DeadReckoningStateBlock {
public:
	using DeadReckoningStateBlock::DeadReckoningStateBlock;

	TestableDRSB(const TestableDRSB& block) : DeadReckoningStateBlock(block) {}

	navtk::not_null<std::shared_ptr<StateBlock<>>> clone() {
		return std::make_shared<TestableDRSB>(*this);
	}
};

TEST_F(DeadReckoningStateBlockTests, test_propagate_first_order) {
	test_strategy(navtk::filtering::first_order_discretization_strategy, 1);
}

TEST_F(DeadReckoningStateBlockTests, test_propagate_second_order) {
	test_strategy(navtk::filtering::second_order_discretization_strategy, 2);
}

TEST_F(DeadReckoningStateBlockTests, test_propagate_full_order) {
	test_strategy(navtk::filtering::full_order_discretization_strategy, 3);
}

TEST_F(DeadReckoningStateBlockTests, test_altitude_aux) {
	auto strat = navtk::filtering::first_order_discretization_strategy;
	auto block =
	    TestableDRSB("block", lat_sig, lon_sig, time_const, process_sigmas, init_alt, strat);

	// With an altitude change, the Phi components will be different
	navtk::filtering::DynamicsModel dyn_model_01 = block.generate_dynamics(gen_x_p, t_from, t_to);
	auto alt_aux = std::make_shared<aspn_xtensor::MeasurementAltitude>(
	    aspn_xtensor::TypeHeader(ASPN_UNDEFINED, 0, 0, 0, 0),
	    aspn_xtensor::TypeTimestamp(int64_t(dt * 1e9)),
	    ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL,
	    200000.0,
	    10.0,
	    ASPN_MEASUREMENT_ALTITUDE_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	block.receive_aux_data({alt_aux});
	navtk::filtering::DynamicsModel dyn_model_02 = block.generate_dynamics(gen_x_p, t_from, t_to);
	ASSERT_NE(dyn_model_01.Phi(0, 2), dyn_model_02.Phi(0, 2));
	ASSERT_NE(dyn_model_01.Phi(1, 3), dyn_model_02.Phi(1, 3));
}

TEST_F(DeadReckoningStateBlockTests, test_bad_callback) {
	this->gen_x_p = {};
	EXPECT_THROW(test_strategy(navtk::filtering::first_order_discretization_strategy, 1),
	             std::bad_function_call);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, DeadReckoningStateBlockTests, test_nullptr_callback) {

	auto block = DeadReckoningStateBlock("block",
	                                     test.lat_sig,
	                                     test.lon_sig,
	                                     test.time_const,
	                                     test.process_sigmas,
	                                     test.init_alt,
	                                     navtk::filtering::first_order_discretization_strategy);

	EXPECT_HONORS_MODE_EX(
	    (void)block.generate_dynamics(
	        navtk::filtering::NULL_GEN_XHAT_AND_P_FUNCTION, test.t_from, test.t_to),
	    "received nullptr",
	    std::runtime_error);
}
