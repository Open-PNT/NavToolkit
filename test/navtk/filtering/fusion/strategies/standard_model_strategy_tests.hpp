#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <tensor_assert.hpp>

#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/fusion/strategies/StandardModelStrategy.hpp>
#include <navtk/tensors.hpp>

using navtk::eye;
using navtk::Matrix;
using navtk::num_rows;
using navtk::ones;
using navtk::Vector;
using navtk::zeros;
using navtk::filtering::StandardDynamicsModel;
using navtk::filtering::StandardMeasurementModel;
using navtk::filtering::StandardModelStrategy;


// A version of ASSERT_ALLCLOSE that uses specified tolerances for its values
// of RTOL and ATOL.
#define ASSERT_ALLCLOSE_T(...) ASSERT_ALLCLOSE_EX(__VA_ARGS__, 3e-3, 3e-3)
#define ASSERT_ALLCLOSE_L(...) ASSERT_ALLCLOSE_EX(__VA_ARGS__, 2e-1, 2e-1)

/**
 * A set of tests anything claiming to be a StandardModelStrategy should pass.
 */
void test_propagate_affine(StandardModelStrategy& strategy) {
	auto I = eye(3, 3);
	auto g = [&](Vector it) -> Vector { return 2 * it + 1.5 * ones(3); };
	auto Q = 0.001 * I;
	strategy.propagate(StandardDynamicsModel(g, 2 * I, Q));

	Vector mat{4.5, 4.5, 4.5};
	ASSERT_ALLCLOSE_T(strategy.get_estimate(), mat);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), Q);
}

void test_state_reset_not_changing_covariance(StandardModelStrategy& strategy) {
	auto I = eye(3, 3);

	auto h = [&](Vector it) -> Vector { return 2 * it(0) + zeros(1); };
	Matrix H{{2, 0, 0}};
	Matrix noise{{0.01}};
	Vector meas{3.01};

	strategy.propagate(StandardDynamicsModel(2 * I, 1.1e-3 * I));
	strategy.update(StandardMeasurementModel(meas, h, H, noise));

	Matrix cov = strategy.get_covariance();

	strategy.set_estimate_slice(zeros(3), 0);

	ASSERT_ALLCLOSE_T(zeros(3), strategy.get_estimate());
	ASSERT_ALLCLOSE_T(cov, strategy.get_covariance());
}

void test_update_non_linear(StandardModelStrategy& strategy) {
	auto I = eye(3, 3);
	Matrix noise{{0.01}};
	Vector meas{4.53};

	auto h = [&](Vector it) -> Vector { return 2 * it[0] + 1.5 * Vector{1}; };

	strategy.update(StandardMeasurementModel(meas, h, Matrix{{2, 0, 0}}, noise));

	Matrix expected_cov;
	expected_cov       = 0.004 * I;
	expected_cov(0, 0) = 0.00153846;

	Vector mat{1.50923076, 1.5, 1.5};

	ASSERT_ALLCLOSE_T(strategy.get_estimate(), mat);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), expected_cov);
}

void test_update(StandardModelStrategy& strategy) {

	Matrix noise{{5e-5}};
	Vector meas{5.004};

	auto h = [&](Vector it) -> Vector { return 2 * it(0) + zeros(1); };
	Matrix H{{2, 0}};

	strategy.update(StandardMeasurementModel(meas, h, H, noise));

	Matrix expected_cov{{1.24875e-5, 0.}, {0.0, 0.001}};

	Vector mat{2.5018181818, 2.0};

	ASSERT_ALLCLOSE_T(strategy.get_estimate(), mat);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), expected_cov);
}

void test_basic_propagation(StandardModelStrategy& strategy) {
	auto I = eye(3, 3);

	strategy.propagate(StandardDynamicsModel(2 * I, 1.1e-4 * I));

	auto mat = zeros(3, 1) + 3;
	ASSERT_ALLCLOSE_T(strategy.get_estimate(), mat);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), 1.1e-4 * I);

	strategy.propagate(StandardDynamicsModel(2 * I, 1.2e-4 * I));

	auto mat2 = zeros(3, 1) + 6;
	ASSERT_ALLCLOSE_T(strategy.get_estimate(), mat2);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), 5.6e-4 * I);

	{
		Matrix P = 1.2e-4 * I;
		P(0, 1)  = 0.5e-4;
		P(1, 0)  = 0.5e-4;
		strategy.set_covariance_slice(std::move(P));
	}

	strategy.propagate(StandardDynamicsModel(2 * I, 1.4e-4 * I));

	Matrix expected{{6.2e-4, 2e-4, 0}, {2e-4, 6.2e-4, 0}, {0, 0, 6.2e-4}};

	mat2 = zeros(3, 1) + 12;

	ASSERT_ALLCLOSE_T(strategy.get_estimate(), mat2);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), expected);
}

void test_basic_update(StandardModelStrategy& strategy) {
	auto I     = eye(2, 2);
	auto noise = zeros(2, 2);
	auto meas  = Vector{1.6, 1.4};

	strategy.update(StandardMeasurementModel(meas, I, noise));

	Vector expect   = {1.6, 1.4};
	Matrix expect_m = zeros(2, 2);

	ASSERT_ALLCLOSE_T(strategy.get_estimate(), expect);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), expect_m);

	strategy.set_estimate_slice(zeros(2));
	strategy.set_covariance_slice(std::move(0.0002 * I));

	meas = Vector{1.55e-4, 1.5e-4};

	noise = 1e-2 * I;
	strategy.update(StandardMeasurementModel(meas, 0.2 * I, noise));

	expect = {6.1950439e-7, 5.995203836e-7};

	ASSERT_ALLCLOSE_T(strategy.get_estimate(), expect);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), 0.0001998401278 * eye(2, 2));

	strategy.set_estimate_slice(-2. * ones(2));
	strategy.set_covariance_slice(std::move(0.0004 * I));
	meas = Vector{1.56, 1.506};

	noise = 0.04 * I;
	strategy.update(StandardMeasurementModel(meas, 0.1 * I, noise));

	expect = {-1.99824017, -1.99829417058};

	ASSERT_ALLCLOSE_T(strategy.get_estimate(), expect);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), 0.0004 * I);
}

void demonstrate_changeable_update(StandardModelStrategy& strategy) {
	Matrix meas_matrix{{1, 0}};
	Vector meas{1.495};
	auto I = Matrix{{1}};

	strategy.update(StandardMeasurementModel(meas, meas_matrix, 1e-2 * I));

	Vector mat1({1.4975, 1.5});
	Matrix mat2({{0.000909, 0}, {0, 0.001}});
	ASSERT_ALLCLOSE_T(strategy.get_estimate(), mat1);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), mat2);

	meas = Vector{1.505};

	strategy.update(StandardMeasurementModel(meas, meas_matrix, 1e-1 * I));

	Matrix mat3({{1.497857}, {1.5}});
	Matrix mat4({{0.000901, 0}, {0, 0.001}});
	ASSERT_ALLCLOSE_T(strategy.get_estimate(), mat3);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), mat4);

	meas_matrix = Matrix({{0, 1}});
	meas        = Vector({-0.1});

	strategy.update(StandardMeasurementModel(meas, meas_matrix, 10. * I));

	Vector expect({1.497857, 1.498402});
	Matrix mat6({{0.000901, 0}, {0, 0.001}});

	ASSERT_ALLCLOSE_T(strategy.get_estimate(), expect);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), mat6);
}

void test_changeable_standard_dynamics_model(StandardModelStrategy& strategy) {
	auto I     = eye(2, 2);
	auto noise = zeros(2, 2);

	strategy.propagate(StandardDynamicsModel(I, noise));

	Matrix mat({{1.5}, {1.5}});
	Matrix mat2({{0.0001, 0}, {0, 0.0001}});
	ASSERT_ALLCLOSE_T(strategy.get_estimate(), mat);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), mat2);

	strategy.propagate(StandardDynamicsModel(2 * I, noise));

	mat  = Matrix({{3.0}, {3.0}});
	mat2 = Matrix({{0.0004, 0}, {0, 0.0004}});
	ASSERT_ALLCLOSE_T(strategy.get_estimate(), mat);
	ASSERT_ALLCLOSE_T(strategy.get_covariance(), mat2);
}

void test_update_converges_zero_standard_dynamics_model_noise(StandardModelStrategy& strategy) {
	auto I             = eye(2, 2);
	auto noise         = zeros(2, 2);
	auto I2            = 0.0023 * eye(1, 1);
	Matrix meas_matrix = Matrix{{0, 1}};
	Vector meas        = Vector{10.};

	Vector mat{1.5, 10.};
	Matrix mat2{{1., 0}, {0, 0.002}};

	for (size_t i = 0; i < 60; i++) {
		strategy.propagate(StandardDynamicsModel(I, noise));
		strategy.update(StandardMeasurementModel(meas, meas_matrix, I2));
	}
	ASSERT_ALLCLOSE_L(strategy.get_estimate(), mat);
	ASSERT_ALLCLOSE_L(strategy.get_covariance(), mat2);
}

void test_bad_update_model_error(StandardModelStrategy& strategy) {
	int actual_xrows = num_rows(strategy.get_estimate());
	ASSERT_EQ(actual_xrows, strategy.get_num_states());
	spdlog::debug(actual_xrows);
	StandardMeasurementModel measurement_model(Vector{123.45678}, Matrix{{0, 1}}, eye(1, 1));

	try {
		strategy.update(measurement_model);
	} catch (std::runtime_error& e) {
		// When navtk errors are off, _our_ error checking is bypassed but xtensor's isn't, so catch
		// the error they'll throw rather than failing the test.
		if (navtk::get_global_error_mode() == navtk::ErrorMode::OFF) {
			ASSERT_STREQ("Dot: shape mismatch.", e.what());
		} else
			throw;
	}
}

void test_no_op_prop_symmetric(StandardModelStrategy& strategy) {
	int actual_xrows = num_rows(strategy.get_estimate());
	strategy.propagate(StandardDynamicsModel(eye(actual_xrows), zeros(actual_xrows, actual_xrows)));
	Matrix expected = Matrix{{2.0, 0.5}, {0.5, 3.0}};
	ASSERT_TRUE(allclose(expected, strategy.get_covariance(), 0, 1e-10));
}
