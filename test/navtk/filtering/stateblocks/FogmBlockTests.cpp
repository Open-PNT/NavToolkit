#include <memory>

#include <gtest/gtest.h>
#include <tensor_assert.hpp>

#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/stateblocks/FogmAccel.hpp>
#include <navtk/filtering/stateblocks/FogmBlock.hpp>
#include <navtk/filtering/stateblocks/FogmVelocity.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::_;
using navtk::expm;
using navtk::eye;
using navtk::Matrix;
using navtk::Vector;
using navtk::zeros;
using navtk::filtering::DiscretizationStrategy;
using navtk::filtering::FogmAccel;
using navtk::filtering::FogmBlock;
using navtk::filtering::FogmVelocity;
using navtk::filtering::NULL_GEN_XHAT_AND_P_FUNCTION;

TEST(FogmBlockTests, testMultiState_SLOW) {
	double tau   = 3.0;
	double sigma = 4.0;
	double dt    = 0.5;
	for (int num_states = 1; num_states <= 10; ++num_states) {
		// Update block and grab dynamics model.
		auto block = FogmBlock("a", tau, sigma, num_states);
		auto dyn   = block.generate_dynamics(
            NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));

		// Build expected Phi matrix.
		Matrix expected_phi = eye(num_states) * exp(-dt / tau);

		// Check is abs(actual - expected)) <= absolute_tolerance + expected*relative_tolerance
		// So these values result in a check of the percent difference
		auto absolute_tolerance = 0.0;
		auto relative_tolerance = 1e-4;

		EXPECT_ALLCLOSE_EX(expected_phi, dyn.Phi, relative_tolerance, absolute_tolerance);
	}
}

TEST(FogmBlockTests, testDifferentFOGMParameters) {
	Vector tau{3.0, 4.0, 5.0};
	Vector sigma{4.0, 5.0, 6.0};
	double dt = 0.3;
	for (int num_states = 1; num_states <= 3; ++num_states) {
		// Update block and grab dynamics model.
		auto block = FogmBlock("a", tau, sigma, num_states);
		auto dyn   = block.generate_dynamics(
            NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));

		// Build expected Phi matrix.
		Vector phi_elements = {exp(-dt / tau(0)), exp(-dt / tau(1)), exp(-dt / tau(2))};
		phi_elements        = xt::view(phi_elements, xt::range(_, num_states));
		Matrix expected_phi = xt::diag(phi_elements);

		// Check is abs(actual - expected)) <= absolute_tolerance + expected*relative_tolerance
		// So these values result in a check of the percent difference
		auto absolute_tolerance = 0.0;
		auto relative_tolerance = 1e-4;

		EXPECT_ALLCLOSE_EX(expected_phi, dyn.Phi, relative_tolerance, absolute_tolerance);
	}
}

template <typename T>
std::pair<Vector, Matrix> run_steady_state_test(T&& block,
                                                double dt,
                                                size_t states,
                                                size_t iterations = 10000) {
	auto xhat = zeros(states);
	auto P    = zeros(states, states);
	auto dyn  = block.generate_dynamics(
        NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));
	for (decltype(iterations) it = 0; it < iterations; ++it) {
		xhat = navtk::dot(dyn.Phi, xhat);
		P    = navtk::dot(navtk::dot(dyn.Phi, P), xt::transpose(dyn.Phi)) + dyn.Qd;
	}
	return {std::move(xhat), std::move(P)};
}

TEST(FogmBlockTests, testFogmBlockSteadyState) {
	Vector tau{3.0, 4.0, 5.0};
	Vector sigma{4.0, 5.0, 6.0};
	double dt = 0.3;

	// Update block and grab dynamics model.
	auto block   = FogmBlock("a", tau, sigma, 3);
	auto results = run_steady_state_test(block, dt, 3);
	auto xhat    = results.first;
	auto P       = results.second;

	// Build expected xhat and P matrix.
	Vector expected_xhat = zeros(3);
	Vector p_elements    = {sigma(0) * sigma(0), sigma(1) * sigma(1), sigma(2) * sigma(2)};
	Matrix expected_p    = xt::diag(p_elements);

	// Check is abs(actual - expected)) <= absolutionTolerance + expected*relative_tolerance
	// So these values result in a check of the percent difference
	auto absolute_tolerance = 0.0;
	auto relative_tolerance = 1e-4;

	EXPECT_ALLCLOSE_EX(expected_xhat, xhat, relative_tolerance, absolute_tolerance);
	EXPECT_ALLCLOSE_EX(expected_p, P, relative_tolerance, absolute_tolerance);
}

TEST(FogmBlockTests, testVelocityFOGM) {
	double tau   = 3.0;
	double sigma = 4.0;
	double dt    = 0.5;

	for (size_t dim = 2; dim < 4; ++dim) {
		// Update block and grab dynamics model.
		auto block = FogmVelocity("a", tau, sigma, dim);
		auto dyn   = block.generate_dynamics(
            NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));

		// Build expected Phi matrix.
		Matrix expected_f                                          = zeros(dim * 2, dim * 2);
		xt::view(expected_f, xt::range(_, dim), xt::range(dim, _)) = eye(dim);
		xt::view(expected_f, xt::range(dim, _), xt::range(dim, _)) = eye(dim) * (-1.0 / tau);
		Matrix expected_phi                                        = expm(expected_f * dt);

		// Check is abs(actual - expected)) <= absolute_tolerance + expected*relative_tolerance
		// So these values result in a check of the percent difference
		auto absolute_tolerance = 0.0;
		auto relative_tolerance = 1e-4;

		EXPECT_ALLCLOSE_EX(expected_phi, dyn.Phi, relative_tolerance, absolute_tolerance);
	}
}

TEST(FogmBlockTests, testVelocityFOGMSteadyState) {
	Vector tau{3.0, 4.0, 5.0};
	Vector sigma{4.0, 5.0, 6.0};
	double dt = 0.3;

	// Update block and grab dynamics model.
	auto block   = FogmVelocity("a", tau, sigma, 3);
	auto results = run_steady_state_test(block, dt, 6);
	auto xhat    = results.first;
	auto P       = results.second;

	// Build expected xhat and P matrix.
	Vector expected_xhat = zeros(3);
	Vector p_elements    = {sigma(0) * sigma(0), sigma(1) * sigma(1), sigma(2) * sigma(2)};
	Matrix expected_p    = xt::diag(p_elements);

	// Check is abs(actual - expected)) <= absolute_tolerance + expected*relative_tolerance
	// So these values result in a check of the percent difference
	auto absolute_tolerance = 0.0;
	auto relative_tolerance = 1e-4;

	EXPECT_ALLCLOSE_EX(expected_xhat,
	                   Vector(xt::view(xhat, xt::range(3, _))),
	                   relative_tolerance,
	                   absolute_tolerance);
	EXPECT_ALLCLOSE_EX(expected_p,
	                   Matrix(xt::view(P, xt::range(3, _), xt::range(3, _))),
	                   relative_tolerance,
	                   absolute_tolerance);
}

TEST(FogmBlockTests, testAccelFOGM) {
	double tau   = 3.0;
	double sigma = 4.0;
	double dt    = 0.5;

	for (size_t dim = 2; dim < 4; ++dim) {
		// Update block and grab dynamics model.
		auto block = FogmAccel("a", tau, sigma, dim);
		auto dyn   = block.generate_dynamics(
            NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(dt));

		// Build expected Phi matrix.
		Matrix expected_f                                                = zeros(dim * 3, dim * 3);
		xt::view(expected_f, xt::range(_, dim), xt::range(dim, dim * 2)) = eye(dim);
		xt::view(expected_f, xt::range(dim, dim * 2), xt::range(dim * 2, _)) = eye(dim);
		xt::view(expected_f, xt::range(dim * 2, _), xt::range(dim * 2, _)) =
		    eye(dim) * (-1.0 / tau);
		Matrix expected_phi = expm(expected_f * dt);

		// Check is abs(actual - expected)) <= absolute_tolerance + expected*relative_tolerance
		// So these values result in a check of the percent difference
		auto absolute_tolerance = 0.0;
		auto relative_tolerance = 1e-4;

		EXPECT_ALLCLOSE_EX(expected_phi, dyn.Phi, relative_tolerance, absolute_tolerance);
	}
}

TEST(FogmBlockTests, testAccelFOGMSteadyState) {
	Vector tau{3.0, 4.0, 5.0};
	Vector sigma{4.0, 5.0, 6.0};
	double dt = 0.3;

	// Update block and grab dynamics model.
	auto block   = FogmAccel("a", tau, sigma, 3);
	auto results = run_steady_state_test(block, dt, 9);
	auto xhat    = results.first;
	auto P       = results.second;

	// Build expected xhat and P matrix.
	Vector expected_xhat = zeros(3);
	Vector p_elements    = {sigma(0) * sigma(0), sigma(1) * sigma(1), sigma(2) * sigma(2)};
	Matrix expected_p    = xt::diag(p_elements);

	// Check is abs(actual - expected)) <= absolute_tolerance + expected*relative_tolerance
	// So these values result in a check of the percent difference
	auto absolute_tolerance = 0.0;
	auto relative_tolerance = 1e-4;

	EXPECT_ALLCLOSE_EX(expected_xhat,
	                   Vector(xt::view(xhat, xt::range(6, _))),
	                   relative_tolerance,
	                   absolute_tolerance);
	EXPECT_ALLCLOSE_EX(expected_p,
	                   Matrix(xt::view(P, xt::range(6, _), xt::range(6, _))),
	                   relative_tolerance,
	                   absolute_tolerance);
}

class TestableFOGMB : public FogmBlock {
public:
	TestableFOGMB(const std::string& label,
	              Vector time_constants,
	              Vector state_sigmas,
	              Vector::shape_type::value_type num_states,
	              DiscretizationStrategy discretization_strategy)
	    : FogmBlock(label, time_constants, state_sigmas, num_states, discretization_strategy) {}

	TestableFOGMB(const TestableFOGMB& block) : FogmBlock(block) {}

	navtk::not_null<std::shared_ptr<StateBlock<>>> clone() {
		return std::make_shared<TestableFOGMB>(*this);
	}

	using FogmBlock::discretization_strategy;
	using FogmBlock::state_sigmas;
	using FogmBlock::time_constants;
};

class TestableFOGMV : public FogmVelocity {
public:
	TestableFOGMV(const std::string& label,
	              Vector time_constants,
	              Vector state_sigmas,
	              Vector::shape_type::value_type num_states,
	              DiscretizationStrategy discretization_strategy)
	    : FogmVelocity(label, time_constants, state_sigmas, num_states, discretization_strategy) {}

	TestableFOGMV(const TestableFOGMV& block) : FogmVelocity(block) {}

	navtk::not_null<std::shared_ptr<StateBlock<>>> clone() {
		return std::make_shared<TestableFOGMV>(*this);
	}

	using FogmVelocity::discretization_strategy;
	using FogmVelocity::state_sigmas;
	using FogmVelocity::time_constants;
};

class TestableFOGMA : public FogmAccel {
public:
	TestableFOGMA(const std::string& label,
	              Vector time_constants,
	              Vector state_sigmas,
	              Vector::shape_type::value_type num_states,
	              DiscretizationStrategy discretization_strategy)
	    : FogmAccel(label, time_constants, state_sigmas, num_states, discretization_strategy) {}

	TestableFOGMA(const TestableFOGMA& block) : FogmAccel(block) {}

	navtk::not_null<std::shared_ptr<StateBlock<>>> clone() {
		return std::make_shared<TestableFOGMA>(*this);
	}

	using FogmAccel::discretization_strategy;
	using FogmAccel::state_sigmas;
	using FogmAccel::time_constants;
};

TEST(FogmBlockTests, test_clone_FOGM) {
	auto block = TestableFOGMB(
	    "block", Vector{1}, Vector{1}, 1, &navtk::filtering::full_order_discretization_strategy);
	auto block_copy_cast = std::dynamic_pointer_cast<TestableFOGMB>(block.clone());
	auto& block_copy     = *block_copy_cast;

	ASSERT_EQ(block.get_label(), block_copy.get_label());
	ASSERT_EQ(block.get_num_states(), block_copy.get_num_states());

	// Compare the function pointer address
	typedef std::pair<Matrix, Matrix>(FnType)(
	    const Matrix& F, const Matrix& G, const Matrix& Q, double dt);
	ASSERT_EQ((size_t)*block.discretization_strategy.template target<FnType*>(),
	          (size_t)*block_copy.discretization_strategy.template target<FnType*>());

	// Validate clone() as implemented returns a deep copy by modifying all of the clone's public
	// properties and asserting they do not equal the original's.
	ASSERT_EQ(block.state_sigmas, block_copy.state_sigmas);
	block_copy.state_sigmas += 1;
	ASSERT_NE(block.state_sigmas, block_copy.state_sigmas);

	ASSERT_EQ(block.time_constants, block_copy.time_constants);
	block_copy.time_constants += 1;
	ASSERT_NE(block.time_constants, block_copy.time_constants);
}

TEST(FogmBlockTests, test_clone_FOGM_vel) {
	auto block = TestableFOGMV(
	    "block", Vector{1}, Vector{1}, 1, &navtk::filtering::full_order_discretization_strategy);
	auto block_copy_cast = std::dynamic_pointer_cast<TestableFOGMV>(block.clone());
	auto& block_copy     = *block_copy_cast;

	// Validate clone() as implemented returns a deep copy by modifying all of the clone's public
	// properties and asserting they do not equal the original's.
	ASSERT_EQ(block.get_label(), block_copy.get_label());
	ASSERT_EQ(block.get_num_states(), block_copy.get_num_states());

	// Compare the function pointer address
	typedef std::pair<Matrix, Matrix>(FnType)(
	    const Matrix& F, const Matrix& G, const Matrix& Q, double dt);
	ASSERT_EQ((size_t)*block.discretization_strategy.template target<FnType*>(),
	          (size_t)*block_copy.discretization_strategy.template target<FnType*>());

	ASSERT_EQ(block.state_sigmas, block_copy.state_sigmas);
	block_copy.state_sigmas += 1;
	ASSERT_NE(block.state_sigmas, block_copy.state_sigmas);

	ASSERT_EQ(block.time_constants, block_copy.time_constants);
	block_copy.time_constants += 1;
	ASSERT_NE(block.time_constants, block_copy.time_constants);
}

TEST(FogmBlockTests, test_clone_FOGM_accel) {
	auto block = TestableFOGMA(
	    "block", Vector{1}, Vector{1}, 1, &navtk::filtering::full_order_discretization_strategy);
	auto block_copy_cast = std::dynamic_pointer_cast<TestableFOGMA>(block.clone());
	auto& block_copy     = *block_copy_cast;

	// Validate clone() as implemented returns a deep copy by modifying all of the clone's public
	// properties and asserting they do not equal the original's.
	ASSERT_EQ(block.get_label(), block_copy.get_label());
	ASSERT_EQ(block.get_num_states(), block_copy.get_num_states());

	// Compare the function pointer address
	typedef std::pair<Matrix, Matrix>(FnType)(
	    const Matrix& F, const Matrix& G, const Matrix& Q, double dt);
	ASSERT_EQ((size_t)*block.discretization_strategy.template target<FnType*>(),
	          (size_t)*block_copy.discretization_strategy.template target<FnType*>());

	ASSERT_EQ(block.state_sigmas, block_copy.state_sigmas);
	block_copy.state_sigmas += 1;
	ASSERT_NE(block.state_sigmas, block_copy.state_sigmas);

	ASSERT_EQ(block.time_constants, block_copy.time_constants);
	block_copy.time_constants += 1;
	ASSERT_NE(block.time_constants, block_copy.time_constants);
}

TEST(FogmBlockTests, AccelCloneIsNotObjectSlice) {
	FogmAccel block("Accel", 1.0, 1.0, 2);
	auto clone = block.clone();
	auto dyn   = block.generate_dynamics(
        NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(1.0), to_type_timestamp(2.0));
	auto clone_dyn = clone->generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(1.0), to_type_timestamp(2.0));
	auto& clone_ref = *clone;
	EXPECT_EQ(std::string(typeid(block).name()), std::string(typeid(clone_ref).name()));
	EXPECT_ALLCLOSE(dyn.Qd, clone_dyn.Qd);
}

TEST(FogmBlockTests, VeloCloneIsNotObjectSlice) {
	FogmVelocity block("Velocity", 1.0, 1.0, 2);
	auto clone = block.clone();
	auto dyn   = block.generate_dynamics(
        NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(1.0), to_type_timestamp(2.0));
	auto clone_dyn = clone->generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(1.0), to_type_timestamp(2.0));
	auto& clone_ref = *clone;
	EXPECT_EQ(std::string(typeid(block).name()), std::string(typeid(clone_ref).name()));
	EXPECT_ALLCLOSE(dyn.Qd, clone_dyn.Qd);
}
