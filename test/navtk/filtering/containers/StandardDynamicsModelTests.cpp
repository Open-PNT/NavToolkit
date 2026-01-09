#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/tensors.hpp>

using navtk::Matrix;
using navtk::ones;
using navtk::Vector;
using navtk::filtering::StandardDynamicsModel;

struct StandardDynamicsModelTests : public ::testing::Test {
	Vector x                               = {2.0};
	std::function<Vector(const Vector&)> g = [](const Vector& x) { return x * 2; };
	Matrix Phi                             = {{2}};
	Matrix Qd                              = {{1}};

	void check_model_fields(StandardDynamicsModel model) {
		ASSERT_ALLCLOSE(g(x), model.g(x));
		ASSERT_ALLCLOSE(Phi, model.Phi);
		ASSERT_ALLCLOSE(Qd, model.Qd);
	}
};

TEST_F(StandardDynamicsModelTests, constructorEquivalence) {
	check_model_fields(StandardDynamicsModel{g, Phi, Qd});
	check_model_fields(StandardDynamicsModel{Phi, Qd});
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, StandardDynamicsModelTests, badMatrixSize) {
	EXPECT_HONORS_MODE_EX(
	    (void)StandardDynamicsModel(test.g, ones(2, 1), test.Qd), "dimension", std::range_error);
	EXPECT_HONORS_MODE_EX(
	    (void)StandardDynamicsModel(test.g, test.Phi, ones(2, 1)), "dimension", std::range_error);

	EXPECT_HONORS_MODE_EX(
	    (void)StandardDynamicsModel(ones(2, 1), test.Qd), "dimension", std::range_error);
	EXPECT_HONORS_MODE_EX(
	    (void)StandardDynamicsModel(test.Phi, ones(2, 1)), "dimension", std::range_error);
}
