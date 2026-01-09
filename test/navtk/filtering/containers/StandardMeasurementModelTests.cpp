#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/tensors.hpp>

using navtk::Matrix;
using navtk::ones;
using navtk::Vector;
using navtk::filtering::StandardMeasurementModel;

struct StandardMeasurementModelTests : public ::testing::Test {
	Vector x                               = {1.0, 2.0};
	Vector z                               = {3.0};
	std::function<Vector(const Vector&)> h = [](const Vector& x) { return Vector{x(0) * 2}; };
	Matrix H                               = {{2, 0}};
	Matrix R                               = {{1}};

	void check_model_fields(StandardMeasurementModel model) {
		ASSERT_ALLCLOSE(z, model.z);
		ASSERT_ALLCLOSE(h(x), model.h(x));
		ASSERT_ALLCLOSE(H, model.H);
		ASSERT_ALLCLOSE(R, model.R);
	}
};

TEST_F(StandardMeasurementModelTests, constructorEquivalence) {
	check_model_fields(StandardMeasurementModel{z, h, H, R});
	check_model_fields(StandardMeasurementModel{z, H, R});
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, StandardMeasurementModelTests, badMatrixSize) {
	EXPECT_HONORS_MODE_EX((void)StandardMeasurementModel(ones(2), test.h, test.H, test.R),
	                      "dimension",
	                      std::range_error);
	EXPECT_HONORS_MODE_EX((void)StandardMeasurementModel(test.z, test.h, ones(2, 1), test.R),
	                      "dimension",
	                      std::range_error);
	EXPECT_HONORS_MODE_EX((void)StandardMeasurementModel(test.z, test.h, test.H, ones(2, 1)),
	                      "dimension",
	                      std::range_error);

	EXPECT_HONORS_MODE_EX(
	    (void)StandardMeasurementModel(ones(2), test.H, test.R), "dimension", std::range_error);
	EXPECT_HONORS_MODE_EX(
	    (void)StandardMeasurementModel(test.z, ones(2, 1), test.R), "dimension", std::range_error);
	EXPECT_HONORS_MODE_EX(
	    (void)StandardMeasurementModel(test.z, test.H, ones(2, 1)), "dimension", std::range_error);
}
