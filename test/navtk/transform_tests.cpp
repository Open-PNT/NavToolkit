#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/tensors.hpp>
#include <navtk/transform.hpp>

using namespace navtk;

TEST(TensorsTests, DropRangeSimpleCase) {
	Matrix input{{0, 1, 2, 3}, {5, 6, 7, 8}};
	Matrix output = xt::view(input, xt::all(), drop_range(1, 3));
	ASSERT_ALLCLOSE(Matrix({{0, 3}, {5, 8}}), output);
}

TEST(TensorsTests, DropRangeSteppedCase) {
	Vector input{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
	Vector output = xt::view(input, drop_range(0, 11, 2));
	ASSERT_ALLCLOSE(Vector({1, 3, 5, 7, 9}), output);
}

TEST(TensorsTests, DropRangeSimpleCase2) {
	Matrix input{{0, 1, 2, 3}, {5, 6, 7, 8}};
	Matrix output = xt::view(input, xt::all(), drop_range(0, 4));
	ASSERT_ALLCLOSE(Matrix({{}, {}}), output);
}

TEST(TensorsTests, DropRangeNonsenseRanges) {
	// -1 "overflows" to a large number when casted to size_t.
	EXPECT_UB_OR_DIE(drop_range(-1, 2), "start_val must be <= stop_val", std::invalid_argument);
	EXPECT_UB_OR_DIE(drop_range(0, 4, -1), "invalid step size", std::invalid_argument);
	EXPECT_UB_OR_DIE(drop_range(0, 4, 0), "invalid step size", std::invalid_argument);
}
