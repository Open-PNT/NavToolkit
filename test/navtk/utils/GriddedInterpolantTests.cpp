#include <gtest/gtest.h>
#include <error_mode_assert.hpp>

#include <navtk/factory.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/GriddedInterpolant.hpp>

using navtk::eye;
using navtk::Matrix;
using navtk::Vector;
using navtk::zeros;
using navtk::utils::GriddedInterpolant;

TEST(GriddedInterpolantTests, badConstruct) {
	EXPECT_UB_OR_DIE(GriddedInterpolant(zeros(2), zeros(3), eye(3)),
	                 "x_vector must have at least three elements",
	                 std::invalid_argument);
	EXPECT_UB_OR_DIE(GriddedInterpolant(zeros(3), zeros(2), eye(3)),
	                 "y_vector must have at least three elements",
	                 std::invalid_argument);
	EXPECT_UB_OR_DIE(GriddedInterpolant(zeros(3), zeros(3), eye(3)),
	                 "Vectors are not monotonically increasing",
	                 std::invalid_argument);
	EXPECT_UB_OR_DIE(GriddedInterpolant(Vector{1, 2, 4, 5}, Vector{1, 2, 3, 4}, eye(4)),
	                 "x_vector spacing not uniform",
	                 std::invalid_argument);
	EXPECT_UB_OR_DIE(GriddedInterpolant(Vector{1, 2, 3, 4}, Vector{1, 2, 4, 5}, eye(4)),
	                 "y_vector spacing not uniform",
	                 std::invalid_argument);
}

TEST(GriddedInterpolantTests, badInterpolate) {
	GriddedInterpolant grid(Vector{1, 2, 3}, Vector{1, 2, 3}, eye(3));
	EXPECT_UB_OR_DIE(
	    grid.interpolate(0, 20), "Query point must be inside grid", std::invalid_argument);
}