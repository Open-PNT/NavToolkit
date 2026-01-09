#include <gtest/gtest.h>

#include <navtk/utils/algorithm.hpp>

TEST(Algorithm, TrapezoidalArea) {
	auto val = navtk::utils::trapezoidal_area(10.0, 7.5, 11.0, 15.5);
	EXPECT_DOUBLE_EQ(val, 11.5);
}
