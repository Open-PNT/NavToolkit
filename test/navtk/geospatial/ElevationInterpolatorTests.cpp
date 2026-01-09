#include <gtest/gtest.h>

#include <navtk/geospatial/ElevationInterpolator.hpp>

namespace navtk {
namespace geospatial {

TEST(ElevationInterpolator, TopLeft) {
	EXPECT_NEAR(
	    10,
	    ElevationInterpolator(10, 100, 200, 500).interpolate(std::pair<double, double>(0, 0)),
	    0.1);
}

TEST(ElevationInterpolator, TopRight) {
	EXPECT_NEAR(
	    100,
	    ElevationInterpolator(10, 100, 200, 500).interpolate(std::pair<double, double>(1, 0)),
	    0.1);
}

TEST(ElevationInterpolator, BottomLeft) {
	EXPECT_NEAR(
	    200,
	    ElevationInterpolator(10, 100, 200, 500).interpolate(std::pair<double, double>(0, 1)),
	    0.1);
}

TEST(ElevationInterpolator, BottomRight) {
	EXPECT_NEAR(
	    500,
	    ElevationInterpolator(10, 100, 200, 500).interpolate(std::pair<double, double>(1, 1)),
	    0.1);
}

TEST(ElevationInterpolator, CloseTopLeft) {
	EXPECT_NEAR(
	    40.1,
	    ElevationInterpolator(10, 100, 200, 500).interpolate(std::pair<double, double>(0.1, 0.1)),
	    0.1);
}

TEST(ElevationInterpolator, Somewhere) {
	EXPECT_NEAR(265.258,
	            ElevationInterpolator(10, 100, 200, 500)
	                .interpolate(std::pair<double, double>(0.3876, 0.812)),
	            0.1);
}
}  // namespace geospatial
}  // namespace navtk
