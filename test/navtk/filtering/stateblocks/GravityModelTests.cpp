#include <gtest/gtest.h>

#include <navtk/filtering/stateblocks/EarthModel.hpp>
#include <navtk/filtering/stateblocks/GravityModel.hpp>
#include <navtk/filtering/stateblocks/GravityModelSchwartz.hpp>
#include <navtk/filtering/stateblocks/GravityModelTittertonAndWeston.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/tensors.hpp>

using navtk::Vector3;
using navtk::filtering::EarthModel;
using navtk::filtering::GravityModel;
using navtk::filtering::GravityModelSchwartz;
using navtk::filtering::GravityModelTittertonAndWeston;
using navtk::navutils::PI;
using std::abs;

TEST(GravityModelTests, garbage) {
	const GravityModel &gm  = GravityModelTittertonAndWeston();
	const GravityModel &gm2 = GravityModelSchwartz();

	EarthModel point1(Vector3{6.5 * PI / 180.0, 0.0, 1000.0}, Vector3{0.0, 0.0, 0.0}, gm);
	EarthModel point2(Vector3{50.0 * PI / 180.0, 0.0, 1000.0}, Vector3{0.0, 0.0, 0.0}, gm);
	EarthModel point3(Vector3{84.0 * PI / 180.0, 0.0, 1000.0}, Vector3{0.0, 0.0, 0.0}, gm);

	EarthModel point4(Vector3{6.5 * PI / 180.0, 0.0, 1000.0}, Vector3{0.0, 0.0, 0.0}, gm2);
	EarthModel point5(Vector3{50.0 * PI / 180.0, 0.0, 1000.0}, Vector3{0.0, 0.0, 0.0}, gm2);
	EarthModel point6(Vector3{84.0 * PI / 180.0, 0.0, 1000.0}, Vector3{0.0, 0.0, 0.0}, gm2);

	// Test values taken from https://www.sensorsone.com/local-gravity-calculator/. It references
	// GRS80, but values are coarse enough that they can be used for both models.
	ASSERT_TRUE(std::abs(gm.calculate_gravity(point1, 1000.0) - 9.77790) < 1e-5);
	ASSERT_TRUE(std::abs(gm.calculate_gravity(point2, 1000.0) - 9.80762) < 1e-5);
	ASSERT_TRUE(std::abs(gm.calculate_gravity(point3, 1000.0) - 9.82853) < 1e-5);

	ASSERT_TRUE(std::abs(gm2.calculate_gravity(point4, 1000.0) - 9.77790) < 1e-5);
	ASSERT_TRUE(std::abs(gm2.calculate_gravity(point5, 1000.0) - 9.80762) < 1e-5);
	ASSERT_TRUE(std::abs(gm2.calculate_gravity(point6, 1000.0) - 9.82853) < 1e-5);
}

// TODO: This tests current behavior, which linearly scales gravity at altitudes
// below sea level based on a fraction of radius. The reality is definitely
// more complicated, though in practice it doesn't matter much for the spaces
// we operate in.
TEST(GravityModelTests, belowSeaLevel) {
	const GravityModel &gm = GravityModelTittertonAndWeston();
	EarthModel point1(Vector3{6.5 * PI / 180.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}, gm);
	ASSERT_TRUE(gm.calculate_gravity(point1, -0.5 * point1.r_zero) ==
	            0.5 * gm.calculate_gravity(point1, 0.0));
	ASSERT_TRUE(gm.calculate_gravity(point1, -point1.r_zero) == 0.0);
}
