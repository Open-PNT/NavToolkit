#include <gtest/gtest.h>
#include <tensor_assert.hpp>
#include <xtensor/misc/xmanipulation.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/leverarms.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/human_readable.hpp>

using navtk::dot;
using navtk::eye;
using navtk::Matrix3;
using navtk::to_vec;
using navtk::Vector3;
using navtk::zeros;
using navtk::navutils::obs_in_platform_to_sensor;
using navtk::navutils::obs_in_sensor_to_platform;
using navtk::navutils::PI;
using navtk::navutils::platform_to_sensor;
using navtk::navutils::rpy_to_dcm;
using navtk::navutils::sensor_to_platform;
using navtk::utils::repr;
using xt::transpose;

struct LeverArmTests : public ::testing::Test {
public:
	Vector3 sensor_pos;
	Matrix3 C_nav_to_sensor;
	Vector3 lever_arm;
	Matrix3 orientation;
	Matrix3 C_j_to_i;
	std::pair<Vector3, Matrix3> start;
	Vector3 arm0;
	Matrix3 I;


	std::pair<Vector3, Matrix3> home_in_e;
	std::pair<Vector3, Matrix3> s1_in_e;
	std::pair<Vector3, Matrix3> s1_in_h;
	std::pair<Vector3, Matrix3> s2_in_e;
	std::pair<Vector3, Matrix3> s2_in_h;
	std::pair<Vector3, Matrix3> point_in_e;
	std::pair<Vector3, Matrix3> point_in_h;
	std::pair<Vector3, Matrix3> point_in_s1;
	std::pair<Vector3, Matrix3> point_in_s2;

	std::pair<Vector3, Matrix3> PinA;
	std::pair<Vector3, Matrix3> PinB;
	std::pair<Vector3, Matrix3> PinC;
	std::pair<Vector3, Matrix3> toA;
	std::pair<Vector3, Matrix3> toC;

	LeverArmTests()
	    : ::testing::Test(),
	      sensor_pos({62383.14, -766952.2344, 16531.23663}),
	      C_nav_to_sensor(xt::transpose(rpy_to_dcm({0.334, -1.6536, 0.746528}))),
	      lever_arm({1212.1, 4234.244, 124141.2341}),
	      orientation(xt::transpose(rpy_to_dcm({0.44542, -2.213215, -1.232141}))),
	      C_j_to_i(xt::transpose(rpy_to_dcm({-1.8238, 0.823724, 1.19883}))),
	      start({sensor_pos, C_nav_to_sensor}),
	      arm0({0.0, 0.0, 0.0}),
	      I(eye(3)),


	      home_in_e({{5, 10, 12}, {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}}),
	      s1_in_e({{1, 13, 12}, {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}}}),
	      s1_in_h({{3, -4, 0}, {{0, -1, 0}, {0, 0, -1}, {1, 0, 0}}}),
	      s2_in_e({{13, 10, 10}, {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}}),
	      s2_in_h({{0, 8, 2}, eye(3)}),
	      point_in_e({{20, 15, 0}, transpose(Matrix3{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}})}),
	      point_in_h({{5, 15, 12}, eye(3)}),
	      point_in_s1({{-19, -12, 2}, transpose(Matrix3{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}})}),
	      point_in_s2({Vector3{5, 7, 10}, eye(3)}),

	      PinA({Vector3{1, 2, -3}, eye(3)}),
	      PinB({Vector3{8, 5, 6}, {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}}),
	      PinC({Vector3{10, -13, -8}, {{0, 0, -1}, {0, -1, 0}, {-1, 0, 0}}}),
	      toA({Vector3{6, 4, 3}, {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}}),
	      toC({Vector3{-5, -3, -4}, {{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}}) {}
};

/**
 * Convenience function for 'swapping' the reference frame and observation
 * frame of a lever arm/orientation pair.
 *
 * @param orig A Vector3 pointing from the origin of frame A to the
 * origin of frame B in frame A coordinates, and the DCM that rotates
 * from frame A to frame B. Units on the lever arm should be consistent
 * across elements.
 *
 * @return A Vector3 that points from the frame B origin to the frame A
 * origin in frame B coordinates, and the DCM that rotates from frame B
 * to frame A.
 */
std::pair<Vector3, Matrix3> swap_frame(std::pair<Vector3, Matrix3> orig) {
	return {to_vec(dot(orig.second, -orig.first)), transpose(orig.second)};
}

TEST_F(LeverArmTests, PointInSameFrame) {
	auto res1 = obs_in_sensor_to_platform(PinA, zeros(3), eye(3));
	auto res2 = obs_in_platform_to_sensor(PinA, zeros(3), eye(3));
	auto res3 = sensor_to_platform(PinA, zeros(3), eye(3));
	auto res4 = platform_to_sensor(PinA, zeros(3), eye(3));
	ASSERT_ALLCLOSE(res1.first, PinA.first);
	ASSERT_ALLCLOSE(res1.second, PinA.second);
	ASSERT_ALLCLOSE(res2.first, PinA.first);
	ASSERT_ALLCLOSE(res2.second, PinA.second);
	ASSERT_ALLCLOSE(res3.first, PinA.first);
	ASSERT_ALLCLOSE(res3.second, PinA.second);
	ASSERT_ALLCLOSE(res4.first, PinA.first);
	ASSERT_ALLCLOSE(res4.second, PinA.second);
}

TEST_F(LeverArmTests, PointInPlatformFromA) {
	auto res1 = obs_in_sensor_to_platform(PinA, toA.first, toA.second);
	ASSERT_ALLCLOSE(res1.first, PinB.first);
	ASSERT_ALLCLOSE(res1.second, PinB.second);
}

// Single hops
TEST_F(LeverArmTests, PointInPlatformFromC) {
	auto res1 = obs_in_sensor_to_platform(PinC, toC.first, toC.second);
	ASSERT_ALLCLOSE(res1.first, PinB.first);
	ASSERT_ALLCLOSE(res1.second, PinB.second);
}

TEST_F(LeverArmTests, PointInCFromPlatform) {
	auto res1 = obs_in_platform_to_sensor(PinB, toC.first, toC.second);
	ASSERT_ALLCLOSE(res1.first, PinC.first);
	ASSERT_ALLCLOSE(res1.second, PinC.second);
}

TEST_F(LeverArmTests, PointInAFromPlatform) {
	auto res1 = obs_in_platform_to_sensor(PinB, toA.first, toA.second);
	ASSERT_ALLCLOSE(res1.first, PinA.first);
	ASSERT_ALLCLOSE(res1.second, PinA.second);
}

// Double hops
TEST_F(LeverArmTests, PointInCFromA) {
	auto res1 = obs_in_platform_to_sensor(
	    obs_in_sensor_to_platform(PinA, toA.first, toA.second), toC.first, toC.second);
	ASSERT_ALLCLOSE(res1.first, PinC.first);
	ASSERT_ALLCLOSE(res1.second, PinC.second);
}

TEST_F(LeverArmTests, PointInAFromC) {
	auto res1 = obs_in_platform_to_sensor(
	    obs_in_sensor_to_platform(PinC, toC.first, toC.second), toA.first, toA.second);
	ASSERT_ALLCLOSE(res1.first, PinA.first);
	ASSERT_ALLCLOSE(res1.second, PinA.second);
}

TEST_F(LeverArmTests, RecoverLA) {
	auto res1 = obs_in_sensor_to_platform({zeros(3), eye(3)}, toA.first, toA.second);
	ASSERT_ALLCLOSE(res1.first, toA.first);
	ASSERT_ALLCLOSE(res1.second, toA.second);
}

TEST_F(LeverArmTests, RecoverLC) {
	auto res1 = obs_in_sensor_to_platform({zeros(3), eye(3)}, toC.first, toC.second);
	ASSERT_ALLCLOSE(res1.first, toC.first);
	ASSERT_ALLCLOSE(res1.second, toC.second);
}

TEST_F(LeverArmTests, RecoverLAB) {
	auto res1      = obs_in_platform_to_sensor({zeros(3), eye(3)}, toA.first, toA.second);
	auto swapped_a = swap_frame(toA);
	ASSERT_ALLCLOSE(res1.first, swapped_a.first);
	ASSERT_ALLCLOSE(res1.second, swapped_a.second);
}

TEST_F(LeverArmTests, RecoverLAC) {
	auto res1      = obs_in_platform_to_sensor({zeros(3), eye(3)}, toC.first, toC.second);
	auto swapped_c = swap_frame(toC);
	ASSERT_ALLCLOSE(res1.first, swapped_c.first);
	ASSERT_ALLCLOSE(res1.second, swapped_c.second);
}

TEST_F(LeverArmTests, InertialEx) {
	// Time varying
	auto C_i_e = xt::transpose(rpy_to_dcm({0.5, 1.2, 0.6}));  // C_I_E
	Vector3 l_ei_e{10000.0, 20033.0, 736132.0};               // L_EI_E

	// Fixed, lever arm between inertial and platform
	auto C_i_p = xt::transpose(rpy_to_dcm({-1.0, -1.5, 0.2}));
	Vector3 l_pi_p{30.0, 15.0, -22.0};

	// Expected platform frame wrt earth
	auto C_p_e  = dot(transpose(C_i_p), C_i_e);
	auto l_eb_e = l_ei_e + to_vec(dot(dot(transpose(C_i_e), C_i_p), -l_pi_p));

	// We don't have a platform to earth (that is what we are trying to calculate)
	// We can take an observation of the earth frame in the inertial frame and
	// find that in the platform frame, and then call swap_frame to switch the inertial/earth
	// references
	auto platform_obs_in_earth = sensor_to_platform({l_ei_e, C_i_e}, l_pi_p, C_i_p);

	// Can get earth pos in platform, equivalent to a 'lever arm' to the earth frame
	auto earth_obs_in_platform = swap_frame(platform_obs_in_earth);

	ASSERT_ALLCLOSE(platform_obs_in_earth.first, l_eb_e);
	ASSERT_ALLCLOSE(platform_obs_in_earth.second, C_p_e);

	// Get the earth frame as an 'observation' in the inertial sensor frame
	auto earth_in_inertial = obs_in_platform_to_sensor(earth_obs_in_platform, l_pi_p, C_i_p);

	ASSERT_ALLCLOSE(earth_in_inertial.first, to_vec(dot(C_i_e, -l_ei_e)));
	ASSERT_ALLCLOSE(earth_in_inertial.second, transpose(C_i_e));
}

TEST_F(LeverArmTests, TestSamePointInNewFrame) {
	auto e_in_home = swap_frame(home_in_e);
	auto res1      = obs_in_platform_to_sensor(
        obs_in_sensor_to_platform(point_in_s2, s2_in_h.first, s2_in_h.second),
        s1_in_h.first,
        s1_in_h.second);
	ASSERT_ALLCLOSE(res1.first, point_in_s1.first);
	ASSERT_ALLCLOSE(res1.second, point_in_s1.second);

	auto res2 = obs_in_platform_to_sensor(
	    obs_in_sensor_to_platform(point_in_s1, s1_in_h.first, s1_in_h.second),
	    s2_in_h.first,
	    s2_in_h.second);
	ASSERT_ALLCLOSE(res2.first, point_in_s2.first);
	ASSERT_ALLCLOSE(res2.second, point_in_s2.second);

	auto res3 = obs_in_platform_to_sensor(
	    obs_in_sensor_to_platform(point_in_s2, s2_in_h.first, s2_in_h.second),
	    e_in_home.first,
	    e_in_home.second);
	ASSERT_ALLCLOSE(res3.first, point_in_e.first);
	ASSERT_ALLCLOSE(res3.second, point_in_e.second);

	auto res4 = obs_in_sensor_to_platform(point_in_s2, s2_in_h.first, s2_in_h.second);
	ASSERT_ALLCLOSE(res4.first, point_in_h.first);
	ASSERT_ALLCLOSE(res4.second, point_in_h.second);
}

TEST_F(LeverArmTests, TestNonDef) {
	Vector3 orig{1.0, 2.0, 3.0};
	auto rot = xt::transpose(rpy_to_dcm({1.0, 2.0, 3.0}));
	std::pair<Vector3, Matrix3> my_boy{orig, rot};
	ASSERT_ALLCLOSE(my_boy.first, orig);
	ASSERT_ALLCLOSE(my_boy.second, rot);
}

TEST_F(LeverArmTests, TestSwappers) {
	std::pair<Vector3, Matrix3> def;
	ASSERT_ALLCLOSE(def.first, swap_frame(def).first);
	ASSERT_ALLCLOSE(def.second, swap_frame(def).second);

	std::pair<Vector3, Matrix3> shifty{{1.0, 2.0, 3.0}, eye(3)};
	Vector3 shifty_exp{-1.0, -2.0, -3.0};
	ASSERT_ALLCLOSE(shifty_exp, swap_frame(shifty).first);

	std::pair<Vector3, Matrix3> rotted{{1.0, 2.0, 3.0}, {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}};
	Vector3 rotted_exp_vec{-2.0, -1.0, 3.0};
	Matrix3 rotted_exp_mat{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}};  // It just happens to be the same
	ASSERT_ALLCLOSE(rotted_exp_vec, swap_frame(rotted).first);
	ASSERT_ALLCLOSE(rotted_exp_mat, swap_frame(rotted).second);
	// Should be invertible
	ASSERT_ALLCLOSE(rotted.first, swap_frame(swap_frame(rotted)).first);
	ASSERT_ALLCLOSE(rotted.second, swap_frame(swap_frame(rotted)).second);
}


TEST_F(LeverArmTests, ReversibleSharedFrameIdentity) {
	auto there = sensor_to_platform({sensor_pos, I}, lever_arm, orientation);
	auto back  = platform_to_sensor(there, lever_arm, orientation);
	ASSERT_ALLCLOSE(sensor_pos, back.first);
	ASSERT_ALLCLOSE(I, back.second);
}

TEST_F(LeverArmTests, ReversibleSharedFrame) {
	auto there = sensor_to_platform(start, lever_arm, orientation);
	auto back  = platform_to_sensor(there, lever_arm, orientation);
	ASSERT_ALLCLOSE(sensor_pos, back.first);
	ASSERT_ALLCLOSE(C_nav_to_sensor, back.second);
}

TEST_F(LeverArmTests, ReversibleDiffFrame) {
	auto there =
	    sensor_to_platform({sensor_pos, C_nav_to_sensor}, lever_arm, orientation, C_j_to_i);
	auto back = platform_to_sensor(there, lever_arm, orientation, C_j_to_i);
	ASSERT_ALLCLOSE(sensor_pos, back.first);
	ASSERT_ALLCLOSE(C_nav_to_sensor, back.second);
}

TEST_F(LeverArmTests, NoOpSharedFrame) {
	auto tx1 = sensor_to_platform({sensor_pos, C_nav_to_sensor}, arm0, I);
	auto tx2 = platform_to_sensor({sensor_pos, C_nav_to_sensor}, arm0, I);
	ASSERT_ALLCLOSE(sensor_pos, tx1.first);
	ASSERT_ALLCLOSE(C_nav_to_sensor, tx1.second);
	ASSERT_ALLCLOSE(sensor_pos, tx2.first);
	ASSERT_ALLCLOSE(C_nav_to_sensor, tx2.second);
}

TEST_F(LeverArmTests, NoOpDiffFrame) {
	auto tx1 = sensor_to_platform({sensor_pos, C_nav_to_sensor}, arm0, I, C_j_to_i);
	auto tx2 = platform_to_sensor({sensor_pos, C_nav_to_sensor}, arm0, I, C_j_to_i);
	ASSERT_ALLCLOSE(sensor_pos, tx1.first);
	ASSERT_ALLCLOSE(C_nav_to_sensor, tx1.second);
	ASSERT_ALLCLOSE(sensor_pos, tx2.first);
	ASSERT_ALLCLOSE(C_nav_to_sensor, tx2.second);
}

TEST_F(LeverArmTests, PlatformAlignedAtOrigin) {
	auto tx1 = sensor_to_platform({sensor_pos, I}, sensor_pos, I);
	Vector3 expected_platform_pos{0.0, 0.0, 0.0};
	ASSERT_ALLCLOSE(expected_platform_pos, tx1.first);
	ASSERT_ALLCLOSE(I, tx1.second);
	auto tx2 = platform_to_sensor(tx1, lever_arm, C_nav_to_sensor);
	ASSERT_ALLCLOSE(lever_arm, tx2.first);
	ASSERT_ALLCLOSE(C_nav_to_sensor, tx2.second);
}

TEST_F(LeverArmTests, HandWorkedValuesIdentityRot) {
	Vector3 pos_platform{10.0, 20.0, 30.0};
	Vector3 pos_sensor1{14.0, 23.0, 30.0};
	Vector3 pos_sensor2{12.0, 18.0, 33.0};
	Matrix3 csensor1_ref  = xt::transpose(rpy_to_dcm({0.0, 0.0, PI / 2.0}));
	Matrix3 csensor2_ref  = xt::transpose(rpy_to_dcm({-PI / 2.0, 0.0, -PI / 4.0}));
	Matrix3 cplatform_ref = I;

	Vector3 lever1{4.0, 3.0, 0.0};
	Vector3 lever2{2.0, -2.0, 3.0};
	Matrix3 cenu_ned{{0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}};
	Matrix3 csensor1_platform = xt::transpose(rpy_to_dcm({0.0, 0.0, PI / 2.0}));
	Matrix3 csensor2_platform = xt::transpose(rpy_to_dcm({-PI / 2.0, 0.0, -PI / 4.0}));

	auto tx1 = sensor_to_platform({pos_sensor1, csensor1_ref}, lever1, csensor1_platform);
	ASSERT_ALLCLOSE(pos_platform, tx1.first);
	ASSERT_ALLCLOSE(cplatform_ref, tx1.second);
	auto tx2 = platform_to_sensor(tx1, lever2, csensor2_platform);
	ASSERT_ALLCLOSE(pos_sensor2, tx2.first);
	ASSERT_ALLCLOSE(csensor2_ref, tx2.second);
}

TEST_F(LeverArmTests, HandWorkedValuesComplex) {
	Vector3 lever1{4.0, 3.0, 0.0};
	Vector3 lever2{2.0, -2.0, 3.0};

	Vector3 pos_platform{10.0, 20.0, 30.0};
	double platform_heading = PI / 8.0;

	// How platform frame is oriented wrt ref. Only heading allowed in this test
	// because of how we are adding in Csensor initialization below
	Matrix3 C_nav_to_platform = xt::transpose(rpy_to_dcm({0.0, 0.0, platform_heading}));
	// Put lever arms in ref frame
	Vector3 rot1 = dot(transpose(C_nav_to_platform), lever1);
	Vector3 rot2 = dot(transpose(C_nav_to_platform), lever2);
	Vector3 pos_sensor1{10.0 + rot1[0], 20.0 + rot1[1], 30.0 + rot1[2]};
	Vector3 pos_sensor2{10.0 + rot2[0], 20.0 + rot2[1], 30.0 + rot2[2]};
	Matrix3 csensor1_ref = xt::transpose(rpy_to_dcm({0.0, 0.0, PI / 2.0 + platform_heading}));
	Matrix3 csensor2_ref =
	    xt::transpose(rpy_to_dcm({-PI / 2.0, 0.0, -PI / 4.0 + platform_heading}));
	Matrix3 cplatform_ref = xt::transpose(rpy_to_dcm({0.0, 0.0, PI / 8.0}));

	Matrix3 csensor1_platform = xt::transpose(rpy_to_dcm({0.0, 0.0, PI / 2.0}));
	Matrix3 csensor2_platform = xt::transpose(rpy_to_dcm({-PI / 2.0, 0.0, -PI / 4.0}));

	auto tx1 = sensor_to_platform({pos_sensor1, csensor1_ref}, lever1, csensor1_platform);
	ASSERT_ALLCLOSE(pos_platform, tx1.first);
	ASSERT_ALLCLOSE(cplatform_ref, tx1.second);
	auto tx2 = platform_to_sensor(tx1, lever2, csensor2_platform);
	ASSERT_ALLCLOSE(pos_sensor2, tx2.first);
	ASSERT_ALLCLOSE(csensor2_ref, tx2.second);
}
