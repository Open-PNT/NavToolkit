#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/factory.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/quaternions.hpp>

#define EXPECT_MATRIX_EQUAL(expected, actual, eps)                            \
	GTEST_ASSERT_((::navtk::filtering::testing::AllCloseHelper{0, eps, true}) \
	                  .compare(#expected, #actual, expected, actual),         \
	              GTEST_NONFATAL_FAILURE_)

using namespace navtk::navutils;
using navtk::dot;
using navtk::ErrorMode;
using navtk::ErrorModeLock;
using navtk::expm;
using navtk::eye;
using navtk::Matrix;
using navtk::Matrix3;
using navtk::num_rows;
using navtk::ones;
using navtk::Size;
using navtk::Vector;
using navtk::Vector3;
using navtk::Vector4;
using navtk::zeros;
using xt::transpose;

TEST(dot, Dimensionality) { ASSERT_ALLCLOSE((Vector{2, 4, 6}), dot(Vector{1, 2, 3}, eye(3) * 2)); }

TEST(meridian_radius, CompareToKnownSolution) {
	// Check calculation against reference values found in DMA TECHNICAL REPORT TR8350.2-b - (Second
	// Printing, 1 December 1987) Supplement to DoD WGS 84 Technical Report Part 2 - Parameters,
	// Formulas, and Graphics https://nga-rescue.is4s.us/Sections%201-5.pdf Table 2.7

	// Table values for every 5 degrees latitude from 0 to 90
	double const test_vals[] = {6335439.3273,
	                            6335922.6064,
	                            6337358.1216,
	                            6339703.2990,
	                            6342888.4825,
	                            6346818.8587,
	                            6351377.1037,
	                            6356426.6959,
	                            6361815.8264,
	                            6367381.8156,
	                            6372955.9257,
	                            6378368.4396,
	                            6383453.8572,
	                            6388056.0488,
	                            6392033.1923,
	                            6395262.3228,
	                            6397643.3264,
	                            6399102.2255,
	                            6399593.6258};
	size_t ind               = 0;
	for (double latitude = 0.0; latitude <= 90.0; latitude += 5.0)
		EXPECT_NEAR(meridian_radius(latitude * PI / 180.0), test_vals[ind++], 1e-4);
}

TEST(discretize_first_order, CompareToKnownSolution) {
	// TODO: This test is based on existing behavior in other languages, and therefore requires
	// validation by someone with domain knowledge.
	Matrix3 Qt = 5 * eye(3) + 0.1 * ones(3);
	// clang-format off
	Matrix3 F {{0, 1, 0},
	           {0, 0, 1},
	           {0, 0, 0}};
	double dt = 0.1;
	Matrix3 expected_phi {{1.0 ,  0.1,  0.0},
	                     {0.0 ,  1.0,  0.1},
	                     {0.0 ,  0.0,  1.0}};
	Matrix3 expected_qd  {{0.51,  0.01,  0.01},
	              		 {0.01,  0.51,  0.01},
	              		 {0.01,  0.01,  0.51}};
	// clang-format on
	auto actual = discretize_first_order(F, Qt, dt);
	EXPECT_MATRIX_EQUAL(expected_phi, actual.first, 1e-8);
	EXPECT_MATRIX_EQUAL(expected_qd, actual.second, 1e-8);
}

TEST(discretize_second_order, CompareToKnownSolution) {
	// TODO: This test is based on existing behavior in other languages, and therefore requires
	// validation by someone with domain knowledge.
	Matrix3 Qt = 5 * eye(3) + 0.1 * ones(3);
	// clang-format off
	Matrix3 F {{0, 1, 0},
	           {0, 0, 1},
	           {0, 0, 0}};
	double dt = 0.1;
	Matrix3 expected_phi {{1.0 ,  0.1,  0.005},
	                      {0.0 ,  1.0,  0.1},
	                      {0.0 ,  0.0,  1.0}};
	Matrix3 expected_qd  {{0.51361137, 0.0362025, 0.011775},
	                      {0.0362025,   0.51355,   0.0355},
	                      {0.011775,    0.0355,     0.51}};
	// clang-format on
	auto actual = discretize_second_order(F, Qt, dt);
	EXPECT_MATRIX_EQUAL(expected_phi, actual.first, 1e-8);
	EXPECT_MATRIX_EQUAL(expected_qd, actual.second, 1e-8);
}

TEST(discretize_van_loan, CompareToVanLoan) {
	Matrix3 Qt = 5 * eye(3) + 0.1 * ones(3);
	// clang-format off
	Matrix3 F {{0, 1, 0},
	           {0, 0, 1},
	           {0, 0, 0}};
	// clang-format on
	double dt   = 0.1;
	auto vloan  = calc_van_loan(F, eye(3), Qt, dt);
	Matrix3 Phi = expm(F * dt);
	auto actual = discretize_van_loan(F, Qt, dt);
	EXPECT_MATRIX_EQUAL(Phi, actual.first, 1e-8);
	EXPECT_MATRIX_EQUAL(vloan, actual.second, 1e-8);
}

TEST(discretize, TestBadInput) {
	Matrix3 Qt = 5 * eye(3) + 0.1 * ones(3);
	// clang-format off
	Matrix3 F {{0, 1, 0},
	           {0, 0, 1},
	           {0, 0, 0}};
	// clang-format on
	EXPECT_UB_OR_DIE(
	    discretize_first_order(F, Qt, -1), "`dt` should be >= 0;", std::invalid_argument);
	EXPECT_UB_OR_DIE(
	    discretize_second_order(F, Qt, -1), "`dt` should be >= 0;", std::invalid_argument);
	EXPECT_UB_OR_DIE(discretize_van_loan(F, Qt, -1), "`dt` should be >= 0;", std::invalid_argument);
}

TEST(calc_van_loan, TaylorSeriesApprox) {
	Matrix3 Qt = 5 * eye(3) + 0.1 * ones(3);
	// clang-format off
	Matrix3 F {{0, 1, 0},
	           {0, 0, 1},
	           {0, 0, 0}};
	// clang-format on
	double dt  = 0.1;
	auto vloan = calc_van_loan(F, eye(3), Qt, dt);

	auto Phi      = expm(F * dt);
	Matrix taylor = (dot(dot(Phi, Qt), transpose(Phi)) + Qt) * dt / 2.0;
	EXPECT_MATRIX_EQUAL(taylor, vloan, 1.0e-2);
}

TEST(calc_van_loan, Single) {
	Matrix Qt{{5.0}};
	Matrix F{{1.2}};
	double dt  = 0.1;
	auto vloan = calc_van_loan(F, eye(1), Qt, dt);
	Matrix expected{{0.565102396502926}};
	EXPECT_MATRIX_EQUAL(expected, vloan, 1.0e-15);
}

TEST(calc_van_loan, f_zero) {
	Matrix Qt{{5.0}};
	Matrix F{{0}};
	double dt  = 0.1;
	auto vloan = calc_van_loan(F, eye(1), Qt, dt);
	Matrix expected{{0.5}};
	EXPECT_MATRIX_EQUAL(expected, vloan, 1.0e-15);
}

TEST(rpy_to_dcm, SingleAxisUnitRotations) {
	auto rot = Vector3{0, 0, PI / 2};
	// clang-format off
	auto expected = Matrix3 {{0, -1, 0},
	                         {1, 0, 0},
	                         {0, 0, 1}};
	// clang-format on
	auto out = rpy_to_dcm(rot);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-10);

	rot      = Vector3{0, PI / 2, 0};
	expected = Matrix3{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
	out      = rpy_to_dcm(rot);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-10);

	rot      = Vector3{PI / 2, 0, 0};
	expected = Matrix3{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}};
	out      = rpy_to_dcm(rot);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-10);
}
TEST(rpy_to_dcm, TwoAxisUnitRotations) {
	auto rot      = Vector3{0, PI / 2, PI};
	auto expected = Matrix3{{0, 0, -1}, {0, -1, 0}, {-1, 0, 0}};
	auto out      = rpy_to_dcm(rot);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-5);


	rot      = Vector3{PI / 2, PI / 2, 0};
	expected = Matrix3{{0, 1, 0}, {0, 0, -1}, {-1, 0, 0}};
	out      = rpy_to_dcm(rot);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-10);

	rot      = Vector3{0, PI / 2, PI / 2};
	expected = Matrix3{{0, -1, 0}, {0, 0, 1}, {-1, 0, 0}};
	out      = rpy_to_dcm(rot);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-10);
}
TEST(rpy_to_dcm, SamplePointRotated) {
	auto rot      = Vector3{PI / 4, PI / 4, 0};
	auto point    = Vector{1, 0, 0};
	auto dcm      = rpy_to_dcm(rot);
	Vector out    = dot(dcm, point);
	auto expected = Vector{cos(PI / 4), 0, -cos(PI / 4)};
	EXPECT_MATRIX_EQUAL(expected, out, 1e-10);
}
TEST(rpy_to_dcm, FullDOFRotation) {
	auto rot   = Vector3{PI / 4, PI / 4, PI / 4};
	auto point = Vector3{1, 1, 1};
	auto dcm   = rpy_to_dcm(rot);

	// Generate the individual rotations that make up the full DOF rotation
	auto dcm_roll       = rpy_to_dcm(Vector3{PI / 4, 0, 0});
	auto dcm_pitch      = rpy_to_dcm(Vector3{0, PI / 4, 0});
	auto dcm_yaw        = rpy_to_dcm(Vector3{0, 0, PI / 4});
	Matrix expected_dcm = dot(dot(dcm_yaw, dcm_pitch), dcm_roll);
	EXPECT_MATRIX_EQUAL(expected_dcm, dcm, 1e-10);
	EXPECT_MATRIX_EQUAL(dot(expected_dcm, point), dot(dcm, point), 1e-10);
	EXPECT_MATRIX_EQUAL(dot(expected_dcm, point), dot(dcm, point), 1e-10);
}
TEST(transverse_radius, CompareToKnownSolution) {
	// Check calculation against reference values found in DMA TECHNICAL REPORT TR8350.2-b - (Second
	// Printing, 1 December 1987) Supplement to DoD WGS 84 Technical Report Part 2 - Parameters,
	// Formulas, and Graphics https://nga-rescue.is4s.us/Sections%201-5.pdf Table 2.7

	// Table values for every 5 degrees latitude from 0 to 90
	double const test_vals[] = {6378137.0000,
	                            6378299.1746,
	                            6378780.8437,
	                            6379567.5820,
	                            6380635.8071,
	                            6381953.4572,
	                            6383480.9177,
	                            6385172.1749,
	                            6386976.1657,
	                            6388838.2901,
	                            6390702.0442,
	                            6392510.7274,
	                            6394209.1738,
	                            6395745.4533,
	                            6397072.4882,
	                            6398149.5323,
	                            6398943.4599,
	                            6399429.8215,
	                            6399593.6258};
	size_t ind               = 0;
	for (double latitude = 0.0; latitude <= 90.0; latitude += 5.0)
		EXPECT_NEAR(transverse_radius(latitude * PI / 180.0), test_vals[ind++], 1e-4);
}

Vector3 naive_dcm_to_rpy(const Matrix3& dcm) {
	auto r = atan2(dcm(1, 2), dcm(2, 2));
	auto p = -asin(dcm(0, 2));
	auto y = atan2(dcm(0, 1), dcm(0, 0));
	return {r, p, y};
}

TEST(rpy_to_dcm, DCMConvention) {
	/* Check to make sure DCM rotation direction generated from either euler or quaternions are the
	 * same. rpy_to_dcm->dcm_to_quat->quat_to_rpy should be a no-op */
	auto rpy  = Vector{PI / 4, PI / 8, PI / 4};
	auto dcm  = rpy_to_dcm(rpy);
	auto quat = dcm_to_quat(dcm);
	auto out  = quat_to_rpy(quat);
	EXPECT_MATRIX_EQUAL(rpy, out, 1e-10);
}

TEST(dcm_to_rpy, DCMConvention) {
	/* Check to make sure DCM rotation direction generated from either euler or quaternions are the
	 * same. rpy_to_quat->quat_to_dcm->dcm_to_rpy should be a no-op */
	auto rpy  = Vector{PI / 4, PI / 8, PI / 4};
	auto quat = rpy_to_quat(rpy);
	auto dcm  = quat_to_dcm(quat);
	auto out  = dcm_to_rpy(dcm);
	EXPECT_MATRIX_EQUAL(rpy, out, 1e-10);
}

TEST(dcm_to_rpy, InverseOfRpyToDcm) {
	/*
	 * None of these actually fail with the previous dcm_to_rpy version due to numerics as explained
	 * in InverseOfRpyToDcmAnother comments; this test is to guard against unlikely regression +
	 * sin/cos behavior change.
	 */
	Matrix rot{{PI / 4, PI / 2, PI / 8},
	           {-PI / 4, PI / 2, PI / 8},
	           {PI / 4, -PI / 2, PI / 8},
	           {-PI / 4, -PI / 2, PI / 8},
	           {PI / 4, PI / 2, -PI / 8},
	           {-PI / 4, PI / 2, -PI / 8},
	           {PI / 4, -PI / 2, -PI / 8},
	           {-PI / 4, -PI / 2, -PI / 8}};

	for (Size r = 0; r < num_rows(rot); r++) {
		auto rpy = xt::view(rot, r, xt::all());
		auto dcm = rpy_to_dcm(rpy);
		auto out = dcm_to_rpy(dcm);
		EXPECT_MATRIX_EQUAL(rpy, out, 1e-10);
	}
}

TEST(dcm_to_rpy, InverseOfRpyToDcmAnother) {
	/*
	 * Without the detection of PI/2 pitches, the dcm below is incorrectly converted to
	 * [0, -PI/2, 0] when supplied to dcm_to_rpy. Acceptable equivalents would be either
	 * [0, -PI/2, PI/2] or [PI/2, -PI/2, 0]. However, attempting to generate the offending
	 * DCM through rpy_to_dcm using one of these inputs fails; the math is such that the dcms
	 * generated have small (e-17, e-33) values in place of navtk::zeros that are nevertheless
	 * substantial enough to enable the 'naive' dcm_to_rpy to reproduce the input rpys.
	 *
	 * A dcm of this type could be generated from angles 'in the wild' using llh_to_cen({0, PI/2,
	 * 0})
	 */
	Matrix dcm      = Matrix3{{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}};  // xt::allclose wants just Matrix
	auto rpy_vers_1 = transpose(rpy_to_dcm({0, -PI / 2, PI / 2}));
	auto rpy_vers_2 = transpose(rpy_to_dcm({PI / 2, -PI / 2, 0}));
	auto cen        = llh_to_cen({0, PI / 2, 0});
	auto cse        = transpose(dot(cen, rpy_to_dcm({0, 0, 0})));
	EXPECT_MATRIX_EQUAL(dcm, rpy_vers_1, 1e-10);
	EXPECT_MATRIX_EQUAL(dcm, rpy_vers_2, 1e-10);
	EXPECT_MATRIX_EQUAL(dcm, cse, 1e-10);
	auto rpy      = dcm_to_rpy(transpose(cse));
	auto naive    = transpose(naive_dcm_to_rpy(cse));
	auto out      = transpose(rpy_to_dcm(rpy));
	auto out_fail = transpose(rpy_to_dcm(naive));
	ASSERT_FALSE(xt::allclose(dcm, out_fail));
	EXPECT_MATRIX_EQUAL(dcm, out, 1e-10);
}

TEST(ecef_to_llh, CompareToMatlabSolution) {
	Vector3 Pe{4510731, 4510731, 0};
	auto Pwgs     = ecef_to_llh(Pe);
	auto expected = Vector3{0, 45 * PI / 180, 999.9564};
	EXPECT_MATRIX_EQUAL(expected, Pwgs, 1e-4);
}

// navtk#757
TEST(ecef_to_llh, HangCheck) {
	navtk::Vector3 ecef = {0.0, 0.0, 3e+31};
	ecef_to_llh(ecef);
	// This test case will fail if the function hangs indefinitely
}

TEST(ecef_to_llh, CompareToGoldStandard) {
	// Points taken from GoldData_v6.3, https://nga-rescue.is4s.us/gold/GoldData_v6.3.zip
	Vector3 Pe;
	Vector3 diff;

	Pe               = {6285924.886408, 0.000000, 6040538.783023};
	auto Pwgs        = ecef_to_llh(Pe);
	Vector3 expected = {44.0 * PI / 180.0, 0.0, 2350000};
	diff             = Pwgs - expected;
	EXPECT_NEAR(diff[0], 0, 1e-12);
	EXPECT_NEAR(diff[1], 0, 1e-12);
	EXPECT_NEAR(diff[2], 0, 1e-6);

	Pe       = {152701.349483, 0.000000, 8705419.710257};
	Pwgs     = ecef_to_llh(Pe);
	expected = {89 * PI / 180.0, 0.0, 2350000};
	diff     = Pwgs - expected;
	EXPECT_NEAR(diff[0], 0, 1e-12);
	EXPECT_NEAR(diff[1], 0, 1e-12);
	EXPECT_NEAR(diff[2], 0, 1e-6);

	Pe       = {-2246552.197953, -2246552.197953, -5465836.117787};
	Pwgs     = ecef_to_llh(Pe);
	expected = Vector3{-60 * PI / 180.0, -135 * PI / 180.0, -40000.0};
	diff     = Pwgs - expected;
	EXPECT_NEAR(diff[0], 0, 1e-12);
	EXPECT_NEAR(diff[1], 0, 1e-12);
	EXPECT_NEAR(diff[2], 0, 1e-6);

	Pe       = {0.000000, -3177104.586924, -5465836.117787};
	Pwgs     = ecef_to_llh(Pe);
	expected = {-60 * PI / 180.0, -90 * PI / 180.0, -40000.0};
	diff     = Pwgs - expected;
	EXPECT_NEAR(diff[0], 0, 1e-12);
	EXPECT_NEAR(diff[1], 0, 1e-12);
	EXPECT_NEAR(diff[2], 0, 1e-6);

	Pe       = {-3197104.586924, 0.000000, -5500477.133939};
	Pwgs     = ecef_to_llh(Pe);
	expected = {-60 * PI / 180.0, PI, 0.0};
	diff     = Pwgs - expected;
	EXPECT_NEAR(diff[0], 0, 1e-12);
	EXPECT_NEAR(diff[1], 0, 1e-12);
	EXPECT_NEAR(diff[2], 0, 1e-6);

	Pe       = {-2267765.401388, -2267765.401388, -5517797.642014};
	Pwgs     = ecef_to_llh(Pe);
	expected = {-60.0 * PI / 180.0, -135.0 * PI / 180.0, 20000.0};
	diff     = Pwgs - expected;
	EXPECT_NEAR(diff[0], 0, 1e-12);
	EXPECT_NEAR(diff[1], 0, 1e-12);
	EXPECT_NEAR(diff[2], 0, 1e-6);
}

TEST(axis_angle_to_dcm, CompareToRpyRotations) {
	double angle = PI / 8;

	Vector3 axis     = {1, 0, 0};
	Matrix3 expected = rpy_to_dcm(Vector3{PI / 8, 0, 0});
	auto out         = axis_angle_to_dcm(axis, angle);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-9);

	axis     = {0, 1, 0};
	expected = rpy_to_dcm(Vector3{0, PI / 8, 0});
	out      = axis_angle_to_dcm(axis, angle);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-9);

	axis     = {0, 0, 1};
	expected = rpy_to_dcm(Vector3{0, 0, PI / 8});
	out      = axis_angle_to_dcm(axis, angle);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-9);
}
TEST(axis_angle_to_dcm, SamplePointRotated) {
	double angle    = PI / 4;
	Vector axis     = {0, 0, -1};
	Vector sample   = {sin(PI / 4), sin(PI / 4), 0};
	Vector expected = {1, 0, 0};
	Vector out      = dot(axis_angle_to_dcm(axis, angle), sample);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-9);

	angle    = PI;
	axis     = {1, 1, 0};
	axis     = axis / navtk::norm(axis);
	sample   = {1, 1, -1};
	expected = {1, 1, 1};
	out      = dot(axis_angle_to_dcm(axis, angle), sample);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-9);

	angle    = PI;
	axis     = {1, 1, 0};
	axis     = axis / navtk::norm(axis);
	sample   = {1, 0, 0};
	sample   = sample / navtk::norm(sample);
	expected = {0, 1, 0};
	out      = dot(axis_angle_to_dcm(axis, angle), sample);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-9);

	angle    = PI / 2;
	axis     = {1, 1, 0};
	axis     = axis / navtk::norm(axis);
	sample   = {1, 0, 0};
	sample   = sample / navtk::norm(sample);
	expected = {.5, .5, -1 / sqrt(2)};
	out      = dot(axis_angle_to_dcm(axis, angle), sample);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-9);
}

TEST(axis_angle_to_dcm, CheckNonNormalizedAxis) {
	double angle     = PI;
	Vector3 axis     = {1, 1, 0};
	Vector3 sample   = {1, 1, -1};
	Vector3 expected = {1, 1, 1};
	Vector3 out      = dot(axis_angle_to_dcm(axis, angle), sample);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-9);
}

TEST(llh_to_ecef, InverseOfEcefToLlh) {
	Vector3 Pwgs{1, 2, 3};
	auto Pe  = llh_to_ecef(Pwgs);
	auto out = ecef_to_llh(Pe);
	EXPECT_MATRIX_EQUAL(Pwgs, out, 1e-6);
}

TEST(ecef_to_local_level, CompareToKnownSolution) {
	Vector3 P0e{10000, 10000, 0};
	Vector3 Pe{10001, 10001, 0};
	Vector3 expected{0, 0, -sqrt(2)};
	auto out = ecef_to_local_level(P0e, Pe);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-6);
}

TEST(local_level_to_ecef, InverseOfEcefToLocalLevel) {
	Vector3 P0e{10000, 10000, 0};
	Vector3 Pe{10001, 10001, 0};
	auto ll  = ecef_to_local_level(P0e, Pe);
	auto out = local_level_to_ecef(P0e, ll);
	EXPECT_MATRIX_EQUAL(Pe, out, 1e-6);
}

TEST(llh_to_cen, CompareToKnownSolution) {
	Vector3 Pwgs{0, PI / 4, sqrt(sqrt(2) + sqrt(2))};
	Matrix3 expected = ecef_to_cen(Vector3{1, 1, 0});
	Matrix3 out      = llh_to_cen(Pwgs);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-6);
}

TEST(DISABLED_llhToCen, CompareToKnownSolutionThreeAxis) {  // TODO: Geodetic solution
}

TEST(ecef_to_cen, CompareToKnownSolution) {
	Vector3 Pe{10001, 0, 0};
	// clang-format off
	Matrix3 expected = {{0, 0, -1},
	                    {0, 1,  0},
	                    {1, 0,  0}};
	// clang-format on
	auto out = ecef_to_cen(Pe);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-6);
}
TEST(ecef_to_cen, CompareToKnownSolutionTwoAxis) {
	Vector3 Pe{300, 300, 0};
	// clang-format off
	Matrix3 expected = {{0, -sin(PI/4), -sin(PI/4)},
	                    {0,  sin(PI/4), -sin(PI/4)},
	                    {1,          0,          0}};
	// clang-format on
	auto out = ecef_to_cen(Pe);
	EXPECT_MATRIX_EQUAL(expected, out, 1e-6);
}

TEST(dcm_to_quat, CompareToKnownSolution) {
	auto dcm = transpose(rpy_to_dcm(Vector3{1.8, -0.42, 1.23}));
	// The matlab version of DcmToQuat uses ref->platform dcm. Transpose to compensate.
	auto actual = dcm_to_quat(transpose(dcm));
	Vector4 expected{0.4023476388, 0.7005075035, 0.3361801534, 0.4841368804};
	EXPECT_MATRIX_EQUAL(expected, actual, 2e-9);
}

TEST(dcm_to_quat, CompareToManualRotation) {
	// clang-format off
	Matrix3 dcm = {{0,  0, -1},
	               {1,  0,  0},
	               {0, -1,  0}};
	// clang-format on
	auto q = dcm_to_quat(dcm);
	Vector3 point{0.1, 0.2, 0.3};
	Vector rot1 = dot(dcm, point);
	Vector3 rot2{
	    point[0] * (pow(q[0], 2) + pow(q[1], 2) - pow(q[2], 2) - pow(q[3], 2)) +
	        2 * point[1] * (q[1] * q[2] - q[0] * q[3]) + 2 * point[2] * (q[1] * q[3] + q[0] * q[2]),
	    2 * point[0] * (q[1] * q[2] + q[0] * q[3]) +
	        point[1] * (pow(q[0], 2) - pow(q[1], 2) + pow(q[2], 2) - pow(q[3], 2)) +
	        2 * point[2] * (q[2] * q[3] - q[0] * q[1]),
	    2 * point[0] * (q[1] * q[3] - q[0] * q[2]) + 2 * point[1] * (q[2] * q[3] + q[0] * q[1]) +
	        point[2] * (pow(q[0], 2) - pow(q[1], 2) - pow(q[2], 2) + pow(q[3], 2))};
	EXPECT_MATRIX_EQUAL(rot1, rot2, 2e-9);
}

TEST(delta_lat_to_north, CompareToEcef) {
	Vector3 const lla1{0.7, -1.4, 100};
	Vector3 const lla2{0.70005, -1.4, 100};
	auto const dlat   = (lla2 - lla1)[0];
	auto const ecef1  = llh_to_ecef(lla1);
	auto const ecef2  = llh_to_ecef(lla2);
	auto const decef  = ecef2 - ecef1;
	auto const cen    = ecef_to_cen(ecef1);
	auto const dnav   = dot(transpose(cen), decef);
	auto const actual = delta_lat_to_north(dlat, lla1[0], lla1[2]);
	EXPECT_NEAR(dnav[0], actual, 0.001);
}

TEST(delta_lat_to_north, OffMeridian) {
	Vector3 x{3, 3, 0};
	Vector3 y{3.000005, 3, 0};
	auto x_e = llh_to_ecef(x);
	auto y_e = llh_to_ecef(y);
	EXPECT_NEAR(navtk::norm(Vector{x_e - y_e}), delta_lat_to_north(0.000005, 3.0), 1.0e-6);
}

TEST(delta_lat_to_north, PerfectEarthApprox) {
	double A   = .000015;
	double out = delta_lat_to_north(A, 0);
	EXPECT_NEAR(A * 6353000.0, out, 1);
}

TEST(delta_lat_to_north, VerifyAltitudeCorrection) {
	// The difference between the incorrect version (altitude added directly to R_earth when
	// calculating radii of curvatures) and the correct (altitude added to the radii of curvature)
	// is about 169 um with the given parameters.
	double const expected  = 0.736738181561957;
	double const altitude  = 1000000.0;
	double const latitude  = PI / 4.0;
	double const delta_lat = 1e-7;
	EXPECT_NEAR(delta_lat_to_north(delta_lat, latitude, altitude), expected, 1.0e-10);
}

TEST(delta_lon_to_east, CompareToEcef) {
	Vector3 lla1{0.7, -1.4, 100};
	Vector3 lla2{0.7, -1.40005, 100};
	auto dlon   = (lla2 - lla1)[1];
	auto ecef1  = llh_to_ecef(lla1);
	auto ecef2  = llh_to_ecef(lla2);
	auto decef  = ecef2 - ecef1;
	auto cen    = ecef_to_cen(ecef1);
	auto dnav   = dot(transpose(cen), decef);
	auto actual = delta_lon_to_east(dlon, lla1[0], lla1[2]);
	EXPECT_NEAR(dnav[1], actual, 0.001);
}

TEST(delta_lon_to_east, VerifyAltitudeCorrection) {
	// The difference between the incorrect version (altitude added directly to R_earth when
	// calculating radii of curvatures) and the correct (altitude added to the radii of curvature)
	// is about 119 um with the given parameters.
	double const expected  = 0.522469766003547;
	double const altitude  = 1000000.0;
	double const latitude  = PI / 4.0;
	double const delta_lon = 1e-7;
	EXPECT_NEAR(delta_lon_to_east(delta_lon, latitude, altitude), expected, 1.0e-10);
}

TEST(east_to_delta_lon, CompareToEcef) {
	Vector3 lla1{0.7, -1.4, 100};
	Vector3 lla2{0.7, -1.40005, 100};
	auto dlon   = (lla2 - lla1)[1];
	auto ecef1  = llh_to_ecef(lla1);
	auto ecef2  = llh_to_ecef(lla2);
	auto decef  = ecef2 - ecef1;
	auto cen    = ecef_to_cen(ecef1);
	auto dnav   = dot(transpose(cen), decef);
	auto actual = east_to_delta_lon(dnav[1], lla1[0], lla1[2]);
	EXPECT_NEAR(dlon, actual, 1.6e-10);
}

TEST(east_to_delta_lon, InverseFunction) {
	double expected = 0.0000014;
	double actual   = east_to_delta_lon(delta_lon_to_east(expected, 1.34), 1.34);
	EXPECT_NEAR(expected, actual, 1.6e-10);
}

TEST(north_to_delta_lat, InverseFunction) {
	double expected = 0.000015;
	double actual   = north_to_delta_lat(delta_lat_to_north(expected, 1), 1);
	EXPECT_NEAR(expected, actual, 1.6e-10);
}

TEST(north_to_delta_lat, CompareToEcef) {
	Vector3 lla1  = {0.7, -1.4, 100};
	Vector3 lla2  = {0.70005, -1.4, 100};
	auto ecef1    = llh_to_ecef(lla1);
	auto ecef2    = llh_to_ecef(lla2);
	auto cen      = ecef_to_cen(ecef1);
	auto decef    = ecef2 - ecef1;
	auto dnav     = dot(transpose(cen), decef);
	auto expected = (lla2 - lla1)[0];
	auto actual   = north_to_delta_lat(dnav[0], lla1[0], lla1[2]);
	EXPECT_NEAR(expected, actual, 1.6e-10);  // 1mm error
}

TEST(north_to_delta_lat, PerfectEarthApprox) {
	double a        = 15;
	double actual   = north_to_delta_lat(a, 0);
	double expected = a / 6353000.0;
	EXPECT_NEAR(expected, actual, 1e-6);
}

TEST(correct_dcm_with_tilt, CompareToKnownSolution) {  // No tilt applied
	Vector3 rpy1 = {PI / 4, PI / 5, PI / 8};
	Vector3 rpy2 = {0.0, 0.0, 0.0};
	// C_nav_to_platform
	Matrix3 frame1 = transpose(rpy_to_dcm(rpy1));
	// Must pass C_body_to_nav into function with original tilts
	Matrix3 frame2 = dot(rpy_to_dcm(rpy2), transpose(frame1));
	Matrix3 frame3 = correct_dcm_with_tilt(frame2, rpy2);
	EXPECT_MATRIX_EQUAL(frame3, transpose(frame1), 1e-5);

	// X tilt only
	rpy1   = {-1.4, 0.7, -0.2};
	rpy2   = {0.001, 0.0, 0.0};
	frame1 = transpose(rpy_to_dcm(rpy1));
	frame2 = dot(rpy_to_dcm(rpy2), transpose(frame1));
	frame3 = correct_dcm_with_tilt(frame2, rpy2);
	EXPECT_MATRIX_EQUAL(frame3, transpose(frame1), 1e-5);

	// Y tilt only
	rpy1   = {-1.4, 0.7, -0.2};
	rpy2   = {0.0, 0.025, 0.0};
	frame1 = transpose(rpy_to_dcm(rpy1));
	frame2 = dot(rpy_to_dcm(rpy2), transpose(frame1));
	frame3 = correct_dcm_with_tilt(frame2, rpy2);
	EXPECT_MATRIX_EQUAL(frame3, transpose(frame1), 1e-5);

	// Z tilt only
	rpy1   = {-1.4, 0.7, -0.2};
	rpy2   = {0.0, 0.0, 0.006};
	frame1 = transpose(rpy_to_dcm(rpy1));
	frame2 = dot(rpy_to_dcm(rpy2), transpose(frame1));
	frame3 = correct_dcm_with_tilt(frame2, rpy2);
	EXPECT_MATRIX_EQUAL(frame3, transpose(frame1), 1e-5);

	// Starting from identity
	rpy1   = {0.0, 0.0, 0.0};
	rpy2   = {0.001, 0.002, -0.003};
	frame1 = transpose(rpy_to_dcm(rpy1));
	frame2 = dot(rpy_to_dcm(rpy2), transpose(frame1));
	frame3 = correct_dcm_with_tilt(frame2, rpy2);
	EXPECT_MATRIX_EQUAL(frame3, transpose(frame1), 1e-5);

	// More random starting DCM, tilts
	rpy1   = {PI / 4, PI / 3, PI / 7};
	rpy2   = {0.001, 0.002, -0.003};
	frame1 = transpose(rpy_to_dcm(rpy1));
	frame2 = dot(rpy_to_dcm(rpy2), transpose(frame1));
	frame3 = correct_dcm_with_tilt(frame2, rpy2);
	EXPECT_MATRIX_EQUAL(frame3, transpose(frame1), 1e-5);
}

TEST(DISABLED_tiltToRpyWithCov, CompareToKnownSolution) {  // TODO
}

TEST(DISABLED_calcTilt, CompareToKnownSolution) {  // TODO
}

TEST(DISABLED_rpyToTiltWithCov, CompareToKnownSolution) {  // TODO
}

TEST(quat_to_dcm, CompareToKnownSolution) {
	auto q = rpy_to_quat(Vector3{0.05, -1.2, -0.64});
	// clang-format off
	Matrix3 expected = {{0.290645608093345,  0.559085446603746, -0.776497645773597},
	            	   {-0.216398404872243,  0.828912226128811,  0.515825795926229},
	             		{0.932039087627122,  0.018110349805246,  0.361904896851553}};
	// clang-format on
	auto dcm = quat_to_dcm(q);
	EXPECT_MATRIX_EQUAL(expected, dcm, 2.0e-6);
}

TEST(quat_to_dcm, InverseFunction) {
	auto expected = rpy_to_quat(Vector3{0.3, 1.7, -1.9});
	auto actual   = dcm_to_quat(quat_to_dcm(expected));
	EXPECT_MATRIX_EQUAL(expected, actual, 2.0e-6);
}

TEST(rpy_to_quat, CompareToKnownSolution) {
	Vector3 const rpy      = {0.05, -1.20, -0.64};
	Vector4 const expected = {
	    0.787632964405762, -0.157978233413660, -0.542301048263455, -0.246143787655200};
	auto actual = rpy_to_quat(rpy);
	EXPECT_MATRIX_EQUAL(expected, actual, 1.0e-9);
}

TEST(quat_to_rpy, CompareToKnownSolution) {
	Vector4 const quat     = {0.5, 0.6, 0.7, 0.8};
	Vector3 const expected = {1.957302631908816, -0.263022202908469, 2.225902330463252};
	auto actual            = quat_to_rpy(quat);
	EXPECT_MATRIX_EQUAL(expected, actual, 1.0e-9);
}

TEST(rpy_to_quat, Inverse) {
	Vector3 const original_rpy = {PI / 4, PI / 3, PI / 2};
	auto const quat            = rpy_to_quat(original_rpy);
	auto const actual          = quat_to_rpy(quat);
	EXPECT_MATRIX_EQUAL(original_rpy, actual, 1.0e-9);
}

TEST(quat, PropComp) {
	// Propagation/tilt correction very similar
	Vector3 w{1e-3, -2e-6, -7e-3};  // w_nb_b
	Vector3 rpy{1.2, -0.75, -0.6};
	auto cbn = transpose(rpy_to_dcm(rpy));
	// tw 3.4.1
	auto cnb_prop = transpose(correct_dcm_with_tilt(cbn, w));
	auto q        = dcm_to_quat(transpose(cbn));

	auto qp = quat_prop(q, w);

	// All 3 methods of quat prop match correct dcm with tilt (higher order), but not straight skew
	// mult
	auto q_as_c_prop = quat_to_dcm(qp);
	ASSERT_ALLCLOSE(cnb_prop, q_as_c_prop);
}

TEST(quat, Mults) {
	Vector3 rpy{1.2, -0.75, -0.6};
	Vector3 rpy2{1.0, -1.75, -0.8};
	auto cbn  = transpose(rpy_to_dcm(rpy));
	auto cbn2 = transpose(rpy_to_dcm(rpy2));

	auto cnb2  = dot(transpose(cbn2), transpose(cbn));
	auto q     = dcm_to_quat(transpose(cbn));
	auto q2    = dcm_to_quat(transpose(cbn2));
	auto qm    = quat_mult(q2, q);
	auto q_con = quat_to_dcm(qm);
	ASSERT_ALLCLOSE(q_con, cnb2);

	// Conjugate is analogous to dcm transpose and follows similar rules
	// If C = C_1 * C_2, then C^T = C_2^T * C_1^T.
	auto alt = quat_mult(quat_conj(q), quat_conj(q2));
	ASSERT_ALLCLOSE(qm, quat_conj(alt));

	// 'Identity' quaternion
	Vector iq{1, 0, 0, 0};
	ASSERT_ALLCLOSE(quat_mult(q, quat_conj(q)), iq);
}

TEST(quat, Mults2) {
	Vector3 rpy{1.2, -0.75, -0.6};
	Vector3 rpy2{1.0, -1.35, -0.8};
	auto cbn = transpose(rpy_to_dcm(rpy));
	auto ckb = transpose(rpy_to_dcm(rpy2));

	auto ckn = dot(ckb, cbn);
	auto qkn = dcm_to_quat(ckn);
	auto qnk = dcm_to_quat(transpose(ckn));
	ASSERT_ALLCLOSE(qnk, quat_conj(qkn));

	auto qnb         = rpy_to_quat(rpy);
	auto qbk         = rpy_to_quat(rpy2);
	auto qkn_by_mult = quat_mult(quat_conj(qbk), quat_conj(qnb));
	ASSERT_ALLCLOSE(qkn_by_mult, qkn);

	auto qnk_by_mult = quat_mult(qnb, qbk);
	ASSERT_ALLCLOSE(qnk_by_mult, qnk);

	auto rpy_1 = quat_to_rpy(qnb);
	auto rpy_2 = quat_to_rpy(qbk);

	ASSERT_ALLCLOSE(rpy_1, rpy);
	ASSERT_ALLCLOSE(rpy_2, rpy2);
}

TEST(quat, VectorTx) {
	Vector3 rpy{1.2, -0.75, -0.6};
	Vector3 rb{1.0, 9.34, -752.0};
	auto cnb = rpy_to_dcm(rpy);
	auto q   = dcm_to_quat(cnb);

	auto rn  = dot(cnb, rb);
	auto rnq = quat_rot(q, rb);
	ASSERT_ALLCLOSE(rn, rnq);

	// Inversion
	auto rev = quat_rot(quat_conj(q), rnq);
	ASSERT_ALLCLOSE(rb, rev);
}

TEST(quat, Norm) {
	Vector3 rpy{1.2, -0.75, -0.6};
	auto q = rpy_to_quat(rpy);
	ASSERT_ALLCLOSE(q, quat_norm(q));
	ASSERT_ALLCLOSE(q, quat_norm(1.5 * q));
	ASSERT_ALLCLOSE(-q, quat_norm(-1.5 * q));
	ASSERT_ALLCLOSE(q, quat_norm(0.3 * q));
}

TEST(quat, SignErrorInTw) {
	Vector rpy{-0.64, 1.5472, .58};
	auto cnb = rpy_to_dcm(rpy);

	auto a = 0.5 * sqrt(1 + cnb(0, 0) + cnb(1, 1) + cnb(2, 2));
	auto d = 1.0 / (4 * a) * (cnb(1, 0) - cnb(0, 1));

	auto t1         = cos(rpy(0) / 2.0) * cos(rpy(1) / 2.0) * sin(rpy(2) / 2.0);
	auto t2         = sin(rpy(0) / 2.0) * sin(rpy(1) / 2.0) * cos(rpy(2) / 2.0);
	auto difference = d - (t1 + t2);
	ASSERT_TRUE(abs(difference) > 1e-9);
	ASSERT_DOUBLE_EQ(d, t1 - t2);
}

TEST(quat, Cen) {
	Vector llh{-0.64, 0.73, 100.0};
	Vector test{123.21, -7246.3, 96.11};
	auto cen = llh_to_cen(llh);
	auto q1  = dcm_to_quat(cen);
	auto q2  = llh_to_quat_en(llh);

	ASSERT_ALLCLOSE(dot(cen, test), quat_rot(q1, test));
	ASSERT_ALLCLOSE(dot(cen, test), quat_rot(q2, test));
}

TEST(quat_to_rpy, NanCheck) {
	Vector llh{0.0, 0.0, 0.0};
	Vector rpy{0.0, -PI / 2.0, 0.0};
	auto cbn   = transpose(rpy_to_dcm(rpy));
	auto cen   = llh_to_cen(llh);
	auto cbe   = dot(cbn, transpose(cen));
	auto q     = dcm_to_quat(cbe);
	Vector out = quat_to_rpy(q);  // Cannot auto; either isnan or any doesn't like fixed-size
	ASSERT_FALSE(xt::any(xt::isnan(out)));
}

TEST(tilt_corr, Quat0) {
	Vector rpy{-0.64, 1.1472, .58};
	Vector tilts{0.0, 0.0, 0.0};
	auto cnb_e = rpy_to_dcm(rpy);
	auto qnb_e = rpy_to_quat(rpy);
	auto cnb   = ortho_dcm(dot(eye(3) + skew(tilts), cnb_e));
	auto qnb   = quat_norm(correct_quat_with_tilt(qnb_e, tilts));
	auto qdcm  = quat_to_dcm(qnb);
	ASSERT_ALLCLOSE(cnb, qdcm);
}

TEST(quat, YouShouldNormalizePostTilt) {
	Vector rpy{-0.64, 1.1472, .58};
	Vector tilts{1e-3, 2e-3, -6e-3};
	auto cnb_e = rpy_to_dcm(rpy);
	auto qnb_e = rpy_to_quat(rpy);
	auto cnb   = dot(eye(3) + skew(tilts), cnb_e);
	auto qnb   = correct_quat_with_tilt(qnb_e, tilts);
	auto qdcm  = quat_to_dcm(qnb);
	auto rpy1  = dcm_to_rpy(cnb);
	auto rpy2  = quat_to_rpy(qnb);

	auto rpy3 = dcm_to_rpy(ortho_dcm(cnb));
	auto rpy4 = quat_to_rpy(quat_norm(qnb));
	auto d1   = xt::abs(rpy1 - rpy2);
	auto d2   = xt::abs(rpy3 - rpy4);
	ASSERT_ALLCLOSE(rpy3, rpy4);
	ASSERT_TRUE(xt::all(navtk::to_matrix(d2) < navtk::to_matrix(d1)));
}

TEST(quat, YouShouldNormalizePostTilt2) {
	Vector rpy{0.0, 0.0, 0.0};
	Vector tilts{1e-3, 2e-3, -6e-3};
	auto cnb_e = rpy_to_dcm(rpy);
	auto qnb_e = rpy_to_quat(rpy);
	auto cnb   = dot(eye(3) + skew(tilts), cnb_e);
	auto qnb   = correct_quat_with_tilt(qnb_e, tilts);
	auto qdcm  = quat_to_dcm(qnb);
	auto rpy1  = dcm_to_rpy(cnb);
	auto rpy2  = quat_to_rpy(qnb);

	auto rpy3 = dcm_to_rpy(ortho_dcm(cnb));
	auto rpy4 = quat_to_rpy(quat_norm(qnb));
	auto d1   = xt::abs(rpy1 - rpy2);
	auto d2   = xt::abs(rpy3 - rpy4);
	// An identity DCM results in a bit more error
	// But it is also dependent on tilt magnitude (going to 1e-4 also makes it pass w/ default
	// values)
	ASSERT_ALLCLOSE_EX(rpy3, rpy4, 1e-5, 1e-7);
	ASSERT_TRUE(xt::all(navtk::to_matrix(d2) < navtk::to_matrix(d1)));
}

TEST(quat, EqualConversions) {
	// Probably partially redundant with other navutils tests
	Vector rpy{-0.64, 1.1472, .58};
	auto cbn              = transpose(rpy_to_dcm(rpy));
	auto qnb_from_rpy     = rpy_to_quat(rpy);
	auto qnb_from_cnb     = dcm_to_quat(transpose(cbn));
	auto cnb_from_qnb_rpy = quat_to_dcm(qnb_from_rpy);
	auto cnb_from_qnb_cnb = quat_to_dcm(qnb_from_cnb);

	auto rpy1 = dcm_to_rpy(transpose(cbn));
	auto rpy2 = dcm_to_rpy(cnb_from_qnb_rpy);
	auto rpy3 = dcm_to_rpy(cnb_from_qnb_cnb);
	auto rpy4 = quat_to_rpy(qnb_from_rpy);
	auto rpy5 = quat_to_rpy(qnb_from_cnb);
	ASSERT_ALLCLOSE_EX(rpy, rpy1, 1e-10, 1e-10);
	ASSERT_ALLCLOSE_EX(rpy, rpy2, 1e-10, 1e-10);
	ASSERT_ALLCLOSE_EX(rpy, rpy3, 1e-10, 1e-10);
	ASSERT_ALLCLOSE_EX(rpy, rpy4, 1e-10, 1e-10);
	ASSERT_ALLCLOSE_EX(rpy, rpy5, 1e-10, 1e-10);
}

TEST(geoid_minus_ellipsoid, valid_lat_lon) {
	// clang-format off
    const Matrix query_coordinates = {{ 90        ,  35        },
                                      { 38.6281550, 269.7791550},
                                      {-14.6212170, 305.0211140},
                                      { 46.8743190, 102.4487290},
                                      {-23.6174460, 133.8747120},
                                      { 38.6254730, 359.9995000},
                                      {  -.4667440,    .0023000},
                                      {  0        ,   0        },
                                      {  0        , 360        }};
	// clang-format on

	const Vector expected_undulations = {
	    13.606, -31.628, -2.969, -43.575, 15.871, 50.066, 17.329, 17.162, 17.162};

	// Turn on error mode so that validation is not skipped.
	auto guard = ErrorModeLock(ErrorMode::DIE);

	auto resulting_undulations = zeros(num_rows(query_coordinates));
	for (Size i = 0; i < num_rows(query_coordinates); ++i) {
		auto undulation = geoid_minus_ellipsoid(query_coordinates(i, 0) * DEG2RAD,
		                                        query_coordinates(i, 1) * DEG2RAD);
		EXPECT_TRUE(undulation.first);
		resulting_undulations(i) = undulation.second;
	}

	ASSERT_ALLCLOSE_EX(expected_undulations, resulting_undulations, 0.05, 0.0);
}

TEST(geoid_minus_ellipsoid, invalid_lat_lon) {
	// clang-format off
	const Matrix query_coordinates =   {{        100       ,         180       },
										{         90.00001 ,         180       },
										{       -100       ,         180       },
										{        -90.00001 ,         180       },
										{10000000000       , 10000000000       },
										{-1000000000       , 10000000000       }};
	// clang-format on

	// Turn on error mode so that validation is not skipped.
	auto guard = ErrorModeLock(ErrorMode::DIE);

	for (Size i = 0; i < num_rows(query_coordinates); ++i) {
		auto undulation = EXPECT_WARN(geoid_minus_ellipsoid(query_coordinates(i, 0) * DEG2RAD,
		                                                    query_coordinates(i, 1) * DEG2RAD),
		                              "outside range");
		EXPECT_FALSE(undulation.first);
	}
	const auto nan_point = NAN;
	auto undulation      = EXPECT_WARN(geoid_minus_ellipsoid(nan_point, nan_point), "not a number");
	EXPECT_FALSE(undulation.first);
}

TEST(hae_to_msl, valid_lat_lon_SLOW) {
	// clang-format off
	const Matrix query_coordinates =   {{ 90        ,  35        },
										{ 38.6281550, 269.7791550},
										{-14.6212170, 305.0211140},
										{ 46.8743190, 102.4487290},
										{-23.6174460, 133.8747120},
										{ 38.6254730, 359.9995000},
										{  -.4667440,    .0023000},
										{  0        ,   0        },
										{  0        , 360        },
										{  45       ,-500        },
										{  45       ,-360.00001  },
										{  45       , 500        },
										{  45       , 360.00001  }};
	// clang-format on

	const Vector expected_undulations = {86.394,
	                                     131.628,
	                                     102.969,
	                                     143.575,
	                                     84.229,
	                                     49.944,
	                                     82.671,
	                                     82.838,
	                                     82.838,
	                                     123.638,
	                                     52.86,
	                                     71.844,
	                                     52.86};

	// Turn on error mode so that validation is not skipped.
	auto guard = ErrorModeLock(ErrorMode::DIE);

	const double hae    = 100;
	auto resulting_msls = zeros(num_rows(query_coordinates));
	for (Size i = 0; i < num_rows(query_coordinates); ++i) {
		auto msl =
		    hae_to_msl(hae, query_coordinates(i, 0) * DEG2RAD, query_coordinates(i, 1) * DEG2RAD);
		EXPECT_TRUE(msl.first);
		resulting_msls(i) = msl.second;
	}

	ASSERT_ALLCLOSE_EX(expected_undulations, resulting_msls, 0.05, 0.0);
}

TEST(hae_to_msl, invalid_lat_lon) {
	// clang-format off
	const Matrix query_coordinates = {{        100       ,         180       },
									  {         90.00001 ,         180       },
									  {       -100       ,         180       },
									  {        -90.00001 ,         180       },
									  {10000000000       , 10000000000       },
									  {-1000000000       , 10000000000       }};
	// clang-format on

	const double hae = 100;

	// Turn on error mode so that validation is not skipped.
	auto guard = ErrorModeLock(ErrorMode::DIE);

	for (Size i = 0; i < num_rows(query_coordinates); ++i) {
		auto msl = EXPECT_WARN(
		    hae_to_msl(hae, query_coordinates(i, 0) * DEG2RAD, query_coordinates(i, 1) * DEG2RAD),
		    "outside range");
		EXPECT_FALSE(msl.first);
	}
	const auto nan_point = NAN;
	auto msl             = EXPECT_WARN(hae_to_msl(hae, nan_point, nan_point), "not a number");
	EXPECT_FALSE(msl.first);
}

TEST(msl_to_hae, valid_lat_lon_SLOW) {
	// clang-format off
	const Matrix query_coordinates =   {{  90        ,   35        },
										{  38.6281550,  269.7791550},
										{ -14.6212170,  305.0211140},
										{  46.8743190,  102.4487290},
										{ -23.6174460,  133.8747120},
										{  38.6254730,  359.9995000},
										{   -.4667440,     .0023000},
										{   0        ,    0        },
										{   0        ,  360        },
									    {   45       ,  -500       },
									    {   45       ,  -360.00001 },
									    {   45       ,   500       },
									    {   45       ,   360.00001 }};
	// clang-format on

	const Vector expected_haes = {113.606,
	                              68.372,
	                              97.031,
	                              56.425,
	                              115.871,
	                              150.066,
	                              117.329,
	                              117.162,
	                              117.162,
	                              76.362,
	                              147.14,
	                              128.156,
	                              147.14};

	const double msl    = 100;
	auto resulting_haes = zeros(num_rows(query_coordinates));

	// Turn on error mode so that validation is not skipped.
	auto guard = ErrorModeLock(ErrorMode::DIE);

	for (Size i = 0; i < num_rows(query_coordinates); ++i) {
		auto hae =
		    msl_to_hae(msl, query_coordinates(i, 0) * DEG2RAD, query_coordinates(i, 1) * DEG2RAD);
		EXPECT_TRUE(hae.first);
		resulting_haes(i) = hae.second;
	}

	ASSERT_ALLCLOSE_EX(expected_haes, resulting_haes, 0.05, 0.0);
}

TEST(msl_to_hae, invalid_lat_lon) {
	// clang-format off
	const Matrix query_coordinates = {{        100      ,         180       },
									 {         90.00001 ,         180       },
									 {       -100       ,         180       },
									 {        -90.00001 ,         180       },
									 {10000000000       , 10000000000       },
									 {-1000000000       , 10000000000       }};
	// clang-format on

	const double msl = 100;

	// Turn on error mode so that validation is not skipped.
	auto guard = ErrorModeLock(ErrorMode::DIE);

	for (Size i = 0; i < num_rows(query_coordinates); ++i) {
		auto hae = EXPECT_WARN(
		    msl_to_hae(msl, query_coordinates(i, 0) * DEG2RAD, query_coordinates(i, 1) * DEG2RAD),
		    "outside range");
		EXPECT_FALSE(hae.first);
	}
	const auto nan_point = NAN;
	auto hae             = EXPECT_WARN(msl_to_hae(msl, nan_point, nan_point), "not a number");
	EXPECT_FALSE(hae.first);
}

TEST(compare_tilt_corr, comp) {
	Vector3 rpy{0.3, -0.7, -0.4};
	Vector3 tilts{1e-4, 2e-4, 3e-4};
	// DCM that rotates a vector in the b frame to the n frame
	auto cnb = rpy_to_dcm(rpy);
	// Quat that does the same
	auto qnb = rpy_to_quat(rpy);

	// First, verify rotation equivalence
	auto r1 = dot(cnb, rpy);
	auto r2 = quat_rot(qnb, rpy);
	ASSERT_ALLCLOSE(r1, r2);

	// And double check that quat to dcm gives the same, since we will need it
	// to confirm tilt corrections the same
	auto cnb2 = quat_to_dcm(qnb);
	ASSERT_ALLCLOSE(cnb, cnb2);

	// Correct each with tilts. Unfortunately, it looks like the quat and dcm versions have
	// opposite expectations for the tilt vector. correct_dcm_with_tilt is written such that
	// it will accept pinson-estimated (additive error state) tilts directly, while all other
	// functions adhere to the definition of rotation vector given in our tutorial/T+W, requiring a
	// sign flip of the pinson states
	auto cnb_c  = correct_dcm_with_tilt(cnb, tilts);
	auto qnb_c  = correct_quat_with_tilt(qnb, -tilts);
	auto cnb_c2 = quat_to_dcm(qnb_c);
	auto b3     = navtk::navutils::rot_vec_to_dcm(-tilts);
	auto b4     = transpose(navtk::navutils::rot_vec_to_dcm(tilts));
	// Checking the correct_dcm_with_tilt example
	auto tilt_mag = sqrt(pow(tilts[0], 2) + pow(tilts[1], 2) + pow(tilts[2], 2));
	auto b5       = transpose(navtk::navutils::axis_angle_to_dcm(tilts / tilt_mag, tilt_mag));
	ASSERT_ALLCLOSE_EX(cnb_c, cnb_c2, 1e-14, 1e-14);
	ASSERT_ALLCLOSE_EX(cnb_c, dot(b3, cnb), 1e-14, 1e-14);
	ASSERT_ALLCLOSE_EX(cnb_c, dot(b4, cnb), 1e-14, 1e-14);
	ASSERT_ALLCLOSE_EX(cnb_c, dot(b5, cnb), 1e-14, 1e-14);
}

TEST(compare_tilt_corr, comp2) {
	Vector3 orig{0, -1, 0};
	// If we grab the frame that orig is in and rotate 90 deg right handed about z (a frame
	// rotation following the tutorial docs with a magnitude of PI/2 about the unit vector [0, 0,
	// 1]), then the vector orig will be pointing in the -x direction
	Vector3 expected{-1, 0, 0};
	auto dcm_est = eye(3);
	Vector3 tilt{0, 0, PI / 2.0};
	auto corr = correct_dcm_with_tilt(dcm_est, tilt);
	// If true, then tilt is a frame rot from est to corrected
	ASSERT_ALLCLOSE(expected, dot(corr, orig));
}

TEST(compare_tilt_corr, comp3) {
	Vector3 tilt{1e-5, -2e-5, -9e-5};
	ASSERT_ALLCLOSE(rpy_to_dcm(tilt), rot_vec_to_dcm(tilt));
}
