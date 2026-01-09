#include <stdexcept>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/filtering/virtualstateblocks/FirstOrderVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/PlatformToSensorCartesianVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/ScaleVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/SensorToPlatformCartesianVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/ShiftVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/StateExtractor.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::eye;
using navtk::Matrix;
using navtk::Vector;
using navtk::Vector3;
using navtk::filtering::EstimateWithCovariance;
using navtk::filtering::PlatformToSensorCartesianVirtualStateBlock;
using navtk::filtering::SensorToPlatformCartesianVirtualStateBlock;
using navtk::filtering::StateExtractor;
using navtk::navutils::correct_dcm_with_tilt;
using navtk::navutils::rpy_to_dcm;
using xt::diag;

struct VirtualStateBlockTests : public ::testing::Test {
	Vector nominal_no_rot;
	Vector nominal;
	Matrix diag_cov;
	Matrix full_cov;
	Vector3 la;
	navtk::Matrix3 i3;
	EstimateWithCovariance single_ec;
	EstimateWithCovariance five_ec;

	VirtualStateBlockTests()
	    : ::testing::Test(),
	      nominal_no_rot({100000.0, 200000.0, 300000.0, 0.0, 0.0, 0.0}),
	      nominal({100000.0, 200000.0, 300000.0, 0.1, 0.2, 0.3}),
	      diag_cov(diag(Vector{3.0, 2.0, 3.0, 0.005, 0.0016, 0.0021})),
	      full_cov({{3.0, 0.1, 0.2, 0.3, 0.4, 0.5},
	                {0.1, 2.0, -0.4, -0.75, 0.8, 0.9},
	                {0.2, -0.4, 3.0, 0.6, 0.2, 0.15},
	                {0.3, -0.75, 0.6, 0.75, 0.22, 0.11},
	                {0.4, 0.8, 0.2, 0.22, 0.16, 0.55},
	                {0.5, 0.9, 0.15, 0.11, 0.55, 0.21}}),
	      la({10.0, 20.0, 30.0}),
	      i3(eye(3)),
	      single_ec({1.0}, {{10.0}}),
	      five_ec({0.0, 1.0, 2.0, 3.0, 4.0},
	              {{0.00, 0.01, 0.02, 0.03, 0.04},
	               {0.01, 0.11, 0.12, 0.13, 0.14},
	               {0.02, 0.12, 0.22, 0.23, 0.24},
	               {0.03, 0.13, 0.23, 0.33, 0.34},
	               {0.04, 0.14, 0.24, 0.34, 0.44}}) {}
};

void test_first_order_eq(const EstimateWithCovariance& first_ord,
                         const EstimateWithCovariance& cvt) {
	ASSERT_ALLCLOSE(first_ord.estimate, cvt.estimate);
	ASSERT_ALLCLOSE(first_ord.covariance, cvt.covariance);
}

void test_jac_eq(const Matrix& jac_man, const Matrix& jac_num) {
	ASSERT_ALLCLOSE(jac_man, jac_num);
}

/**
 * Shared testing functionality- compare first order numerical results vs. derived
 *
 * @param start_point Alias to be converted (starting state/covariance)
 * @param tx Transformer to be tested
 *
 * @return Result of tx.convert(start_point), or may die of a failed assert
 */
EstimateWithCovariance test_transform(const EstimateWithCovariance& start_point,
                                      navtk::filtering::VirtualStateBlock& tx) {
	// Have to wrap tx.convert to pass into numerical approximators, reattaching
	// covariance to x
	std::function<Vector(const Vector&)> wrapped = [&](const Vector& x) {
		EstimateWithCovariance ec{x, start_point.covariance};
		return tx.convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0)).estimate;
	};

	auto first_ord = navtk::filtering::first_order_approx(start_point, wrapped);
	auto cvt       = tx.convert(start_point, aspn_xtensor::TypeTimestamp((int64_t)0));

	auto jac_man = tx.jacobian(start_point.estimate, aspn_xtensor::TypeTimestamp((int64_t)0));
	auto jac_num = navtk::filtering::calc_numerical_jacobian(wrapped, start_point.estimate, 1e-4);

	// Compiler doesn't like ASSERT macros in a function that returns non-void...
	test_first_order_eq(first_ord, cvt);
	test_jac_eq(jac_man, jac_num);

	return cvt;
}

TEST_F(VirtualStateBlockTests, s2bNoRotCovDiag_SLOW) {
	auto start_point = EstimateWithCovariance(nominal_no_rot, diag_cov);
	auto tx =
	    SensorToPlatformCartesianVirtualStateBlock("wholeBase", "shiftedIdentity", la, i3, i3);

	auto cvt = test_transform(start_point, tx);

	// Results from 100000 Monte carlo runs ie
	// auto mcOut    = navtk::filtering::monte_carlo_approx(start_point.state, wrapped, 100000);
	Vector mc_est{99990.017227, 199980.061172, 299970.098514, -0.000175, 0.000154, -0.000104};

	Matrix mc_cov{
	    {5.290968e+00, -4.139780e-01, -4.680565e-01, 9.918414e-05, -4.790088e-02, 4.196713e-02},
	    {-4.139780e-01, 6.680558e+00, -2.983854e+00, 1.499344e-01, -3.608104e-04, -2.046508e-02},
	    {-4.680565e-01, -2.983854e+00, 5.184003e+00, -1.008157e-01, 1.607546e-02, -5.766601e-04},
	    {9.918414e-05, 1.499344e-01, -1.008157e-01, 5.034892e-03, -8.689377e-06, 2.146093e-05},
	    {-4.790088e-02, -3.608104e-04, 1.607546e-02, -8.689377e-06, 1.592189e-03, -3.367471e-07},
	    {4.196713e-02, -2.046508e-02, -5.766601e-04, 2.146093e-05, -3.367471e-07, 2.112003e-03}};

	// They compare less favorably to monte carlo results, but are close.
	// 1st approx has 0 in it, which rel tol always fails if non-zero
	// in compared, so have to add an absolute tol that covers the diff
	ASSERT_ALLCLOSE_EX(mc_cov, cvt.covariance, 3e-2, 1e-3);
	ASSERT_ALLCLOSE_EX(mc_est, cvt.estimate, 1e-6, 1e-3);
}

TEST_F(VirtualStateBlockTests, s2bNoRotCovFull_SLOW) {
	auto start_point = EstimateWithCovariance(nominal_no_rot, full_cov);
	auto tx =
	    SensorToPlatformCartesianVirtualStateBlock("wholeBase", "shiftedIdentity", la, i3, i3);

	auto cvt = test_transform(start_point, tx);

	Vector mc_est{99985.683438, 199988.258619, 299984.787975, 0.0005, -0.000621, -0.000195};

	Matrix mc_cov{{117.287692, -51.222396, -52.687762, 0.453276, -1.370431, -0.73484},
	              {-51.222396, 466.036974, -20.42805, 17.486702, 5.583602, 1.24511},
	              {-52.687762, -20.42805, 256.501216, -6.870598, -0.170579, 0.021921},
	              {0.453276, 17.486702, -6.870598, 1.171382, 0.380711, 0.121483},
	              {-1.370431, 5.583602, -0.170579, 0.380711, 0.322539, 0.087558},
	              {-0.73484, 1.24511, 0.021921, 0.121483, 0.087558, 0.08149}};

	// Large uncertainty + state correlation x skew lever arm =
	// breakdown of first order approx; negative terms on cov diagonal
	ASSERT_ALLCLOSE_EX(mc_est, cvt.estimate, 3e-2, 1e-3);
	ASSERT_ALLCLOSE_EX(mc_cov, cvt.covariance, 0, 1000);
}

TEST_F(VirtualStateBlockTests, s2bCovDiag_SLOW) {
	auto start_point          = EstimateWithCovariance(nominal_no_rot, diag_cov);
	auto C_platform_to_sensor = xt::transpose(rpy_to_dcm(Vector{-1.0, 0.4, 2.1}));
	auto C_k_to_j             = xt::transpose(rpy_to_dcm(Vector{0.6, 2.4, -1.1}));
	auto tx                   = SensorToPlatformCartesianVirtualStateBlock(
        "wholeBase", "shiftedIdentity", la, C_platform_to_sensor, C_k_to_j);

	auto cvt = test_transform(start_point, tx);

	// Results from 100000 Monte carlo runs
	Vector mc_est{100025.509608, 200002.396246, 299972.905122, -0.458656, 0.980518, -2.571457};

	Matrix mc_cov{{6.277642, 0.985961, 3.157915, 0.183626, -0.065783, 0.179431},
	              {0.985961, 5.654459, 1.259092, 0.122087, -0.04061, 0.029885},
	              {3.157915, 1.259092, 6.077337, 0.181312, -0.066321, 0.169197},
	              {0.183626, 0.122087, 0.181312, 0.013157, -0.002796, 0.010934},
	              {-0.065783, -0.04061, -0.066321, -0.002796, 0.002597, -0.002313},
	              {0.179431, 0.029885, 0.169197, 0.010934, -0.002313, 0.011202}};

	ASSERT_ALLCLOSE_EX(mc_est, cvt.estimate, 1e-2, 1e-3);
	ASSERT_ALLCLOSE_EX(mc_cov, cvt.covariance, 6e-2, 1e-3);
}

TEST_F(VirtualStateBlockTests, b2sNoRotCovDiag_SLOW) {
	// This transform has the same issues as sensor_to_platform, so the Monte Carlo
	// comparisons have been removed
	auto start_point = EstimateWithCovariance(nominal_no_rot, diag_cov);
	auto tx =
	    PlatformToSensorCartesianVirtualStateBlock("wholeBase", "shiftedIdentity", la, i3, i3);
	test_transform(start_point, tx);
}

TEST_F(VirtualStateBlockTests, b2sNoRotCovFull_SLOW) {
	auto start_point = EstimateWithCovariance(nominal_no_rot, full_cov);
	auto tx =
	    PlatformToSensorCartesianVirtualStateBlock("wholeBase", "shiftedIdentity", la, i3, i3);
	test_transform(start_point, tx);
}

TEST_F(VirtualStateBlockTests, b2sCovDiag_SLOW) {
	auto start_point          = EstimateWithCovariance(nominal_no_rot, diag_cov);
	auto C_platform_to_sensor = xt::transpose(rpy_to_dcm(Vector{-1.0, 0.4, 2.1}));
	auto C_k_to_j             = xt::transpose(rpy_to_dcm(Vector{0.6, 2.4, -1.1}));
	auto tx                   = PlatformToSensorCartesianVirtualStateBlock(
        "wholeBase", "shiftedIdentity", la, C_platform_to_sensor, C_k_to_j);
	test_transform(start_point, tx);
}

TEST_F(VirtualStateBlockTests, b2s_inversion) {
	auto start_point          = EstimateWithCovariance(nominal_no_rot, full_cov);
	auto C_platform_to_sensor = xt::transpose(rpy_to_dcm(Vector{-1.0, 0.4, 2.1}));
	auto C_k_to_j             = xt::transpose(rpy_to_dcm(Vector{0.6, 2.4, -1.1}));
	auto tx                   = PlatformToSensorCartesianVirtualStateBlock(
        "wholeBase", "shiftedIdentity", la, C_platform_to_sensor, C_k_to_j);
	auto tx2 = SensorToPlatformCartesianVirtualStateBlock(
	    "shiftedIdentity", "wholeBase", la, C_platform_to_sensor, C_k_to_j);
	auto fin = tx2.convert(tx.convert(start_point, aspn_xtensor::TypeTimestamp((int64_t)0)),
	                       aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(fin.estimate, start_point.estimate);
	// Somewhat surprisingly, the 'corrupt' covariance is mapped back to the
	// original correctly
	ASSERT_ALLCLOSE(fin.covariance, start_point.covariance);
}

TEST_F(VirtualStateBlockTests, genericShiftVsSpecific) {
	auto start_point          = EstimateWithCovariance(nominal_no_rot, full_cov);
	auto C_platform_to_sensor = xt::transpose(rpy_to_dcm(Vector{-1.0, 0.4, 2.1}));
	auto C_k_to_j             = xt::transpose(rpy_to_dcm(Vector{0.6, 2.4, -1.1}));
	auto tx                   = PlatformToSensorCartesianVirtualStateBlock(
        "wholeBase", "shiftedIdentity", la, C_platform_to_sensor, C_k_to_j);

	// Wrapping these functions is a little underhanded since they'll have
	// access to the state of the other transformer, but it's way easier
	std::function<Vector(const Vector&, const Vector3&, const Matrix&)> wrapped_fx =
	    [&](const Vector& x, const Vector3&, const Matrix&) {
		    EstimateWithCovariance ec{x, start_point.covariance};
		    return tx.convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0)).estimate;
	    };

	std::function<Matrix(const Vector&, const Vector3&, const Matrix&)> wrapped_jx =
	    [&](const Vector& x, const Vector3&, const Matrix&) {
		    return tx.jacobian(x, aspn_xtensor::TypeTimestamp((int64_t)0));
	    };

	auto tx2 = navtk::filtering::ShiftVirtualStateBlock(
	    "wholeBase", "shiftedIdentity", la, C_platform_to_sensor, wrapped_fx, wrapped_jx);

	auto cvt1 = tx.convert(start_point, aspn_xtensor::TypeTimestamp((int64_t)0));
	auto cvt2 = tx2.convert(start_point, aspn_xtensor::TypeTimestamp((int64_t)0));
	auto jac1 = tx.jacobian(start_point.estimate, aspn_xtensor::TypeTimestamp((int64_t)0));
	auto jac2 = tx2.jacobian(start_point.estimate, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(cvt1.estimate, cvt2.estimate);
	ASSERT_ALLCLOSE(cvt1.covariance, cvt2.covariance);
	ASSERT_ALLCLOSE(jac1, jac2);
}

TEST_F(VirtualStateBlockTests, testScale) {
	auto start_point = EstimateWithCovariance(nominal, full_cov);
	Vector sf{10.0, -0.6, 2.4, 0.002, 8.1, -0.987};
	auto tx = navtk::filtering::ScaleVirtualStateBlock("wholeBase", "scaled", sf);
	test_transform(start_point, tx);
	// No need for Monte Carlo check because 1st order captures all info
}

Vector3 old_dcm_to_rpy2(const navtk::Matrix3& dcm) {
	auto r = atan2(dcm(1, 2), dcm(2, 2));
	auto p = -asin(dcm(0, 2));
	auto y = atan2(dcm(0, 1), dcm(0, 0));
	return {r, p, y};
}

/*
 * Uses the 'classic' version of rpy_to_dcm because of a small numerical difference
 * because the jacobian (tilt_corr_jac_simple) matches that version of the function.
 * TODO: PNTOS-352
 */
Vector tilt_corr_simple(const Vector& tilt, const Vector& rpy) {
	return old_dcm_to_rpy2(
	    navtk::dot(eye(3) - navtk::navutils::skew(tilt), xt::transpose(rpy_to_dcm(rpy))));
}

// 'Work in progress' implementations that do not belong to any
// Transformers as yet but will figure into some in the near future
Matrix tilt_corr_jac_simple(const Vector3& tilts, const Vector3& rpy) {
	auto C_nav_to_platform           = xt::transpose(rpy_to_dcm(rpy));
	auto C_nav_to_platform_corrected = correct_dcm_with_tilt(C_nav_to_platform, tilts);
	auto den1 =
	    pow(C_nav_to_platform_corrected(1, 2), 2.0) + pow(C_nav_to_platform_corrected(2, 2), 2.0);
	auto den3 =
	    pow(C_nav_to_platform_corrected(0, 1), 2.0) + pow(C_nav_to_platform_corrected(0, 0), 2.0);

	Matrix dx{{0.0, 0.0, 0.0},
	          {C_nav_to_platform(2, 0), C_nav_to_platform(2, 1), C_nav_to_platform(2, 2)},
	          {-C_nav_to_platform(1, 0), -C_nav_to_platform(1, 1), -C_nav_to_platform(1, 2)}};
	Matrix dy{{-C_nav_to_platform(2, 0), -C_nav_to_platform(2, 1), -C_nav_to_platform(2, 2)},
	          {0, 0, 0},
	          {C_nav_to_platform(0, 0), C_nav_to_platform(0, 1), C_nav_to_platform(0, 2)}};
	Matrix dz{{C_nav_to_platform(1, 0), C_nav_to_platform(1, 1), C_nav_to_platform(1, 2)},
	          {-C_nav_to_platform(0, 0), -C_nav_to_platform(0, 1), -C_nav_to_platform(0, 2)},
	          {0, 0, 0}};

	auto d1dx = (dx(1, 2) * C_nav_to_platform_corrected(2, 2) -
	             C_nav_to_platform_corrected(1, 2) * dx(2, 2)) /
	            den1;
	auto d1dy = (dy(1, 2) * C_nav_to_platform_corrected(2, 2) -
	             C_nav_to_platform_corrected(1, 2) * dy(2, 2)) /
	            den1;
	auto d1dz = (dz(1, 2) * C_nav_to_platform_corrected(2, 2) -
	             C_nav_to_platform_corrected(1, 2) * dz(2, 2)) /
	            den1;

	auto d2dx = -dx(0, 2) / sqrt(1.0 - pow(C_nav_to_platform_corrected(0, 2), 2.0));
	auto d2dy = -dy(0, 2) / sqrt(1.0 - pow(C_nav_to_platform_corrected(0, 2), 2.0));
	auto d2dz = -dz(0, 2) / sqrt(1.0 - pow(C_nav_to_platform_corrected(0, 2), 2.0));

	auto d3dx = (dx(0, 1) * C_nav_to_platform_corrected(0, 0) -
	             C_nav_to_platform_corrected(0, 1) * dx(0, 0)) /
	            den3;
	auto d3dy = (dy(0, 1) * C_nav_to_platform_corrected(0, 0) -
	             C_nav_to_platform_corrected(0, 1) * dy(0, 0)) /
	            den3;
	auto d3dz = (dz(0, 1) * C_nav_to_platform_corrected(0, 0) -
	             C_nav_to_platform_corrected(0, 1) * dz(0, 0)) /
	            den3;
	Matrix dump{{d1dx, d1dy, d1dz}, {d2dx, d2dy, d2dz}, {d3dx, d3dy, d3dz}};
	return dump;
}

Matrix tilt_corr_jac(const Vector3& tilts, const Vector3& rpy) {
	/*
	 * TODO:
	 * The derivatives have been checked repeatedly but there is still a
	 * small difference between the return value of this function and the
	 * equivalent numerical Jacobian calculation.
	 *
	 * The one 'hint' at what might be wrong- if the 'sig2' value (the sum
	 * of the squares of the tilt angles) is set to an individual tilt
	 * (and all derived powers change accordingly), then the column of the
	 * Jacobian corresponding to that angle will match the numeric
	 * calculation. Whether this indicates an error in the derivation or a
	 * higher order effect that the numeric Jacobian does not capture I am
	 * unsure.
	 */
	auto C_nav_to_platform           = xt::transpose(rpy_to_dcm(rpy));
	auto C_nav_to_platform_corrected = correct_dcm_with_tilt(C_nav_to_platform, tilts);

	auto sig2 = xt::sum(xt::pow(tilts, 2))[0];
	auto sig  = sqrt(sig2);
	auto sig3 = sig2 * sig;
	auto sig4 = sig2 * sig2;
	auto sSig = sin(sig);
	auto cSig = cos(sig);
	auto sk   = navtk::navutils::skew(tilts);

	Matrix d_skdx{{0, 0, 0}, {0, 0, -1}, {0, 1, 0}};
	Matrix d_skdy{{0, 0, 1}, {0, 0, 0}, {-1, 0, 0}};
	Matrix d_skdz{{0, -1, 0}, {1, 0, 0}, {0, 0, 0}};

	auto dx1 = tilts[0] * (-cSig * sig + sSig) / sig3 * sk - (sSig / sig) * d_skdx;
	auto dy1 = tilts[1] * (-cSig * sig + sSig) / sig3 * sk - (sSig / sig) * d_skdy;
	auto dz1 = tilts[2] * (-cSig * sig + sSig) / sig3 * sk - (sSig / sig) * d_skdz;

	auto dx2 = (sSig * sig2 - 2.0 * tilts[0] * (1 - cSig)) / sig4 * navtk::dot(sk, sk) +
	           ((1.0 - cSig) / sig2) * (navtk::dot(d_skdx, sk) + navtk::dot(sk, d_skdx));
	auto dy2 = (sSig * sig2 - 2.0 * tilts[1] * (1 - cSig)) / sig4 * navtk::dot(sk, sk) +
	           ((1.0 - cSig) / sig2) * (navtk::dot(d_skdy, sk) + navtk::dot(sk, d_skdy));
	auto dz2 = (sSig * sig2 - 2.0 * tilts[2] * (1 - cSig)) / sig4 * navtk::dot(sk, sk) +
	           ((1.0 - cSig) / sig2) * (navtk::dot(d_skdz, sk) + navtk::dot(sk, d_skdz));

	auto dx = navtk::dot(dx1 + dx2, C_nav_to_platform);
	auto dy = navtk::dot(dy1 + dy2, C_nav_to_platform);
	auto dz = navtk::dot(dz1 + dz2, C_nav_to_platform);

	auto den1 =
	    pow(C_nav_to_platform_corrected(1, 2), 2) + pow(C_nav_to_platform_corrected(2, 2), 2);
	auto den3 =
	    pow(C_nav_to_platform_corrected(0, 1), 2) + pow(C_nav_to_platform_corrected(0, 0), 2);

	auto d1dx = (dx(1, 2) * C_nav_to_platform_corrected(2, 2) -
	             C_nav_to_platform_corrected(1, 2) * dx(2, 2)) /
	            den1;
	auto d1dy = (dy(1, 2) * C_nav_to_platform_corrected(2, 2) -
	             C_nav_to_platform_corrected(1, 2) * dy(2, 2)) /
	            den1;
	auto d1dz = (dz(1, 2) * C_nav_to_platform_corrected(2, 2) -
	             C_nav_to_platform_corrected(1, 2) * dz(2, 2)) /
	            den1;

	auto d2dx = -dx(0, 2) / sqrt(1.0 - pow(C_nav_to_platform_corrected(0, 2), 2));
	auto d2dy = -dy(0, 2) / sqrt(1.0 - pow(C_nav_to_platform_corrected(0, 2), 2));
	auto d2dz = -dz(0, 2) / sqrt(1.0 - pow(C_nav_to_platform_corrected(0, 2), 2));

	auto d3dx = (dx(0, 1) * C_nav_to_platform_corrected(0, 0) -
	             C_nav_to_platform_corrected(0, 1) * dx(0, 0)) /
	            den3;
	auto d3dy = (dy(0, 1) * C_nav_to_platform_corrected(0, 0) -
	             C_nav_to_platform_corrected(0, 1) * dy(0, 0)) /
	            den3;
	auto d3dz = (dz(0, 1) * C_nav_to_platform_corrected(0, 0) -
	             C_nav_to_platform_corrected(0, 1) * dz(0, 0)) /
	            den3;
	Matrix dump{{d1dx, d1dy, d1dz}, {d2dx, d2dy, d2dz}, {d3dx, d3dy, d3dz}};
	return dump;
}

TEST_F(VirtualStateBlockTests, testTiltCorr) {
	Vector3 rpy{1.0, -1.2, 2.04};
	Vector3 tilts{0.00034, 0.00012, -0.00056};
	auto tilt_cov = diag(Vector{0.0001, 0.0002, 0.0003});

	std::function<Vector(const Vector&)> fx = [&](const Vector& x) {
		return tilt_corr_simple(x, rpy);
	};
	std::function<Matrix(const Vector&)> jx = [&](const Vector& x) {
		return tilt_corr_jac_simple(x, rpy);
	};

	auto start_point = EstimateWithCovariance(tilts, tilt_cov);
	auto tx          = navtk::filtering::FirstOrderVirtualStateBlock("errState", "corr", fx, jx);
	auto cvt         = test_transform(start_point, tx);

	// small tilt mc results- 150000 iterations of fx
	Vector mc_out_est{1.000666, -1.199758, 2.039663};

	Matrix mc_out_cov{{0.001612696790351, 0.000119299362384, -0.001623109327946},
	                  {0.000119299362384, 0.0002716626364844, -0.0001290778325520},
	                  {-0.001623109327946, -0.0001290778325520, 0.001742012472697}};

	// Linearization error is sensitive to everything- 4% is representative
	// of 'good' inputs, and error may generally be larger especially in
	// cross covariance terms
	ASSERT_ALLCLOSE_EX(mc_out_est, cvt.estimate, 4e-2, 0.0);
	ASSERT_ALLCLOSE_EX(mc_out_cov, cvt.covariance, 4e-2, 0.0);
}

TEST(JacobianTests, nullJacArg) {
	std::function<Vector(const Vector&)> fx = [&](const Vector& x) { return x; };
	auto tx   = navtk::filtering::FirstOrderVirtualStateBlock("A", "B", fx);
	auto jac1 = tx.jacobian(navtk::zeros(1), aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(jac1, eye(1));
}

TEST(JacobianTests, compareNumericalTiltCorr) {
	Vector3 rpy{0.3, -1.2, 2.6};
	Vector3 tilts{1e-5, 2e-5, 3e-5};

	// 'By hand'
	Matrix jac_simple = tilt_corr_jac_simple(tilts, rpy);
	Matrix jac        = tilt_corr_jac(tilts, rpy);

	std::function<Vector(const Vector&)> fx = [rpy = rpy](const Vector& tilt) {
		return navtk::navutils::dcm_to_rpy(
		    xt::transpose(correct_dcm_with_tilt(xt::transpose(rpy_to_dcm(rpy)), tilt)));
	};

	std::function<Vector(const Vector&)> fx_simple = [rpy = rpy](const Vector& tilt) {
		return tilt_corr_simple(tilt, rpy);
	};

	// Numerically calculated
	Matrix via_numerical_simple = navtk::filtering::calc_numerical_jacobian(fx_simple, tilts, 1e-4);
	Matrix via_numerical        = navtk::filtering::calc_numerical_jacobian(fx, tilts, 1e-4);

	ASSERT_ALLCLOSE_EX(via_numerical_simple, jac_simple, 0.0, 1e-7);

	// This test is not strenuous enough. There is a minor difference
	// in the 2 calculations but they are 'close'; the error is in the
	// ballpark of what you'd get from doing the first order
	// approximation. Details and TODO are in the function implementation.
	ASSERT_ALLCLOSE_EX(via_numerical, jac, 0.0, 1e-4);
}

TEST_F(VirtualStateBlockTests, Extractor0) {
	// If incoming_state_size parameter 0 or provided block 0 throw invalid_argument
	EXPECT_UB_OR_DIE(StateExtractor("a", "b", 0, {0}),
	                 "Argument incoming_state_size must be 1 or greater.",
	                 std::invalid_argument);
}

TEST_F(VirtualStateBlockTests, Extractor1) {
	// Effectively just a alias rename
	auto ex        = StateExtractor("a", "b", 1, {0});
	auto converted = ex.convert(single_ec, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(converted.estimate, single_ec.estimate);
	ASSERT_ALLCLOSE(converted.covariance, single_ec.covariance);
	ASSERT_ALLCLOSE(ex.jacobian(single_ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0)),
	                eye(1));
}

TEST_F(VirtualStateBlockTests, AllNormal) {
	auto ex        = StateExtractor("a", "b", 5, {0, 1, 2, 3, 4});
	auto converted = ex.convert(five_ec, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(converted.estimate, five_ec.estimate);
	ASSERT_ALLCLOSE(converted.covariance, five_ec.covariance);
	ASSERT_ALLCLOSE(ex.jacobian(single_ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0)),
	                eye(5));
}

TEST_F(VirtualStateBlockTests, AllReordered) {
	Matrix ex_jac{
	    {0, 0, 0, 1, 0}, {0, 0, 1, 0, 0}, {0, 0, 0, 0, 1}, {1, 0, 0, 0, 0}, {0, 1, 0, 0, 0}};
	Vector ex_state{3.0, 2.0, 4.0, 0.0, 1.0};
	Matrix ex_cov{{0.33, 0.23, 0.34, 0.03, 0.13},
	              {0.23, 0.22, 0.24, 0.02, 0.12},
	              {0.34, 0.24, 0.44, 0.04, 0.14},
	              {0.03, 0.02, 0.04, 0.0, 0.01},
	              {0.13, 0.12, 0.14, 0.01, 0.11}};
	auto ex        = StateExtractor("a", "b", 5, {3, 2, 4, 0, 1});
	auto converted = ex.convert(five_ec, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(converted.estimate, ex_state);
	ASSERT_ALLCLOSE(converted.covariance, ex_cov);
	ASSERT_ALLCLOSE(ex.jacobian(single_ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0)),
	                ex_jac);
}

TEST_F(VirtualStateBlockTests, ExtractorOrderedChunk) {
	Matrix ex_jac{{1, 0, 0, 0, 0}, {0, 1, 0, 0, 0}, {0, 0, 1, 0, 0}};
	Vector ex_state = xt::view(five_ec.estimate, xt::range(0, 3));
	Matrix ex_cov   = xt::view(five_ec.covariance, xt::range(0, 3), xt::range(0, 3));
	auto ex         = StateExtractor("a", "b", 5, {0, 1, 2});
	auto converted  = ex.convert(five_ec, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(converted.estimate, ex_state);
	ASSERT_ALLCLOSE(converted.covariance, ex_cov);
	ASSERT_ALLCLOSE(ex.jacobian(single_ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0)),
	                ex_jac);
}

TEST_F(VirtualStateBlockTests, ExtractorManyOrdered) {
	Matrix ex_jac{{1, 0, 0, 0, 0}, {0, 0, 1, 0, 0}, {0, 0, 0, 0, 1}};
	Vector ex_state{0.0, 2.0, 4.0};
	Matrix ex_cov{{0.0, 0.02, 0.04}, {0.02, 0.22, 0.24}, {0.04, 0.24, 0.44}};
	auto ex        = StateExtractor("a", "b", 5, {0, 2, 4});
	auto converted = ex.convert(five_ec, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(converted.estimate, ex_state);
	ASSERT_ALLCLOSE(converted.covariance, ex_cov);
	ASSERT_ALLCLOSE(ex.jacobian(single_ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0)),
	                ex_jac);
}

TEST_F(VirtualStateBlockTests, ExtractorManyOrderAltered) {
	Matrix ex_jac{{0, 0, 1, 0, 0}, {0, 0, 0, 0, 1}, {0, 1, 0, 0, 0}};
	Vector ex_state{2.0, 4.0, 1.0};
	Matrix ex_cov{{0.22, 0.24, 0.12}, {0.24, 0.44, 0.14}, {0.12, 0.14, 0.11}};
	auto ex        = StateExtractor("a", "b", 5, {2, 4, 1});
	auto converted = ex.convert(five_ec, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(converted.estimate, ex_state);
	ASSERT_ALLCLOSE(converted.covariance, ex_cov);
	ASSERT_ALLCLOSE(ex.jacobian(single_ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0)),
	                ex_jac);
}

TEST_F(VirtualStateBlockTests, OOB) {
	// Some index too large, throw out_of_range
	EXPECT_UB_OR_DIE(
	    StateExtractor("a", "b", 5, {5}),
	    "One or more indices provided exceeds the length of the expected state vector.",
	    std::invalid_argument);
	EXPECT_UB_OR_DIE(
	    StateExtractor("a", "b", 5, {0, 1, 5, 3}),
	    "One or more indices provided exceeds the length of the expected state vector.",
	    std::invalid_argument);
}

TEST_F(VirtualStateBlockTests, InvalidStateSize) {
	EXPECT_UB_OR_DIE(StateExtractor("a", "b", 0, {1}),
	                 "Argument incoming_state_size must be 1 or greater.",
	                 std::invalid_argument);
}

TEST_F(VirtualStateBlockTests, ExtractorChain) {
	Matrix ex_jac{{0, 0, 1, 0, 0}, {0, 0, 0, 0, 1}};
	Vector ex_state{2.0, 4.0};
	Matrix ex_cov{{0.22, 0.24}, {0.24, 0.44}};
	auto ex1       = StateExtractor("a", "b", 5, {2, 3, 4});
	auto ex2       = StateExtractor("a", "b", 3, {0, 2});
	auto c1        = ex1.convert(five_ec, aspn_xtensor::TypeTimestamp((int64_t)0));
	auto converted = ex2.convert(c1, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(converted.estimate, ex_state);
	ASSERT_ALLCLOSE(converted.covariance, ex_cov);
	ASSERT_ALLCLOSE(
	    navtk::dot(ex2.jacobian(c1.estimate, aspn_xtensor::TypeTimestamp((int64_t)0)),
	               ex1.jacobian(single_ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0))),
	    ex_jac);
}

TEST_F(VirtualStateBlockTests, RepeatIndices) {
	EXPECT_UB_OR_DIE(StateExtractor("a", "b", 5, {0, 1, 2, 1}),
	                 "Repeat indices are not allowed.",
	                 std::invalid_argument);
}

TEST_F(VirtualStateBlockTests, WrongStateSize) {
	auto ex = StateExtractor("a", "b", 5, {0, 1, 2, 3});
	EXPECT_UB_OR_DIE(ex.convert(single_ec, aspn_xtensor::TypeTimestamp((int64_t)0)),
	                 "State block to alias does not contain the expected number of states.",
	                 std::invalid_argument);
}

TEST_F(VirtualStateBlockTests, EmptyIndices) {
	EXPECT_UB_OR_DIE(StateExtractor("a", "b", 5, {}),
	                 "Must provide at least 1 index for an element to keep.",
	                 std::invalid_argument);
}
