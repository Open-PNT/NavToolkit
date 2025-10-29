#include <gtest/gtest.h>
#include <tensor_assert.hpp>
#include <xtensor/views/xview.hpp>

#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

using navtk::dot;
using navtk::eye;
using navtk::Matrix;
using navtk::Matrix3;
using navtk::num_cols;
using navtk::num_rows;
using navtk::Vector;
using navtk::Vector3;
using navtk::navutils::ortho_dcm;
using navtk::navutils::PI;
using navtk::navutils::rpy_to_dcm;
using navtk::navutils::wrap_to_pi;
using navtk::utils::repr;
using xt::range;
using xt::transpose;
using xt::view;

/**
 * Verify that the ortho_dcm function produces an orthonormalized DCM
 * that represents a more accurate rotation that the non orthonormalized
 * DCM.
 *
 * @param original 3x3 'perfect' DCM to use as baseline.
 * @param corrupt_mat 3x3 DCM that is not orthonormal; corrupted version
 * of original.
 */
void orthog_test(const Matrix3& original, const Matrix3& corrupt_mat) {
	auto cv       = Vector{1.65, 2.75, 6.61};
	auto expected = dot(original, cv);
	auto be_ortho = ortho_dcm(corrupt_mat);

	// Orthogonality
	ASSERT_ALLCLOSE_EX(dot(be_ortho, transpose(be_ortho)), eye(3), 1e-15, 1e-15);

	// Normality
	for (int k = 0; k < 3; k++) {
		auto row = view(be_ortho, range(k, k + 1), xt::all());
		auto col = view(be_ortho, xt::all(), range(k, k + 1));
		ASSERT_TRUE(std::abs(navtk::norm(navtk::to_matrix(row)) - 1.0) < 1e-15);
		ASSERT_TRUE(std::abs(navtk::norm(navtk::to_matrix(col)) - 1.0) < 1e-15);
	}

	// Valid rotation
	auto pre_ortho_rotated_error  = navtk::norm(navtk::to_vec(expected - dot(corrupt_mat, cv)));
	auto post_ortho_rotated_error = navtk::norm(navtk::to_vec(expected - dot(be_ortho, cv)));
	ASSERT_LE(post_ortho_rotated_error, pre_ortho_rotated_error);
}

struct OrthoTest : public ::testing::Test {
	Matrix3 m;
	Matrix3 m_corrupt;
	Matrix3 big_nasty;
	OrthoTest()
	    : m(xt::transpose(rpy_to_dcm(Vector3{0.1, 0.2, 0.3}))),
	      m_corrupt(Matrix({{0.0, 0.0, 0.0001}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}})),
	      big_nasty(Matrix(
	          {{0.1, -0.0074, 0.003}, {0.0002, -0.001, 0.004}, {0.000027, 0.0006, -0.0000089}})) {
		m_corrupt += m;
		big_nasty += m;
	}
};

TEST_F(OrthoTest, isOrthogonal) { orthog_test(m, m_corrupt); }

TEST_F(OrthoTest, isOrthogonalBigNasty) { orthog_test(m, big_nasty); }

// TODO: Failure on some platforms when using BLAS/LAPACK.  See
// https://git.aspn.us/pntos/navtk/-/issues/104
TEST_F(OrthoTest, DISABLED_isAlreadyOrtho) { orthog_test(m, m); }

TEST_F(OrthoTest, isAlreadyOrthoEye) { orthog_test(eye(3), eye(3)); }

void test_wrapping(double val, double expected) {
	ASSERT_FLOAT_EQ(expected, wrap_to_pi(val));
	ASSERT_FLOAT_EQ(expected, wrap_to_pi(expected));
	ASSERT_FLOAT_EQ(expected, wrap_to_pi(val + 2 * PI));
	ASSERT_FLOAT_EQ(expected, wrap_to_pi(val + 10 * PI));
	ASSERT_FLOAT_EQ(expected, wrap_to_pi(val - 2 * PI));
	ASSERT_FLOAT_EQ(expected, wrap_to_pi(val - 10 * PI));
}

TEST(WrapTest, wrap_stuff) {
	// Yes, the last 2 test values look odd. This is because 10 * PI + 1e-12 - 10 * PI != 1e-12 to
	// the precision required by ASSERT_FLOAT_EQ, so forcing the rounding to happen before sending
	// it into the algorithm (this rounding error is inflicted in the last step of the wrap
	// algorithm). For raw values, somewhere between 1e-9 and 1e-10 is the smallest magnitude angle
	// that will pass this test without the pre-rounding.
	std::vector<double> to_test{0.0,
	                            PI,
	                            PI / 4.0,
	                            -PI / 4.0,
	                            3.0 * PI / 4.0,
	                            -PI / 4.0,
	                            PI / 2.0,
	                            -PI / 2.0,
	                            PI - 1e-12,
	                            -PI + 1e-12,
	                            10 * PI + 1e-12 - 10 * PI,
	                            10 * PI + -1e-12 - 10 * PI};
	for (auto d = to_test.begin(); d != to_test.end(); d++) {
		test_wrapping(*d, *d);
	}

	// The one case where the raw value doesn't equal expected
	test_wrapping(-PI, PI);
}
