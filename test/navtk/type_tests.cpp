#include <type_traits>

#include <gtest/gtest.h>
#include <tensor_assert.hpp>

#include <navtk/tensors.hpp>

using navtk::eye;
using navtk::Matrix;
using navtk::num_cols;
using navtk::num_rows;
using navtk::ones;
using navtk::to_matrix;
using navtk::to_vec;
using navtk::Vector;
using navtk::zeros;

// To check for compiler conversion warnings, comment out "using navtk::Size" below and typedef
// Size to a different type.  Enable -Wall, -Wextra, -Wcomparison to catch all compiler
// warnings.
using navtk::Size;
// typedef char Size;
// typedef long long Size;


// The purpose of this test is to quickly give an error if the platform that
// it is compiled for has a mismatch between what we are using for Size
// and what xtensor is using for their size_type.
TEST(TypeTests, NavtkSize) {

	auto is_same_result = std::is_same<navtk::Vector::size_type, Size>::value;
	EXPECT_TRUE(is_same_result);

	is_same_result = std::is_same<navtk::Matrix::size_type, Size>::value;
	EXPECT_TRUE(is_same_result);

	is_same_result = std::is_same<navtk::Matrix3::size_type, Size>::value;
	EXPECT_TRUE(is_same_result);

	is_same_result = std::is_same<navtk::Vector3::size_type, Size>::value;
	EXPECT_TRUE(is_same_result);

	is_same_result = std::is_same<navtk::Vector4::size_type, Size>::value;
	EXPECT_TRUE(is_same_result);
}

// This test provides a place to check for compiler conversion warnings.
// See comments above before "using navtk::Size;"
TEST(TypeTests, CompilerWarnings) {
	Size example = 4;

	// gcc 7.4 [-Wnarrowing]
	// clang 6 [-Wc++11-narrowing]
	// clang 9 [-Wc++11-narrowing]
	Vector v({{example}}, 1.0);

	// clang 9 [-Wsign-conversion]
	ASSERT_ALLCLOSE(v, ones(example));

	auto m = Matrix{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

	// gcc 7.4 [-Wsign-compare]
	// clang 6 [-Wsign-compare]
	// clang 9 [-Wsign-compare]
	if (example > num_rows(v)) {
		if (example < num_rows(m)) {
			if (example < num_cols(v)) {
				if (example > num_cols(m)) {
					if (example == v.size()) {
						if (example <= v.shape()[0]) {
						}
					}
				}
			}
		}
	}

	Size two = 2;

	EXPECT_NEAR(v(two), 1.0, 1e-10);

	// gcc 7.4 [-Wconversion]
	// clang 6 [-Wconversion]
	// clang 9 [-Wimplicit-int-conversion]
	Size size = m.size();

	m.at(Size{0}, Size{1});

	// gcc 7.4 [-Wconversion]
	// clang 6 [-Wconversion]
	// clang 9 [-Wimplicit-int-conversion]
	Size offset = m.data_offset();

	// gcc 7.4 [-Wconversion]
	// clang 6 [-Wconversion]
	// clang 9 [-Wimplicit-int-conversion]
	Size dim = m.dimension();

	auto in_bounds = v.in_bounds(Size{0});
	m(Size{2}, Size{0});
	m[Size{2}];
	m.periodic(Size{2});

	auto m2 = Matrix{{7, 8, 9}};

	auto was_vector = to_matrix(v);
	auto was_matrix = to_vec(m2);

	zeros(Size{1}, Size{2});
	ones(Size{4}, Size{5});
	eye(Size{5}, Size{3});
	eye(Size{9});

	zeros(Size{15});
	ones(Size{14});

	// Get rid of warnings for -Wunused-variable
	EXPECT_NE(size, 0);
	EXPECT_NE(offset, 1000);
	EXPECT_NE(dim, 1000);
	EXPECT_TRUE(in_bounds);
}
