#include <gtest/gtest.h>
#include <xtensor/views/xview.hpp>

#include <navtk/factory.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/tensors.hpp>

using namespace navtk;

TEST(TensorsTests, HasZeroSizeSimpleCases) {
	EXPECT_FALSE(has_zero_size(eye(3)));
	EXPECT_FALSE(has_zero_size(eye(3, 3)));
	EXPECT_FALSE(has_zero_size(zeros(3)));
	EXPECT_FALSE(has_zero_size(xt::empty<Scalar>({1, 2, 3})));
	EXPECT_FALSE(has_zero_size(xt::empty<Scalar>(std::array<Size, 0>())));
	EXPECT_TRUE(has_zero_size(Vector{}));
	EXPECT_TRUE(has_zero_size(Matrix{}));
}

TEST(TensorsTests, HasZeroSizeNosenseSeenInWild) {
	EXPECT_TRUE(has_zero_size(zeros(3, 0)));
	EXPECT_TRUE(has_zero_size(eye(3, 0)));
	EXPECT_TRUE(has_zero_size(xt::empty<Scalar>({3, 2, 0})));
	EXPECT_TRUE(has_zero_size(xt::view(eye(2), xt::drop(std::vector<Size>{0, 1}), xt::all())));
	EXPECT_TRUE(has_zero_size(zeros(0) + dot(Matrix{}, Matrix{})));
}

TEST(TensorsTests, NumRowsColsSimpleCases) {
	EXPECT_EQ(3, num_rows(eye(3)));
	EXPECT_EQ(3, num_cols(eye(3)));
	EXPECT_EQ(3, num_rows(eye(3, 2)));
	EXPECT_EQ(2, num_cols(eye(3, 2)));
	EXPECT_EQ(2, num_rows(zeros(2)));
	EXPECT_EQ(2, num_cols(zeros(2)));
	EXPECT_EQ(0, num_rows(Matrix{}));
	EXPECT_EQ(0, num_cols(Matrix{}));
}

TEST(TensorsTests, NumRowsColsZeroDAreScalarsNotEmpty) {
	EXPECT_EQ(1, num_rows(to_matrix(xt::empty<Scalar>(std::array<Size, 0>()))));
	EXPECT_EQ(1, num_cols(to_matrix(xt::empty<Scalar>(std::array<Size, 0>()))));
}

TEST(TensorsTests, NumRowsColsReturnZeroWhenTensorHasNoData) {
	EXPECT_EQ(0, num_rows(zeros(3, 0)));
	EXPECT_EQ(0, num_rows(to_vec(xt::view(eye(2), xt::drop(std::vector<int>{0, 1}), xt::all()))));
	EXPECT_EQ(0,
	          num_rows(to_vec(xt::view(eye(2, 2), xt::drop(std::vector<int>{0, 1}), xt::all()))));
	EXPECT_EQ(0, num_cols(zeros(3, 0)));
	EXPECT_EQ(0, num_cols(to_vec(xt::view(eye(2), xt::drop(std::vector<int>{0, 1}), xt::all()))));
	EXPECT_EQ(0,
	          num_cols(to_vec(xt::view(eye(2, 2), xt::drop(std::vector<int>{0, 1}), xt::all()))));
}

TEST(TensorsTests, ToMatrixSurvivesMalformedDims) {
	EXPECT_TRUE(has_zero_size(to_matrix(zeros(0))));
	EXPECT_TRUE(has_zero_size(to_matrix(zeros(4, 0))));
	EXPECT_TRUE(
	    has_zero_size(to_matrix(xt::view(eye(2), xt::drop(std::vector<int>{0, 1}), xt::all()))));
	EXPECT_TRUE(
	    has_zero_size(to_matrix(xt::view(eye(2, 2), xt::drop(std::vector<int>{0, 1}), xt::all()))));
}

TEST(TensorsTests, ToVecSurvivesMalformedDims) {
	EXPECT_TRUE(has_zero_size(to_vec(zeros(0))));
	EXPECT_TRUE(has_zero_size(to_vec(zeros(3, 0))));
	EXPECT_TRUE(
	    has_zero_size(to_vec(xt::view(eye(2), xt::drop(std::vector<int>{0, 1}), xt::all()))));
	EXPECT_TRUE(
	    has_zero_size(to_vec(xt::view(eye(2, 2), xt::drop(std::vector<int>{0, 1}), xt::all()))));
}

TEST(TensorsTests, Diagonals) {
	EXPECT_TRUE(is_diagonal(zeros(1, 1)));
	EXPECT_TRUE(is_diagonal(zeros(2, 2)));
	EXPECT_TRUE(is_diagonal(zeros(3, 3)));
	EXPECT_TRUE(is_diagonal(eye(1)));
	EXPECT_TRUE(is_diagonal(eye(2)));
	EXPECT_TRUE(is_diagonal(eye(3)));
	Matrix should_fail{{1.0, 1e-20}, {0.0, 1.0}};
	EXPECT_FALSE(is_diagonal(should_fail));
}

TEST(TensorsTests, Identity) {
	EXPECT_TRUE(is_identity(eye(1)));
	EXPECT_TRUE(is_identity(eye(2)));
	EXPECT_TRUE(is_identity(eye(3)));
	EXPECT_TRUE(is_identity(eye(10)));
	EXPECT_FALSE(is_identity(eye(1) * 2.0));
	EXPECT_FALSE(is_identity(eye(3) * 2.0));
	Matrix should_fail{{1.0, 1e-20}, {0.0, 1.0}};
	EXPECT_FALSE(is_identity(should_fail));
	Matrix should_also_fail = eye(9);
	should_also_fail(8, 8)  = 1.00000000001;
	EXPECT_FALSE(is_identity(should_also_fail));
	EXPECT_FALSE(is_identity(Matrix{}));
}
