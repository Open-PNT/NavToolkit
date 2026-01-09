#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/factory.hpp>
#include <navtk/tensors.hpp>

using namespace navtk;

TEST(TensorsTests, ZerosShapeBasedOnParamCount) {
	auto target1 = zeros(10);
	EXPECT_EQ(1, target1.shape().size());
	Vector expected1 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	EXPECT_ALLCLOSE(expected1, target1);

	auto target2 = zeros(3, 3);
	EXPECT_EQ(2, target2.shape().size());
	Matrix expected2 = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	EXPECT_ALLCLOSE(expected2, target2);

	auto target3 = zeros(2, 2, 2);
	EXPECT_EQ(3, target3.shape().size());
	EXPECT_DOUBLE_EQ(0.0, std::accumulate(target3.begin(), target3.end(), 0));
}

TEST(TensorsTests, OnesShapeBasedOnParamCount) {
	auto target1 = ones(10);
	EXPECT_EQ(1, target1.shape().size());
	Vector expected1 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
	EXPECT_ALLCLOSE(expected1, target1);

	auto target2 = ones(3, 3);
	EXPECT_EQ(2, target2.shape().size());
	Matrix expected2 = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}};
	EXPECT_ALLCLOSE(expected2, target2);

	auto target3 = ones(2, 2, 2);
	EXPECT_EQ(3, target3.shape().size());
	EXPECT_DOUBLE_EQ(8.0, std::accumulate(target3.begin(), target3.end(), 0));
}


TEST(TensorsTests, FillShapeBasedOnParamCount) {
	auto target1 = fill(2, 10);
	EXPECT_EQ(1, target1.shape().size());
	Vector expected1 = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
	EXPECT_ALLCLOSE(expected1, target1);

	auto target2 = fill(2, 3, 3);
	EXPECT_EQ(2, target2.shape().size());
	Matrix expected2 = {{2, 2, 2}, {2, 2, 2}, {2, 2, 2}};
	EXPECT_ALLCLOSE(expected2, target2);

	auto target3 = fill(10, 2, 2, 2);
	EXPECT_EQ(3, target3.shape().size());
	EXPECT_DOUBLE_EQ(80.0, std::accumulate(target3.begin(), target3.end(), 0));
}

TEST(TensorsTests, SurvivesMalformedDims) {
	ASSERT_ALLCLOSE(Matrix{}, dot(Matrix{}, eye(3)));
	ASSERT_ALLCLOSE(Matrix{}, dot(Matrix{}, eye(3, 3)));
}

TEST(TensorsTests, HasDiagonalIndex) {
	// clang-format off
	Matrix expected_1{{0, 1, 0, 0},
	                  {0, 0, 1, 0},
	                  {0, 0, 0, 1},
	                  {0, 0, 0, 0}};
	Matrix expected_n1{{0, 0, 0, 0},
	                   {1, 0, 0, 0},
	                   {0, 1, 0, 0},
	                   {0, 0, 1, 0}};
	Matrix expected_0{{1, 0, 0, 0},
	                  {0, 1, 0, 0},
	                  {0, 0, 1, 0},
	                  {0, 0, 0, 1}};
	Matrix expected_3_4_1{{0, 1, 0, 0},
	                      {0, 0, 1, 0},
	                      {0, 0, 0, 1}};
	// clang-format on
	ASSERT_ALLCLOSE(expected_1, eye(4, 4, 1));
	ASSERT_ALLCLOSE(expected_n1, eye(4, 4, -1));
	ASSERT_ALLCLOSE(expected_0, eye(4, 4));
	ASSERT_ALLCLOSE(expected_0, eye(4, 4, 0));
	ASSERT_ALLCLOSE(expected_3_4_1, eye(3, 4, 1));
}

ERROR_MODE_SENSITIVE_TEST(TEST, TensorsTests, BadDiagonalIndex) {
	EXPECT_HONORS_MODE_EX(eye(4, 4, 4), "Diagonal index", std::invalid_argument);
	EXPECT_HONORS_MODE_EX(eye(4, 4, -4), "Diagonal index", std::invalid_argument);
	EXPECT_HONORS_MODE_EX(eye(4, 4, 10), "Diagonal index", std::invalid_argument);
	EXPECT_HONORS_MODE_EX(eye(4, 4, -10), "Diagonal index", std::invalid_argument);
}

TEST(TensorsTests, BlockDiagonalSimpleCase) {
	Matrix a = zeros(2, 2) + 2;
	Matrix b = zeros(3, 3) + 3;
	// clang-format off
	Matrix expected{{2, 2, 0, 0, 0},
	                {2, 2, 0, 0, 0},
	                {0, 0, 3, 3, 3},
	                {0, 0, 3, 3, 3},
	                {0, 0, 3, 3, 3}};
	// clang-format on
	ASSERT_ALLCLOSE(expected, block_diag(a, b));
}

TEST(TensorsTests, BlockDiagonalThreeMatrices) {
	Matrix a = zeros(2, 2) + 2;
	Matrix b = zeros(3, 3) + 3;
	Matrix c = zeros(4, 4) + 4;
	// clang-format off
	Matrix expected{{2, 2, 0, 0, 0, 0, 0, 0, 0},
	                {2, 2, 0, 0, 0, 0, 0, 0, 0},
	                {0, 0, 3, 3, 3, 0, 0, 0, 0},
	                {0, 0, 3, 3, 3, 0, 0, 0, 0},
	                {0, 0, 3, 3, 3, 0, 0, 0, 0},
	                {0, 0, 0, 0, 0, 4, 4, 4, 4},
	                {0, 0, 0, 0, 0, 4, 4, 4, 4},
	                {0, 0, 0, 0, 0, 4, 4, 4, 4},
	                {0, 0, 0, 0, 0, 4, 4, 4, 4}};
	// clang-format on
	ASSERT_ALLCLOSE(expected, block_diag(a, b, c));
}

TEST(TensorsTests, BlockDiagonalInlineInitialized) {
	Matrix b = zeros(3, 3) + 3;
	// clang-format off
	Matrix expected{{2, 2, 0, 0, 0},
	                {2, 2, 0, 0, 0},
	                {0, 0, 3, 3, 3},
	                {0, 0, 3, 3, 3},
	                {0, 0, 3, 3, 3}};
	// clang-format on
	ASSERT_ALLCLOSE(expected, block_diag(Matrix{{2, 2}, {2, 2}}, b));
}

TEST(TensorsTests, BlockDiagonalEmptyMatrix) {
	// Make sure passing in an empty matrix doesn't crash it
	ASSERT_ALLCLOSE(eye(3), block_diag(eye(3), Matrix{}));
}

TEST(TensorsTests, BlockDiagonalNonSquareInputs) {
	// This test replicates the behavior of scipy.linalg.block_diag, and is
	// based on the following python interaction:
	/* def demo_block_diag():
	    from scipy.linalg import block_diag
	    from numpy import array, eye, zeros
	    v = array([2, 2])
	    tall = zeros([4, 2]) + 1
	    wide = zeros([2, 4]) + 3
	    print(block_diag(eye(2), v))
	    print(block_diag(eye(2) + 4, tall, v, wide, eye(2) + 5)
	*/
	Vector v{2, 2};
	Matrix tall = zeros(4, 2) + 1;
	Matrix wide = zeros(2, 4) + 3;
	// clang-format off
	// scipy assumes that vectors are horizontal
	Matrix vector_orientation{{1, 0, 0, 0},
	                          {0, 1, 0, 0},
	                          {0, 0, 2, 2}};
	// it also "trusts" the dimensions of its inputs -- it won't zero-pad
	// incoming matrices to make them square, but instead joins each matrix
	// on adjacent corners.
	Matrix corner_chain{{5, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	                    {4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	                    {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
	                    {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
	                    {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
	                    {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
	                    {0, 0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0},
	                    {0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0},
	                    {0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0},
	                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 5},
	                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 6}};
	// clang-format on
	ASSERT_ALLCLOSE(vector_orientation, block_diag(eye(2), v));
	ASSERT_ALLCLOSE(corner_chain, block_diag(eye(2) + 4, tall, v, wide, eye(2) + 5));
}

TEST(TensorsTests, BlockDiagonalNoParameters) {
	// Make sure empty arguments list isn't crashy
	ASSERT_ALLCLOSE(Matrix{}, block_diag());
}

TEST(TensorsTests, EmptyMatrixFirst) { ASSERT_ALLCLOSE(eye(3), block_diag(Matrix{}, eye(3))); }

TEST(TensorsTests, BlockDiagonalMalformedEmptyMatrix) {
	ASSERT_ALLCLOSE(eye(3), block_diag(Matrix({1, 0}), eye(3)));
}

TEST(TensorsTests, Tensor0toMatrixTest) {
	auto original = Tensor<0>{1.0};

	{
		auto converted = to_matrix(original);
		EXPECT_EQ(2, converted.dimension());
		EXPECT_EQ(1, converted.shape()[0]);
		EXPECT_EQ(1, converted.shape()[1]);
	}

	// R-value references
	auto rv_converted = to_matrix(std::move(original));
	EXPECT_EQ(2, rv_converted.dimension());
	EXPECT_EQ(1, rv_converted.shape()[0]);
	EXPECT_EQ(1, rv_converted.shape()[1]);
}

TEST(TensorsTests, VectorToMatrixTest) {
	auto original = Vector{1.0, 2.0, 3.0};

	{
		auto converted1 = to_matrix(original);
		EXPECT_EQ(2, converted1.dimension());
		EXPECT_EQ(3, converted1.shape()[0]);
		EXPECT_EQ(1, converted1.shape()[1]);

		auto converted2 = to_matrix(original, 0);
		EXPECT_EQ(2, converted2.dimension());
		EXPECT_EQ(1, converted2.shape()[0]);
		EXPECT_EQ(3, converted2.shape()[1]);
	}

	// R-value references
	auto rv_converted = to_matrix(std::move(original));
	EXPECT_EQ(2, rv_converted.dimension());
	EXPECT_EQ(3, rv_converted.shape()[0]);
	EXPECT_EQ(1, rv_converted.shape()[1]);
}

TEST(TensorsTests, MatrixToMatrixTest) {
	auto original1 = Matrix{{1.0, 2.0}, {3.0, 4.0}};
	auto original2 = Matrix{{9.5}};
	auto original3 = Matrix{{10.6}, {11.7}};
	auto original4 = Matrix{{10.6, 11.7}};

	{
		auto converted1 = to_matrix(original1);
		EXPECT_EQ(2, converted1.dimension());
		EXPECT_EQ(2, converted1.shape()[0]);
		EXPECT_EQ(2, converted1.shape()[1]);

		auto converted2 = to_matrix(original2);
		EXPECT_EQ(2, converted2.dimension());
		EXPECT_EQ(1, converted2.shape()[0]);
		EXPECT_EQ(1, converted2.shape()[1]);

		auto converted3 = to_matrix(original3);
		EXPECT_EQ(2, converted3.dimension());
		EXPECT_EQ(2, converted3.shape()[0]);
		EXPECT_EQ(1, converted3.shape()[1]);

		auto converted4 = to_matrix(original4);
		EXPECT_EQ(2, converted4.dimension());
		EXPECT_EQ(1, converted4.shape()[0]);
		EXPECT_EQ(2, converted4.shape()[1]);
	}

	auto rv_converted1 = to_matrix(std::move(original1));
	EXPECT_EQ(2, rv_converted1.dimension());
	EXPECT_EQ(2, rv_converted1.shape()[0]);
	EXPECT_EQ(2, rv_converted1.shape()[1]);

	auto rv_converted2 = to_matrix(std::move(original2));
	EXPECT_EQ(2, rv_converted2.dimension());
	EXPECT_EQ(1, rv_converted2.shape()[0]);
	EXPECT_EQ(1, rv_converted2.shape()[1]);

	auto rv_converted3 = to_matrix(std::move(original3));
	EXPECT_EQ(2, rv_converted3.dimension());
	EXPECT_EQ(2, rv_converted3.shape()[0]);
	EXPECT_EQ(1, rv_converted3.shape()[1]);

	auto rv_converted4 = to_matrix(std::move(original4));
	EXPECT_EQ(2, rv_converted4.dimension());
	EXPECT_EQ(1, rv_converted4.shape()[0]);
	EXPECT_EQ(2, rv_converted4.shape()[1]);
}

TEST(TensorsTests, Tensor3ToMatrixTest) {

	auto original1 =
	    Tensor<3>{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}}, {{7.0, 8.0, 9.0}, {10.0, 11.0, 12.0}}};
	auto original2 = Tensor<3>{{{1.0}, {4.0}}, {{7.0}, {10.0}}};
	auto original3 = Tensor<3>{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}}};
	auto original4 = Tensor<3>{{{1.0, 2.0, 3.0}}, {{7.0, 8.0, 9.0}}};
	auto original5 = Tensor<3>{{{1.0}}};

	{
		auto converted1 = to_matrix(original1);
		EXPECT_EQ(2, converted1.dimension());
		EXPECT_EQ(12, converted1.shape()[0]);
		EXPECT_EQ(1, converted1.shape()[1]);

		auto converted2 = to_matrix(original2);
		EXPECT_EQ(2, converted2.dimension());
		EXPECT_EQ(2, converted2.shape()[0]);
		EXPECT_EQ(2, converted2.shape()[1]);

		auto converted3 = to_matrix(original3);
		EXPECT_EQ(2, converted3.dimension());
		EXPECT_EQ(2, converted3.shape()[0]);
		EXPECT_EQ(3, converted3.shape()[1]);

		auto converted4 = to_matrix(original4);
		EXPECT_EQ(2, converted4.dimension());
		EXPECT_EQ(2, converted4.shape()[0]);
		EXPECT_EQ(3, converted4.shape()[1]);

		auto converted5 = to_matrix(original5);
		EXPECT_EQ(2, converted5.dimension());
		EXPECT_EQ(1, converted5.shape()[0]);
		EXPECT_EQ(1, converted5.shape()[1]);
	}

	// R-value references
	auto rv_converted1 = to_matrix(std::move(original1));
	EXPECT_EQ(2, rv_converted1.dimension());
	EXPECT_EQ(12, rv_converted1.shape()[0]);
	EXPECT_EQ(1, rv_converted1.shape()[1]);

	auto rv_converted2 = to_matrix(std::move(original2));
	EXPECT_EQ(2, rv_converted2.dimension());
	EXPECT_EQ(2, rv_converted2.shape()[0]);
	EXPECT_EQ(2, rv_converted2.shape()[1]);

	auto rv_converted3 = to_matrix(std::move(original3));
	EXPECT_EQ(2, rv_converted3.dimension());
	EXPECT_EQ(2, rv_converted3.shape()[0]);
	EXPECT_EQ(3, rv_converted3.shape()[1]);

	auto rv_converted4 = to_matrix(std::move(original4));
	EXPECT_EQ(2, rv_converted4.dimension());
	EXPECT_EQ(2, rv_converted4.shape()[0]);
	EXPECT_EQ(3, rv_converted4.shape()[1]);

	auto rv_converted5 = to_matrix(std::move(original5));
	EXPECT_EQ(2, rv_converted5.dimension());
	EXPECT_EQ(1, rv_converted5.shape()[0]);
	EXPECT_EQ(1, rv_converted5.shape()[1]);
}

TEST(TensorsTests, Matrix3ToMatrixTest) {
	auto original = Matrix3{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

	{
		auto converted = to_matrix(original);
		EXPECT_EQ(2, converted.dimension());
		EXPECT_EQ(3, converted.shape()[0]);
		EXPECT_EQ(3, converted.shape()[1]);
	}

	auto rv_converted = to_matrix(std::move(original));
	EXPECT_EQ(2, rv_converted.dimension());
	EXPECT_EQ(3, rv_converted.shape()[0]);
	EXPECT_EQ(3, rv_converted.shape()[1]);
}

TEST(TensorsTests, MatrixN31ToMatrixTest) {
	auto original = MatrixN<3, 1>{{3}, {6}, {9}};

	{
		auto converted = to_matrix(original);
		EXPECT_EQ(2, converted.dimension());
		EXPECT_EQ(3, converted.shape()[0]);
		EXPECT_EQ(1, converted.shape()[1]);
	}

	auto rv_converted = to_matrix(std::move(original));
	EXPECT_EQ(2, rv_converted.dimension());
	EXPECT_EQ(3, rv_converted.shape()[0]);
	EXPECT_EQ(1, rv_converted.shape()[1]);
}

TEST(TensorsTests, MatrixN13ToMatrixTest) {
	auto original = MatrixN<1, 3>{{1, 2, 3}};

	{
		auto converted = to_matrix(original);
		EXPECT_EQ(2, converted.dimension());
		EXPECT_EQ(1, converted.shape()[0]);
		EXPECT_EQ(3, converted.shape()[1]);
	}

	auto rv_converted = to_matrix(std::move(original));
	EXPECT_EQ(2, rv_converted.dimension());
	EXPECT_EQ(1, rv_converted.shape()[0]);
	EXPECT_EQ(3, rv_converted.shape()[1]);
}

TEST(TensorsTests, Vector3ToMatrixTest) {
	auto original = Vector3{1, 2, 3};

	{
		auto converted = to_matrix(original);
		EXPECT_EQ(2, converted.dimension());
		EXPECT_EQ(3, converted.shape()[0]);
		EXPECT_EQ(1, converted.shape()[1]);
	}

	auto rv_converted = to_matrix(std::move(original));
	EXPECT_EQ(2, rv_converted.dimension());
	EXPECT_EQ(3, rv_converted.shape()[0]);
	EXPECT_EQ(1, rv_converted.shape()[1]);
}

TEST(TensorsTests, Vector4ToMatrixTest) {
	auto original = Vector4{1, 2, 3, 4};

	{
		auto converted = to_matrix(original);
		EXPECT_EQ(2, converted.dimension());
		EXPECT_EQ(4, converted.shape()[0]);
		EXPECT_EQ(1, converted.shape()[1]);
	}

	auto rv_converted = to_matrix(std::move(original));
	EXPECT_EQ(2, rv_converted.dimension());
	EXPECT_EQ(4, rv_converted.shape()[0]);
	EXPECT_EQ(1, rv_converted.shape()[1]);
}

TEST(TensorsTests, toMatrixCopyTest) {
	auto original  = Matrix{{1.0, 2.0}, {3.0, 4.0}};
	auto copy      = original;
	auto converted = to_matrix(original);

	// Change converted, confirm original doesn't change
	converted(0, 0) = 5555.0;

	EXPECT_EQ(original, copy);
	EXPECT_NE(converted, original);
}

TEST(TensorsTests, XFunctionToMatrix) {
	Matrix a{{1, 1}, {1, 1}};
	Matrix b{{2, 2}, {2, 2}};
	auto xfunction = a + b;

	auto converted = to_matrix(xfunction);
	EXPECT_EQ(2, converted.dimension());
	EXPECT_EQ(2, converted.shape()[0]);
	EXPECT_EQ(2, converted.shape()[1]);
}

TEST(TensorsTests, Tensor0ToVecTest) {
	auto original = Tensor<0>{1.0};

	{
		auto converted = to_vec(original);
		EXPECT_EQ(1, converted.dimension());
		EXPECT_EQ(1, converted.shape()[0]);
	}

	// R-value references
	auto rv_converted = to_vec(std::move(original));
	EXPECT_EQ(1, rv_converted.dimension());
	EXPECT_EQ(1, rv_converted.shape()[0]);
}

TEST(TensorsTests, VectorToVecTest) {
	auto original = Vector{1.0, 2.0, 3.0};

	{
		auto converted = to_vec(original);
		EXPECT_EQ(1, converted.dimension());
		EXPECT_EQ(3, converted.shape()[0]);
	}

	// R-value references
	auto rv_converted = to_vec(std::move(original));
	EXPECT_EQ(1, rv_converted.dimension());
	EXPECT_EQ(3, rv_converted.shape()[0]);
}

TEST(TensorsTests, MatrixToVecTest) {
	auto original1 = Matrix{{1.0, 2.0}, {3.0, 4.0}};
	auto original2 = Matrix{{9.5}};
	auto original3 = Matrix{{10.6}, {11.7}};
	auto original4 = Matrix{{10.6, 11.7}};

	{
		auto converted1 = to_vec(original1);
		EXPECT_EQ(1, converted1.dimension());
		EXPECT_EQ(4, converted1.shape()[0]);

		auto converted2 = to_vec(original2);
		EXPECT_EQ(1, converted2.dimension());
		EXPECT_EQ(1, converted2.shape()[0]);

		auto converted3 = to_vec(original3);
		EXPECT_EQ(1, converted3.dimension());
		EXPECT_EQ(2, converted3.shape()[0]);

		auto converted4 = to_vec(original4);
		EXPECT_EQ(1, converted4.dimension());
		EXPECT_EQ(2, converted4.shape()[0]);
	}

	auto rv_converted1 = to_vec(std::move(original1));
	EXPECT_EQ(1, rv_converted1.dimension());
	EXPECT_EQ(4, rv_converted1.shape()[0]);

	auto rv_converted2 = to_vec(std::move(original2));
	EXPECT_EQ(1, rv_converted2.dimension());
	EXPECT_EQ(1, rv_converted2.shape()[0]);

	auto rv_converted3 = to_vec(std::move(original3));
	EXPECT_EQ(1, rv_converted3.dimension());
	EXPECT_EQ(2, rv_converted3.shape()[0]);

	auto rv_converted4 = to_vec(std::move(original4));
	EXPECT_EQ(1, rv_converted4.dimension());
	EXPECT_EQ(2, rv_converted4.shape()[0]);
}

TEST(TensorsTests, Tensor3ToVecTest) {

	auto original1 =
	    Tensor<3>{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}}, {{7.0, 8.0, 9.0}, {10.0, 11.0, 12.0}}};
	auto original2 = Tensor<3>{{{1.0}, {4.0}}, {{7.0}, {10.0}}};
	auto original3 = Tensor<3>{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}}};
	auto original4 = Tensor<3>{{{1.0, 2.0, 3.0}}, {{7.0, 8.0, 9.0}}};
	auto original5 = Tensor<3>{{{1.0}}};

	{
		auto converted1 = to_vec(original1);
		EXPECT_EQ(1, converted1.dimension());
		EXPECT_EQ(12, converted1.shape()[0]);

		auto converted2 = to_vec(original2);
		EXPECT_EQ(1, converted2.dimension());
		EXPECT_EQ(4, converted2.shape()[0]);

		auto converted3 = to_vec(original3);
		EXPECT_EQ(1, converted3.dimension());
		EXPECT_EQ(6, converted3.shape()[0]);

		auto converted4 = to_vec(original4);
		EXPECT_EQ(1, converted4.dimension());
		EXPECT_EQ(6, converted4.shape()[0]);

		auto converted5 = to_vec(original5);
		EXPECT_EQ(1, converted5.dimension());
		EXPECT_EQ(1, converted5.shape()[0]);
	}

	// R-value references
	auto rv_converted1 = to_vec(std::move(original1));
	EXPECT_EQ(1, rv_converted1.dimension());
	EXPECT_EQ(12, rv_converted1.shape()[0]);

	auto rv_converted2 = to_vec(std::move(original2));
	EXPECT_EQ(1, rv_converted2.dimension());
	EXPECT_EQ(4, rv_converted2.shape()[0]);

	auto rv_converted3 = to_vec(std::move(original3));
	EXPECT_EQ(1, rv_converted3.dimension());
	EXPECT_EQ(6, rv_converted3.shape()[0]);

	auto rv_converted4 = to_vec(std::move(original4));
	EXPECT_EQ(1, rv_converted4.dimension());
	EXPECT_EQ(6, rv_converted4.shape()[0]);

	auto rv_converted5 = to_vec(std::move(original5));
	EXPECT_EQ(1, rv_converted5.dimension());
	EXPECT_EQ(1, rv_converted5.shape()[0]);
}

TEST(TensorsTests, Matrix3ToVecTest) {
	auto original = Matrix3{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

	{
		auto converted = to_vec(original);
		EXPECT_EQ(1, converted.dimension());
		EXPECT_EQ(9, converted.shape()[0]);
	}

	auto rv_converted = to_vec(std::move(original));
	EXPECT_EQ(1, rv_converted.dimension());
	EXPECT_EQ(9, rv_converted.shape()[0]);
}

TEST(TensorsTests, MatrixN31ToVecTest) {
	auto original = MatrixN<3, 1>{{3}, {6}, {9}};

	{
		auto converted = to_vec(original);
		EXPECT_EQ(1, converted.dimension());
		EXPECT_EQ(3, converted.shape()[0]);
	}

	auto rv_converted = to_vec(std::move(original));
	EXPECT_EQ(1, rv_converted.dimension());
	EXPECT_EQ(3, rv_converted.shape()[0]);
}

TEST(TensorsTests, MatrixN13ToVecTest) {
	auto original = MatrixN<1, 3>{{1, 2, 3}};

	{
		auto converted = to_vec(original);
		EXPECT_EQ(1, converted.dimension());
		EXPECT_EQ(3, converted.shape()[0]);
	}

	auto rv_converted = to_vec(std::move(original));
	EXPECT_EQ(1, rv_converted.dimension());
	EXPECT_EQ(3, rv_converted.shape()[0]);
}

TEST(TensorsTests, Vector3ToVecTest) {
	auto original = Vector3{1, 2, 3};

	{
		auto converted = to_vec(original);
		EXPECT_EQ(1, converted.dimension());
		EXPECT_EQ(3, converted.shape()[0]);
	}

	auto rv_converted = to_vec(std::move(original));
	EXPECT_EQ(1, rv_converted.dimension());
	EXPECT_EQ(3, rv_converted.shape()[0]);
}

TEST(TensorsTests, Vector4ToVecTest) {
	auto original = Vector4{1, 2, 3, 4};

	{
		auto converted = to_vec(original);
		EXPECT_EQ(1, converted.dimension());
		EXPECT_EQ(4, converted.shape()[0]);
	}

	auto rv_converted = to_vec(std::move(original));
	EXPECT_EQ(1, rv_converted.dimension());
	EXPECT_EQ(4, rv_converted.shape()[0]);
}

TEST(TensorsTests, toVecCopyTest) {
	auto original  = Vector{1.0, 2.0, 3.0, 4.0};
	auto copy      = original;
	auto converted = to_vec(original);

	// Change converted, confirm original doesn't change
	converted(0, 0) = 5555.0;

	EXPECT_EQ(original, copy);
	EXPECT_NE(converted, original);
}

TEST(TensorsTests, XFunctionToVector) {
	Vector a{1, 1, 1, 1};
	Vector b{2, 2, 2, 2};
	auto xfunction = a + b;

	auto converted = to_vec(xfunction);
	EXPECT_EQ(1, converted.dimension());
	EXPECT_EQ(4, converted.shape()[0]);
}

class EigenLikeClass {
public:
	EigenLikeClass(int rows, int cols) : internal_matrix{}, num_rows{rows}, num_cols{cols} {

		for (auto i = 0; i < rows; ++i) {
			auto next_row = std::vector<double>{};
			for (auto j = 0; j < cols; ++j) {
				next_row.push_back(i * num_cols + j);
			}
			internal_matrix.push_back(next_row);
		}
	}

	int rows() const { return num_rows; }

	int cols() const { return num_cols; }

	const double& operator()(int row, int col) const {
		if (row < 0 || row > num_rows || col < 0 || col > num_cols) {
			throw std::range_error("Requested element out of range.");
		}
		return internal_matrix[row][col];
	}

private:
	std::vector<std::vector<double>> internal_matrix;
	int num_rows;
	int num_cols;
};

TEST(TensorsTests, EigenLikeMatrixToMatrixTest) {
	auto eigen_matrix = EigenLikeClass(3, 4);

	{
		auto converted = to_matrix(eigen_matrix);
		EXPECT_EQ(2, converted.dimension());
		EXPECT_EQ(3, converted.shape()[0]);
		EXPECT_EQ(4, converted.shape()[1]);
		EXPECT_DOUBLE_EQ(0.0, converted(0, 0));
		EXPECT_DOUBLE_EQ(3.0, converted(0, 3));
		EXPECT_DOUBLE_EQ(8.0, converted(2, 0));
		EXPECT_DOUBLE_EQ(11.0, converted(2, 3));
	}

	auto rv_converted = to_matrix(std::move(eigen_matrix));
	EXPECT_EQ(2, rv_converted.dimension());
	EXPECT_EQ(3, rv_converted.shape()[0]);
	EXPECT_EQ(4, rv_converted.shape()[1]);
	EXPECT_DOUBLE_EQ(0.0, rv_converted(0, 0));
	EXPECT_DOUBLE_EQ(3.0, rv_converted(0, 3));
	EXPECT_DOUBLE_EQ(8.0, rv_converted(2, 0));
	EXPECT_DOUBLE_EQ(11.0, rv_converted(2, 3));
}

TEST(TensorsTests, EigenLikeMatrixToVectorTest) {
	auto eigen_matrix = EigenLikeClass(3, 4);

	{
		auto converted = to_vec(eigen_matrix);
		EXPECT_EQ(1, converted.dimension());
		EXPECT_EQ(12, converted.shape()[0]);
		EXPECT_DOUBLE_EQ(0.0, converted(0));
		EXPECT_DOUBLE_EQ(3.0, converted(3));
		EXPECT_DOUBLE_EQ(8.0, converted(8));
		EXPECT_DOUBLE_EQ(11.0, converted(11));
	}

	auto rv_converted = to_vec(std::move(eigen_matrix));
	EXPECT_EQ(1, rv_converted.dimension());
	EXPECT_EQ(12, rv_converted.shape()[0]);
	EXPECT_DOUBLE_EQ(0.0, rv_converted(0));
	EXPECT_DOUBLE_EQ(3.0, rv_converted(3));
	EXPECT_DOUBLE_EQ(8.0, rv_converted(8));
	EXPECT_DOUBLE_EQ(11.0, rv_converted(11));
}

TEST(TensorsTests, EigenLikeRowVectorToMatrixTest) {
	auto eigen_row_vector = EigenLikeClass(5, 1);

	{
		auto converted = to_matrix(eigen_row_vector);
		EXPECT_EQ(2, converted.dimension());
		EXPECT_EQ(5, converted.shape()[0]);
		EXPECT_EQ(1, converted.shape()[1]);
		EXPECT_DOUBLE_EQ(0.0, converted(0, 0));
		EXPECT_DOUBLE_EQ(4.0, converted(4, 0));
	}

	auto rv_converted = to_matrix(std::move(eigen_row_vector));
	EXPECT_EQ(2, rv_converted.dimension());
	EXPECT_EQ(5, rv_converted.shape()[0]);
	EXPECT_EQ(1, rv_converted.shape()[1]);
	EXPECT_DOUBLE_EQ(0.0, rv_converted(0, 0));
	EXPECT_DOUBLE_EQ(4.0, rv_converted(4, 0));
}

TEST(TensorsTests, EigenLikeColVectorToMatrixTest) {
	auto eigen_col_vector = EigenLikeClass(1, 5);

	{
		auto converted = to_matrix(eigen_col_vector);
		EXPECT_EQ(2, converted.dimension());
		EXPECT_EQ(1, converted.shape()[0]);
		EXPECT_EQ(5, converted.shape()[1]);
		EXPECT_DOUBLE_EQ(0.0, converted(0, 0));
		EXPECT_DOUBLE_EQ(4.0, converted(0, 4));
	}

	auto rv_converted = to_matrix(std::move(eigen_col_vector));
	EXPECT_EQ(2, rv_converted.dimension());
	EXPECT_EQ(1, rv_converted.shape()[0]);
	EXPECT_EQ(5, rv_converted.shape()[1]);
	EXPECT_DOUBLE_EQ(0.0, rv_converted(0, 0));
	EXPECT_DOUBLE_EQ(4.0, rv_converted(0, 4));
}

TEST(TensorsTests, EigenLikeRowVectorToVectorTest) {
	auto eigen_row_vector = EigenLikeClass(5, 1);

	{
		auto converted = to_vec(eigen_row_vector);
		EXPECT_EQ(1, converted.dimension());
		EXPECT_EQ(5, converted.shape()[0]);
		EXPECT_DOUBLE_EQ(0.0, converted(0));
		EXPECT_DOUBLE_EQ(4.0, converted(4));
	}

	auto rv_converted = to_vec(std::move(eigen_row_vector));
	EXPECT_EQ(1, rv_converted.dimension());
	EXPECT_EQ(5, rv_converted.shape()[0]);
	EXPECT_DOUBLE_EQ(0.0, rv_converted(0));
	EXPECT_DOUBLE_EQ(4.0, rv_converted(4));
}

TEST(TensorsTests, EigenLikeColVectorToVectorTest) {
	auto eigen_col_vector = EigenLikeClass(1, 5);

	{
		auto converted = to_vec(eigen_col_vector);
		EXPECT_EQ(1, converted.dimension());
		EXPECT_EQ(5, converted.shape()[0]);
		EXPECT_DOUBLE_EQ(0.0, converted(0));
		EXPECT_DOUBLE_EQ(4.0, converted(4));
	}

	auto rv_converted = to_vec(std::move(eigen_col_vector));
	EXPECT_EQ(1, rv_converted.dimension());
	EXPECT_EQ(5, rv_converted.shape()[0]);
	EXPECT_DOUBLE_EQ(0.0, rv_converted(0));
	EXPECT_DOUBLE_EQ(4.0, rv_converted(4));
}

TEST(TensorsTests, FixedSizeArrayToMatrixTest) {
	double array[2][3]{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}};
	Matrix expected{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}};
	EXPECT_ALLCLOSE(expected, to_matrix(array));
}
