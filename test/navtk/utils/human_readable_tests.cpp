#include <string>

#include <gtest/gtest.h>

#include <navtk/factory.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/human_readable.hpp>

using namespace navtk;
using std::string;

TEST(TensorsTests, reprMatrix) {
	string expected =
	    "Matrix {{ 1.,  0.,  0.},\n"
	    "        { 0.,  1.,  0.},\n"
	    "        { 0.,  0.,  1.}}";
	Matrix target = eye(3);
	ASSERT_EQ(expected, utils::repr(target));
}


TEST(TensorsTests, DISABLED_reprVector) {  // TODO: Fix repr to work with vectors.
	string expected = "Vector { 1.,  1.,  2.}";
	Vector target   = Vector{1, 1, 2};
	ASSERT_EQ(expected, utils::repr(target));
}


TEST(TensorsTests, reprExpr) {
	string expected =
	    "(expr) {{ 2.,  0.,  0.},\n"
	    "        { 0.,  2.,  0.},\n"
	    "        { 0.,  0.,  2.}}";
	auto target = eye(3) * 2;
	ASSERT_EQ(expected, utils::repr(target));
}

TEST(TensorsTests, diffExpr) {
	string actual_trans_res =
	    "//Transposed:\n"
	    "//	before.shape() == {1, 7} /* size: 7 */\n"
	    "//	after.shape() == {7, 1} /* size: 7 */\n"
	    "after = xt::transpose(before)";

	string trans_shape_diff_val =
	    "//Possibly transposed:\n"
	    "//	before.shape() == {1, 7} /* size: 7 */\n"
	    "//	after.shape() == {7, 1} /* size: 7 */\n"
	    "//after and xt::transpose(before) have 1 coefficients (of 7) in common.";

	string diff_shape =
	    "//Different shapes:\n"
	    "//	before.shape() == {1, 3} /* size: 3 */\n"
	    "//	after.shape() == {1, 7} /* size: 7 */";

	string sq_trans =
	    "//Transposed:\n"
	    "//	before.shape() == {2, 2} /* size: 4 */\n"
	    "//	after.shape() == {2, 2} /* size: 4 */\n"
	    "after = xt::transpose(before)";

	string paren_names =
	    "//Transposed:\n"
	    "//	DaddyMatrix.shape() == {1, 7} /* size: 7 */\n"
	    "//	(Little Matrix Jr.).shape() == {7, 1} /* size: 7 */\n"
	    "Little Matrix Jr. = xt::transpose(DaddyMatrix)";

	auto a  = Matrix{{5.0, 1.0, 3.0}};
	auto b  = Matrix{{5.0, -1.0, 3.0}};
	auto c  = Matrix{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}};
	auto d  = Matrix{{2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 7.0}};
	auto sq = Matrix{{1.0, 2.0}, {3.0, 4.0}};


	ASSERT_EQ("", utils::diff(a, a));
	ASSERT_EQ("//after and before have no coefficients in common.", utils::diff(a, a + 1.0));
	ASSERT_EQ("\nafter(0, 1) = -1;  // before(0, 1) == 1", utils::diff(a, b));
	ASSERT_EQ("//after and before have 1 coefficients (of 7) in common.", utils::diff(c, d));
	ASSERT_EQ(diff_shape, utils::diff(a, c));
	ASSERT_EQ(paren_names, utils::diff("DaddyMatrix", "Little Matrix Jr.", c, xt::transpose(c)));
	ASSERT_EQ(actual_trans_res, utils::diff(c, xt::transpose(c)));
	ASSERT_EQ(trans_shape_diff_val, utils::diff(c, xt::transpose(d)));
	ASSERT_EQ(sq_trans, utils::diff(sq, xt::transpose(sq)));
}
