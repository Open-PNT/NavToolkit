#pragma once

#include <gtest/gtest.h>
#include <xtensor/core/xmath.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/human_readable.hpp>

namespace navtk {
namespace filtering {
namespace testing {

using navtk::utils::diff;
using navtk::utils::repr;
using ::testing::AssertionFailure;
using ::testing::AssertionResult;
using ::testing::AssertionSuccess;

struct AllCloseHelper {
	double rtol                 = 1e-05;
	double atol                 = 1e-08;
	bool user_specified_epsilon = false;

	bool allclose(Matrix lhs, Matrix rhs) {
		return lhs.shape() == rhs.shape() && (!lhs.size() || xt::allclose(lhs, rhs, rtol, atol));
	}

	template <typename T1, typename T2>
	AssertionResult compare(const char* lhs_expr,
	                        const char* rhs_expr,
	                        const T1& lhs_raw,
	                        const T2& rhs_raw) {
		Matrix lhs = to_matrix(lhs_raw);
		Matrix rhs = to_matrix(rhs_raw);
		if (allclose(lhs, rhs)) return AssertionSuccess();
		auto out = AssertionFailure() << "xt::allclose(" << lhs_expr << ", " << rhs_expr;
		if (user_specified_epsilon) out = out << ", " << rtol << ", " << atol;
		return out << ") evaluates to false.\n"
		           << lhs_expr << ":\n"
		           << repr(lhs) << "\n"
		           << rhs_expr << ":\n"
		           << repr(rhs) << "\n\n"
		           << diff(lhs_expr, rhs_expr, lhs, rhs, rtol, atol);
	}
};
}  // namespace testing
}  // namespace filtering
}  // namespace navtk


#define ASSERT_ALLCLOSE(expected, actual)                                \
	GTEST_ASSERT_(::navtk::filtering::testing::AllCloseHelper().compare( \
	                  #expected, #actual, expected, actual),             \
	              GTEST_FATAL_FAILURE_)

#define EXPECT_ALLCLOSE(expected, actual)                                \
	GTEST_ASSERT_(::navtk::filtering::testing::AllCloseHelper().compare( \
	                  #expected, #actual, expected, actual),             \
	              GTEST_NONFATAL_FAILURE_)

#define ASSERT_ALLCLOSE_EX(expected, actual, rtol, atol)                          \
	GTEST_ASSERT_((::navtk::filtering::testing::AllCloseHelper{rtol, atol, true}) \
	                  .compare(#expected, #actual, expected, actual),             \
	              GTEST_FATAL_FAILURE_)

#define EXPECT_ALLCLOSE_EX(expected, actual, rtol, atol)                          \
	GTEST_ASSERT_((::navtk::filtering::testing::AllCloseHelper{rtol, atol, true}) \
	                  .compare(#expected, #actual, expected, actual),             \
	              GTEST_NONFATAL_FAILURE_)
