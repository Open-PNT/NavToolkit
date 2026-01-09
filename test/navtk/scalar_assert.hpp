#pragma once

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

namespace navtk {
namespace filtering {
namespace testing {

using ::testing::AssertionFailure;
using ::testing::AssertionResult;
using ::testing::AssertionSuccess;

struct IsNearHelper {
	double rtol;
	double atol;
	bool nan_equivalency;

	bool is_near(double a, double b) {
		if (std::isnan(a)) {
			return (std::isnan(b) ? nan_equivalency : false);
		}
		if (std::isinf(a)) {
			return (std::isinf(b) ? a == b : false);
		}
		if (std::isnan(b)) return false;
		if (std::isinf(b)) return false;

		auto diff = std::fabs(a - b);
		return diff <= atol || diff <= rtol * std::max(std::fabs(a), std::fabs(b));
	}

	AssertionResult compare(double a, double b) {
		if (is_near(a, b)) return AssertionSuccess();
		return AssertionFailure() << "The difference between " << a << " and " << b << " is "
		                          << std::fabs(a - b) << ", which exceeds the given rtol=" << rtol
		                          << " atol=" << atol;
	}
};

}  // namespace testing
}  // namespace filtering
}  // namespace navtk

#define ASSERT_NEAR_EX_NAN(expected, actual, rtol, atol, nan_equivalency)                  \
	GTEST_ASSERT_((::navtk::filtering::testing::IsNearHelper{rtol, atol, nan_equivalency}) \
	                  .compare(expected, actual),                                          \
	              GTEST_FATAL_FAILURE_)

#define EXPECT_NEAR_EX_NAN(expected, actual, rtol, atol, nan_equivalency)                  \
	GTEST_ASSERT_((::navtk::filtering::testing::IsNearHelper{rtol, atol, nan_equivalency}) \
	                  .compare(expected, actual),                                          \
	              GTEST_NONFATAL_FAILURE_)

#define ASSERT_NEAR_EX(expected, actual, rtol, atol) \
	ASSERT_NEAR_EX_NAN(expected, actual, rtol, atol, false)

#define EXPECT_NEAR_EX(expected, actual, rtol, atol) \
	EXPECT_NEAR_EX_NAN(expected, actual, rtol, atol, false)
