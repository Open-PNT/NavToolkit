#pragma once

#include <error_mode_assert.hpp>

#include <navtk/utils/ValidationContext.hpp>
#include <navtk/utils/ValidationResult.hpp>

namespace validationassert {
// The overloads of this helper function allows us to pass either a ValidationContext or a
// ValidationResult to EXPECT_VALIDATION_FAILURE_P and EXPECT_VALIDATION_FAILURE_G
inline auto validate(navtk::utils::ValidationContext& vc) { return vc.validate(); }
inline auto validate(navtk::utils::ValidationResult vr) { return vr; }
}  // namespace validationassert

#define SECOND(A, B, ...) B
#define FIRST(X, ...) X
#define EXPECT_VALIDATION_FAILURE_(macro, context_expr, ...)                \
	{                                                                       \
		auto result = macro(::validationassert::validate((context_expr)),   \
		                    FIRST(__VA_ARGS__, x),                          \
		                    SECOND(__VA_ARGS__, std::runtime_error, x));    \
		if (mode == navtk::ErrorMode::OFF) {                                \
			EXPECT_EQ(navtk::utils::ValidationResult::NOT_CHECKED, result); \
		} else if (mode == navtk::ErrorMode::LOG) {                         \
			EXPECT_EQ(navtk::utils::ValidationResult::BAD, result);         \
		}                                                                   \
	}

// EXPECT_VALIDATION_FAILURE_P(expression, "error message", [optional: exception type]) expects the
// expression to honor the mode parameter and return a ValidationResult. EXPECT_VALIDATION_FAILURE_G
// takes the same parameters and expects the expression to honor the global error mode.
//
// When ErrorMode::OFF the expected behavior is to return ValidationResult::NOT_CHECKED.
// When ErrorMode::LOG the expected behavior is to log a message and return ValidationResult::BAD.
// When ErrorMode::DIE the expected behavior is to throw the given exception type (or
//                     std::runtime_error if that parameter is omitted)
//
// Use these macros inside an ERROR_MODE_SENSITIVE_TEST for maximum effect.
#define EXPECT_VALIDATION_FAILURE_P(...) \
	EXPECT_VALIDATION_FAILURE_(EXPECT_HONORS_MODE_PARAM_EX, __VA_ARGS__)
#define EXPECT_VALIDATION_FAILURE_G(...) \
	EXPECT_VALIDATION_FAILURE_(EXPECT_HONORS_MODE_EX, __VA_ARGS__)
