#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <misc_test_helpers.hpp>
#include <validation_assert.hpp>

#include <navtk/navutils/math.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/ValidationContext.hpp>

using navtk::ErrorMode;
using navtk::ErrorModeLock;
using navtk::Matrix;
using navtk::Vector;
using navtk::navutils::PI;
using navtk::utils::ValidationContext;
using navtk::utils::ValidationResult;


ENUM_PRINT_TEST(TEST,
                Validation,
                ValidationResult,
                ValidationResult::NOT_CHECKED,
                ValidationResult::GOOD,
                ValidationResult::BAD)

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, SimpleBoundsCheck) {
	auto foo = Matrix{{1, 2, 3}};
	ValidationContext ctx{};
	ctx.add_matrix(foo).dim(2, 2);

	EXPECT_HONORS_MODE_EX(ctx.validate(), "dimension", std::range_error);
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, LetterBoundsCheck) {
	auto foo = Matrix{{1, 2, 3}};
	auto bar = Matrix{{1, 2, 3, 4}};

	ValidationContext ctx{};
	ctx.add_matrix(foo).dim(1, 'N').add_matrix(bar).dim(1, 'N');

	EXPECT_HONORS_MODE_EX(ctx.validate(), "dimension", std::range_error);
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, SimpleBoundsCheckNxN) {
	auto foo = Matrix{{1, 2, 3}};
	ValidationContext ctx{};
	ctx.add_matrix(foo).dim('N', 'N');

	EXPECT_HONORS_MODE_EX(ctx.validate(), "dimension", std::range_error);
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, NonIdiomaticFormChainable) {
	auto foo = Matrix{{1, 2}, {3, 4}};
	auto bar = Matrix{{1, 2}, {2, 1}};
	ValidationContext ctx{};
	ctx.add_matrix(bar).dim(2, 2).symmetric().min(0.0).max(5.0).validate();

	ValidationContext ctx2{};
	EXPECT_HONORS_MODE_EX(ctx2.add_matrix(foo).symmetric().dim(2, 2).min(0.0).max(5.0).validate(),
	                      "symm",
	                      std::domain_error);
}

TEST(Validation, DimensionCheckOverloadsExist) {
	auto foo = Matrix{{1, 2, 3}};
	ValidationContext ctx{ErrorMode::DIE};
	ctx.add_matrix(foo);

	ctx.dim(1, 3).validate();
	ctx.dim(1, 'M').validate();
	ctx.dim('N', 3).validate();
	ctx.dim('N', 'M').validate();
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, DimensionCalledTooEarly) {
	ValidationContext ctx{mode};

	EXPECT_HONORS_MODE_PARAM(ctx.dim(1, 3), "no matrices").validate();
	EXPECT_HONORS_MODE_PARAM(ctx.dim(1, 'M'), "no matrices");
	EXPECT_HONORS_MODE_PARAM(ctx.dim('N', 3), "no matrices");
	EXPECT_HONORS_MODE_PARAM(ctx.dim('N', 'M'), "no matrices");
}

TEST(Validation, Min) {
	auto mat = Matrix{{1, 2, 3}};

	ValidationContext ctx{ErrorMode::DIE};

	EXPECT_THROW(EXPECT_ERROR(ctx.min(1.0), "no matrices"), std::runtime_error);

	ctx.add_matrix(mat).min(1.0);

	EXPECT_THROW(EXPECT_ERROR(ctx.min(2.0), "< 2"), std::invalid_argument);
}

TEST(Validation, Max) {
	auto mat = Matrix{{1, 2, 3}};

	ValidationContext ctx{ErrorMode::DIE};

	EXPECT_THROW(EXPECT_ERROR(ctx.max(3.0), "no matrices"), std::runtime_error);

	ctx.add_matrix(mat).max(3.0);

	EXPECT_THROW(EXPECT_ERROR(ctx.max(2.0), "> 2"), std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, Symmetric) {
	auto sym = Matrix{{1, 2, 3}, {2, 4, 5}, {3, 5, 6}};

	ValidationContext ctx{};

	EXPECT_HONORS_MODE(ctx.symmetric(), "no matrices");

	ctx.add_matrix(sym).symmetric();

	auto asym = Matrix{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

	EXPECT_VALIDATION_FAILURE_G(ctx.add_matrix(asym).symmetric(), "symm", std::domain_error);

	auto not_square = Matrix{{1, 2, 3}};

	EXPECT_VALIDATION_FAILURE_G(ctx.add_matrix(not_square).symmetric(), "symm", std::out_of_range);
}

TEST(Validation, SymmetricAndBoundsInSameExpr) {
	auto sym = Matrix{{1, 2}, {2, 1}};
	ValidationContext ctx{ErrorMode::DIE};
	ctx.add_matrix(sym).symmetric().dim(2, 2).validate();
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, NameInExceptionDim) {
	auto foo = Matrix{{1, 2, 3}};
	ValidationContext ctx{};
	EXPECT_VALIDATION_FAILURE_G(ctx.add_matrix(foo, "foo").dim(2, 2), "foo", std::range_error);
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, ProblemInExceptionDim) {
	auto foo = Matrix{{1, 2, 3}};
	ValidationContext ctx{mode};
	EXPECT_VALIDATION_FAILURE_P(
	    ctx.add_matrix(foo, "foo").dim(2, 2), "dimension", std::range_error);
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, NameInExceptionMax) {
	auto bar = Matrix{{1, 2, 3}};
	ValidationContext ctx{};
	EXPECT_VALIDATION_FAILURE_G(ctx.add_matrix(bar, "bar").max(2.0), "bar", std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, ProblemInExceptionMax) {
	auto bar = Matrix{{1, 2, 3}};
	ValidationContext ctx{mode};
	EXPECT_VALIDATION_FAILURE_P(ctx.add_matrix(bar, "bar").max(2.0), "max", std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, NameInExceptionMin) {
	auto baz = Matrix{{1, 2, 3}};
	ValidationContext ctx{};
	EXPECT_VALIDATION_FAILURE_G(ctx.add_matrix(baz, "baz").min(2.0), "baz", std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, ProblemInExceptionMin) {
	auto baz = Matrix{{1, 2, 3}};
	ValidationContext ctx{mode};
	EXPECT_VALIDATION_FAILURE_P(ctx.add_matrix(baz, "baz").min(2.0), "min", std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, NameInExceptionSymmetric) {
	auto foo = Matrix{{1, 2, 3}};
	ValidationContext ctx{};
	EXPECT_VALIDATION_FAILURE_G(ctx.add_matrix(foo, "foo").symmetric(), "foo", std::out_of_range);
}

ERROR_MODE_SENSITIVE_TEST(TEST, Validation, ProblemInExceptionSymmetric) {
	auto foo = Matrix{{1, 2, 3}};
	ValidationContext ctx{};
	EXPECT_VALIDATION_FAILURE_G(ctx.add_matrix(foo, "foo").symmetric(), "symm", std::out_of_range);
}

TEST(Validation, MaxExceptionMessage) {
	ErrorModeLock guard{ErrorMode::DIE};
	auto foo = Matrix{{15, 7}};
	EXPECT_THROW(
	    EXPECT_ERROR(ValidationContext{}.add_matrix(foo, "CaridinaCantonensis").max(5.0).validate(),
	                 "caridina"),
	    std::invalid_argument);
}

TEST(Validation, MinExceptionMessage) {
	ErrorModeLock guard{ErrorMode::DIE};
	auto foo = Matrix{{15, 7}};
	EXPECT_THROW(
	    EXPECT_ERROR(
	        ValidationContext{}.add_matrix(foo, "NeocaridinaZhanghjiajiensis").min(10.0).validate(),
	        "caridina"),
	    std::invalid_argument);
}

TEST(Validation, DimensionExceptionMessages) {
	auto first    = Matrix{{1, 2, 3}};
	auto second   = Matrix{{2, 3}, {4, 5}};
	auto third    = Matrix{{1}};
	auto longname = Matrix{{1}, {2}};

	ValidationContext ctx{ErrorMode::DIE};
	ctx.add_matrix(first, "first").dim(1, 2);
	ctx.add_matrix(second, "second").dim('M', 15);
	ctx.add_matrix(third, "third").dim('N', 'M');
	ctx.add_matrix(longname, "This is a really long name, isn't it?").dim('M', 'N');

	EXPECT_THROW(EXPECT_ERROR(ctx.validate(), "dimension"), std::range_error);
}

TEST(Validation, MoreDimensionExceptionMessages) {

	auto first  = Matrix{{1}, {2}, {3}};
	auto second = Matrix{{2, 3}, {4, 5}};
	auto third  = Matrix{{1}};

	ValidationContext ctx{ErrorMode::DIE};

	ctx.add_matrix(first, "first").dim(2, 1);
	ctx.add_matrix(second, "second").dim(15, 'M');
	ctx.add_matrix(third, "third").dim('N', 'M');
	EXPECT_THROW(EXPECT_ERROR(ctx.validate(), "dimension"), std::range_error);
}

TEST(Validation, SymmetricExceptionMessage) {
	ErrorModeLock guard{ErrorMode::DIE};
	auto foo = Matrix{{1, 2}, {3, 4}};
	EXPECT_THROW(
	    EXPECT_ERROR(ValidationContext{}.add_matrix(foo, "PenaeusMonodon").symmetric().validate(),
	                 "symm"),
	    std::domain_error);
}

TEST(Validation, SymmetricImpossibleBoundsExceptionMessage) {
	auto foo = Matrix{{1, 2}};

	ValidationContext ctx{ErrorMode::DIE};

	ctx.add_matrix(foo, "MacrobrachiumCarcinus");
	EXPECT_THROW(EXPECT_ERROR(ctx.symmetric().validate(), "symm"), std::out_of_range);
}

TEST(Validation, CombinedCriteria) {
	auto foo = Matrix{{1, 2}};
	auto bar = Matrix{{1, 2}, {2, 1}};
	auto baz = Matrix{{1, 2, 3}, {4, 5, 6}};

	ValidationContext ctx{ErrorMode::DIE};

	ctx.add_matrix(foo, "foo").dim(1, 'N').max(2.0 * PI).min(-2.0 * PI);
	ctx.add_matrix(bar, "bar").dim('N', 'N').symmetric();
	ctx.add_matrix(baz, "baz").dim('N', 1);
	EXPECT_THROW(EXPECT_ERROR(ctx.validate(), "dimension"), std::range_error);
}

TEST(Validation, TestVector) {
	auto foo = Vector{1, 2, 3, 4};
	auto bar = Vector{1, 2, 2, 1};

	ValidationContext ctx{ErrorMode::DIE};
	ctx.add_matrix(bar).dim(4, 1).min(0.0).max(5.0).validate();

	ValidationContext ctx2{ErrorMode::DIE};
	EXPECT_THROW(
	    EXPECT_ERROR(ctx2.add_matrix(foo).dim(2, 2).min(0.0).max(5.0).validate(), "dimension"),
	    std::range_error);
}

TEST(Validation, ValidationNeededWarning) {
	auto foo = Matrix{{1, 2}, {3, 4}};
	auto ctx = std::make_unique<ValidationContext>(ErrorMode::DIE);
	ctx->add_matrix(foo).dim(2, 2);
	EXPECT_ERROR(ctx.reset(), "destroyed without calling validate");
}

TEST(Validation, SymmetricNonsquareWithNoErrors) {
	ErrorModeLock guard{ErrorMode::OFF};
	auto foo = Matrix{{1, 2}};

	auto result = EXPECT_NO_LOG(ValidationContext{}.add_matrix(foo, "foo").symmetric().validate());
	EXPECT_EQ(result, ValidationResult::NOT_CHECKED);
}

TEST(Validation, SymmetricNonsquareWithLogOnly) {
	ErrorModeLock guard{ErrorMode::LOG};
	auto foo = Matrix{{1, 2}};

	auto result =
	    EXPECT_ERROR(ValidationContext{}.add_matrix(foo, "foo").symmetric().validate(), "symm");
	EXPECT_EQ(result, ValidationResult::BAD);
}

TEST(Validation, SymmetricSquareWithNoErrors) {
	ErrorModeLock guard{ErrorMode::OFF};
	auto foo = Matrix{{1, 2}, {3, 4}};

	auto result = EXPECT_NO_LOG(ValidationContext{}.add_matrix(foo, "foo").symmetric().validate());
	EXPECT_EQ(result, ValidationResult::NOT_CHECKED);
}

TEST(Validation, SymmetricSquareWithLogOnly) {
	ErrorModeLock guard{ErrorMode::LOG};
	auto foo = Matrix{{1, 2}, {3, 4}};

	auto result =
	    EXPECT_ERROR(ValidationContext{}.add_matrix(foo, "foo").symmetric().validate(), "symm");
	EXPECT_EQ(result, ValidationResult::BAD);
}

TEST(Validation, MaxWithNoErrors) {
	auto result = EXPECT_NO_LOG(
	    ValidationContext{ErrorMode::OFF}.add_matrix(Matrix{{12}}, "twelve").max(10).validate());
	EXPECT_EQ(result, ValidationResult::NOT_CHECKED);
}

TEST(Validation, MaxWithLogOnly) {
	auto result = EXPECT_ERROR(
	    ValidationContext{ErrorMode::LOG}.add_matrix(Matrix{{12}}, "twelve").max(10).validate(),
	    "> 10");
	EXPECT_EQ(result, ValidationResult::BAD);
}

TEST(Validation, MinWithNoErrors) {
	auto result = EXPECT_NO_LOG(
	    ValidationContext{ErrorMode::OFF}.add_matrix(Matrix{{12}}, "twelve").min(20).validate());
	EXPECT_EQ(result, ValidationResult::NOT_CHECKED);
}

TEST(Validation, MinWithLogOnly) {
	auto result = EXPECT_ERROR(
	    ValidationContext{ErrorMode::LOG}.add_matrix(Matrix{{12}}, "twelve").min(20).validate(),
	    "< 20");
	EXPECT_EQ(result, ValidationResult::BAD);
}
