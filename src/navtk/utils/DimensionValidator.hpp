#pragma once

#include <string>

#include <navtk/not_null.hpp>
#include <navtk/utils/ValidationContext.hpp>
#include <navtk/utils/ValidationResult.hpp>

namespace navtk {
namespace utils {

/**
 * Forward declaration of class used in implementation.
 */
class DimensionValidatorPrivate;

/**
 * The interface to a class that validates dimensions.
 *
 * Note that this class is only expected to be used by the ValidationContext class.
 *
 * This validator can function across multiple matrices. Add matrices and expected
 * dimensions with a dim() function.  After all matrices have been loaded, call perform_validation()
 * to validate all of the matrices.
 */
class DimensionValidator {
public:
	DimensionValidator();

	~DimensionValidator();

	/**
	 * Deleted to prevent pointer invalidation.
	 */
	DimensionValidator(const DimensionValidator& other) = delete;

	/**
	 * Deleted to prevent pointer invalidation.
	 */
	DimensionValidator(DimensionValidator&& other) = delete;

	/**
	 * Deleted to prevent pointer invalidation.
	 */
	DimensionValidator& operator=(const DimensionValidator& other) = delete;

	/**
	 * Deleted to prevent pointer invalidation.
	 */
	DimensionValidator& operator=(DimensionValidator&& other) = delete;

	/**
	 * Provide a matrix, name, and dimensions to validate.
	 * Validation is not done here.  After all matrices are loaded,
	 * call perform_validation().
	 *
	 * @param name The name of the matrix to show in errors.
	 * @param matrix The matrix to validate.
	 * @param rows The desired number of rows in the matrix.
	 * @param cols The desired number of columns in the matrix.
	 */
	void dim(const std::string& name, const Matrix& matrix, Size rows, Size cols);

	/**
	 * Provide a matrix, name, and dimensions to validate.
	 * Validation is not done here.  After all matrices are loaded,
	 * call perform_validation().
	 *
	 * @param name The name of the matrix to show in errors.
	 * @param matrix The matrix to validate.
	 * @param rows The desired number of rows in the matrix.
	 * @param cols Variable name for number of columns in the matrix.
	 */
	void dim(const std::string& name, const Matrix& matrix, Size rows, char cols);

	/**
	 * Provide a matrix, name, and dimensions to validate.
	 * Validation is not done here.  After all matrices are loaded,
	 * call perform_validation().
	 *
	 * @param name The name of the matrix to show in errors.
	 * @param matrix The matrix to validate.
	 * @param rows Variable name for number of rows in the matrix.
	 * @param cols The desired number of columns in the matrix.
	 */
	void dim(const std::string& name, const Matrix& matrix, char rows, Size cols);

	/**
	 * Provide a matrix, name, and dimensions to validate.
	 * Validation is not done here.  After all matrices are loaded,
	 * call perform_validation().
	 *
	 * @param name The name of the matrix to show in errors.
	 * @param matrix The matrix to validate.
	 * @param rows Variable name for number of rows in the matrix.
	 * @param cols Variable name for number of columns in the matrix.
	 */
	void dim(const std::string& name, const Matrix& matrix, char rows, char cols);

	/**
	 * Check matrices against set rules and populate the given ValidationResult, optionally throwing
	 * an exception or logging an error in the case of a bad result.
	 *
	 * @param mode How to behave when validation fails.
	 * @param result_out Reference that will be set to ValidationResult::GOOD or
	 * ValidationResult::BAD depending on the outcome of validation. This is an out parameter,
	 * rather than a return value, to ensure its value is written before any exception is thrown.
	 * @throw std::range_error if dimension validation fails and mode is ErrorMode::DIE.
	 */
	void perform_validation(ErrorMode mode, ValidationResult& result_out);

private:
	/**
	 * Member responsible for validating dimension checks.  Holds an internal pointer to various
	 * matrices to validate.
	 */
	not_null<std::unique_ptr<DimensionValidatorPrivate>> implementation;
};

}  // namespace utils
}  // namespace navtk
