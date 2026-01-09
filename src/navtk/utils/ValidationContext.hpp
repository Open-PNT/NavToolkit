#pragma once

#include <list>
#include <memory>
#include <string>

#include <navtk/errors.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/ValidationResult.hpp>

namespace navtk {
namespace utils {

/**
 * Forward declaration of class used in implementation.
 */
class DimensionValidator;

/**
 * A class to validate matrices.  Instantiate and then call add_matrix() with a matrix to validate,
 * followed by the validation functions to perform on that matrix.  A context can hold multiple
 * matrices, to validate across all matrices in the context if needed.  After the specific checks to
 * the first matrix are added, call the next matrix with add_matrix() and follow that with the
 * validation functions for the second matrix.
 *
 * Some validation functions apply only to one matrix.  The validations will be performed
 * immediately.
 *
 * Validation functions that apply across matrices will not be performed until validate() is called.
 *
 * To disable matrix validation, compile with `-DNO_MATRIX_VAL`.  This will increase performance.
 */
class ValidationContext {
public:
	/**
	 * Constructor that uses the current value of get_global_error_mode() to determine validation
	 * behavior. A copy of the current ErrorMode is stored, so subsequent calls to
	 * set_global_error_mode() will not affect this instance.
	 */
	ValidationContext();

	/**
	 * Create validation context in the given mode, where ErrorMode::OFF bypasses all checks,
	 * ErrorMode::LOG prints a log message and continues, and ErrorMode::DIE throws an exception in
	 * the event of a validation failure.
	 * @param mode Error mode for this ValidationContext
	 */
	ValidationContext(ErrorMode mode);

	~ValidationContext();

	/**
	 * Deleted to prevent pointer invalidation.
	 */
	ValidationContext(const ValidationContext& other) = delete;

	/**
	 * Deleted to prevent pointer invalidation.
	 */
	ValidationContext(ValidationContext&& other) = delete;

	/**
	 * Deleted to prevent pointer invalidation.
	 */
	ValidationContext& operator=(const ValidationContext& other) = delete;

	/**
	 * Deleted to prevent pointer invalidation.
	 */
	ValidationContext& operator=(ValidationContext&& other) = delete;

	/**
	 * Add a 2-D matrix to validate in this context with the default name.  It is possible to add
	 * multiple matrices to this context.  Specify the validations to occur on this matrix before
	 * adding another.
	 *
	 * In general it is preferable to use the other version of this function that lets you specify a
	 * name. All matrices that are validated with this method will appear in the error message with
	 * the default name.
	 *
	 * @param matrix A matrix to validate.
	 *
	 * @return A validation context that can be used to chain add_matrix() and validation functions.
	 */
	ValidationContext& add_matrix(const Matrix& matrix);

	/**
	 * Add a 2-D matrix to validate in this context with the given name.  It is possible to add
	 * multiple matrices to this context.  Specify the validations to occur on this matrix before
	 * adding another.
	 *
	 * \pre
	 * {@code
	 * ValidationContext ctx {};
	 * ctx.add_matrix(myMatrix, "myMatrix").dim(1, 4).validate() // validates myMatrix is 1 by 4.
	 * }
	 *
	 * @param matrix A matrix to validate.
	 * @param name The name of the matrix (used in displayed errors).
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 */
	ValidationContext& add_matrix(const Matrix& matrix, const std::string& name);

	/**
	 * Add a 1-D matrix to validate in this context with the default name.  It is possible to add
	 * multiple matrices to this context.  Specify the validations to occur on this matrix before
	 * adding another.
	 *
	 * In general it is preferable to use the other version of this function that lets you specify a
	 * name. All matrices that are validated with this method will appear in the error message with
	 * the default name.
	 *
	 * @param matrix A matrix to validate.
	 *
	 * @return A validation context that can be used to chain add_matrix() and validation functions.
	 */
	ValidationContext& add_matrix(const Vector& matrix);

	/**
	 * Add a 1-D matrix to validate in this context with the given name.  It is possible to add
	 * multiple matrices to this context.  Specify the validations to occur on this matrix before
	 * adding another.
	 *
	 * \pre
	 * {@code
	 * ValidationContext ctx {};
	 * ctx.add_matrix(myMatrix, "myMatrix").dim(1, 4).validate() // validates myMatrix is 1 by 4.
	 * }
	 *
	 * @param matrix A matrix to validate.
	 * @param name The name of the matrix (used in displayed errors).
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 */
	ValidationContext& add_matrix(const Vector& matrix, const std::string& name);

	/**
	 * Checks to see if the current matrix in the context is symmetric.
	 * Call add_matrix() before this function.  This is an immediate validation.
	 *
	 * @param rtol Relative tolerance for symmetric values in order to be considered equivalent.
	 * @param atol Absolute tolerance for symmetric values in order to be considered equivalent.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 * @throw std::out_of_range if the matrix is not square and the error mode is ErrorMode::DIE.
	 * @throw std::domain_error if the matrix is square but not symmetric and the error mode is
	 * ErrorMode::DIE.
	 */
	ValidationContext& symmetric(double rtol = 1e-5, double atol = 1e-8);

	/**
	 * Ensure all of the elements in the current matrix are <= the given limit.
	 * Call add_matrix() before this function.  This is an immediate validation.
	 *
	 * @param limit The maximum value required of all elements in the matrix.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 * @throw std::invalid_argument if an element in the matrix is greater than the limit and the
	 * error mode is ErrorMode::DIE.
	 */
	ValidationContext& max(double limit);

	/**
	 * Ensure all of the elements in the current matrix are >= the given limit.
	 * Call add_matrix() before this function.  This is an immediate validation.
	 *
	 * @param limit The minimum value required of all elements in the matrix.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 * @throw std::invalid_argument if an element in the matrix is less than the limit and the
	 * error mode is ErrorMode::DIE.
	 */
	ValidationContext& min(double limit);

	/**
	 * Require the current matrix to have exactly the given number of rows and columns.
	 * Call add_matrix() before this function.  After all matrices and dim() functions have been
	 * added to the context, call validate() to perform dimension validation.
	 *
	 * @param rows Fixed number of rows to require.
	 * @param cols Fixed number of columns to require.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 */
	ValidationContext& dim(Size rows, Size cols);

	/**
	 * Require the current matrix to have exactly the given number of rows and columns.
	 * Call add_matrix() before this function.  After all matrices and dim() functions have been
	 * added to the context, call validate() to perform dimension validation.
	 *
	 * @param rows Fixed number of rows to require.
	 * @param cols Fixed number of columns to require.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 */
	ValidationContext& dim(Size rows, int cols);

	/**
	 * Require the current matrix to have exactly the given number of rows and columns.
	 * Call add_matrix() before this function.  After all matrices and dim() functions have been
	 * added to the context, call validate() to perform dimension validation.
	 *
	 * @param rows Fixed number of rows to require.
	 * @param cols Fixed number of columns to require.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 */
	ValidationContext& dim(int rows, Size cols);

	/**
	 * Require the current matrix to have exactly the given number of rows and columns.
	 * Call add_matrix() before this function.  After all matrices and dim() functions have been
	 * added to the context, call validate() to perform dimension validation.
	 *
	 * @param rows Fixed number of rows to require.
	 * @param cols Fixed number of columns to require.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 */
	ValidationContext& dim(int rows, int cols);

	/**
	 * Require the current matrix to have exactly the given number of rows and match the number of
	 * columns with other dimensions in other matrices based on the cols variable name. Call
	 * add_matrix() before this function.  After all matrices and dim() functions have been added to
	 * the context, call validate() to perform dimension validation.
	 *
	 * @param rows Fixed number of rows to require.
	 * @param cols Variable name for the number of cols in the matrix.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 */
	ValidationContext& dim(int rows, char cols);

	/**
	 * Require the current matrix to have exactly the given number of rows and columns.
	 * Call add_matrix() before this function.  After all matrices and dim() functions have been
	 * added to the context, call validate() to perform dimension validation.
	 *
	 * @param rows Fixed number of rows to require.
	 * @param cols Fixed number of columns to require.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 */
	ValidationContext& dim(Size rows, char cols);

	/**
	 * Require the current matrix to have exactly the given number of columns and match the number
	 * of rows with other dimensions in other matrices based on the rows variable name. Call
	 * add_matrix() before this function.  After all matrices and dim() functions have been added to
	 * the context, call validate() to perform dimension validation.
	 *
	 * @param rows Variable name for the number of rows in the matrix.
	 * @param cols Fixed number of columns to require.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 */
	ValidationContext& dim(char rows, int cols);

	/**
	 * Require the current matrix to have exactly the given number of rows and columns.
	 * Call add_matrix() before this function.  After all matrices and dim() functions have been
	 * added to the context, call validate() to perform dimension validation.
	 *
	 * @param rows Fixed number of rows to require.
	 * @param cols Fixed number of columns to require.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 */
	ValidationContext& dim(char rows, Size cols);

	/**
	 * Require the current matrix's dimensions to correspond to the given variable names. Compares
	 * with other dimensions in other matrices in the context that are assigned the same variable
	 * name. Call add_matrix() before this function.  After all matrices and dim() functions have
	 * been added to the context, call validate() to perform dimension validation.
	 *
	 * @param rows Variable name for the number of rows in the matrix.
	 * @param cols Variable name for the number of cols in the matrix.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 */
	ValidationContext& dim(char rows, char cols);

	/** TODO #587: Not implemented
	 * Accept a transposed version of the matrix as satisfying the dimensions check. For example,
	 * allow a 1 x 3 matrix when dimensions are declared as 3 x 1. Call add_matrix() and dim()
	 * before this function.  After all matrices and dim() functions have been added to the context,
	 * call validate() to perform dimension validation.
	 *
	 * @return The validation context, which can be used to chain add_matrix() and validation
	 * functions.
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() or dimensions with
	 * dim() and the error mode is ErrorMode::DIE for either case.
	 */
	// ValidationContext& transposable();

	/**
	 * Check declared matrices against any rules that apply across multiple matrices.
	 * As good practice, always finish your validations by calling this function.
	 * @return ValidationResult showing the outcome of the validation.
	 * @throw std::range_error if dimension validation fails and the error mode is ErrorMode::DIE.
	 */
	ValidationResult validate();

	/**
	 * This is either the error mode passed in to the constructor, or if default-constructed, the
	 * value of get_global_error_mode() at time of construction.
	 *
	 * @return The ErrorMode of this ValidationContext.
	 */
	ErrorMode get_mode() const;

	/**
	 * Convenience method for `get_mode() != ErrorMode::OFF`
	 *
	 * @return `true` if any validation is to be performed. `false` if validation should be bypassed
	 */
	bool is_enabled() const;

	/**
	 * Convenience cast, returning `get_mode() != ErrorMode::OFF`
	 *
	 * This can be used to wrap all validation in an `if` statement that uses a single context.
	 *
	 * ```
	 * if (ValidationContext validation{}) {
	 *    // do extra computation for validation here
	 * }
	 * ```
	 *
	 * It is not necessary to use this construct if your validation routines are written entirely in
	 * terms of other ValidationContext methods, as these will automatically be skipped when
	 * ErrorMode::OFF. If however you're doing extra manual validation, or computing values to use
	 * in ValidationContext method calls, you can use this technique to save processing time.
	 *
	 * @return `true` if any validation is to be performed. `false` if validation should be
	 * bypassed.
	 */
	operator bool() const;

protected:
	/**
	 * Member responsible for validating dimension checks.  Holds an internal pointer to various
	 * matrices to validate.
	 */
	not_null<std::unique_ptr<DimensionValidator>> dimension_validator;

	/**
	 * Storage for a copy of all matrices and names that we are going to validate in this context.
	 */
	std::list<std::pair<Matrix, std::string>> matrices_to_validate;

	/**
	 * Pointer to latest matrix added with add_matrix().  Shortcut for
	 * `matrices_to_validate.back().first`.
	 */
	const Matrix* current_matrix;

	/**
	 * Pointer to latest matrix name added with add_matrix().  Shortcut for
	 * `matrices_to_validate.back().second`.
	 */
	const std::string* current_matrix_name;

	/**
	 * Checks the current_matrix member variable and potentially throws an exception if it is null.
	 *
	 * @throw std::runtime_error if a matrix has not been added with add_matrix() and the error mode
	 * is ErrorMode::DIE.
	 * @return `true` if current_matrix is non-null. `false` if current_matrix is `nullptr` but
	 * exceptions are disabled.
	 */
	bool check_current_matrix() const;

	/**
	 * Updates the value of cached_result to indicate that validation information has been added but
	 * validation has not yet been performed.
	 */
	void mark_validation_needed();

	/**
	 * Check whether information has been loaded into this ValidationContext but not yet validated.
	 * @return `true` if validation calculations need to be performed.
	 */
	bool validation_needed() const;

	/**
	 * Keeps track of whether the validate() function needs to be called in order to
	 * complete the validation.
	 */
	ValidationResult cached_result;

	/**
	 * Behavior of assorted validation functions, OFF to skip checks, LOG to log a message, DIE to
	 * throw an exception.
	 */
	ErrorMode mode;
};

}  // namespace utils
}  // namespace navtk
