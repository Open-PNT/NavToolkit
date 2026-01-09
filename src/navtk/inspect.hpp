#pragma once

#include <navtk/tensors.hpp>

namespace navtk {

/**
 * Checks whether there is any data in a given iterable object.
 * Intended as a safer alternative to `!expression.size()`, since some xtensor
 * types can crash when `.size()` is called.
 * @param expression An iterable object.
 * @return The boolean result.
 */
template <typename T>
bool has_zero_size(const T& expression) {
	return expression.begin() == expression.end();
}

/**
 * @param mat The matrix whose symmetry is evaluated.
 * @param rtol Relative tolerance allowed between 2 elements; unitless.
 * @param atol Absolute tolerance allowed between 2 elements; units determined by elements being
 * compared.
 * @return `true` if \p mat is symmetric within a tolerance and `false` otherwise. Does not check if
 *  square.
 */
bool is_symmetric(const Matrix& mat, double rtol = 1e-5, double atol = 1e-8);

/**
 * Check if a square matrix is diagonal.
 *
 * @param mat Matrix to check. Equal rows/cols are required but not checked.
 *
 * @return True if all off-diagonal elements are 0.
 */
bool is_diagonal(const Matrix& mat);

/**
 * Check if a square matrix is identity.
 *
 * @param mat Matrix to check. Equal rows/cols are required but not checked.
 *
 * @return True if all off-diagonal elements are 0 and all diagonal elements are
 *  1. Returns false if matrix has 0 size.
 */
bool is_identity(const Matrix& mat);

/**
 * Generate a set of row/column indices for non-symmetric matrix elements.
 *
 * @param mat Matrix to check. Equal rows/cols are required but not checked.
 * @param rtol Relative tolerance allowed between 2 elements; unitless.
 * @param atol Absolute tolerance allowed between 2 elements; units determined by elements being
 * compared.
 *
 * @return A vector of pairs of row/col indices (upper triangular only) for corresponding elements
 * that do not match within tolerance.
 */
std::vector<std::pair<Size, Size>> non_symmetric_elements(const Matrix& mat,
                                                          double rtol = 1e-5,
                                                          double atol = 1e-8);

/**
 * Templated struct which allows us to obtain metadata information from a matrix/vector type.
 * This template specialization is matched if no additional information can be obtained.
 *
 * @tparam T A type from which we are attempting to gain additional information.
 * @tparam class class = void permits this to be a fallback if template specializations match.
 */
template <typename T, class = void>
struct TensorMeta {
	/**
	 * Indicates whether the dimensions of the tensor are fixed.
	 */
	static constexpr bool FIXED_DIMS = false;
};

/**
 * Templated struct which allows us to obtain metadata information from an xtensor type,
 * intended to support template functions and classes that use xtensor objects.
 *
 * @tparam T An xtensor type from which we are attempting to gain additional information.
 * @tparam std::enable_if_t<> Invalidates this structure for types that are not xtensor expressions.
 */
template <typename T>
struct TensorMeta<T, std::enable_if_t<xt::is_xexpression<T>::value>> {
	/**
	 * Whether the dimensions of the tensor are fixed.
	 */
	static constexpr bool FIXED_DIMS = true;

	/**
	 * The number of dimensions.
	 */
	static constexpr auto dimCount =
	    std::tuple_size<typename std::remove_reference<T>::type::shape_type>::value;
};


/**
 * `IfTensorOfDim` can be used in a template definition to invalidate the template. To be valid,
 * type T must be a Tensor type with `Dim` number of dimensions.
 */
template <typename T, std::size_t Dim>
using IfTensorOfDim = std::enable_if_t<TensorMeta<T>::dimCount == Dim>;

/**
 * `IfBothTensorsOfDim` can be used in a template definition to invalidate the template. To be
 * valid, type A and type B must both be Tensor types with `Dim` number of dimensions.
 */
template <typename A, typename B, std::size_t Dim>
using IfBothTensorsOfDim =
    std::enable_if_t<TensorMeta<A>::dimCount == Dim && TensorMeta<B>::dimCount == Dim>;

/**
 * `IfFirstTensorOfDim` can be used in a template definition to invalidate the template. To be
 * valid, type A must have `Dim` number of dimensions, and type B must not.  Both must be Tensor
 * types.
 */
template <typename A, typename B, std::size_t Dim>
using IfFirstTensorOfDim =
    std::enable_if_t<TensorMeta<A>::dimCount == Dim && TensorMeta<B>::dimCount != Dim>;

/**
 * `IfSecondTensorOfDim` can be used in a template definition to invalidate the template. To be
 * valid, type B must have `Dim` number of dimensions, and type A must not.  Both must be Tensor
 * types.
 */
template <typename A, typename B, std::size_t Dim>
using IfSecondTensorOfDim =
    std::enable_if_t<TensorMeta<A>::dimCount != Dim && TensorMeta<B>::dimCount == Dim>;

/**
 * `IfEigenInterface` can be used in a template definition to invalidate the template. To be valid,
 * type `T` must have member functions named `rows()` and `cols()`.  This enables support for
 * Eigen-like interfaces.
 */
template <typename T>
using IfEigenInterface =
    std::enable_if_t<std::is_member_function_pointer<decltype(&T::rows)>::value &&
                     std::is_member_function_pointer<decltype(&T::cols)>::value>;

/**
 * Returns the number of rows in a Matrix. For empty matrices, returns zero.
 *
 * @param m The Matrix to inspect.
 *
 * @return The number of rows.
 */
Size num_rows(const Matrix& m);

/**
 * Returns the number of rows in a column Vector. For empty vectors, returns
 * zero.
 *
 * @param c The Vector to inspect.
 *
 * @return The number of rows.
 */
Size num_rows(const Vector& c);

/**
 * Returns the number of columns in a Matrix. For empty matrices, returns zero.
 * @param m The Matrix to inspect.
 *
 * @return The number of columns.
 */
Size num_cols(const Matrix& m);

/**
 * Returns the number of columns in a row Vector. For empty vectors, returns
 * zero.
 *
 * @param r The Vector to inspect.
 *
 * @return The number of columns.
 */
Size num_cols(const Vector& r);

}  // namespace navtk
