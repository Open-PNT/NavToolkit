#pragma once

#include <xtensor/core/xmath.hpp>

#include <navtk/inspect.hpp>
#include <navtk/tensors.hpp>

namespace navtk {

/**
 * Converts an xtensor container object of 0 dimensions to a 1-D vector.
 *
 * @tparam T An xtensor container object.
 * @tparam IfTensorOfDim<> Invalidates the template for non-matching types. (Has no effect on the
 * function).
 * @param m Input object containing coefficients.
 * @return 1-D vector of the container object.
 */
template <class T, IfTensorOfDim<T, 0>* = nullptr>
Vector to_vec(const T& m) {
	if (has_zero_size(m)) return Vector{};
	return Vector{m()};
}

#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND
/**
 * Converts an xtensor container object of 1 dimension to a 1-D vector.
 *
 * @tparam T An xtensor container object.
 * @tparam IfTensorOfDim<> Invalidates the template for non-matching types. (Has no effect on the
 * function).
 * @param m Input object containing coefficients.
 */
template <class T, IfTensorOfDim<T, 1>* = nullptr>
Vector to_vec(const T& m) {
	return m;
}

/**
 * Converts an xtensor container object of 2 dimensions to a 1-D vector.
 * Data is not lost in this conversion.  It may be flattened if necessary.
 *
 * @tparam T An xtensor container object.
 * @tparam IfTensorOfDim<> Invalidates the template for non-matching types. (Has no effect on the
 * function).
 * @param m Input object containing coefficients.
 */
template <class T, IfTensorOfDim<T, 2>* = nullptr>
Vector to_vec(const T& m) {
	if (has_zero_size(m)) {
		return Vector{};
	} else if (m.shape()[0] == 1 && m.shape()[1] == 1) {
		return Tensor<1>{m(0, 0)};
	} else if (m.shape()[0] == 1 || m.shape()[1] == 1) {
		return xt::squeeze(m);
	}

	// Combine data from all rows into one row
	return xt::flatten(m);
}

/**
 * Converts an xtensor container object of 3 dimensions to a 1-D vector.
 * Data is not lost in this conversion.  It may be flattened if necessary.
 *
 * @tparam T An xtensor container object.
 * @tparam IfTensorOfDim<> invalidates the template for non-matching types. (Has no effect on the
 * function).
 * @param m Input object containing coefficients.
 */
template <class T, IfTensorOfDim<T, 3>* = nullptr>
Vector to_vec(const T& m) {
	if (has_zero_size(m)) {
		return Vector{};
	} else if (m.shape()[0] == 1 && m.shape()[1] == 1 && m.shape()[2] == 1) {
		return Tensor<1>{m(0, 0, 0)};
	}

	// if there is at least one dimmension that is not being used
	if (m.shape()[0] == 1 || m.shape()[1] == 1 || m.shape()[2] == 1) {
		auto squeezed = xt::squeeze(m);
		if (squeezed.dimension() == 1) {
			return squeezed;
		} else if (squeezed.dimension() == 2) {
			return xt::flatten(squeezed);
		}
	}

	return xt::flatten(m);
}

/**
 * Converts an Eigen-like container object to a 1-D vector.  The container must have
 * implemented a `rows()` function, a `cols()` function, and the `operator() `to work properly.
 *
 * @tparam T An Eigen-like container object.
 * @tparam IfEigenInterface<> invalidates the template for non-matching types. (Has no effect on the
 * function).
 * @param m Input object containing coefficients.
 */
template <typename T, IfEigenInterface<T>* = nullptr>
Vector to_vec(const T& m) {

	// TODO: PNTOS-56 Instead, could attempt to block memory copy if we can determine memory layout
	// of m
	auto rows = m.rows();
	auto cols = m.cols();

	Vector out = xt::zeros<Scalar>({rows * cols});
	for (decltype(rows) i = 0; i < rows; i++) {
		for (decltype(cols) j = 0; j < cols; j++) {
			out(i * cols + j) = m(i, j);
		}
	}
	return out;
}
#endif

/**
 * Converts an xtensor container object of 0 dimensions to a 2-D matrix.
 *
 * @tparam T An xtensor container object.
 * @tparam IfTensorOfDim<> Invalidates the template for non-matching types. (Has no effect on the
 * function).
 * @param m Input object containing coefficients.
 * @return 2-D matrix of the container object.
 */
template <class T, IfTensorOfDim<T, 0>* = nullptr>
Matrix to_matrix(const T& m, std::size_t = 1) {
	if (has_zero_size(m)) return Matrix{};
	return Matrix{{m()}};
}

#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND
/**
 * Converts an xtensor container object of 1 dimension to a 2-D matrix.
 *
 * @tparam T An xtensor matrix or expression type.
 * @tparam IfTensorOfDim<> Invalidates the template for non-matching types. (Has no effect on the
 * function).
 * @param m Input object containing coefficients.
 * @param axis Which axis to expand. When 0, a 4-element 1-D vector will return a 1x4 matrix. When
 * 1, a 4-element vector will return a 4x1 matrix.
 */
template <class T, IfTensorOfDim<T, 1>* = nullptr>
Matrix to_matrix(const T& m, std::size_t axis = 1) {
	return xt::expand_dims(m, axis);
}

/**
 * Converts an xtensor container object of 2 dimensions to a 2-D matrix.
 *
 * @tparam T An xtensor container object.
 * @tparam IfTensorOfDim<> Invalidates the template for non-matching types. (Has no effect on the
 * function).
 * @param m Input object containing coefficients.
 */
template <class T, IfTensorOfDim<T, 2>* = nullptr>
Matrix to_matrix(const T& m, std::size_t = 1) {
	return m;
}

/**
 * Converts an xtensor container object of 3 dimensions to a 2-D matrix.
 * Data is not lost in this conversion.  It may be flattened and placed into one of the matrix
 * dimensions, depending on the axis parameter.
 *
 * @tparam T An xtensor container object.
 * @tparam IfTensorOfDim<> Invalidates the template for non-matching types. (Has no effect on the
 * function).
 * @param m Input object containing coefficients.
 * @param axis Which axis to place flattened data into, if shape could not be squeezed.  When 0,
 * the result will be a 1xN matrix. When 1, the result will be an Nx1 matrix.
 */
template <class T, IfTensorOfDim<T, 3>* = nullptr>
Matrix to_matrix(const T& m, std::size_t axis = 1) {
	if (has_zero_size(m)) {
		return Matrix{};
	}

	if (m.shape()[0] == 1 && m.shape()[1] == 1 && m.shape()[2] == 1) {
		return Tensor<2>{{m(0, 0, 0)}};
	}

	if (m.shape()[0] == 1 || m.shape()[1] == 1 || m.shape()[2] == 1) {
		auto squeezed = xt::squeeze(m);
		if (squeezed.dimension() == 1) {
			return xt::expand_dims(squeezed, axis);
		} else if (squeezed.dimension() == 2) {
			return squeezed;
		}
	}

	auto flattened = xt::flatten(m);

	return xt::expand_dims(flattened, axis);
}

/**
 * Converts an Eigen-like container object to a 2-D matrix.  The container must have
 * implemented a `rows()` function, a `cols()` function, and the `operator()` to work properly.
 *
 * @tparam T An Eigen-like container object.
 * @tparam IfEigenInterface<> Invalidates the template for non-matching types. (Has no effect on the
 * function).
 * @param m Input object containing coefficients.
 */
template <typename T, IfEigenInterface<T>* = nullptr>
Matrix to_matrix(const T& m, std::size_t = 1) {

	// TODO: PNTOS-56 Instead, could attempt to block memory copy if we can determine memory layout
	// of m
	auto rows = m.rows();
	auto cols = m.cols();

	Matrix out = xt::zeros<Scalar>({rows, cols});
	for (decltype(rows) i = 0; i < rows; i++) {
		for (decltype(cols) j = 0; j < cols; j++) {
			out(i, j) = m(i, j);
		}
	}
	return out;
}

/**
 * Converts any fixed-size 2d array to a Matrix.
 */
template <typename T, std::size_t rows, std::size_t cols>
Matrix to_matrix(T (&data)[rows][cols]) {
	Matrix out = xt::zeros<Scalar>({rows, cols});
	for (Size ii = 0; ii < rows; ++ii)
		for (Size jj = 0; jj < cols; ++jj) out(ii, jj) = data[ii][jj];
	return out;
}
#endif

/**
 * Returns the NxM Matrix with given row and column size. The
 * placement of diagonal ones is specified with  `diagonal_index`.
 * By default, the diagonal ones are placed starting at `diagonal_index=0`,
 * or matrix index (0,0). `diagonal_index` can range from `-(rows-1)` to
 * `(cols-1)`.
 *
 * @param rows Number of rows for desired matrix.
 * @param cols Number of columns for desired matrix.
 * @param diagonal_index Placement index of the diagonal.
 *
 * @throw Throws an `invalid_argument` exception when `diagonal_index` is less than `-(cols-1)` or
 * greater than `(rows-1)` and the error mode is ErrorMode::DIE for either case.
 *
 * @return NxM Matrix where N is the number of rows and M is the number of columns.
 */
Matrix eye(Size rows, Size cols, int diagonal_index = 0);

/**
 * Returns the square identity Matrix with given number size.
 *
 * @param size Size of desired square Matrix.
 *
 * @return Identity Matrix of given size.
 */
Matrix eye(Size size);

#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND
/**
 * Returns a tensor filled with zeros.
 *
 * The return value's dimensionality will match the number of arguments you pass in. In other words,
 * the function returns a Vector given one parameter, a Matrix given two parameters, and
 * higher-order Tensors for more parameters.
 *
 * @param dim dimensions of the Tensor you're building.
 * @return A Tensor filled with zeros.
 */
template <typename... T>
Tensor<sizeof...(T)> zeros(T... dim) {
	return xt::zeros<Scalar>({Size(dim)...});
}

/**
 * Returns a tensor filled with the given value.
 *
 * The return value's dimensionality will match the number of arguments in the \p dim tuple. In
 * other words, the function returns a Vector given one parameter, a Matrix given two parameters,
 * and higher-order Tensors for more parameters.
 *
 * @param value Number with which to populate the resulting tensor.
 * @param dim dimensions of the Tensor you're building.
 * @return A Tensor filled with copies of the given value.
 */
template <typename... T>
Tensor<sizeof...(T)> fill(Scalar value, T... dim) {
	return zeros(dim...) + value;
}

/**
 * Returns a tensor filled with ones.
 *
 * The return value's dimensionality will match the number of arguments you pass in. In other words,
 * the function returns a Vector given one parameter, a Matrix given two parameters, and
 * higher-order Tensors for more parameters.
 *
 * @param dim dimensions of the Tensor you're building.
 * @return A Tensor filled with ones.
 */
template <typename... T>
Tensor<sizeof...(T)> ones(T... dim) {
	return xt::ones<Scalar>({Size(dim)...});
}
#endif

/**
 * Create a block-diagonal matrix from the provided matrices.
 *
 * Given square matrix inputs A, B and C, the output will have these arrays
 * arranged along the diagonal:
 *
 * ```
 * {{A, 0, 0},
 *  {0, B, 0},
 *  {0, 0, C}}
 * ```
 *
 * Vectors are assumed to be horizontal matrices (single-row), matching the
 * behavior of `scipy.linalg.block_diag`.
 *
 * @param matrices A series of matrices to be arranged.
 *
 * @return A single matrix containing the input matrices arranged diagonally,
 * such that the index of the top-left of a given matrix is (1, 1) plus the
 * index of the bottom-right of the preceding matrix.
 */
Matrix block_diag(std::initializer_list<Matrix> matrices);

/**
 * Create a block-diagonal matrix from the provided matrices.
 *
 * Given square matrix inputs A, B and C, the output will have these arrays
 * arranged along the diagonal:
 *
 * ```
 * {{A, 0, 0},
 *  {0, B, 0},
 *  {0, 0, C}}
 * ```
 *
 * Vectors are assumed to be horizontal matrices (single-row), matching the
 * behavior of `scipy.linalg.block_diag`.
 *
 * @param matrices A series of Tensor objects to be arranged.
 *
 * @return A single matrix containing the input matrices arranged diagonally,
 * such that the index of the top-left of a given matrix is (1, 1) plus the
 * index of the bottom-right of the preceding matrix.
 */
template <typename... T>
Matrix block_diag(T&&... matrices) {
	return block_diag({to_matrix(matrices, 0)...});
}

}  // namespace navtk
