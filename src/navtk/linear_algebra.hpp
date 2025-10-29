#pragma once

#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/reducers/xnorm.hpp>

#include <navtk/factory.hpp>
#include <navtk/inspect.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/ValidationContext.hpp>
#include <xtensor/misc/xpad.hpp>

namespace navtk {

/**
 * Computes the matrix exponential (e^A) using Pade approximation.
 *
 * Reference: this function was ported from Koma: https://koma.kyonifer.com
 * whose expm function was ported from scipy:
 *
 * Copyright (c) 2001-2002 Enthought, Inc.  2003-2019, SciPy Developers.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * References
 *    ----------
 *   .. [1] Awad H. Al-Mohy and Nicholas J. Higham (2009)
 *          "A New Scaling and Squaring Algorithm for the Matrix Exponential."
 *          SIAM Journal on Matrix Analysis and Applications.
 *          31 (3). pp. 970-989. ISSN 1095-7162.
 *
 * @param matrix A square matrix (NxN).
 *
 * @return The matrix exponential of the input matrix (e^matrix).
 * @throw std::range_error If matrix is not NxN and the error mode is ErrorMode::DIE.
 */
Matrix expm(const Matrix& matrix);

/**
 * Raise a square matrix to n power (ie dot n times).
 *
 * @param matrix A square matrix (NxN).
 * @param n Power to raise to. If negative the result is equivalent to matrix_power(inv(matrix),
 * -n).
 *
 * @return Result of squaring. If n = 0 returns identity matrix of same shape as \p matrix.
 */
Matrix matrix_power(const Matrix& matrix, long n);

/**
 * Calculate the Cholesky decomposition of a matrix.
 * @param matrix A square matrix (NxN).
 *
 * @return Lower triangular Cholesky Matrix, same size as A. Though, if A is singular (zero
 * determinant) then return the square root of the main diagonal elements.
 * @throw std::runtime_error If A is non-square or contains a negative element along diagonal and
 * the error mode is ErrorMode::DIE.
 */
Matrix chol(const Matrix& matrix);

/**
 * Calculate the square root of all values on the main diagonal as an approximation to the Cholesky
 * decomposition of a matrix. This may be useful when matrix is not positive definite but an
 * approximation is acceptable.
 * @param matrix A square matrix (NxN).
 *
 * @return Diagonal matrix, same size as A (NxN).
 * @throw std::runtime_error If A is non-square or contains a negative element along diagonal and
 * the error mode is ErrorMode::DIE.
 */
Matrix sqrt_of_main_diagonal(const Matrix& matrix);

/**
 * Calculate the covariance of a NxK matrix, where K is the number of samples and N the number of
 * states.
 * @param matrix A square matrix (NxN).
 *
 * @return Covariance.
 */
Matrix calc_cov(const Matrix& matrix);

/**
 * Calculate the weighted covariance of a NxK matrix, where K is the number of samples and N the
 * number of states.
 * @param matrix A square matrix (NxN).
 * @param weights A vector of sample weights.
 *
 * @return Weighted covariance.
 */
Matrix calc_cov_weighted(const Matrix& matrix, const Vector& weights);

/**
 * Returns the matrix product of \p a and \p b .
 *
 * Don't use this function directly. Use dot() instead. This function exists as an implementation
 * detail: in particular, to prevent tensors.hpp from bringing in any particular set of linear
 * algebra headers.
 *
 * @param a The left matrix.
 * @param b The right matrix.
 *
 * @return The matrix (inner) product \p a *\p b.
 */
Matrix _dot(const Matrix& a, const Matrix& b);

/**
 * @tparam A An xtensor container object with 1 dimension.
 * @tparam B An xtensor container object with 2 dimensions.
 * @tparam IfFirstTensorOfDim<> Invalidates the template for non-matching types. (Has no effect on
 * the function).
 * @param a The left 1-D matrix.
 * @param b The right matrix.
 *
 * @return the matrix product of \p a and \p b .
 */
template <typename A, typename B, IfFirstTensorOfDim<A, B, 1>* = nullptr>
Vector dot(A&& a, B&& b) {
	return to_vec(_dot(to_matrix(std::forward<A>(a), 0), to_matrix(std::forward<B>(b))));
}

#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND
/**
 * Returns the matrix product of \p a and \p b .
 * @tparam A An xtensor container object with 2 dimensions.
 * @tparam B An xtensor container object with 1 dimension.
 * @tparam IfSecondTensorOfDim<> Invalidates the template for non-matching types. (Has no effect on
 * the function).
 * @param a The left matrix.
 * @param b The right 1-D matrix.
 *
 * @return The matrix (inner) product \p a *\p b (as a Vector).
 */
template <typename A, typename B, IfSecondTensorOfDim<A, B, 1>* = nullptr>
Vector dot(A&& a, B&& b) {
	return to_vec(_dot(to_matrix(std::forward<A>(a)), to_matrix(std::forward<B>(b), 1)));
}

/**
 * Returns the vector dot product of \p a and \p b .
 * @tparam A An xtensor container object with 1 dimension.
 * @tparam B An xtensor container object with 1 dimension.
 * @tparam IfBothTensorsOfDim<> Invalidates the template for non-matching types. (Has no effect on
 * the function).
 * @param a The left 1-D matrix.
 * @param b The right 1-D matrix.
 *
 * @return The vector dot product \p a * \p b.
 */
template <typename A, typename B, IfBothTensorsOfDim<A, B, 1>* = nullptr>
Vector dot(A&& a, B&& b) {
	return to_vec(
	    _dot(xt::transpose(to_matrix(std::forward<A>(a), 1)), to_matrix(std::forward<B>(b), 1)));
}

/**
 * Returns the matrix product of \p a and \p b .
 * @tparam A An xtensor container object with 2 dimensions.
 * @tparam B An xtensor container object with 2 dimensions.
 * @tparam IfBothTensorsOfDim<> Invalidates the template for non-matching types. (Has no effect on
 * the function).
 * @param a The left matrix.
 * @param b The right matrix.
 *
 * @return The matrix (inner) product \p a * \p b.
 */
template <typename A, typename B, IfBothTensorsOfDim<A, B, 2>* = nullptr>
Matrix dot(A&& a, B&& b) {
	Matrix a_mat = to_matrix(std::forward<A>(a));
	Matrix b_mat = to_matrix(std::forward<B>(b));

#	ifdef __aarch64__
	// Manually multiply on ARM because xt::linalg::dot seems to trigger UB (#906)
	if (has_zero_size(a_mat) || has_zero_size(b_mat)) return Matrix{};

	if (utils::ValidationContext validation{}) {
		validation.add_matrix(a_mat, "a_mat")
		    .dim('X', 'N')
		    .add_matrix(b_mat, "b_mat")
		    .dim('N', 'Y')
		    .validate();
	}

	auto left_rows = num_rows(a_mat);
	auto left_cols = num_cols(a_mat);
	// auto right_rows = right.shape().at(0);
	auto right_cols = num_cols(b_mat);
	Matrix out      = zeros(left_rows, right_cols);
	for (size_t i = 0; i < left_rows; i++) {
		for (size_t idx = 0; idx < left_cols; idx++) {
			for (size_t j = 0; j < right_cols; j++) {
				out(i, j) += a(i, idx) * b(idx, j);
			}
		}
	}
	return out;
#	else
	return _dot(a_mat, b_mat);
#	endif
}

/**
 * Returns the matrix product of \p a and transpose( \p b ) .
 * @tparam A An xtensor container object with 2 dimensions.
 * @tparam B An xtensor container object with 2 dimensions.
 * @tparam IfBothTensorsOfDim<> Invalidates the template for non-matching types.
 *
 * @param a The left matrix.
 * @param b The right matrix.
 *
 * @return The matrix (inner) product \p a * transpose( \p b ).
 */
template <typename A, typename B, IfBothTensorsOfDim<A, B, 2>* = nullptr>
Matrix transpose_a_dot_b(A&& a, B&& b) {
	Matrix a_mat = to_matrix(std::forward<A>(a));
	Matrix b_mat = to_matrix(std::forward<B>(b));

#	ifdef __aarch64__
	// Manually multiply on ARM because xt::linalg::dot seems to trigger UB (#906)
	if (has_zero_size(a_mat) || has_zero_size(b_mat)) return Matrix{};

	if (utils::ValidationContext validation{}) {
		validation.add_matrix(a_mat, "a_mat")
		    .dim('X', 'N')
		    .add_matrix(b_mat, "b_mat")
		    .dim('Y', 'N')
		    .validate();
	}

	Size rows      = num_rows(a_mat);
	Size columns   = num_rows(b_mat);
	Size join_size = num_cols(a_mat);

	Matrix out = zeros(rows, columns);
	for (size_t i = 0; i < rows; i++)
		for (size_t j = 0; j < columns; j++)
			for (size_t idx = 0; idx < join_size; idx++) {
				out(i, j) += a(i, idx) * b(j, idx);
			}

	return out;
#	else
	return _dot(a_mat, xt::transpose(b_mat));
#	endif
}

/**
 * Returns the matrix product of transpose ( \p a ) and \p b .
 * @tparam A An xtensor container object with 2 dimensions.
 * @tparam B An xtensor container object with 2 dimensions.
 * @tparam IfBothTensorsOfDim<> Invalidates the template for non-matching types.
 *
 * @param a The left matrix.
 * @param b The right matrix.
 *
 * @return The matrix (inner) product transpose ( \p a ) * \p b.
 */
template <typename A, typename B, IfBothTensorsOfDim<A, B, 2>* = nullptr>
Matrix a_dot_transpose_b(A&& a, B&& b) {
	Matrix a_mat = to_matrix(std::forward<A>(a));
	Matrix b_mat = to_matrix(std::forward<B>(b));

#	ifdef __aarch64__
	// Manually multiply on ARM because xt::linalg::dot seems to trigger UB (#906)
	if (has_zero_size(a_mat) || has_zero_size(b_mat)) return Matrix{};

	if (utils::ValidationContext validation{}) {
		validation.add_matrix(a_mat, "a_mat")
		    .dim('N', 'X')
		    .add_matrix(b_mat, "b_mat")
		    .dim('N', 'Y')
		    .validate();
	}

	Size rows      = num_cols(a_mat);
	Size columns   = num_cols(b_mat);
	Size join_size = num_rows(a_mat);

	Matrix out = zeros(rows, columns);
	for (size_t idx = 0; idx < join_size; idx++)
		for (size_t i = 0; i < rows; i++)
			for (size_t j = 0; j < columns; j++) {
				out(i, j) += a(idx, i) * b(idx, j);
			}

	return out;
#	else
	return _dot(xt::transpose(a_mat), b_mat);
#	endif
}
#endif

/**
 * Returns the matrix inverse of \p m.
 * @param m Input Matrix.
 *
 * @return Inverse of \p m. If matrix is not invertible, will log or throw depending on
 * navtk::ErrorMode setting, and will return a zero filled matrix the same size as \p m if
 * non-throwing.
 *
 * @throw std::runtime_error if ErrorMode::DIE and matrix is not invertible.
 */
Matrix inverse(const Matrix& m);

/**
 * Returns the norm of \p m.
 * @param m Input Matrix.
 *
 * @return Norm of \p m.
 */
double norm(const Matrix& m);

/**
 * Returns the norm of \p m.
 * @param m Input Vector.
 *
 * @return Norm of \p m.
 */
double norm(const Vector& m);

/**
 * Calculates cross product of two Vector3.
 *
 * @param m First Vector.
 * @param n Second Vector.
 *
 * @return The result of pre-multiplying the Vector \p n by the skew-symmetric matrix of \p m,
 * `dot(skew(m), n)`.
 */
Vector3 cross(const Vector3& m, const Vector3& n);

/**
 * Recursively solve a system of equations of the form Ax = b, where A is a tridiagonal matrix
 * (zeros everywhere except on diagonal, and first upper and lower diagonals).
 * See https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm.
 *
 * @param low Vector representation of lower diagonal of A, length N - 1.
 * @param mid Vector representation of diagonal of A, length N. Diagonal must be free of zeros to
 * avoid division by zero, but is not checked.
 * @param up Vector representation of upper diagonal of A, length N - 1.
 * @param b Solution vector of system to solve, length N.
 *
 * @return Vector x such that Ax = b.
 */
Vector solve_tridiagonal(const Vector& low, const Vector& mid, const Vector& up, const Vector& b);

/**
 * Recursively solve a system of equations of the form Ax = b, where A is a tridiagonal matrix
 * (zeros everywhere except on diagonal, and first upper and lower diagonals). Uses the 'overwrite'
 * algorithm; all inputs will be modified. See
 * https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm.
 *
 * @param low Vector representation of lower diagonal of A, length N - 1.
 * @param mid Vector representation of diagonal of A, length N. Diagonal must be free of zeros to
 * avoid division by zero, but is not checked.
 * @param up Vector representation of upper diagonal of A, length N - 1.
 * @param b Solution vector of system to solve, length N.
 *
 * @return Vector x such that Ax = b.
 */
Vector solve_tridiagonal_overwrite(Vector& low, Vector& mid, Vector& up, Vector& b);

/**
 * Estimate the rotation matrix that relates two fixed frames from common measurements in each of
 * the frames.
 *
 * @param outer The sum of outer products of the measurements in the platform frame and reference
 * frame, generated from 2 or more related pairs. In other words, for a 3 dimensional value \f$ m_k
 * \f$: \f$ b = \sum_{k = 0}^n m_{platform, k}m_{ref, k}^T \f$
 *
 * @return Estimate of `C_ref_to_platform`. No validation of the result is performed; testing of the
 * return value before use is recommended.
 */
Matrix3 solve_wahba_svd(const Matrix3& outer);

/**
 * Estimate the rotation matrix that relates two fixed frames from common measurements in each of
 * the frames.
 *
 * @param p A collection of platform frame observations.
 * @param r A collection of reference frame observations, the same size as \p p, where the ith
 *  observation in both \p p and \p r are of the same event. If \p p and \p r are different sizes,
 *  or either contains fewer than 2 measurements, a warning will be generated or an error thrown,
 * depending on the value of navtk::ErrorMode.
 *
 * @return Estimate of `C_ref_to_platform`. No validation of the result is performed; testing of the
 * return value before use is recommended. In the case of a warning being generated due to input
 * sizes, `navtk::zeros(3, 3)` will be returned.
 */
Matrix3 solve_wahba_svd(const std::vector<Vector3>& p, const std::vector<Vector3>& r);

/**
 * Implements the "Davenports' Q" method of determining the rotation between two fixed frames from
 * common measurements in each of the frames, as described in
 * 'Survey of Nonlinear Attitude Estimation Methods', Crassidis, Markley and Cheng.
 * http://www.acsu.buffalo.edu/~johnc/att_survey07.pdf.
 *
 * @param p A collection of platform frame observations.
 * @param r A collection of reference frame observations, the same size as \p p, where the ith
 *  observation in both \p p and \p r are of the same event. If \p p and \p r are different sizes,
 *  or either contains fewer than 2 measurements, a warning will be generated or an error thrown,
 * depending on the value of navtk::ErrorMode.
 *
 * @return A set of reference to platform DCMs \f$ C_{ref}^{platform}\f$ that minimize the loss
 * function \f$ J(C) = \frac{1}{2}\sum\limits^m_{i = 1}||p_i - C_{ref}^{platform}r_i||^2 \f$, which
 * differs slightly from the reference document in that all observations are weighted equally. If
 * the output is of size greater than 1, then a unique solution could not be determined from the
 * inputs. The solutions will be ordered from most to least likely, based upon the associated
 * eigenvalue magnitudes.  In the case of a warning being generated due to input sizes, a single
 * `navtk::zeros(3, 3)` will be returned.
 */
std::vector<Matrix3> solve_wahba_davenport(const std::vector<Vector3>& p,
                                           const std::vector<Vector3>& r);

/**
 * Implements the "Davenports' Q" method of determining the rotation between two fixed frames from
 * common measurements in each of the frames, as described in
 * 'Survey of Nonlinear Attitude Estimation Methods', Crassidis, Markley and Cheng.
 * http://www.acsu.buffalo.edu/~johnc/att_survey07.pdf.
 *
 * @param outer Given N platform \f$p\f$ and reference frame measurements \f$r\f$, the sum of the
 * possibly weighted \f$a_i\f$ outer products of the platform and reference frame observations
 * \f$ \sum\limits^N_{i=1}a_ip_ir_i^T \f$.
 * @param cr Given the same set of observations used in \p outer, the sum of the cross products
 * \f$ \sum\limits^N_{i=1}a_ip_i \times r_i \f$.
 *
 * @return A set of reference to platform DCMs \f$ C_{ref}^{platform}\f$ that minimize the loss
 * function \f$ J(C) = \frac{1}{2}\sum\limits^m_{i = 1}a_i|pb_i - C_{ref}^{platform}r_i||^2 \f$. If
 * the output is of size greater than 1, then a unique solution could not be determined from the
 * inputs. The solutions will be ordered from most to least likely, based upon the associated
 * eigenvalue magnitudes.
 */
std::vector<Matrix3> solve_wahba_davenport(const Matrix3& outer, const Vector3& cr);
}  // namespace navtk
