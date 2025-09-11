#include <navtk/linear_algebra.hpp>

#include <navtk/errors.hpp>
#include <navtk/navutils/navigation.hpp>

using navtk::utils::ValidationContext;
using navtk::utils::ValidationResult;

namespace navtk {

struct ExpmHelper {
	Matrix A;
	Matrix A2;
	Matrix A4;
	Matrix A6;
	Matrix A8;
	ExpmHelper(const Matrix& a)
	    : A(a),
	      A2(matrix_power(a, 2)),
	      A4(matrix_power(A2, 2)),
	      A6(matrix_power(A2, 3)),
	      A8(matrix_power(A4, 2)) {}
	ExpmHelper(const ExpmHelper&)            = delete;
	ExpmHelper& operator=(const ExpmHelper&) = delete;
};

Matrix pade_approximation(const Matrix& U, const Matrix& V) {
	auto P = U + V;
	auto Q = xt::fma(U, -1, V);
	return xt::linalg::solve(Q, P);
}

Matrix pade_3(const ExpmHelper& expm_helper) {
	auto ident = eye(num_rows(expm_helper.A));
	auto U     = dot(expm_helper.A, xt::fma(ident, 60.0, expm_helper.A2));
	auto V     = xt::fma(expm_helper.A2, 12.0, ident * 120.0);
	return pade_approximation(U, V);
}

Matrix pade_5(const ExpmHelper& expm_helper) {
	Vector b{30240, 15120, 3360, 420, 30};
	auto ident = eye(num_rows(expm_helper.A));
	auto U =
	    dot(expm_helper.A, xt::fma(ident, b[1], xt::fma(expm_helper.A2, b[3], expm_helper.A4)));
	auto V = xt::fma(expm_helper.A4, b[4], xt::fma(expm_helper.A2, b[2], ident * b[0]));
	return pade_approximation(U, V);
}

Matrix pade_7(const ExpmHelper& expm_helper) {
	Vector b{17297280, 8648640, 1995840, 277200, 25200, 1512, 56};
	auto ident = eye(num_rows(expm_helper.A));
	auto U     = dot(expm_helper.A,
                 xt::fma(expm_helper.A4,
                         b[5],
                         xt::fma(expm_helper.A2, b[3], xt::fma(ident, b[1], expm_helper.A6))));
	auto V     = xt::fma(expm_helper.A6,
                     b[6],
                     xt::fma(expm_helper.A4, b[4], xt::fma(expm_helper.A2, b[2], ident * b[0])));
	return pade_approximation(U, V);
}

Matrix pade_9(const ExpmHelper& expm_helper) {
	Vector b{17643225600, 8821612800, 2075673600, 302702400, 30270240, 2162160, 110880, 3960, 90};
	auto ident = eye(num_rows(expm_helper.A));
	auto U =
	    dot(expm_helper.A,
	        xt::fma(expm_helper.A6,
	                b[7],
	                xt::fma(expm_helper.A4,
	                        b[5],
	                        xt::fma(expm_helper.A2, b[3], xt::fma(ident, b[1], expm_helper.A8)))));
	auto V = xt::fma(
	    expm_helper.A8,
	    b[8],
	    xt::fma(expm_helper.A6,
	            b[6],
	            xt::fma(expm_helper.A4, b[4], xt::fma(expm_helper.A2, b[2], ident * b[0]))));
	return pade_approximation(U, V);
}

Matrix pade_13(const ExpmHelper& expm_helper) {
	const auto max_norm = 5.371920351148152;
	auto n_squarings    = std::max(0.0, ceil(logb(xt::norm_l1(expm_helper.A)[0] / max_norm)));
	ExpmHelper expm_helper_scaled(std::move(expm_helper.A) * pow(2.0, -n_squarings));
	Vector b{64764752532480000,
	         32382376266240000,
	         7771770303897600,
	         1187353796428800,
	         129060195264000,
	         10559470521600,
	         670442572800,
	         33522128640,
	         1323241920,
	         40840800,
	         960960,
	         16380,
	         182};
	Matrix ident = eye(num_rows(expm_helper_scaled.A));
	Matrix U     = dot(expm_helper_scaled.A,

                   dot(expm_helper_scaled.A6,
                       xt::fma(expm_helper_scaled.A2,
                               b[9],
                               xt::fma(expm_helper_scaled.A4, b[11], expm_helper_scaled.A6))) +
                       xt::fma(expm_helper_scaled.A6,
                               b[7],
                               xt::fma(expm_helper_scaled.A4,
                                       b[5],
                                       xt::fma(expm_helper_scaled.A2, b[3], ident * b[1]))));

	Matrix V = dot(expm_helper_scaled.A6,
	               xt::fma(expm_helper_scaled.A6,
	                       b[12],
	                       xt::fma(expm_helper_scaled.A4, b[10], expm_helper_scaled.A2 * b[8]))) +
	           xt::fma(expm_helper_scaled.A6,
	                   b[6],
	                   xt::fma(expm_helper_scaled.A4,
	                           b[4],
	                           xt::fma(expm_helper_scaled.A2, b[2], ident * b[0])));
	auto R = pade_approximation(U, V);
	return matrix_power(R, pow(2, n_squarings));
}

Matrix _dot(const Matrix& m1, const Matrix& m2) {
	if (has_zero_size(m1) || has_zero_size(m2)) return Matrix{};
	return xt::linalg::dot(m1, m2);
}

Matrix inverse(const Matrix& m) {
	try {
		return xt::linalg::inv(m);
	} catch (const std::runtime_error& e) {
		log_or_throw<std::runtime_error>(e.what());
		return zeros(num_rows(m), num_cols(m));
	}
}

double norm(const Matrix& m) { return xt::linalg::norm(m, xt::linalg::normorder::frob); }

double norm(const Vector& m) { return xt::linalg::norm(m, 2); }

Vector3 cross(const Vector3& m, const Vector3& n) { return dot(navutils::skew(m), n); }

double d4(const ExpmHelper& expm_helper) {
	return std::pow(xt::amax(norm_l1(expm_helper.A4, {0}))(), 1.0 / 4.0);
}

double d6(const ExpmHelper& expm_helper) {
	return std::pow(xt::amax(norm_l1(expm_helper.A6, {0}))(), 1.0 / 6.0);
}

double d8(const ExpmHelper& expm_helper) {
	return std::pow(xt::amax(norm_l1(expm_helper.A8, {0}))(), 1.0 / 8.0);
}

// Returns the number of permutations for n choose k
unsigned int comb(unsigned int n, unsigned int k) {
	if (k == 0) {
		return 1;
	}
	return n * comb(n - 1, k - 1) / k;
}

// Returns the factorial of n (n!). Needs to be a type that can contain up to 19! (~1e17).
double factorial(unsigned int n) {
	double factorial = 1;
	for (unsigned int ii = 1; ii <= n; ++ii) {
		factorial *= ii;
	}
	return factorial;
}

int ell(const Matrix& A, int m) {
	auto choose_2m_m = comb(2 * m, m);
	auto abs_c_recip = choose_2m_m * factorial(2 * m + 1);

	// This is explained after Eq. (1.2) of the 2009 expm paper.
	// It is the "unit roundoff" of IEEE double-precision arithmetic.
	auto u = std::pow(2, -53);

	// Compute the one-norm of matrix power p of abs(A).
	auto a_abs         = xt::abs(A);
	auto v             = xt::sum(matrix_power(a_abs, 2 * m + 1), 0);
	auto a_abs_onenorm = xt::amax(v)();

	// Treat zero norm as a special case.
	if (a_abs_onenorm == 0.0) {
		return 0;
	}

	auto alpha            = a_abs_onenorm / (xt::amax(norm_l1(A, {0}))() * abs_c_recip);
	auto log2_alpha_div_u = std::log2(alpha / u);
	auto value            = int(std::ceil(log2_alpha_div_u / (2 * m)));
	return std::max(value, 0);
}

Matrix expm(const Matrix& matrix) {
	if (ValidationContext validation{}) {
		validation.add_matrix(matrix, "matrix").dim('N', 'N').validate();
	}

	// If matrix is diagonal, expm is just the exponential of the diagonal elements
	if (is_diagonal(matrix)) {
		return xt::diag(xt::exp(xt::diagonal(matrix)));
	}
	ExpmHelper expm_helper(matrix);
	auto d6_res = d6(expm_helper);

	auto eta_1 = std::max(d4(expm_helper), d6_res);
	if (eta_1 < 1.495585217958292e-2 && ell(expm_helper.A, 3) == 0) {
		return pade_3(expm_helper);
	}
	if (eta_1 < 2.539398330063230e-1 && ell(expm_helper.A, 5) == 0) {
		return pade_5(expm_helper);
	}
	auto eta_2 = std::max(d6_res, d8(expm_helper));
	if (eta_2 < 9.504178996162932e-1 && ell(expm_helper.A, 7) == 0) {
		return pade_7(expm_helper);
	}
	if (eta_2 < 2.097847961257068 && ell(expm_helper.A, 9) == 0) {
		return pade_9(expm_helper);
	}
	return pade_13(expm_helper);
}

Matrix matrix_power(const Matrix& matrix, long n) {
	// TODO: xtensor-blas has a bug with n even and greater than 3
	// Need to open issues and submit a patch, and then if/when we bump this
	// function should be able to just call directly into the xtensor version
	using xtype = xt::xtensor<double, 2>;

	// copy input matrix
	xtype mat = matrix;

	XTENSOR_ASSERT(mat.dimension() == 2);
	XTENSOR_ASSERT(mat.shape()[0] == mat.shape()[1]);

	xtype res(mat.shape());

	if (n < 0) {
		mat = xt::linalg::inv(mat);
		n   = -n;
	}

	// Okay to use here
	if (n < 3 || (n & 1)) {
		return xt::linalg::matrix_power(mat, n);
	}

	// Below is patched xtensor-blas version
	// if n > 3, do a binary decomposition (copied from NumPy)
	long bits, var = n, i = 0;
	for (bits = 0; var != 0; ++bits) {
		var >>= 1;
	}

	xtype temp = mat;
	res        = xt::eye(mat.shape()[0]);
	for (; i < bits; ++i) {
		if (i > 0) {
			xt::blas::gemm(mat, mat, temp);
			mat = temp;
		}

		if (n & (1 << i)) {
			temp = res;
			xt::blas::gemm(temp, mat, res);
		}
	}
	return res;
}

Matrix chol(const Matrix& matrix) {
	auto n = num_rows(matrix);

	ValidationContext validation;
	if (validation) {
		if (n == 0) log_or_throw<std::range_error>("Matrix passed to chol must be NxN, with N > 0");

		validation.add_matrix(matrix, "matrix").dim('N', 'N').validate();
	}
	if (n == 1 && matrix(0, 0) > 0) {
		return sqrt(matrix);
	}

	try {
		// allow xtensor to check if matrix is a positive definite matrix
		Matrix out = xt::linalg::cholesky(matrix);

		if (xt::linalg::det(out) == 0.) {
			out = sqrt_of_main_diagonal(matrix);
		}
		return out;

	} catch (const std::exception& e) {

		// try to approximate the decomposition
		return sqrt_of_main_diagonal(matrix);
	}
}

Matrix sqrt_of_main_diagonal(const Matrix& matrix) {
	auto n             = num_rows(matrix);
	Matrix chol_matrix = zeros(n, n);
	for (size_t i = 0; i < n; i++) {
		if (matrix(i, i) < 0) {
			log_or_throw<std::runtime_error>(
			    "Matrix must have positive diagonal entries for Cholesky Decomposition");
		} else {
			// return square root of diagonal entries
			chol_matrix(i, i) = sqrt(matrix(i, i));
		}
	}
	return chol_matrix;
}

Matrix calc_cov(const Matrix& matrix) {
	auto N = num_rows(matrix);
	auto K = num_cols(matrix);
	Matrix P;
	if (K > 1) {
		Vector xbar = xt::mean(matrix, {1});
		Matrix S    = ones(K, 1);

		Matrix M = xt::transpose(matrix) - dot(S, to_matrix(xbar, 0));
		P        = dot(xt::transpose(M), M);
		P *= 1.0 / ((double)(K - 1));
	} else {
		P = zeros(N, N);
	}
	return P;
}

Matrix calc_cov_weighted(const Matrix& matrix, const Vector& weights) {
	auto N           = num_rows(matrix);
	auto K           = num_cols(matrix);
	auto sum_weights = xt::sum(weights)(0);

	Matrix P;
	if (K > 1 && sum_weights > 0.) {
		Vector norm_weights = weights / sum_weights;

		Matrix weight_tiles = xt::tile(view(norm_weights, xt::newaxis(), xt::all()), N);
		Vector xbar         = sum(weight_tiles * matrix, {1});

		Matrix S = ones(K, 1);

		norm_weights *= (double)K;

		Matrix M  = xt::transpose(matrix) - dot(S, to_matrix(xbar, 0));
		Matrix wM = dot(diag(norm_weights), M);

		P = dot(xt::transpose(wM), M);

		P *= 1.0 / ((double)(K - 1));
	} else {
		P = zeros(N, N);
	}
	return P;
}

Vector solve_tridiagonal(const Vector& low, const Vector& mid, const Vector& up, const Vector& b) {
	Vector c = zeros(num_rows(low));
	Vector y = zeros(num_rows(b));
	for (Size k = 0; k < num_rows(b); k++) {
		if (k == 0) {
			c[k] = up[k] / mid[k];
			y[k] = b[k] / mid[k];
		} else if (k == num_rows(b) - 1) {
			y[k] = (b[k] - low[k - 1] * y[k - 1]) / (mid[k] - low[k - 1] * c[k - 1]);
		} else {
			c[k] = up[k] / (mid[k] - low[k - 1] * c[k - 1]);
			y[k] = (b[k] - low[k - 1] * y[k - 1]) / (mid[k] - low[k - 1] * c[k - 1]);
		}
	}

	Vector out           = zeros(num_rows(b));
	out[num_rows(b) - 1] = y[num_rows(b) - 1];
	for (Size k = num_rows(b) - 1; k-- > 0;) {
		out[k] = (y[k] - c[k] * out[k + 1]);
	}
	return out;
}

Vector solve_tridiagonal_overwrite(Vector& low, Vector& mid, Vector& up, Vector& b) {

	auto n = num_rows(b);

	for (Size k = 1; k < n; k++) {
		auto w = low[k - 1] / mid[k - 1];
		mid[k] -= w * up[k - 1];
		b[k] -= w * b[k - 1];
	}

	Vector x = zeros(n);
	x[n - 1] = b[n - 1] / mid[n - 1];
	for (Size k = n - 1; k-- > 0;) {
		x[k] = (b[k] - up[k] * x[k + 1]) / mid[k];
	}
	return x;
}

bool verify_wahba_inputs(const std::vector<Vector3>& p, const std::vector<Vector3>& r) {
	if (p.size() != r.size()) {
		log_or_throw<std::runtime_error>("Input vectors must be the same size");
		return false;
	}
	if (p.size() < 2 || r.size() < 2) {
		log_or_throw<std::runtime_error>("At least 2 vector observations are required");
		return false;
	}
	return true;
}

Matrix3 solve_wahba_svd(const Matrix3& outer) {
	auto svd = xt::linalg::svd(outer);
	auto m   = eye(3);
	m(2, 2)  = xt::linalg::det(dot(std::get<0>(svd), std::get<2>(svd)));
	return dot(dot(std::get<0>(svd), m), std::get<2>(svd));

	/* There are at least 2 methods given in various references for calculating the covariance of
	 * the associated rotation
	 * Eq 16c from "Quaternion Attitude Estimated Using Vector Observations", Markley and Mortari,
	 * https://malcolmdshuster.com/FC_MarkleyMortari_2000_J_MDSscan.pdf
	 * gives
	 * P = U K U.T
	 * where K is diag([1/(s_2 + s_3), 1/(s_3 + s_1), 1/(s_1 + s_2)]
	 * and s_1 = sigma[0], s_2 = sigma[1], s_3 = sigma[2] * det(U) * det(V)
	 * (sigma being the vector of singular values from the decomposition)
	 *
	 *
	 * The same paper quotes Shuster in eq 33 giving
	 * P = inverse[sum(a_i (I - b_i * b_i^T))
	 * where a_i are the weights associated with each measurement, and b_i are the 'true' body
	 * frame measurements, obtained by rotating the reference vectors using the true dcm Cbn * r_i.
	 * Generally we only have the estimated dcm, so we calculate using that. This is with relation
	 * to another algorithm, but the underlying 'B' matrix is the same, so it should be equally
	 * applicable.
	 *
	 * In either case, neither of these compares favorably with Monte Carlo based covariances.
	 * Just for fun, implemented approach #1 using numpy separately with same results
	 * At any rate, leaving initial attempt commented as a starting point for possible future work.
	 */
	// auto sig = std::get<1>(svd);
	// auto s1  = sig[0];
	// auto s2  = sig[1];
	// auto s3  = xt::linalg::det(std::get<0>(svd)) * xt::linalg::det(std::get<2>(svd)) * sig[2];
	// Matrix3 mid{{1.0 / (s2 + s3), 0, 0}, {0, 1.0 / (s1 + s3), 0}, {0, 0, 1.0 / (s1 + s2)}};
	// auto cov = dot(std::get<0>(svd), dot(mid, xt::transpose(std::get<0>(svd))));
}

Matrix3 solve_wahba_svd(const std::vector<Vector3>& p, const std::vector<Vector3>& r) {
	Matrix outer     = zeros(3, 3);
	auto can_proceed = verify_wahba_inputs(p, r);
	if (!can_proceed) {
		return outer;
	}
	for (Size k = 0; k < p.size(); k++) {
		outer += xt::linalg::outer(p[k], r[k]);
	}
	return solve_wahba_svd(outer);
}

std::vector<Matrix3> solve_wahba_davenport(const std::vector<Vector3>& p,
                                           const std::vector<Vector3>& r) {
	Matrix outer     = zeros(3, 3);
	auto can_proceed = verify_wahba_inputs(p, r);
	if (!can_proceed) {
		return std::vector<Matrix3>(1, outer);
	}
	Vector cr = zeros(3);
	for (Size k = 0; k < p.size(); k++) {
		outer += xt::linalg::outer(p[k], r[k]);
		cr += navtk::cross(p[k], r[k]);
	}
	return solve_wahba_davenport(outer, cr);
}

std::vector<Matrix3> solve_wahba_davenport(const Matrix3& outer, const Vector3& cr) {
	double tr       = xt::linalg::trace(to_matrix(outer))[0];
	auto ul         = outer + xt::transpose(outer) - navtk::eye(3) * tr;
	navtk::Matrix K = navtk::zeros(4, 4);
	xt::view(K, xt::range(0, 3), xt::range(0, 3)) = ul;
	xt::view(K, 3, xt::range(0, 3))               = cr;
	xt::view(K, xt::range(0, 3), 3)               = cr;
	xt::view(K, 3, 3)                             = tr;
	auto eigs                                     = xt::linalg::eig(K);
	auto eig_vals                                 = std::get<0>(eigs);
	// auto will cause view/shape failures, force into Matrix
	Matrix eig_vecs = xt::real(std::get<1>(eigs));
	// num_rows/cols doesn't support complex types
	navtk::Size nc = eig_vecs.shape()[1];
	// Reference material uses the 'real on the bottom' definition of quaternions [qx, qy, qz, q0]
	// as opposed to our 'real on top' [q0, qx, qy, qz].

	std::vector<Matrix3> out;
	double mx = 0;
	for (navtk::Size k = 0; k < nc; k++) {
		auto rl = std::real(eig_vals[k]);
		if (rl > 0) {
			Vector q                     = navtk::zeros(4);
			xt::view(q, xt::range(1, 4)) = xt::view(eig_vecs, xt::range(0, 3), k);
			q[0]                         = eig_vecs(3, k);
			// Because quaternion 'directions' are harder to document
			auto to_add = xt::transpose(navtk::navutils::quat_to_dcm(q));

			if (rl > mx) {
				out.insert(out.begin(), to_add);
				mx = rl;
			} else {
				out.push_back(to_add);
			}
		}
	}
	// 4 possible solutions, best is supposed to be indicated by largest eigenvalue. Unfortunately
	// there are occasionally ties (or very close), so we'll probably have to return a vector
	// containing possibilities we couldn't reject
	return out;
}

}  // namespace navtk
