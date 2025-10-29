#include <navtk/filtering/utils.hpp>

#include <xtensor/generators/xrandom.hpp>
#include <xtensor/views/xview.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/ValidationContext.hpp>

using navtk::navutils::dcm_to_rpy;
using navtk::navutils::rpy_to_dcm;
using navtk::utils::ValidationContext;
using xt::all;
using xt::keep;
using xt::range;
using xt::transpose;
using xt::view;

namespace navtk {
namespace filtering {

/**
 * Generates a vector of perturbations to use for each state
 * when calculating numerical derivatives when using the lazy
 * 'single eps' methods. Finer grain control is available in the
 * Vector eps overloads of each differentiation function.
 *
 * @param x Nominal state vector to be perturbed.
 * @param eps Nominal amount to perturb by. If 0 for some reason,
 * will be set to 0.001 instead.
 *
 * @return A Vector the same size as x where each element is
 * the perturbation for the corresponding state, either eps * state
 * when state is non-zero, or simply eps if the state is 0 (i.e. eps is
 * interpreted as a scale factor or additive value as required to
 * avoid divide by zeros).
 */
Vector gen_perturbation_vector(const Vector& x, Scalar eps) {
	if (eps == 0.0) {
		// TODO: PNTOS-51 Log a warning here.
		eps = 0.001;
	}
	Vector eps_vec = x * eps;
	for (Size i = 0; i < num_rows(x); i++) {
		if (eps_vec[i] == 0) eps_vec[i] = eps;
	}
	return eps_vec;
}

Vector partial_derivative(const std::function<Vector(const Vector&)>& f,
                          const Vector& x,
                          Scalar eps,
                          int ind) {
	Vector dxx = zeros(num_rows(x));
	dxx[ind]   = eps;
	Scalar two(2);
	Vector a = f(x + dxx);
	Vector b = f(x - dxx);
	double c = (two * eps);

	return (a - b) / c;
}


Matrix calc_numerical_jacobian(const std::function<Vector(const Vector&)>& f,
                               const Vector& x,
                               const Vector& eps) {

	if (ValidationContext validation{}) {
		validation.add_matrix(x, "x").dim('N', 1).add_matrix(eps, "eps").dim('N', 1).validate();
	}
	auto cols    = num_rows(x);
	auto partial = partial_derivative(f, x, eps[0], 0);
	auto rows    = num_rows(partial);
	Matrix out   = zeros(rows, cols);

	view(out, all(), 0) = partial;
	for (Size ind = 1; ind < cols; ++ind)
		view(out, all(), ind) = partial_derivative(f, x, eps[ind], ind);
	return out;
}

Vector partial_derivative_rpy(const std::function<Vector(const Vector&)>& f,
                              const Vector& x,
                              Scalar eps,
                              int ind) {
	Vector dxx = zeros(num_rows(x));
	dxx[ind]   = eps;
	Scalar two(2);
	auto cns_nom = rpy_to_dcm(f(x));
	auto csn_p   = xt::transpose(rpy_to_dcm(f(x + dxx)));
	auto csn_m   = xt::transpose(rpy_to_dcm(f(x - dxx)));
	auto tilt_p  = dcm_to_rpy(xt::transpose(dot(cns_nom, csn_p)));
	auto tilt_m  = dcm_to_rpy(xt::transpose(dot(cns_nom, csn_m)));
	return (tilt_p - tilt_m) / (two * eps);
}

Matrix calc_numerical_jacobian_rpy(const std::function<Vector(const Vector&)>& f,
                                   const Vector& x,
                                   const Vector& eps) {
	if (ValidationContext validation{}) {
		validation.add_matrix(x, "x").dim('N', 1).add_matrix(eps, "eps").dim('N', 1).validate();
	}
	auto cols  = num_rows(x);
	Matrix out = zeros(3, cols);
	for (Size ind = 0; ind < cols; ++ind)
		view(out, all(), ind) = partial_derivative_rpy(f, x, eps[ind], ind);
	return out;
}

Matrix calc_numerical_jacobian_rpy(const std::function<Vector(const Vector&)>& f,
                                   const Vector& x,
                                   Scalar eps) {

	auto eps_vec = gen_perturbation_vector(x, eps);
	return calc_numerical_jacobian_rpy(f, x, eps_vec);
}

Matrix calc_numerical_jacobian(const std::function<Vector(const Vector&)>& f,
                               const Vector& x,
                               Scalar eps) {

	auto eps_vec = gen_perturbation_vector(x, eps);
	return calc_numerical_jacobian(f, x, eps_vec);
}

std::vector<Matrix> calc_numerical_hessians(const std::function<Vector(const Vector&)>& f,
                                            const Vector& x,
                                            const Vector& eps) {
	auto rows         = num_rows(x);
	Vector dummy      = f(x);
	auto num_hessians = num_rows(dummy);
	std::vector<Matrix> out_vec(num_hessians, zeros(rows, rows));

	for (Size j = 0; j < rows; j++) {
		for (Size k = j; k < rows; k++) {
			Vector dx0 = x;
			dx0[j] += eps[j];
			dx0[k] += eps[k];
			Vector dx1 = x;
			dx1[j] += eps[j];
			dx1[k] -= eps[k];
			Vector dx2 = x;
			dx2[j] -= eps[j];
			dx2[k] += eps[k];
			Vector dx3 = x;
			dx3[j] -= eps[j];
			dx3[k] -= eps[k];
			Vector ele = (f(dx0) - f(dx1) - f(dx2) + f(dx3)) / (4.0 * eps[j] * eps[k]);
			for (Size p = 0; p < num_hessians; p++) {
				out_vec[p](j, k) = ele[p];
				if (k != j) out_vec[p](k, j) = ele[p];
			}
		}
	}
	return out_vec;
}

std::vector<Matrix> calc_numerical_hessians(const std::function<Vector(const Vector&)>& f,
                                            const Vector& x,
                                            Scalar eps) {

	auto eps_vec = gen_perturbation_vector(x, eps);
	return calc_numerical_hessians(f, x, eps_vec);
}

EstimateWithCovariance calc_mean_cov(const Matrix& samples) {
	auto num_samples = num_cols(samples);
	auto num_states  = num_rows(samples);

	auto mn        = to_matrix(xt::mean(samples, {1}));
	Matrix cov_sum = zeros(num_states, num_states);
	for (Size k = 0; k < num_samples; k++) {
		auto df = view(samples, all(), keep(k)) - mn;
		cov_sum += dot(df, transpose(df));
	}
	Matrix cov = cov_sum / (num_samples - 1);
	return EstimateWithCovariance{to_vec(mn), cov};
}

EstimateWithCovariance calc_mean_cov_rpy(const Matrix& samples) {
	// Get a 3xn Matrix containing rpy
	std::vector<Matrix3> dcms;
	auto dcm_mean = zeros(3, 3);
	for (Size k = 0; k < num_cols(samples); k++) {
		dcms.push_back(xt::transpose(navutils::rpy_to_dcm(xt::view(samples, xt::all(), k))));
		dcm_mean = dcm_mean + dcms.back();
	}
	// orthogonalize?
	dcm_mean       = dcm_mean / dcms.size();
	Matrix cov_sum = zeros(3, 3);
	auto dcm_t     = xt::transpose(dcm_mean);
	for (Size k = 0; k < num_cols(samples); k++) {
		Vector df = navutils::dcm_to_rpy(xt::transpose(dot(dcm_t, dcms[k])));
		cov_sum += xt::linalg::outer(df, transpose(df));
	}
	Matrix cov = cov_sum / (dcms.size() - 1);
	return EstimateWithCovariance{navutils::dcm_to_rpy(xt::transpose(dcm_mean)), cov};
}

EstimateWithCovariance monte_carlo_approx(const EstimateWithCovariance& ec,
                                          const std::function<Vector(const Vector&)>& fx,
                                          Size num_samples) {
	auto nr       = num_rows(ec.estimate);
	auto c        = chol(ec.covariance);
	auto alt_rand = xt::random::randn<double>({nr, num_samples});

	Vector samp             = to_vec(dot(c, view(alt_rand, all(), keep(0)))) + ec.estimate;
	auto init_res           = fx(samp);
	Matrix txd              = zeros(num_rows(init_res), num_samples);
	view(txd, xt::all(), 0) = init_res;

	for (Size k = 1; k < num_samples; k++) {
		samp                    = to_vec(dot(c, view(alt_rand, all(), keep(k)))) + ec.estimate;
		view(txd, xt::all(), k) = fx(samp);
	}
	return calc_mean_cov(txd);
}

EstimateWithCovariance monte_carlo_approx_rpy(const EstimateWithCovariance& ec,
                                              const std::function<Vector(const Vector&)>& fx,
                                              Size num_samples) {
	auto nr       = num_rows(ec.estimate);
	auto c        = chol(ec.covariance);
	auto alt_rand = xt::random::randn<double>({nr, num_samples});

	Vector samp             = to_vec(dot(c, view(alt_rand, all(), keep(0)))) + ec.estimate;
	auto init_res           = fx(samp);
	Matrix txd              = zeros(num_rows(init_res), num_samples);
	view(txd, xt::all(), 0) = init_res;

	for (Size k = 1; k < num_samples; k++) {
		samp                    = to_vec(dot(c, view(alt_rand, all(), keep(k)))) + ec.estimate;
		view(txd, xt::all(), k) = fx(samp);
	}
	return calc_mean_cov_rpy(txd);
}


EstimateWithCovariance first_order_approx(const EstimateWithCovariance& ec,
                                          std::function<Vector(const Vector&)>& fx,
                                          std::function<Matrix(const Vector&)> jx) {
	Matrix jac;
	if (jx != 0)
		jac = jx(ec.estimate);
	else
		jac = calc_numerical_jacobian(fx, ec.estimate);

	Vector outX = fx(ec.estimate);
	Matrix outP = dot(jac, dot(ec.covariance, transpose(jac)));
	return EstimateWithCovariance{outX, outP};
}

EstimateWithCovariance first_order_approx_rpy(const EstimateWithCovariance& ec,
                                              std::function<Vector(const Vector&)>& fx) {

	Matrix jac  = calc_numerical_jacobian_rpy(fx, ec.estimate, 0.01);
	Vector outX = fx(ec.estimate);
	Matrix outP = dot(jac, dot(ec.covariance, transpose(jac)));
	return EstimateWithCovariance{outX, outP};
}


EstimateWithCovariance second_order_approx(const EstimateWithCovariance& ec,
                                           std::function<Vector(const Vector&)>& fx,
                                           std::function<Matrix(const Vector&)> jx,
                                           std::function<std::vector<Matrix>(const Vector&)> hx) {

	std::vector<Matrix> hessians;

	if (hx != 0)
		hessians = hx(ec.estimate);
	else
		hessians = calc_numerical_hessians(fx, ec.estimate);

	auto app1   = first_order_approx(ec, fx, jx);
	Vector outX = app1.estimate;
	Matrix outP = app1.covariance;

	for (Size k = 0; k < num_rows(outX); k++) {
		view(outX, keep(k)) += xt::linalg::trace(dot(hessians[k], ec.covariance));

		// Valid due to cyclic property of trace- tr(ABCD) = tr(BCDA) = tr(CDAB)
		for (Size j = k; j < num_rows(outX); j++) {
			view(outP, keep(k), keep(j)) +=
			    0.5 * xt::linalg::trace(
			              dot(ec.covariance, dot(hessians[k], dot(ec.covariance, hessians[j]))));
			if (j != k) view(outP, keep(j), keep(k)) = view(outP, keep(k), keep(j));
		}
	}

	return EstimateWithCovariance{outX, outP};
}
}  // namespace filtering
}  // namespace navtk
