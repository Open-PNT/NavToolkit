#include <navtk/filtering/experimental/fusion/strategies/RbpfStrategy.hpp>

#include <cmath>
#include <iterator>
#include <vector>

#include <xtensor/core/xmath.hpp>
#include <xtensor/misc/xpad.hpp>
#include <xtensor/views/xindex_view.hpp>
#include <xtensor/views/xstrided_view.hpp>
#include <xtensor/views/xview.hpp>

#include <navtk/experimental/random.hpp>
#include <navtk/tensors.hpp>

using navtk::chol;
using navtk::experimental::rand_n;
using navtk::filtering::EstimateWithCovariance;
using navtk::utils::ValidationResult;
using xt::all;
using xt::diagonal;
using xt::keep;
using xt::newaxis;
using xt::square;
using xt::sum;
using xt::tile;
using xt::transpose;
using xt::xtensor;
using xt::linalg::det;

namespace navtk {
namespace filtering {
namespace experimental {

RbpfStrategy::RbpfStrategy(Size num_particles,
                           double resampling_threshold,
                           bool calc_single_jacobian,
                           ResamplingFunction resamp_fun)
    : StandardModelStrategy(),
      RbpfModel(num_particles, calc_single_jacobian),
      resampling_threshold(resampling_threshold),
      _resampling_fun(std::move(resamp_fun)) {}


void RbpfStrategy::propagate(const StandardDynamicsModel &dynamics_model) {
	if (ValidationResult::BAD ==
	    this->validate_linearized_propagate(dynamics_model.Phi, dynamics_model.Qd))
		return;

	this->symmetricize_covariance();

	if (det(dynamics_model.Qd) == 0.) {
		propagate_bank_ekf(dynamics_model);
	} else {
		if (!any_nonlinear()) {
			propagate_single_ekf(dynamics_model);
		} else {
			Matrix chol_qd = chol(dynamics_model.Qd);
			if (!any_linear()) {
				propagate_particle_filter(dynamics_model, chol_qd);
			} else {
				propagate_rbpf(dynamics_model, chol_qd);
			}
		}
	}
	this->covariance_stale = true;
}  // end propagate

void RbpfStrategy::update(const StandardMeasurementModel &measurement_model) {
	if (ValidationResult::BAD == this->check_update_args(measurement_model)) return;

	this->symmetricize_covariance();

	if (!any_nonlinear()) {
		update_as_single_ekf(measurement_model);
	} else {
		if (det(measurement_model.R) == 0.) {
			update_as_bank_ekf(measurement_model);
		} else {
			update_with_particle_method(measurement_model);
		}
	}
	this->covariance_stale = true;

}  // end update

void RbpfStrategy::update_as_single_ekf(const StandardMeasurementModel &measurement_model) {

	// All linear states so treat as EKF
	Matrix cov_mat = view(this->covariance_particles, 0, all(), all());

	EstimateWithCovariance ecov =
	    ekf_update(view(this->state_particles, all(), 0), measurement_model, cov_mat);

	view(this->state_particles, all(), 0)             = ecov.estimate;
	view(this->covariance_particles, 0, all(), all()) = ecov.covariance;

	this->estimate   = std::move(ecov.estimate);
	this->covariance = std::move(ecov.covariance);
	this->symmetricize_covariance();
}

void RbpfStrategy::update_as_bank_ekf(const StandardMeasurementModel &measurement_model) {

	// process each particle state as an EKF
	auto particle_count = count_particles();

	Matrix cov_mat = this->covariance;
	if (this->calc_single_jacobian && this->any_linear()) {
		view(cov_mat, keep_linear, keep_linear) = view(this->covariance_particles, 0, all(), all());
	}
	// TODO (PNTOS-596): doesn't handle the case of a measurement matrix of zeros.
	Matrix m_inv = inverse(measurement_model.R);

	for (decltype(particle_count) i = 0; i < particle_count; i++) {
		if (!this->calc_single_jacobian && this->any_linear()) {
			view(cov_mat, keep_linear, keep_linear) =
			    view(this->covariance_particles, i, all(), all());
		}
		EstimateWithCovariance ecov =
		    ekf_update(view(this->state_particles, all(), i), measurement_model, cov_mat);
		view(this->state_particles, all(), i) = ecov.estimate;

		if (!this->calc_single_jacobian && this->any_linear()) {
			view(this->covariance_particles, i, all(), all()) =
			    view(ecov.covariance, keep_linear, keep_linear);
		}
		Vector res = measurement_model.z - measurement_model.h(ecov.estimate);

		Matrix errM                     = dot(dot(to_matrix(res, 0), m_inv), to_matrix(res, 1));
		this->state_particle_weights(i) = std::exp(-0.5 * errM(0, 0));
	}
	this->estimate = calc_weighted_estimate();

	if (this->calc_single_jacobian && this->any_linear()) {
		EstimateWithCovariance ecov = ekf_update(this->estimate, measurement_model, cov_mat);
		view(this->covariance_particles, 0, all(), all()) =
		    view(ecov.covariance, keep_linear, keep_linear);
	}
}

void RbpfStrategy::update_with_particle_method(const StandardMeasurementModel &measurement_model) {
	auto particle_count       = count_particles();
	double residual_threshold = 1e15;
	double min_res = residual_threshold;  // holds the minimum residual value - a metric for how far
	                                      // the measurement is from the expectation.

	Vector recovery_states = calc_weighted_estimate();

	Vector weights;
	double number_effective_particles = 0.;

	Matrix residual = zeros(num_rows(measurement_model.z), particle_count);

	Matrix C_linear = any_linear() ? view(measurement_model.H, all(), keep_linear) : Matrix{};
	Matrix C_linear_transpose = any_linear() ? transpose(C_linear) : Matrix{};
	Matrix m_inv;
	if (!any_linear() || this->calc_single_jacobian) {
		m_inv = inverse(measurement_model.R);
	}
	Vector res_linear, res;
	if (any_linear()) {
		res_linear = to_vec(dot(C_linear, to_matrix(view(this->estimate, keep_linear), 1)));
	}
	for (decltype(particle_count) i = 0; i < particle_count; i++) {
		if (!this->calc_single_jacobian && any_linear()) {
			Matrix cov_mat = view(this->covariance_particles, i, all(), all());

			Matrix M = dot(dot(C_linear, cov_mat), C_linear_transpose) + measurement_model.R;
			m_inv    = inverse(M);
		}
		if (any_linear()) {
			Vector h_nonlinear =
			    measurement_model.h(view(this->state_particles, all(), i)) -
			    to_vec(dot(C_linear, to_matrix(view(this->state_particles, keep_linear, i), 1)));
			res = measurement_model.z - h_nonlinear - res_linear;
		} else {
			res = measurement_model.z - measurement_model.h(view(this->state_particles, all(), i));
		}
		view(residual, all(), i) = res;

		Matrix errM = dot(dot(to_matrix(res, 0), m_inv), to_matrix(res, 1));
		if (errM(0, 0) < min_res) {
			min_res = errM(0, 0);
		}
		this->state_particle_weights(i) *= std::exp(-0.5 * errM(0, 0));
	}

	auto sum_wt = sum(this->state_particle_weights)(0);
	if (sum_wt > 1e-120) {
		this->state_particle_weights /= sum_wt;
	} else {
		if (min_res < residual_threshold) {
			// use EKF bank to update since not bad enough to reset
			update_as_bank_ekf(measurement_model);
		} else {
			if (this->covariance_stale) {
				this->get_covariance();
			}
			reset_particles(recovery_states);
		}
		this->state_particle_weights = fill(1.0 / particle_count, particle_count);
		return;
	}
	number_effective_particles =
	    1 / (static_cast<double>(particle_count) * sum(square(this->state_particle_weights), 0)[0]);
	if (number_effective_particles <= this->resampling_threshold) {
		// resample scheme
		weights                   = this->state_particle_weights;
		ResamplingResult resamp   = _resampling_fun(weights, nullptr);
		std::vector<size_t> index = resamp.index;

		copy_particles_by(index);
		this->state_particle_weights = fill(1.0 / particle_count, particle_count);
	}

	if (any_linear()) {
		Matrix gain;

		Matrix new_covariance;
		if (this->calc_single_jacobian) {
			Matrix cov_linear = view(this->covariance_particles, 0, all(), all());
			Matrix M     = dot(dot(C_linear, cov_linear), C_linear_transpose) + measurement_model.R;
			Matrix m_inv = inverse(M);

			gain = dot(dot(cov_linear, C_linear_transpose), m_inv);

			auto n         = num_rows(gain);
			Matrix temp    = eye(n) - dot(gain, C_linear);
			new_covariance = dot(dot(temp, cov_linear), transpose(temp)) +
			                 dot(dot(gain, measurement_model.R), transpose(gain));

			new_covariance = symmetricize_covariance(new_covariance);
		}

		Vector x_contribution =
		    measurement_model.h(this->estimate) -
		    to_vec(dot(C_linear, to_matrix(view(this->estimate, keep_linear), 1)));

		for (decltype(particle_count) i = 0; i < particle_count; i++) {
			if (!this->calc_single_jacobian) {
				Matrix covariance_particle = view(this->covariance_particles, i, all(), all());

				Matrix M = dot(dot(C_linear, covariance_particle), C_linear_transpose) +
				           measurement_model.R;
				Matrix m_inv = inverse(M);

				gain = dot(dot(covariance_particle, C_linear_transpose), m_inv);

				auto n           = num_rows(gain);
				Matrix temp      = eye(n) - dot(gain, C_linear);
				Matrix temp_full = dot(dot(temp, covariance_particle), transpose(temp)) +
				                   dot(dot(gain, measurement_model.R), transpose(gain));

				view(this->covariance_particles, i, all(), all()) =
				    symmetricize_covariance(temp_full);
			}
			Vector x_nl_contribution =
			    to_vec(dot(C_linear, view(this->state_particles, keep_linear, i)));
			Vector correction = to_vec(
			    dot(gain, to_matrix(measurement_model.z - x_nl_contribution - x_contribution)));

			view(this->state_particles, keep_linear, i) += correction;
		}
		if (this->calc_single_jacobian)
			view(this->covariance_particles, 0, all(), all()) = new_covariance;
	}

	this->estimate         = calc_weighted_estimate();
	this->covariance_stale = true;

}  // end update_with_particle_method

void RbpfStrategy::propagate_particle_filter(const StandardDynamicsModel &dynamics_model,
                                             const Matrix &chol_qd) {

	auto particle_count = count_particles();

	Matrix rng = rand_n(num_rows(chol_qd), particle_count);
	for (decltype(particle_count) i = 0; i < particle_count; i++) {
		Vector prior_state = view(this->state_particles, all(), i);
		Vector add_noise   = to_vec(dot(chol_qd, to_matrix(view(rng, all(), i), 1)));
		Vector new_state   = dynamics_model.g(prior_state) + add_noise;

		view(this->state_particles, all(), i) = new_state;
	}
	this->estimate         = calc_weighted_estimate();
	this->covariance_stale = true;
}  // end propagate_particle_filter

void RbpfStrategy::propagate_bank_ekf(const StandardDynamicsModel &dynamics_model) {
	// process each particle state as an EKF
	auto particle_count = count_particles();

	Matrix p_part = this->covariance;
	if (this->calc_single_jacobian && this->any_linear()) {
		view(p_part, keep_linear, keep_linear) = view(this->covariance_particles, 0, all(), all());
	}
	for (decltype(particle_count) i = 0; i < particle_count; i++) {
		if (!this->calc_single_jacobian && this->any_linear()) {
			view(p_part, keep_linear, keep_linear) =
			    view(this->covariance_particles, i, all(), all());
		}

		Vector prior_state = view(this->state_particles, all(), i);

		EstimateWithCovariance ecov = ekf_propagate(prior_state, dynamics_model, p_part);

		if (!this->calc_single_jacobian && this->any_linear()) {
			view(this->covariance_particles, i, all(), all()) =
			    view(ecov.covariance, keep_linear, keep_linear);
		}
		view(this->state_particles, all(), i) = ecov.estimate;
	}
	this->estimate = calc_weighted_estimate();

	if (this->calc_single_jacobian && this->any_linear()) {
		EstimateWithCovariance ecov = ekf_propagate(this->estimate, dynamics_model, p_part);
		view(this->covariance_particles, 0, all(), all()) =
		    view(ecov.covariance, keep_linear, keep_linear);
	}
}  // end propagate_bank_ekf

void RbpfStrategy::propagate_rbpf(const StandardDynamicsModel &dynamics_model,
                                  const Matrix &chol_qd) {

	auto particle_count = count_particles();

	Matrix p_val_l, k_mat, p_part;

	Matrix a_full_transpose = transpose(dynamics_model.Phi);
	Matrix phi_n            = view(dynamics_model.Phi, drop_linear, keep_linear);
	Matrix phi_l            = view(dynamics_model.Phi, keep_linear, keep_linear);
	Matrix phi_l_t          = transpose(phi_l);
	Matrix phi_n_tr         = transpose(phi_n);
	Matrix qd_l_n           = view(dynamics_model.Qd, keep_linear, drop_linear);
	Matrix qd_n_l           = view(dynamics_model.Qd, drop_linear, keep_linear);
	Matrix qd_n_n           = view(dynamics_model.Qd, drop_linear, drop_linear);
	Matrix qd_n_n_i         = inverse(qd_n_n);
	Matrix qd_l_n_n_i       = dot(qd_l_n, qd_n_n_i);
	Matrix a_bar            = phi_l - dot(qd_l_n_n_i, phi_n);
	Matrix q_bar = view(dynamics_model.Qd, keep_linear, keep_linear) - dot(qd_l_n_n_i, qd_n_l);

	Matrix chol_qd_nn = view(chol_qd, drop_linear, drop_linear);

	if (this->calc_single_jacobian) {
		p_val_l      = view(this->covariance_particles, 0, all(), all());
		Matrix n_arr = dot(dot(phi_n, p_val_l), phi_n_tr) + qd_n_n;
		Matrix n_mat = inverse(n_arr);
		k_mat        = dot(dot(dot(a_bar, p_val_l), phi_n_tr), n_mat);
	}
	Matrix rng_output = rand_n(nonlinear_states.size(), particle_count);

	for (decltype(particle_count) i = 0; i < particle_count; i++) {
		if (!this->calc_single_jacobian) {
			p_val_l      = view(this->covariance_particles, i, all(), all());
			Matrix n_arr = dot(dot(phi_n, p_val_l), phi_n_tr) + qd_n_n;
			Matrix n_mat = inverse(n_arr);
			k_mat        = dot(dot(dot(a_bar, p_val_l), phi_n_tr), n_mat);
		}
		Vector prior_state  = view(this->state_particles, all(), i);
		Matrix prior_linear = to_matrix(view(prior_state, keep_linear), 1);
		Vector new_state    = dynamics_model.g(prior_state);
		Matrix add_noise    = dot(chol_qd_nn, to_matrix(view(rng_output, all(), i), 1));
		view(new_state, drop_linear) += to_vec(add_noise);

		Matrix z     = add_noise + dot(phi_n, prior_linear);
		Vector k_z   = to_vec(dot(k_mat, add_noise));
		Vector q_q_z = to_vec(dot(qd_l_n_n_i, z));

		Vector f_l      = view(new_state, keep_linear) - to_vec(dot(phi_l, prior_linear));
		Vector a_linear = to_vec(dot(a_bar, prior_linear));

		view(new_state, keep_linear)          = a_linear + k_z + q_q_z + f_l;
		view(this->state_particles, all(), i) = new_state;

		if (!this->calc_single_jacobian) {
			Matrix p_part = dot(dot(phi_l, p_val_l), phi_l_t) + q_bar;
			view(this->covariance_particles, i, all(), all()) = symmetricize_covariance(p_part);
		}
	}

	if (this->calc_single_jacobian) {
		p_part = dot(dot(phi_l, p_val_l), phi_l_t) + q_bar;
		view(this->covariance_particles, 0, all(), all()) = symmetricize_covariance(p_part);
	}
	this->estimate = calc_weighted_estimate();
}  // end propagate_rbpf

void RbpfStrategy::propagate_single_ekf(const StandardDynamicsModel &dynamics_model) {
	// EKF propagate
	Matrix p_part   = view(this->covariance_particles, 0, all(), all());
	Vector estimate = view(this->state_particles, all(), 0);

	EstimateWithCovariance e_cov = ekf_propagate(estimate, dynamics_model, p_part);
	view(this->covariance_particles, 0, all(), all()) = e_cov.covariance;
	view(this->state_particles, all(), 0)             = e_cov.estimate;

	this->estimate   = std::move(e_cov.estimate);
	this->covariance = std::move(e_cov.covariance);
}

EstimateWithCovariance RbpfStrategy::ekf_propagate(const Vector &x,
                                                   const StandardDynamicsModel &dynamics_model,
                                                   const Matrix &cov_mat) {

	Vector estimate = dynamics_model.g(x);

	Matrix cov =
	    dot(dot(dynamics_model.Phi, cov_mat), transpose(dynamics_model.Phi)) + dynamics_model.Qd;
	return EstimateWithCovariance{estimate, cov};
}

EstimateWithCovariance RbpfStrategy::ekf_update(const Vector &x,
                                                const StandardMeasurementModel &measurement_model,
                                                const Matrix &cov_mat) {
	auto num_states    = num_rows(x);
	Matrix h_transpose = transpose(measurement_model.H);

	Matrix I   = eye(num_states);
	Vector res = measurement_model.z - measurement_model.h(x);
	Matrix K1  = dot(cov_mat, h_transpose);
	Matrix K2  = inverse(dot(dot(measurement_model.H, cov_mat), h_transpose) + measurement_model.R);

	Matrix K    = dot(K1, K2);
	Matrix temp = I - dot(K, measurement_model.H);
	Matrix p_new =
	    dot(dot(temp, cov_mat), transpose(temp)) + dot(dot(K, measurement_model.R), transpose(K));
	Matrix cov = symmetricize_covariance(p_new);

	Vector estimate = x + dot(K, res);
	return EstimateWithCovariance{estimate, cov};
}

Vector RbpfStrategy::calc_weighted_estimate() {
	Vector new_estimate;

	auto sum_wt = sum(this->state_particle_weights)(0);
	if (sum_wt > 0.) {
		this->state_particle_weights /= sum_wt;
		Matrix weight_tiles =
		    tile(view(this->state_particle_weights, newaxis(), all()), this->get_num_states());
		new_estimate = sum(weight_tiles * this->state_particles, {1});
	} else {
		new_estimate = mean(this->state_particles, {1});
	}
	return new_estimate;
}

void RbpfStrategy::reset_particles(Vector const &recovery_states) {
	auto particle_count = count_particles();

	this->estimate = recovery_states;
	Matrix chol_m  = chol(symmetricize_covariance(this->covariance));

	Matrix rng = rand_n(num_rows(chol_m), particle_count);

	for (Size ii = 0; ii < particle_count; ii++) {
		view(state_particles, all(), ii) =
		    recovery_states + to_vec(dot(chol_m, to_matrix(view(rng, all(), ii), 1)));
	}
}

std::pair<bool, Vector> RbpfStrategy::jitter_contribution() {
	auto marked_states = get_marked_states();
	auto jitter_amount = get_jitter_scaling();
	auto num_jitter    = marked_states.size();

	Vector scale = zeros(num_jitter);

	bool jitter = false;
	for (decltype(num_jitter) i = 0; i < num_jitter; i++) {
		size_t ind       = marked_states[i];
		double cov_entry = sqrt(std::fabs(this->covariance(ind, ind)));
		scale[i]         = static_cast<double>(jitter_amount[ind]) * cov_entry;
		if (std::fabs(scale[i]) > 0.) jitter = true;
	}

	return {jitter, scale};
}

void RbpfStrategy::copy_particles_by(const std::vector<size_t> &index) {

	Matrix state_particles_copy                  = this->state_particles;
	xtensor<double, 3> covariance_particles_copy = this->covariance_particles;

	auto marked_states  = get_marked_states();
	auto num_jitter     = marked_states.size();
	auto particle_count = count_particles();

	bool jitter;
	Vector scale, res_vec;

	std::tie(jitter, scale) = jitter_contribution();

	Matrix noise;
	if (jitter) {
		noise = rand_n(num_jitter, particle_count);
	}

	for (decltype(particle_count) i = 0; i < particle_count; i++) {
		Vector state = view(state_particles_copy, all(), index[i]);
		if (jitter) {
			view(state, keep(marked_states)) += scale * view(noise, all(), i);
		}
		view(this->state_particles, all(), i) = state;

		if (!this->calc_single_jacobian) {
			view(this->covariance_particles, i, all(), all()) =
			    view(covariance_particles_copy, index[i], all(), all());
		}
	}
}

std::vector<bool> RbpfStrategy::get_particle_state_marks() const { return this->is_nonlinear; }

Matrix RbpfStrategy::get_state_particles() const { return this->state_particles; }

Tensor<3> RbpfStrategy::get_state_particles_cov() const { return this->covariance_particles; }

Vector RbpfStrategy::get_state_particles_weights() const { return this->state_particle_weights; }

not_null<std::shared_ptr<FusionStrategy>> RbpfStrategy::clone() const {
	return std::make_shared<RbpfStrategy>(*this);
}

}  // namespace experimental
}  // namespace filtering
}  // namespace navtk
