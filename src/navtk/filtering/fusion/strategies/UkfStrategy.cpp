#include <navtk/filtering/fusion/strategies/UkfStrategy.hpp>

#include <memory>

#include <navtk/filtering/containers/LinearizedStrategyBase.hpp>

using navtk::utils::ValidationResult;
using xt::all;
using xt::range;
using xt::transpose;

namespace navtk {
namespace filtering {

void UkfStrategy::propagate(const StandardDynamicsModel &dynamics_model) {
	this->symmetricize_covariance();

	this->validate_linearized_propagate(dynamics_model.Phi, dynamics_model.Qd);

	auto num_states = get_num_states();

	int kappa = default_kappa(num_states);

	Matrix sigma_points  = mean_sigma_points(kappa, this->estimate, this->covariance);
	Vector sigma_weights = mean_weights(kappa, num_states);


	for (size_t i = 0; i < 2 * num_states + 1; i++)
		view(sigma_points, all(), i) = dynamics_model.g(view(sigma_points, all(), i));

	this->estimate = reconstruct_x_from_sigma_points(sigma_points, sigma_weights);
	this->covariance =
	    std::move(reconstruct_p_from_sigma_points(sigma_points, sigma_weights, this->estimate) +
	              dynamics_model.Qd);
}

void UkfStrategy::update(const StandardMeasurementModel &measurement_model) {
	if (ValidationResult::BAD == this->check_update_args(measurement_model)) return;
	this->symmetricize_covariance();

	auto num_states       = get_num_states();
	auto num_observations = num_rows(measurement_model.z);

	Matrix sigma_points_prop = zeros(num_observations, 2 * num_states + 1);

	Vector hx = measurement_model.h(this->estimate);

	if (ValidationResult::BAD ==
	    this->validate_linearized_update(
	        measurement_model.H, measurement_model.R, measurement_model.z, hx))
		return;

	int kappa = default_kappa(num_states);

	Matrix sigma_points = mean_sigma_points(kappa, this->estimate, this->covariance);

	Vector sigma_weights = mean_weights(kappa, num_states);

	for (size_t i = 0; i < 2 * num_states + 1; i++) {
		auto predictions                  = measurement_model.h(view(sigma_points, all(), i));
		view(sigma_points_prop, all(), i) = view(predictions, all());
	}

	Vector prediction_mean = reconstruct_x_from_sigma_points(sigma_points_prop, sigma_weights);

	Matrix Pzz =
	    reconstruct_p_from_sigma_points(sigma_points_prop, sigma_weights, prediction_mean) +
	    measurement_model.R;

	Matrix Pxz = calc_weighted_cov(
	    sigma_points, this->estimate, sigma_points_prop, prediction_mean, sigma_weights);

	// Kalman gain
	Matrix K = dot(Pxz, inverse(Pzz));

	auto offset = dot(K, (measurement_model.z - prediction_mean));
	this->estimate += offset;

	Matrix ktranspose = transpose(K);
	this->covariance -= dot(dot(K, Pzz), ktranspose);
}


not_null<std::shared_ptr<FusionStrategy>> UkfStrategy::clone() const {
	return std::make_shared<UkfStrategy>(*this);
}

int UkfStrategy::default_kappa(int tuning) {
	if (tuning != 3) {
		return 3 - tuning;
	} else {
		return 1;
	}
}

Matrix UkfStrategy::mean_sigma_points(int kappa, Vector x, Matrix P) {
	auto num_states = num_rows(x);

	Matrix sigma_points = zeros(num_states, 2 * num_states + 1);
	Matrix C;

	for (size_t i = 0; i < 2 * num_states + 1; i++) {
		view(sigma_points, all(), i) = view(x, all());
	}
	C = chol(P) * sqrt(num_states + kappa);

	view(sigma_points, all(), range(1, num_states + 1)) += C;
	view(sigma_points, all(), range(num_states + 1, 2 * num_states + 1)) += -C;

	return sigma_points;
}

double UkfStrategy::weight_off(int kappa, Size num_states) {
	return (1.0 / (2.0 * (kappa + num_states)));
}

double UkfStrategy::mean_weight0(int kappa, Size num_states) {
	return ((double)kappa / (double)(kappa + num_states));
}

Vector UkfStrategy::mean_weights(int kappa, Size num_states) {
	double weight = weight_off(kappa, num_states);
	Vector out    = zeros(2 * num_states + 1);
	for (size_t i = 1; i < 2 * num_states + 1; i++) {
		out(i) = weight;
	}
	out(0) = mean_weight0(kappa, num_states);
	return out;
}

Vector UkfStrategy::reconstruct_x_from_sigma_points(Matrix sigma_points, Vector sigma_weights) {
	auto M = num_cols(sigma_points);
	auto N = num_rows(sigma_points);

	Vector A = zeros(N);
	for (size_t i = 0; i < N; i++) {
		for (size_t j = 0; j < M; j++) {
			A(i) += sigma_points(i, j) * sigma_weights(j);
		}
	}
	return A;
}

Matrix UkfStrategy::reconstruct_p_from_sigma_points(Matrix sigma_points,
                                                    Vector sigma_weights,
                                                    Vector new_mean) {
	auto M     = num_cols(sigma_points);
	auto N     = num_rows(new_mean);
	Matrix out = zeros(N, N);
	for (size_t i = 0; i < N; i++) {
		for (size_t j = 0; j < M; j++) {
			for (size_t k = 0; k < N; k++) {
				out(i, k) += sigma_weights(j) * (sigma_points(i, j) - new_mean(i)) *
				             (sigma_points(k, j) - new_mean(k));
			}
		}
	}
	return out;
}

Matrix UkfStrategy::calc_weighted_cov(
    Matrix pred1, Vector mean1, Matrix pred2, Vector mean2, Vector weights) {
	auto N     = num_rows(mean1);
	auto K     = num_rows(mean2);
	auto M     = num_rows(weights);
	Matrix out = zeros(N, K);
	for (size_t i = 0; i < N; i++) {
		for (size_t p = 0; p < K; p++) {
			for (size_t j = 0; j < M; j++) {
				out(i, p) += weights(j) * (pred1(i, j) - mean1(i)) * (pred2(p, j) - mean2(p));
			}
		}
	}
	return out;
}

}  // namespace filtering
}  // namespace navtk
