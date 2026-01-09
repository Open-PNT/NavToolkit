#include <navtk/filtering/fusion/strategies/EkfStrategy.hpp>

#include <memory>

#include <xtensor-blas/xlinalg.hpp>

using navtk::dot;
using navtk::utils::ValidationResult;

namespace navtk {
namespace filtering {

void EkfStrategy::propagate(const StandardDynamicsModel& dynamics_model) {
	this->symmetricize_covariance();

	this->validate_linearized_propagate(dynamics_model.Phi, dynamics_model.Qd);

	this->estimate   = dynamics_model.g(this->estimate);
	this->covariance = std::move(dot(dot(dynamics_model.Phi, std::move(this->covariance)),
	                                 xt::transpose(dynamics_model.Phi)) +
	                             dynamics_model.Qd);
}

void EkfStrategy::update(const StandardMeasurementModel& measurement_model) {
	if (ValidationResult::BAD == this->check_update_args(measurement_model)) return;

	auto h           = measurement_model.h;
	const Matrix& H  = measurement_model.H;
	const Matrix& R  = measurement_model.R;
	const Vector& z  = measurement_model.z;
	Matrix& P        = this->covariance;
	Vector& estimate = this->estimate;
	Vector hx        = h(estimate);

	if (ValidationResult::BAD == this->validate_linearized_update(H, R, z, hx)) return;

	Matrix I   = eye(num_rows(P));
	Vector res = z - hx;
	Matrix K1  = transpose_a_dot_b(P, H);
	Matrix K2  = inverse(transpose_a_dot_b(dot(H, P), H) + R);
	Matrix K   = dot(K1, K2);
	auto temp  = I - dot(K, H);
	P          = transpose_a_dot_b(dot(temp, P), temp) + transpose_a_dot_b(dot(K, R), K);
	estimate   = estimate + dot(K, res);
}

not_null<std::shared_ptr<FusionStrategy>> EkfStrategy::clone() const {
	return std::make_shared<EkfStrategy>(*this);
}

}  // namespace filtering
}  // namespace navtk
