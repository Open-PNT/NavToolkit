#include <navtk/filtering/fusion/strategies/StandardModelStrategy.hpp>

#include <navtk/inspect.hpp>
#include <navtk/utils/ValidationContext.hpp>

using navtk::utils::ValidationContext;
using navtk::utils::ValidationResult;

namespace navtk {
namespace filtering {

ValidationResult StandardModelStrategy::check_update_args(
    const StandardMeasurementModel& measurement_model) {
	if (navtk::get_global_error_mode() == navtk::ErrorMode::OFF)
		return ValidationResult::NOT_CHECKED;
	return ValidationContext{}
	    .add_matrix(measurement_model.H, "measurement_model.H jacobian")
	    .dim('N', get_num_states())
	    .validate();
}

ValidationResult StandardModelStrategy::validate_linearized_propagate(
    const Matrix& dynamics_jacobian, const Matrix& dynamics_noise_covariance) {
	if (navtk::get_global_error_mode() == navtk::ErrorMode::OFF)
		return ValidationResult::NOT_CHECKED;
	return ValidationContext{}
	    .add_matrix(dynamics_jacobian, "jacobian")
	    .dim('N', 'N')
	    .add_matrix(dynamics_noise_covariance, "dynamics_noise_covariance")
	    .dim('N', 'N')
	    .symmetric()
	    .validate();
}

ValidationResult StandardModelStrategy::validate_linearized_update(
    const Matrix& measurement_jacobian,
    const Matrix& measurement_noise_covariance,
    const Vector& measurement,
    const Vector& hx) {
	if (navtk::get_global_error_mode() == navtk::ErrorMode::OFF)
		return ValidationResult::NOT_CHECKED;
	return ValidationContext{}
	    .add_matrix(measurement_noise_covariance, "measurement_noise_covariance")
	    .dim('M', 'M')
	    .symmetric()
	    .add_matrix(measurement, "measurement")
	    .dim('M', 1)
	    .add_matrix(measurement_jacobian, "jacobian")
	    .dim('M', 'N')
	    .add_matrix(hx, "measurementFunction(estimate)")
	    .dim('M', 1)
	    .validate();
}

}  // namespace filtering
}  // namespace navtk
