#include <navtk/magnetic/MagnetometerCalibrationScaleFactorBias.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/utils/ValidationContext.hpp>

namespace navtk {
namespace magnetic {

std::pair<Matrix, Vector> MagnetometerCalibrationScaleFactorBias::get_calibration_params() {
	return std::make_pair(scale_factor, bias);
}

void MagnetometerCalibrationScaleFactorBias::set_calibration_params(Matrix const& sf,
                                                                    Vector const& b) {
	if (navtk::utils::ValidationContext validation{}) {
		validation.add_matrix(sf, "scale_factor")
		    .dim('N', 'N')
		    .add_matrix(b, "bias")
		    .dim('N', 1)
		    .validate();
	}

	scale_factor = sf;
	bias         = b;

	calibrated = true;
}

Vector MagnetometerCalibrationScaleFactorBias::apply_calibration(const Vector& mag) const {
	if (calibrated) {
		return dot(scale_factor, mag + bias);
	} else {
		spdlog::warn(
		    "Magnetometer calibration parameters have not been set. Not applying calibration to "
		    "measurement.");
	}
	return mag;
}

}  // namespace magnetic
}  // namespace navtk
