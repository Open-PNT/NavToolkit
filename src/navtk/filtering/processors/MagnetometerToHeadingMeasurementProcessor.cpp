#include <navtk/filtering/processors/MagnetometerToHeadingMeasurementProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/magnetic/MagnetometerCalibration.hpp>
#include <navtk/magnetic/magnetic.hpp>

using aspn_xtensor::MeasurementMagneticField;
using navtk::magnetic::mag_to_heading;
using navtk::magnetic::MagnetometerCalibration;

namespace navtk {
namespace filtering {

MagnetometerToHeadingMeasurementProcessor::MagnetometerToHeadingMeasurementProcessor(
    std::string label,
    const std::string& state_block_label,
    const std::shared_ptr<MagnetometerCalibration>& calibration,
    double heading_var,
    double magnetic_declination,
    const Matrix& dcm)
    : MagnetometerToHeadingMeasurementProcessor(std::move(label),
                                                std::vector<std::string>(1, state_block_label),
                                                calibration,
                                                heading_var,
                                                magnetic_declination,
                                                dcm) {}

MagnetometerToHeadingMeasurementProcessor::MagnetometerToHeadingMeasurementProcessor(
    std::string label,
    std::vector<std::string> state_block_labels,
    const std::shared_ptr<MagnetometerCalibration>& calibration,
    double heading_var,
    double magnetic_declination,
    const Matrix& dcm)
    : MeasurementProcessor(std::move(label), std::move(state_block_labels)),
      calibration(calibration),
      heading_var(heading_var),
      magnetic_declination(magnetic_declination),
      dcm(dcm) {}

std::shared_ptr<StandardMeasurementModel> MagnetometerToHeadingMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction) {

	std::shared_ptr<MeasurementMagneticField> input_meas =
	    std::dynamic_pointer_cast<MeasurementMagneticField>(measurement);
	if (input_meas == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (MeasurementMagneticField). Unable to perform "
		    "update.");
		return nullptr;
	}

	auto x_field_strength = input_meas->get_x_field_strength();
	auto y_field_strength = input_meas->get_y_field_strength();
	auto z_field_strength = input_meas->get_z_field_strength();

	if (isnan(x_field_strength) || isnan(y_field_strength) || isnan(z_field_strength))
		log_or_throw(
		    "Expected MeasurementMagneticField to be a 3D measurement but received NaN's for one "
		    "or more terms.");

	// Convert from initial reference frame to desired frame
	Vector3 mag_input_frame{x_field_strength, y_field_strength, z_field_strength};
	Vector3 mag_output_frame    = dot(dcm, mag_input_frame);
	Matrix3 covariance          = input_meas->get_covariance();
	Matrix mag_cov_output_frame = dot(dot(dcm, covariance), xt::transpose(dcm));

	if (calibration == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Magnetometer calibration not set inside the "
		    "MagnetometerToHeadingMeasurementProcessor. Returning nullptr.");
		return nullptr;
	}

	auto mag_calibrated = calibration->apply_calibration(mag_output_frame);

	Vector heading{mag_to_heading(mag_calibrated(0), mag_calibrated(1), magnetic_declination)};

	Matrix R = eye(1);
	if (heading_var >= 0.0) {
		R(0, 0) = heading_var;
	} else {
		double m = x_field_strength * x_field_strength + y_field_strength * y_field_strength;
		Matrix pd{{-y_field_strength / m, x_field_strength / m}};
		R = dot(dot(pd, view(mag_cov_output_frame, xt::range(0, 2), xt::range(0, 2))),
		        xt::transpose(pd));
	}
	return std::make_shared<StandardMeasurementModel>(heading, H, R);
}

not_null<std::shared_ptr<MeasurementProcessor<>>>
MagnetometerToHeadingMeasurementProcessor::clone() {
	return std::make_shared<MagnetometerToHeadingMeasurementProcessor>(*this);
}

}  // namespace filtering
}  // namespace navtk
