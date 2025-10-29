#include <navtk/magnetic/MagnetometerCalibrationEllipse2d.hpp>

#include <algorithm>

#include <xtensor/generators/xbuilder.hpp>
#include <xtensor/views/xview.hpp>

#include <navtk/factory.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/magnetic/MagnetometerCalibrationCaruso2d.hpp>

namespace navtk {
namespace magnetic {

using navtk::num_rows;
using navtk::zeros;
using xt::col;
using xt::eye;
using xt::range;

MagnetometerCalibrationEllipse2d::MagnetometerCalibrationEllipse2d(bool calibrate_caruso)
    : calibrate_caruso(calibrate_caruso) {}

void MagnetometerCalibrationEllipse2d::generate_calibration(const Matrix& mag) {
	size_t dims = num_rows(mag);
	if (dims != 2) {
		log_or_throw<std::invalid_argument>(
		    "Magnetic field matrix must have 2 rows, but {} were given. Cannot generate "
		    "magnetometer calibration");
	}
	size_t num_meas = num_cols(mag);
	Matrix mag_cpy  = mag;
	Matrix caruso_scale_factor;
	Vector caruso_bias;
	if (calibrate_caruso) {
		MagnetometerCalibrationCaruso2d caruso_calibration;
		caruso_calibration.generate_calibration(mag_cpy);
		auto caruso_calibration_params = caruso_calibration.get_calibration_params();
		caruso_scale_factor            = caruso_calibration_params.first;
		caruso_bias                    = caruso_calibration_params.second;
		for (size_t idx = 0; idx < num_meas; idx++) {
			col(mag_cpy, idx) = caruso_calibration.apply_calibration(col(mag_cpy, idx));
		}
	}

	// Calculate the mean magnitude of the magnetic field measurements
	// TODO #887: there's no mean in Shockley's paper.
	Vector magnitudes     = xt::norm_l2(mag_cpy, {0});
	double mean_magnitude = xt::mean(magnitudes)(0);

	// Eqn 2.25
	Matrix D      = zeros(num_meas, 5);
	xt::col(D, 0) = xt::row(mag_cpy, 0) * xt::row(mag_cpy, 0);
	xt::col(D, 1) = xt::row(mag_cpy, 1) * xt::row(mag_cpy, 1);
	xt::col(D, 2) = 2 * xt::row(mag_cpy, 0) * xt::row(mag_cpy, 1);
	xt::col(D, 3) = -2 * xt::row(mag_cpy, 0);
	xt::col(D, 4) = -2 * xt::row(mag_cpy, 1);

	// Eqn. 2.26
	Vector mean_magnitude_vec = ones(num_meas) * (mean_magnitude * mean_magnitude);
	Vector p = dot(dot(navtk::inverse(dot(transpose(D), D)), transpose(D)), mean_magnitude_vec);
	Matrix Q = zeros(2, 2);
	Q(0, 0)  = p[0];
	Q(1, 1)  = p[1];
	Q(0, 1)  = p[2];
	Q(1, 0)  = p[2];

	// Eqn 2.27
	// NOTE: Sign of bias is flipped here, so that it can be added instead of subtracted in
	// apply_calibration(). Adding the bias makes Eqn 2.28 look more like Eqn 2.11, which is used
	// for the Caruso calibration method.
	bias                    = zeros(dims);
	scale_factor            = eye(dims);
	view(bias, range(0, 2)) = -dot(navtk::inverse(Q), view(p, xt::range(3, 5)));
	auto svd                = xt::linalg::svd(Q);
	view(scale_factor, range(0, 2), range(0, 2)) =
	    dot(dot(std::get<0>(svd), diag(sqrt(std::get<1>(svd)))), transpose(std::get<0>(svd)));

	if (calibrate_caruso) {
		scale_factor = dot(scale_factor, caruso_scale_factor);
		bias         = dot(navtk::inverse(caruso_scale_factor), bias) + caruso_bias;
	}

	calibrated = true;
}

}  // namespace magnetic
}  // namespace navtk
