#pragma once

#include <utility>
#include <vector>

#include <navtk/magnetic/MagnetometerCalibration.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace magnetic {

/**
 * General class for calibrating magnetometer measurements using a scale factor Matrix and a bias
 * Vector.
 */
class MagnetometerCalibrationScaleFactorBias : public MagnetometerCalibration {
public:
	/**
	 * Get the Matrix of scale factors and distortions for each axis.
	 *
	 * @return a pair containing the scale factor matrix and the bias vector
	 */
	std::pair<Matrix, Vector> get_calibration_params();

	/**
	 * Set the Matrix of scale factors and distortions for each axis.
	 *
	 * @param sf scale factor matrix to assign to [scale_factor]
	 * @param b the bias vector to assign to [bias]
	 */
	void set_calibration_params(Matrix const &sf, Vector const &b);

	/**
	 * Apply calibration parameters to a magnetometer measurement to obtain a calibrated
	 * measurement.
	 *
	 * @param mag magnetic field measurement vector (any units)
	 *
	 * @return Vector containing the calibrated magnetic field vector in the same units as the input
	 * values.
	 */
	virtual Vector apply_calibration(const Vector &mag) const override;

protected:
	/**
	 * The scale factor and distortion matrix, with a row for each axis.
	 */
	Matrix scale_factor;

	/**
	 * The bias vector, with an element for each axis.
	 */
	Vector bias;
};
}  // namespace magnetic
}  // namespace navtk
