#pragma once

#include <utility>
#include <vector>

#include <navtk/magnetic/MagnetometerCalibrationScaleFactorBias.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace magnetic {

/**
 * Class for calibrating magnetometer measurements using the Ellipse2D method.
 *
 * This method is taken from Section 2.4.5.1 of Shockley PhD Dissertation: Ground Vehicle Navigation
 * Using Magnetic Field Variation, 2012 (available on dtic.mil)
 */
class MagnetometerCalibrationEllipse2d : public MagnetometerCalibrationScaleFactorBias {
public:
	/**
	 * Constructor
	 *
	 * @param calibrate_caruso if true, compute calibration parameters with the 2D Caruso method
	 * first, then further calibrate with the Ellipse2D method.
	 */
	MagnetometerCalibrationEllipse2d(bool calibrate_caruso = false);

	/**
	 * Given x and y magnetometer values, compute the calibration parameters using the Ellipse2D
	 * method.
	 *
	 * @param mag Matrix of magnetometer measurements(any units), with each row corresponding to an
	 * axis, and each column to a measurement. Must have 2 rows, with the first being the x-axis,
	 * and the second being the y-axis.
	 */
	virtual void generate_calibration(const Matrix& mag) override;

private:
	bool calibrate_caruso;  // calibrate with Caruso method first
};

}  // namespace magnetic
}  // namespace navtk
