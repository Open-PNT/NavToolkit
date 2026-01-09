#pragma once

#include <utility>
#include <vector>

#include <navtk/magnetic/MagnetometerCalibrationScaleFactorBias.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace magnetic {

/**
 * Class for calibrating magnetometer measurements using the 2D Caruso method.
 *
 * This method is taken from Section 2.4.1.2 of Shockley PhD Dissertation: Ground Vehicle Navigation
 * Using Magnetic Field Variation, 2012 (available on dtic.mil)
 */
class MagnetometerCalibrationCaruso2d : public MagnetometerCalibrationScaleFactorBias {
public:
	/**
	 * Given x and y magnetometer values, compute the calibration parameters using the Caruso
	 * method.
	 *
	 * @param mag Matrix of magnetometer measurements(any units), with each row corresponding to an
	 * axis, and each column to a measurement. Must have at least 2 rows, with the first being the
	 * x-axis, and the second being the y-axis.
	 */
	virtual void generate_calibration(const Matrix& mag) override;
};

}  // namespace magnetic
}  // namespace navtk
