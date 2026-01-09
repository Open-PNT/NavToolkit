#pragma once

#include <utility>
#include <vector>

#include <navtk/tensors.hpp>

namespace navtk {
namespace magnetic {

/**
 * An abstract class for calibrating magnetometer measurements.
 */
class MagnetometerCalibration {
public:
	virtual ~MagnetometerCalibration() = default;

	/**
	 * Given x and y magnetometer values, compute the calibration parameters.
	 *
	 * @param mag Matrix of magnetometer measurements(any units), with each row corresponding to an
	 * axis, and each column to a measurement.
	 */
	virtual void generate_calibration(const Matrix& mag) = 0;

	/**
	 * Apply calibration parameters to a magnetometer measurement to obtain a calibrated
	 * measurement.
	 *
	 * @param mag magnetic field measurement vector (any units)
	 *
	 * @return Vector containing the calibrated magnetic field vector in the same units as the input
	 * values.
	 */
	virtual Vector apply_calibration(const Vector& mag) const = 0;

	/**
	 * Get status of [calibrated].
	 *
	 * @return whether or not the calibration parameters have been generated yet. If `false`, all
	 * calls to #apply_calibration will return uncalibrated_measurements.
	 * @return false
	 */
	bool is_calibrated();

protected:
	/**
	 * Indicates whether the calibration parameters have been generated yet. If this is `false`, all
	 * calls to #apply_calibration will return uncalibrated measurements.
	 */
	bool calibrated = false;
};

}  // namespace magnetic
}  // namespace navtk
