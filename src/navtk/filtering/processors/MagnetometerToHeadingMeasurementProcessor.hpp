#pragma once

#include <memory>
#include <string>
#include <utility>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/magnetic/MagnetometerCalibration.hpp>
#include <navtk/not_null.hpp>

using navtk::magnetic::MagnetometerCalibration;

namespace navtk {
namespace filtering {

/**
 * Processes 3D magnetometer Measurements of aspn type MeasurementMagneticField to update heading
 * (in 2D space) and heading bias states relative to true north or magnetic north.
 */
class MagnetometerToHeadingMeasurementProcessor : public MeasurementProcessor<> {

public:
	/**
	 * Constructor for processor that updates only one state block.
	 *
	 * @param label Name of this processor.
	 * @param state_block_label A label referring to a StateBlock this processor updates. It is
	 * required that the state block label represents heading [radians] as state 1 and heading bias
	 * [radians] as state 2.
	 * @param calibration The calibration object used to calibrate magnetometer measurements.
	 * @param heading_var The heading variance for the calibrated heading from the magnetometer
	 * measurements in radians. This is an optional parameter. If set to a negative value, the
	 * heading will be calculated using the following formula: \parblock
	 *
	 * \f[ \sigma^2_{heading} = \begin{bmatrix} \frac{-y}{y^2+x^2} & \frac{x}{y^2+x^2} \end{bmatrix}
	 * \begin{bmatrix} \sigma^2_{x} & \sigma_{x}\sigma_{y} \\  \sigma_{x}\sigma_{y} & \sigma^2_{y}
	 * \end{bmatrix} \begin{bmatrix} \frac{-y}{y^2+x^2} \\ \frac{x}{y^2+x^2} \end{bmatrix} \f]
	 *
	 * where:
	 *
	 * \f$ x \f$ is the measured x magnetometer field strength.
	 *
	 * \f$ y \f$ is the measured y magnetometer field strength.
	 *
	 * \f$ \sigma^2_{x} \f$ is the measurement x magnetometer field strength variance.
	 *
	 * \f$ \sigma^2_{y} \f$ is the measurement y magnetometer field strength variance.
	 *
	 * \f$ \sigma_{xy} \f$ is the measurement x and y magnetometer field strength covariance.
	 * \endparblock
	 * @param magnetic_declination magnetic declination (the angle on the horizontal plane between
	 * magnetic north and true north in radians). This value varies depending on position on the
	 * Earth's surface. If heading is desired relative to magnetic north, leave this argument set to
	 * 0. TODO #751: Support lookup model.
	 * @param dcm A direction cosine Matrix used to convert the MeasurementMagneticField
	 * measurements from the incoming measurement's frame to the platform frame for the heading
	 * calculation.
	 */
	MagnetometerToHeadingMeasurementProcessor(
	    std::string label,
	    const std::string& state_block_label,
	    const std::shared_ptr<MagnetometerCalibration>& calibration,
	    double heading_var          = -1.0,
	    double magnetic_declination = 0,
	    const Matrix& dcm           = {{1., 0, 0}, {0, 1., 0}, {0, 0, 1.}});

	/**
	 * Constructor for processor that updates multiple state blocks.
	 *
	 * @param label Name of this processor.
	 * @param state_block_labels A vector of labels referring to StateBlocks this processor updates.
	 * It is required that the state block labels represent heading [radians] as state 1 and heading
	 * bias [radians] as state 2.
	 * @param calibration The calibration object used to calibrate magnetometer measurements.
	 * @param heading_var The heading variance for the calibrated heading from the magnetometer
	 * measurements in radians. This is an optional parameter. If set to a negative value, the
	 * heading will be calculated using the following formula: \parblock
	 *
	 * \f[ \sigma^2_{heading} = \begin{bmatrix} \frac{-y}{y^2+x^2} & \frac{x}{y^2+x^2} \end{bmatrix}
	 * \begin{bmatrix} \sigma^2_{x} & \sigma_{x}\sigma_{y} \\  \sigma_{x}\sigma_{y} & \sigma^2_{y}
	 * \end{bmatrix} \begin{bmatrix} \frac{-y}{y^2+x^2} \\ \frac{x}{y^2+x^2} \end{bmatrix} \f]
	 *
	 * where:
	 *
	 * \f$ x \f$ is the measured x magnetometer field strength.
	 *
	 * \f$ y \f$ is the measured y magnetometer field strength.
	 *
	 * \f$ \sigma^2_{x} \f$ is the measurement x magnetometer field strength variance.
	 *
	 * \f$ \sigma^2_{y} \f$ is the measurement y magnetometer field strength variance.
	 *
	 * \f$ \sigma_{xy} \f$ is the measurement x and y magnetometer field strength covariance.
	 * \endparblock
	 * @param magnetic_declination magnetic declination (the angle on the horizontal plane between
	 * magnetic north and true north in radians). This value varies depending on position on the
	 * Earth's surface. If heading is desired relative to magnetic north, leave this argument set to
	 * 0. TODO #751: Support lookup model.
	 * @param dcm A direction cosine Matrix used to convert the MeasurementMagneticField
	 * measurements from the incoming measurement's frame to the platform frame for the heading
	 * calculation.
	 */
	MagnetometerToHeadingMeasurementProcessor(
	    std::string label,
	    std::vector<std::string> state_block_labels,
	    const std::shared_ptr<MagnetometerCalibration>& calibration,
	    double heading_var          = -1.0,
	    double magnetic_declination = 0,
	    const Matrix& dcm           = {{1., 0, 0}, {0, 1., 0}, {0, 0, 1.}});

	/**
	 * Calibrates the MeasurementMagneticField measurement and generates a StandardMeasurementModel
	 * that relates the magnetic field measurements to heading (in 2D space) and heading bias states
	 * relative to true north, or magnetic north.
	 *
	 * @param measurement Measurement of type MeasurementMagneticField.
	 *
	 * @return A constructed StandardMeasurementModel if \p measurement contains
	 * MeasurementMagneticField, otherwise `nullptr`.
	 */
	std::shared_ptr<StandardMeasurementModel> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction) override;

	/**
	 * Create a copy of the MeasurementProcessor with the same properties.
	 *
	 * @return A shared pointer to a copy of the MeasurementProcessor.
	 */
	not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override;

private:
	std::shared_ptr<MagnetometerCalibration>
	    calibration;  // object for calibrating magnetometer measurements
	double heading_var;
	double magnetic_declination;  // angle between true north and magnetic north
	Matrix dcm;  // direction cosine matrix, used to convert MeasurementMagneticField measurements
	             // from one reference frame to another.
	// Map states to a 1-element Vector containing a heading measurement in radians
	Matrix H = {{1, 1}};
};


}  // namespace filtering
}  // namespace navtk
