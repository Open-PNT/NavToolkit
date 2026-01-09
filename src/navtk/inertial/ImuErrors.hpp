#pragma once

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Container for estimated IMU sensor errors. Generally, the error model
 * followed is that given in Titterton and Weston, 2nd edition, eq 8.4 and 8.5:
 * \f$ v_{meas} = (1 + e_{scale})v_{true} + e_{misalignment} + e_{bias} + e_{noise} \f$
 *
 * Misalignment errors are currently not represented. Note that care must be
 * taken with signs as many filter formulations do not estimate these values
 * directly, but rather the error-form and these should be negated when
 * assigned to this container.
 */
struct ImuErrors : aspn_xtensor::AspnBase {

	/**
	 * Accel biases in sensor frame, m/s^2, such that
	 * \f$ accel_{meas} = accel_{true} + accel_{bias} \f$
	 * in the absence of other errors.
	 */
	Vector3 accel_biases;

	/**
	 * Gyro biases in sensor frame, rad/s, such that
	 * \f$ gyro_{meas} = gyro_{true} + gyro_{bias} \f$
	 * in the absence of other errors.
	 */
	Vector3 gyro_biases;

	/**
	 * Accel scale factors in sensor frame, unitless, such that
	 * \f$ accel_{meas} = (1 + accel_{scale})accel_{true} \f$
	 * in the absence of other errors.
	 */
	Vector3 accel_scale_factors;

	/**
	 * Gyro scale factors in sensor frame, unitless, such that
	 * \f$ gyro_{meas} = (1 + gyro_{scale})gyro_{true} \f$
	 * in the absence of other errors.
	 */
	Vector3 gyro_scale_factors;

	/**
	 * Time at which the IMU errors are considered to be valid.
	 */
	aspn_xtensor::TypeTimestamp time_validity;

	/**
	 * Constructor.
	 *
	 * @param accel_biases Accel biases in sensor frame, m/s^2.
	 * @param gyro_biases Gyro biases in sensor frame, rad/s.
	 * @param accel_scale_factors Sensor frame, unitless.
	 * @param gyro_scale_factors Sensor frame, unitless.
	 * @param time Time at which values are valid.
	 * @param message_type the ASPN message type assigned to ImuErrors. This will likely be specific
	 * to the running program. Defaults to ASPN_EXTENDED_BEGIN since ImuErrors extends the set of
	 * defined ASPN messages. Note, if this default value is not overridden by a program using
	 * NavToolkit, then the type assigned to ImuErrors may conflict with another type used by that
	 * program. Users should be careful to ensure that all ASPN message types used by their program
	 * have unique types if they are using AspnBase::get_message_type to identify a message type.
	 */
	ImuErrors(const Vector3& accel_biases             = zeros(3),
	          const Vector3& gyro_biases              = zeros(3),
	          const Vector3& accel_scale_factors      = zeros(3),
	          const Vector3& gyro_scale_factors       = zeros(3),
	          const aspn_xtensor::TypeTimestamp& time = aspn_xtensor::TypeTimestamp((int64_t)0),
	          AspnMessageType message_type            = ASPN_EXTENDED_BEGIN)
	    : aspn_xtensor::TypeHeader(message_type, 0, 0, 0, 0),
	      accel_biases(accel_biases),
	      gyro_biases(gyro_biases),
	      accel_scale_factors(accel_scale_factors),
	      gyro_scale_factors(gyro_scale_factors),
	      time_validity(time) {}
};

}  // namespace inertial
}  // namespace navtk
