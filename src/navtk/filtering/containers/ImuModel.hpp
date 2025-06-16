#pragma once

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Represents the errors associated with an IMU as a first-order Gauss-Markov (FOGM) bias + white
 * noise (random walk).
 */
class ImuModel : public aspn_xtensor::AspnBase {
public:
	/**
	 * The sigma for the accelerometer random walk process in m/s^(3/2).
	 */
	Vector3 accel_random_walk_sigma;

	/**
	 * The sigma for the gyro random walk process in rad/s^(1/2).
	 */
	Vector3 gyro_random_walk_sigma;

	/**
	 * The steady-state sigma of the bias state (NOT the input noise) in m/s^2.
	 */
	Vector3 accel_bias_sigma;

	/**
	 * The time constant for the accelerometer's FOGM process in seconds.
	 */
	Vector3 accel_bias_tau;

	/**
	 * The steady-state sigma of the bias state (NOT the input noise) in rad/s.
	 */
	Vector3 gyro_bias_sigma;

	/**
	 * The time constant for the gyro's FOGM process in seconds.
	 */
	Vector3 gyro_bias_tau;

	/**
	 * Accel output scale factors, ppm.
	 */
	Vector3 accel_scale_factor;

	/**
	 * Gyro output scale factors, ppm.
	 */
	Vector3 gyro_scale_factor;

	/**
	 * Uncertainty in the accel bias initial value (turn-on bias) in m/s^2.
	 */
	Vector3 accel_bias_initial_sigma;

	/**
	 * Uncertainty in the gyro bias initial value (turn-on bias) in rad/s.
	 */
	Vector3 gyro_bias_initial_sigma;

	/**
	 * Initializes the fields from the parameters using `std::move`.
	 *
	 * @param accel_random_walk_sigma the value to store in #accel_random_walk_sigma.
	 * @param gyro_random_walk_sigma the value to store in #gyro_random_walk_sigma.
	 * @param accel_bias_sigma the value to store in #accel_bias_sigma.
	 * @param accel_bias_tau the value to store in #accel_bias_tau.
	 * @param gyro_bias_sigma the value to store in #gyro_bias_sigma.
	 * @param gyro_bias_tau the value to store in #gyro_bias_tau.
	 * @param accel_scale_factor the value to store in #accel_scale_factor.
	 * @param gyro_scale_factor the value to store in #gyro_scale_factor.
	 * @param accel_bias_initial_sigma the value to store in #accel_bias_initial_sigma.
	 * @param gyro_bias_initial_sigma the value to store in #gyro_bias_initial_sigma.
	 * @param message_type the ASPN message type assigned to ImuModel. This will likely be
	 * specific to the running program. Defaults to ASPN_EXTENDED_BEGIN since ImuModel
	 * extends the set of defined ASPN messages. Note, if this default value is not overridden by a
	 * program using NavToolkit, then the type assigned to ImuModel may conflict with
	 * another type used by that program. Users should be careful to ensure that all ASPN message
	 * types used by their program have unique types if they are using AspnBase::get_message_type to
	 * identify a message type.
	 */
	ImuModel(Vector3 accel_random_walk_sigma  = zeros(3),
	         Vector3 gyro_random_walk_sigma   = zeros(3),
	         Vector3 accel_bias_sigma         = zeros(3),
	         Vector3 accel_bias_tau           = zeros(3),
	         Vector3 gyro_bias_sigma          = zeros(3),
	         Vector3 gyro_bias_tau            = zeros(3),
	         Vector3 accel_scale_factor       = zeros(3),
	         Vector3 gyro_scale_factor        = zeros(3),
	         Vector3 accel_bias_initial_sigma = zeros(3),
	         Vector3 gyro_bias_initial_sigma  = zeros(3),
	         AspnMessageType message_type     = ASPN_EXTENDED_BEGIN);
};

/**
 * Returns the Honeywell HG1700 IMU model. There are a number of HG1700 models, these specs are for
 * the higher-end versions. Bias tau values are assumed. Scale not present in sheet and are set to
 * 0.
 *
 * See
 * https://aerospace.honeywell.com/content/dam/aerobt/en/documents/learn/products/sensors/brochures/N61-1619-000-001-HG1700InertialMeasurementUnit-bro.pdf
 *
 * Spec sheet values given as (all 1 sigma):
 *
 * Accel Bias: 1 mg = 9.81e-3 m/s^2
 *
 * VRW: 0.065 fps/sqrt(hr) max = 3.302e-4 m/s^(3/2)
 *
 * Gyro bias: 1 deg/hr = 4.8481e-6 rad/s
 *
 * ARW: 0.125 deg/sqrt(hr) max = 3.6361e-5 rad/s^(1/2).
 *
 * Conversions assume gravity value of 9.81 m/s^2.
 *
 * @return The error characteristics of on an HG1700 IMU.
 */
ImuModel hg1700_model();


/**
 * Returns the Honeywell HG9900 IMU model. Bias tau values are assumed.
 *
 * See
 * https://aerospace.honeywell.com/content/dam/aerobt/en/documents/learn/products/sensors/brochures/N61-1638-000-000-hg9900inertialmeasurementunit-bro.pdf
 * Spec sheet values given as (all 1 sigma):
 *
 * Accel Bias: 25 ug = 2.4525e-4 m/s^2
 *
 * VRW: 0 (None listed) set to 1e-12 to avoid numerical issues
 *
 * Gyro bias: 0.0035 deg/hr = 1.6968e-8 rad/s
 *
 * ARW: 0.002 deg/sqrt(hr) max = 5.8178e-7 rad/s^(1/2).
 *
 * Accel scale factor: 100 ppm.
 *
 * Gyro scale factor: 5 ppm.
 *
 * Conversions assume gravity value of 9.81 m/s^2.
 *
 * @return The error characteristics of on an HG9900 IMU.
 */
ImuModel hg9900_model();

/**
 * Returns an IMU model for the Sagem Primus 200.
 *
 * Spec sheet values given as:
 *
 * Accel Bias: 2 mg (rms) =  0.01962 m/s^2
 *
 * VRW: 60ug /sqrt(Hz) at 100 hz sample rate = 5.886e-5 m/s^(3/2)
 *
 * Gyro bias: 0.05 deg/hr = 2.4240684e-07 rad/s
 *
 * ARW: 0.004 deg/sqrt(hr) = 1.1635528e-06 rad/s^(1/2)
 *
 * Accel scale factor: 500 ppm (rms)
 *
 * Gyro scale factor: 10 ppm (rms)
 *
 * @return The Sagem Primus 200 ImuModel.
 */
ImuModel sagem_primus200_model();

/**
 * Returns an IMU model for the STIM300, using the specs for the 10g accelerometers. Note the
 * large number of additional error terms not accounted for- it is unlikely that this model will
 * perform well as is.
 *
 * See https://www.sensonor.com/media/1132/ts1524r9-datasheet-stim300.pdf
 *
 * Spec sheet values given as:
 *
 * Accel Bias: 0.05 mg =  4.905e-4 m/s^2
 *
 * VRW: 0.06 m/s/sqrt(hr) = 1e-3 m/s^(3/2)
 *
 * Gyro bias: 0.5 deg/hr = 2.4241e-6 rad/s
 *
 * ARW: 0.15 deg/sqrt(hr) = 4.3633e-5 rad/s^(1/2)
 *
 * Accel scale factor: 300 ppm
 *
 * Gyro scale factor: 500 ppm
 *
 * Accel Bias on/off: +/- 0.75 mg (valid only for 10g range setting; parameter scales with range)
 *
 * Additional terms given but not modeled here:
 *
 * Gyro non-linearity: 25 to 50 ppm (at 200/400 deg/sec, respectively)
 *
 * Gyro Bias range: +/- 250 deg/hr
 *
 * Gyro Bias over static temp: 5 deg/hr
 *
 * Gyro bias over changing temp: 10 deg/hr for temp change <= 1 deg C/min
 *
 * Gyro bias drift due to acceleration: 1 deg/hr/g (assuming 'g-compensation', see spec sheet)
 *
 * Gyro scale factor due to acceleration: 30 ppm/g (assuming 'g-compensation', see spec sheet)
 *
 * Gyro sensor misalignment: 1 mrad
 *
 * Accel non-linearity: 100 ppm
 *
 * Accel Bias over temp: +/- 2 mg (rms)
 *
 * Accel sensor misalignment: 1 mrad
 *
 * @return The STIM 300 ImuModel.
 */
ImuModel stim300_model();

/**
 * An idealized IMU model.
 *
 * @return An error-free ImuModel with 1 hour time constants.
 */
ImuModel ideal_imu_model();

}  // namespace filtering
}  // namespace navtk
