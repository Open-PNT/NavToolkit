#include <navtk/filtering/containers/ImuModel.hpp>

#include <utility>

#include <navtk/navutils/math.hpp>

using navtk::navutils::PI;

namespace navtk {
namespace filtering {

ImuModel::ImuModel(Vector3 accel_random_walk_sigma,
                   Vector3 gyro_random_walk_sigma,
                   Vector3 accel_bias_sigma,
                   Vector3 accel_bias_tau,
                   Vector3 gyro_bias_sigma,
                   Vector3 gyro_bias_tau,
                   Vector3 accel_scale_factor,
                   Vector3 gyro_scale_factor,
                   Vector3 accel_bias_initial_sigma,
                   Vector3 gyro_bias_initial_sigma,
                   AspnMessageType message_type)
    : aspn_xtensor::AspnBase(message_type, 0, 0, 0, 0),
      accel_random_walk_sigma(std::move(accel_random_walk_sigma)),
      gyro_random_walk_sigma(std::move(gyro_random_walk_sigma)),
      accel_bias_sigma(std::move(accel_bias_sigma)),
      accel_bias_tau(std::move(accel_bias_tau)),
      gyro_bias_sigma(std::move(gyro_bias_sigma)),
      gyro_bias_tau(std::move(gyro_bias_tau)),
      accel_scale_factor(std::move(accel_scale_factor)),
      gyro_scale_factor(std::move(gyro_scale_factor)),
      accel_bias_initial_sigma(std::move(accel_bias_initial_sigma)),
      gyro_bias_initial_sigma(std::move(gyro_bias_initial_sigma)) {}

ImuModel hg1700_model() {
	return ImuModel(zeros(3) + 0.065 * 0.3048 / 60,    // accel_random_walk_sigma, m/s^(3/2)
	                zeros(3) + 0.125 * PI / 180 / 60,  // gyro_random_walk_sigma, rad/s^(1/2)
	                zeros(3) + 9.81e-3,                // accel_bias_sigma, m/s^2
	                zeros(3) + 3600.0,                 // accel_bias_tau, sec
	                zeros(3) + PI / 180 / 3600,        // gyro_bias_sigma, rad/s
	                zeros(3) + 3600.0,                 // gyro_bias_tau, sec
	                zeros(3),                          // accel_scale_factor, ppm
	                zeros(3),                          // gyro_scale_factor, ppm
	                zeros(3) + 9.81e-3,                // initial_accel_bias_sigma, m/s^2
	                zeros(3) + PI / 180 / 3600         // initial_gyro_bias_sigma, rad/s
	);
}

ImuModel hg9900_model() {
	return ImuModel(zeros(3) + 1e-12,
	                zeros(3) + (0.002 * PI / 180 / 60),
	                zeros(3) + 25 * 9.81e-6,
	                zeros(3) + 3600,
	                zeros(3) + 0.0035 * PI / 180 / 3600,
	                zeros(3) + 3600,
	                zeros(3) + 100,
	                zeros(3) + 5,
	                zeros(3) + 25 * 9.81e-6,
	                zeros(3) + 0.003 * PI / 180 / 3600);
}

ImuModel sagem_primus200_model() {
	return ImuModel(zeros(3) + 60e-6 * 9.81 / 10,
	                zeros(3) + 0.004 * PI / 180 / 60,
	                zeros(3) + 2 * 9.81 / 1000.0,
	                zeros(3) + 3600.0,
	                zeros(3) + 0.05 * PI / 180 / 3600,
	                zeros(3) + 3600.0,
	                zeros(3) + 500,
	                zeros(3) + 10,
	                zeros(3) + 2 * 9.81 / 1000.0,
	                zeros(3) + 0.05 * PI / 180 / 3600);
}

ImuModel stim300_model() {
	return ImuModel(zeros(3) + 0.06 / 60,
	                zeros(3) + 0.15 * PI / 180 / 60,
	                zeros(3) + 0.05 * 9.81 / 1000,
	                zeros(3) + 3600.0,
	                zeros(3) + 0.5 * PI / 180 / 3600,
	                zeros(3) + 3600,
	                zeros(3) + 300,
	                zeros(3) + 500,
	                zeros(3) + 0.75 * 9.81 / 1000,
	                zeros(3) + 0.5 * PI / 180 / 3600);
}

ImuModel ideal_imu_model() {
	return ImuModel(zeros(3),
	                zeros(3),
	                zeros(3),
	                zeros(3) + 3600.0,
	                zeros(3),
	                zeros(3) + 3600.0,
	                zeros(3),
	                zeros(3),
	                zeros(3),
	                zeros(3));
}

}  // namespace filtering
}  // namespace navtk
