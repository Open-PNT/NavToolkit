#pragma once

#include <memory>
#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/inertial/MovementDetectorPlugin.hpp>
#include <navtk/inertial/MovementStatus.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Accepts inertial data and after an initial stationary calibration period attempts to
 * classify the inertial as moving/not moving in near-real time.
 */
class MovementDetectorImu : public MovementDetectorPlugin {
public:
	/**
	 * Constructor
	 *
	 * @param window Number of IMU measurements to collect and average together before updating
	 * status. A longer window can potentially reduce classification error by removing more noise,
	 * but will result a longer lag in status (movement status is held constant over the collection
	 * period), with a value of 1 indicating real-time estimation.
	 *
	 * @param calib_time Number of seconds of guaranteed stationary data at the beginning of
	 * processing. This stationary data is used to form a rough estimate of sensor biases and noise
	 * levels that are used as the basis of movement detection.
	 *
	 * @throw std::invalid_argument if \p window is < 1 or \p calib_time is < 0.
	 */
	MovementDetectorImu(const Size window = 10, const double calib_time = 30.0);

	/**
	 * Add another IMU measurement and possibly update status.
	 *
	 * @param data IMU measurement to process. Data is stored for various intervals to first allow
	 * for the estimation of biases and noise parameters, and then for averaging of measurements.
	 * After the initial stationary period, each 'window'th (as per the constructor argument) call
	 * to this function will trigger a new status calculation. Inputs should be sequential in time.
	 *
	 * @return Best estimate of movement status based on the last full data collection. Value will
	 * be constant between every 'window'th measurement.
	 */
	MovementStatus process(not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) override;

	/**
	 * Time of last measurement fully processed. Not updated while buffering IMU data to fill buffer
	 * windows.
	 *
	 * @return Last time; `TypeTimestamp((int64_t)0)` if no measurements yet received.
	 */
	aspn_xtensor::TypeTimestamp get_time() override;

private:
	/*
	 * Empty all IMU data related buffers.
	 */
	void clear_buffers();

	/* Number of measurements to consider collectively when detecting movement */
	Size window;

	/* Minimum number of seconds of stationary data assumed delivered at start */
	double calib_time;

	/* Storage for delta_theta extracted */
	std::vector<Vector3> dth_extract;

	/* Storage for time_validities extracted */
	std::vector<aspn_xtensor::TypeTimestamp> time_extract;

	/* Stationary period has passed and we can start detecting movement */
	bool ready_to_test = false;

	/* Norm of average dth from calibration period */
	double initial_dth_norm = 0.0;

	/* One-sigma uncertainty of dth norms from caibration period */
	double initial_dth_sig = 0.0;

	/* Time of last fully processed IMU measurement */
	aspn_xtensor::TypeTimestamp last_time = aspn_xtensor::TypeTimestamp((int64_t)0);
};
}  // namespace inertial
}  // namespace navtk
