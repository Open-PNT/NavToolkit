#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/inertial/MovementDetector.hpp>
#include <navtk/inertial/MovementDetectorImu.hpp>
#include <navtk/inertial/MovementDetectorPos.hpp>
#include <navtk/inertial/MovementStatus.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Processes Measurements containing a Velocity3d and NavSolution to provide a whole-state velocity
 * update.
 */
class ZuptMeasurementProcessor : public MeasurementProcessor<> {

public:
	/**
	 * Constructor for processor that updates only one state block.
	 *
	 * @param label Name of this processor.
	 * @param state_block_label A label referring to a Pinson-style StateBlock this processor
	 * updates. It is required that the block have an estimate vector of at least length 6 and
	 * elements[3..5] contain NED velocity error state in m/s.
	 * @param cov 3x3 Covariance matrix to be used for the zero velocity update.
	 * @param window (optional) Number of IMU measurements to collect and average together before
	 * updating movement status. A longer window can potentially reduce classification error by
	 * removing more noise, but will result in a longer lag in status (movement status is held
	 * constant over the collection period), with a value of 1 indicating real-time estimation. See
	 * inertial::MovementDetectorImu.
	 * @param calib_time (optional)  Number of seconds of guaranteed stationary data at the
	 * beginning of processing. This stationary data is used to form a rough estimate of sensor
	 * biases and noise levels that are used as the basis of IMU movement detection. See
	 * inertial::MovementDetectorImu.
	 * @param speed_cutoff (optional) Nominal speed threshold for platform to be considered moving
	 * using a position domian movement detector, in m/s. Speed in all axes must be below this value
	 * to be classified as stationary over the period between 2 position inputs. See
	 * inertial::MovementDetectorPos.
	 * @param zero_corr_distance (optional) Threshold distance used in position domain movement
	 * detector, between two sequential position measurements at which they are considered
	 * uncorrelated, in m. Correlation between the the two measurements is linearly scaled based on
	 * the norm distance between the two points; Minimum value is 1, and anything less will be
	 * silently replaced. See inertial::MovementDetectorPos.
	 * @param mov_detect_imu_weight (optional) Relative weight assigned to this plugin. Must be
	 * positive. See inertial::MovementDetector#get_status() for how weights affect results. See
	 * inertial::MovementDetector.
	 * @param mov_detect_imu_stale (optional) Amount of time (seconds) allowed to have elapsed
	 * between this plugin's inertial::MovementDetectorPlugin#get_time result and the latest
	 * inertial::MovementDetectorPlugin#get_time result from all plugins before treating the status
	 * from this plugin as temporarily invalid. See inertial::MovementDetector.
	 * @param mov_detect_pos_weight (optional) Relative weight assigned to this plugin. Must be
	 * positive. See inertial::MovementDetector#get_status() for how weights affect results. See
	 * inertial::MovementDetector.
	 * @param mov_detect_pos_stale (optional) Amount of time (seconds) allowed to have elapsed
	 * between this plugin's inertial::MovementDetectorPlugin#get_time result and the latest
	 * inertial::MovementDetectorPlugin#get_time result from all plugins before treating the status
	 * from this plugin as temporarily invalid. See inertial::MovementDetector.
	 * @param state_count (optional) number of state blocks associated with state_block_label. If 0,
	 * the measurement processor will size the H matrix based off `gen_x_and_p_func()`, otherwise
	 * this field will be used to size the H matrix, which reduces computational expense.
	 */
	ZuptMeasurementProcessor(std::string label,
	                         const std::string &state_block_label,
	                         const Matrix3 &cov,
	                         const Size window                  = 1,
	                         const double calib_time            = 1.0,
	                         const double speed_cutoff          = 0.2,
	                         const double zero_corr_distance    = 100.0,
	                         const double mov_detect_imu_weight = 1.0,
	                         const double mov_detect_imu_stale  = 10.0,
	                         const double mov_detect_pos_weight = 1.0,
	                         const double mov_detect_pos_stale  = 10.0,
	                         int state_count                    = 0);

	/**
	 * Generates a StandardMeasurementModel that relates a zero velocity to a state vector
	 * with velocity error states.
	 *
	 * @param measurement Measurement of type aspn_xtensor::MeasurementImu.
	 *
	 * @param gen_x_and_p_func A function that will generate `xhat` (a Vector of estimated states,
	 * constructed from state blocks referenced by `state_block_labels`) and `P` (covariance Matrix
	 * for `xhat`) when called.
	 *
	 * @return A constructed StandardMeasurementModel.
	 */
	std::shared_ptr<StandardMeasurementModel> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase> measurement,
	    GenXhatPFunction gen_x_and_p_func) override;

	/**
	 * Create a copy of the MeasurementProcessor with the same properties.
	 *
	 * @return A shared pointer to a copy of the MeasurementProcessor.
	 */
	not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override;

private:
	/**
	 *  Detects movement based on sensor data and MovementDetectorPlugin
	 */
	navtk::inertial::MovementDetector detector;

	/**
	 * user provided covariance matrix of zero velocity update.
	 */
	Matrix3 cov;

	/**
	 * number of state blocks associated with state_block_label.
	 */
	int state_count;
};


}  // namespace filtering
}  // namespace navtk
