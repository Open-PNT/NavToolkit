#pragma once

#include <memory>
#include <utility>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/TimestampedDataSeries.hpp>
#include <navtk/inertial/BufferedPva.hpp>
#include <navtk/inertial/ImuErrors.hpp>
#include <navtk/inertial/Inertial.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/MechanizationStandard.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/Ordered.hpp>

namespace navtk {
namespace inertial {

/**
 * Class that wraps Inertial and maintains a time history of and provides access to calculated PVAs,
 * processed measurements and other values of interest.
 */
class BufferedImu : public BufferedPva {

public:
	/**
	 * Constructor with initial PVA.
	 *
	 * @param pva Time-tagged initial position, velocity and attitude. Attitude in PVA should be
	 * w.r.t. frame of IMU measurements to be mechanized.
	 * @param initial_imu IMU measurement measured over the expected_dt period prior to PVA
	 * measurement time; `nullptr` if unknown. Failure to provide an appropriate value will result
	 * in possibly inaccurate IMU-based values (specific forces, rotation rates) being returned from
	 * their respective functions over the first mechanization period.
	 * @param expected_dt The typical delta time over which an IMU measurement is measured, in
	 * seconds.
	 * @param imu_errs Initial values of IMU error parameters. Time of validity is assumed to be the
	 * same as that of \p pva.
	 * @param mech_options Options for underlying mechanization process.
	 * @param buffer_length Amount of data, in seconds, this class should keep in history. Max
	 * number of accessible buffer elements is `buffer_length/expected_dt + 2`.
	 */
	BufferedImu(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva,
	            std::shared_ptr<aspn_xtensor::MeasurementImu> initial_imu = nullptr,
	            double expected_dt                                        = 0.01,
	            const ImuErrors& imu_errs                                 = ImuErrors{},
	            const MechanizationOptions& mech_options                  = MechanizationOptions{},
	            double buffer_length                                      = 60.0);

	/**
	 * Reset some combination of inertial PVA and IMU error model to new values, and recalculate the
	 * trajectory from that point.
	 *
	 * Reset time 't' is taken to be `pva->time_validity` if provided, otherwise
	 * `imu_errs->time_validity`. All historical data prior to 't' contained in buffers will be
	 * untouched; all data equal to and after 't' will be overwritten.
	 *
	 * @param pva Time-tagged position, velocity and attitude. Must be within the range of times
	 * contained in the buffered PVA history (i.e. `in_range(pva->time_validity`) is true). If
	 * `nullptr`, only \p imu_errs will be considered for reset.
	 * @param imu_errs Estimated IMU errors at reset time. If both \p imu_errs and \p pva are valid
	 * pointers but have different \p time_validity fields, \p imu_errs will be assumed valid at the
	 * same time as \p pva. If `nullptr`, no changes to the error model will be made.
	 * @param previous Time-tagged position, velocity and attitude, at 1 `dt` earlier than \p pva.
	 * Useful for some higher-order integration algorithms that rely on multiple data points, such
	 * as trapezoidal. If not provided (`nullptr`), \p pva will be used instead; the resulting error
	 * should be minimal, especially for low-dynamic situations. This value is only used in
	 * mechanization and will not be inserted into the time history of solutions.
	 *
	 * @return `true` if position was reset to requested PVA, false if the reset was rejected.
	 */
	bool reset(
	    std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> pva      = nullptr,
	    std::shared_ptr<ImuErrors> imu_errs                                         = nullptr,
	    std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> previous = nullptr);

	/**
	 * Get solution at time, with the guarantee that there have been no resets since requested time.
	 * This is accomplished by repropagating IMU data from `since` to `time`, ignoring any resets
	 * that were performed in the interim. If a clean repropagation is not possible, a `nullptr` is
	 * returned.
	 *
	 * @param time Requested solution time.
	 * @param since Time after which no resets should be present in the solution. Resets at exactly
	 * `since` will be included.
	 *
	 * @return Solution at time, or nullptr if requested time doesn't lie in time_span().
	 */
	std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> calc_pva_no_reset_since(
	    const aspn_xtensor::TypeTimestamp& time, const aspn_xtensor::TypeTimestamp& since) const;

	/**
	 * Propagate the current inertial solution by integrating the supplied IMU measurement.
	 *
	 * @param data IMU measurement to mechanize; only accepts data of the type
	 * aspn_xtensor::MeasurementImu. IMU must be of ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED type.
	 * IMU data with a delta time between measurements that varies by
	 * 50% or more from the average delta will generate a warning. To account for the possibility
	 * that this class may have been initialized 'between' IMU measurements, the input to the first
	 * call of this function may be scaled accordingly.
	 *
	 * @throw If error mode is DIE and \p data is a type other than aspn_xtensor::MeasurementImu,
	 * or if a type other than ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED
	 * is received.
	 */
	void add_data(not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) override;

	/**
	 * Propagate the current inertial solution by integrating the supplied IMU measurement.
	 *
	 * @param imu IMU measurement to mechanize. IMU must be of
	 * ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED type. IMU data with a delta time between
	 * measurements that varies by 50% or more from the average delta will generate a warning. To
	 * account for the possibility that this class may have been initialized 'between' IMU
	 * measurements, the input to the first call of this function may be scaled accordingly.
	 *
	 * @throw std::invalid_argument If error mode is DIE and a type other than
	 * ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED is received.
	 */
	void mechanize(const aspn_xtensor::MeasurementImu& imu);

	/**
	 * Propagate the current inertial solution by integrating the supplied IMU measurement.
	 *
	 * @param time IMU measurement time of validity.
	 * @param delta_v Delta velocity measurement in inertial sensor frame, m/s.
	 * @param delta_theta Delta rotation measurement in inertial sensor frame, rad.
	 */
	void mechanize(const aspn_xtensor::TypeTimestamp& time,
	               const Vector3& delta_v,
	               const Vector3& delta_theta);

	/**
	 * Get specific force and inertial sensor frame rotation rate at requested time.
	 *
	 * @param time Requested time of force and rate.
	 *
	 * @return If time is within the available history of solutions, the specific force in the NED
	 * frame, in m/s^2, and inertial sensor frame rotation rate in inertial sensor frame in rad/s
	 * calculated from the single IMU measurement that spans time. Otherwise, a nullptr. This force
	 * and rate will be returned as an `aspn_xtensor::MeasurementImu` with the imu type set to
	 * ASPN_MEASUREMENT_IMU_IMU_TYPE_SAMPLED.
	 */
	std::shared_ptr<aspn_xtensor::MeasurementImu> calc_force_and_rate(
	    const aspn_xtensor::TypeTimestamp& time) const override;

	/**
	 * Get average specific force and rotation rate over a range of times.
	 *
	 * @param time1 Start time of range, inclusive.
	 * @param time2 Stop time of range, inclusive.
	 *
	 * @return Average specific force in the NED frame in m/s^2, and average inertial sensor frame
	 * rotation rate, in the inertial sensor frame in rad/s over the requested time range. This
	 * force and rate will be returned as an `aspn_xtensor::MeasurementImu` with the imu type set to
	 * ASPN_MEASUREMENT_IMU_IMU_TYPE_SAMPLED. The
	 * frames are approximations of the frames midway between \p time1 and \p time2. If both \p
	 * time1 and \p time2 occurred within the same mechanization period, then result is same as
	 * `calc_force_and_rate_rate(time2)`. If either \p time1 or \p time2 are not within the
	 * available history of solutions, returns a `nullptr`.
	 */
	std::shared_ptr<aspn_xtensor::MeasurementImu> calc_force_and_rate(
	    const aspn_xtensor::TypeTimestamp& time1,
	    const aspn_xtensor::TypeTimestamp& time2) const override;

	/**
	 * Error params at time.
	 *
	 * @param t Time of request.
	 *
	 * @return Bias and scale factor values for accel and gyro supplied during the last reset prior
	 * to \p t, if available. If there is no such record, returns the IMU error parameter record
	 * nearest to \p t. If never explicitly provided via a reset, the return value will be the
	 * ImuErrors provided at construction of this instance.
	 */
	std::shared_ptr<ImuErrors> get_imu_errors(const aspn_xtensor::TypeTimestamp& t) const;

private:
	// Performs actual mechanization
	Inertial ins;

	// Buffers raw inertial inputs; a sorted container where the earlier times are in the front and
	// the later times are in the back.
	filtering::TimestampedDataSeries<aspn_xtensor::MeasurementImu, aspn_xtensor::TypeTimestamp>
	    imu_buf;

	// Stores IMU error parameters supplied during resets; a sorted container where the earlier
	// times are in the front and the later times are in the back.
	filtering::TimestampedDataSeries<ImuErrors, aspn_xtensor::TypeTimestamp> reset_err_buf;

	// User-provided expected delta time between IMU measurements. Used until estimated_dt() can
	// provide a usable estimate
	double expected_dt;

	// Use to calculate average in estimated_dt()
	double dt_sum;
	long num_dt;

	// Provides an estimate of the delta time between IMU measurements, calculated from
	// measurements presumed valid
	double estimated_dt() const;

	aspn_xtensor::MeasurementPositionVelocityAttitude dummy_pva;
};

}  // namespace inertial
}  // namespace navtk
