#pragma once

#include <memory>
#include <utility>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/TimestampedDataSeries.hpp>
#include <navtk/not_null.hpp>
#include <navtk/utils/Ordered.hpp>

/// @cond
namespace {

template <typename D, typename T>
bool safe_deref(const navtk::filtering::TimestampedDataSeries<D, T>& buf,
                typename navtk::utils::RingBuffer<std::shared_ptr<D>>::const_iterator& it) {
	return it != buf.cend() && *it != nullptr;
}

}  // namespace
/// @endcond

namespace navtk {
namespace inertial {

/**
 * Class that maintains a time history of and provides access to calculated PVAs. It is not meant to
 * be used directly but provides functionality for child classes to build off of.
 */
class BufferedPva {

public:
	virtual ~BufferedPva() = default;

	/**
	 * Copy constructor. Defaulted.
	 */
	BufferedPva(const BufferedPva&) = default;

	/**
	 * Copy assignment. Defaulted.
	 * @return `*this`
	 */
	BufferedPva& operator=(const BufferedPva&) = default;

	/**
	 * Move constructor. Defaulted.
	 */
	BufferedPva(BufferedPva&&) = default;

	/**
	 * Move assignment. Defaulted.
	 * @return `*this`
	 */
	BufferedPva& operator=(BufferedPva&&) = default;

	/**
	 * Child classes will need to have an implementation-specific data class added periodically in
	 * order to increase the time span that this buffer covers. This method provides a common
	 * interface to add data to child classes.
	 *
	 * @param data The data to add.
	 */
	virtual void add_data(not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) = 0;

	/**
	 * Get solution at time.
	 *
	 * @param time Requested solution time.
	 *
	 * @return Solution at time, or `nullptr` if Time is not within the available history of
	 * solutions.
	 */
	std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> calc_pva(
	    const aspn_xtensor::TypeTimestamp& time) const;

	/**
	 * Get most recent solution.
	 *
	 * @throw std::runtime_error If this instance has been 'moved' from.
	 *
	 * @return Most recent solution stored.
	 */
	not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> calc_pva() const;

	/**
	 * Get available time range of solutions.
	 *
	 * @return Pair of times (`first` being the earlier time and `second` being the later` time)
	 * indicating the range of time over which PVA is available. If there is no history of solutions
	 * available, returned times will both equal std::numeric_limits<double>::lowest(). If only one
	 * PVA resides in the buffer, then both times returned will be the time of that PVA.
	 */
	std::pair<aspn_xtensor::TypeTimestamp, aspn_xtensor::TypeTimestamp> time_span() const;

	/**
	 * Get available time range of solutions in a 64-bit represented integer, nanoseconds
	 *
	 * @return Pair of times (`first` being the earlier time and `second` being the later` time)
	 * indicating the range of time over which PVA is available. If there is no history of solutions
	 * available, returned times will both equal std::numeric_limits<double>::lowest(). If only one
	 * PVA resides in the buffer, then both times returned will be the time of that PVA.
	 */
	std::pair<int64_t, int64_t> nsec_time_span() const;

	/**
	 * Determines if a time is within the available history of solutions, which is a prerequisite
	 * for most operations.
	 *
	 * @param t Time to check.
	 *
	 * @return True if \p t lies within the stored history of solutions. This return is invalidated
	 * by
	 * a call to mechanize().
	 */
	bool in_range(const aspn_xtensor::TypeTimestamp& t) const;

	/**
	 * Get the average specific force and inertial sensor frame rotation rate between two solutions
	 * close to the requested time.
	 *
	 * @param time Requested time of force and rate.
	 *
	 * @return If time is within the available history of solutions, the specific force in the NED
	 * frame, in m/s^2, and inertial sensor frame rotation rate in inertial sensor frame in rad/s.
	 * Otherwise, a `nullptr`. This force and rate will be returned as an
	 * `aspn_xtensor::MeasurementImu` with the imu type set to
	 * ASPN_MEASUREMENT_IMU_IMU_TYPE_SAMPLED. If \p time is in between two solutions, then the force
	 * and rate will be calculated from the two bordering solutions. Otherwise, if \p time matches a
	 * solution's time, it uses the two neighboring PVAs to calculate the force and rate. If only
	 * one neighboring PVA exists, then the other one is interpolated from the matched PVA and the
	 * existing neighbor. If neither neighboring PVA exists, then a nullptr is returned.
	 */
	virtual std::shared_ptr<aspn_xtensor::MeasurementImu> calc_force_and_rate(
	    const aspn_xtensor::TypeTimestamp& time) const;

	/**
	 * Get average specific force and rotation rate between the solutions at two times.
	 *
	 * @param time1 Start time of range, inclusive.
	 * @param time2 Stop time of range, inclusive.
	 *
	 * @return Average specific force in the NED frame in m/s^2, and average inertial sensor frame
	 * rotation rate, in the inertial sensor frame in rad/s over the requested time range. This
	 * force and rate will be returned as an `aspn_xtensor::MeasurementImu` with the imu type set to
	 * ASPN_MEASUREMENT_IMU_IMU_TYPE_SAMPLED. The frames are approximations of the frames  midway
	 * between time1 and time2. If either time1 or time2 are not within the available history of
	 * solutions, returns a `nullptr`.
	 */
	virtual std::shared_ptr<aspn_xtensor::MeasurementImu> calc_force_and_rate(
	    const aspn_xtensor::TypeTimestamp& time1, const aspn_xtensor::TypeTimestamp& time2) const;

protected:
	/**
	 * Constructor.
	 *
	 * @param pva Time-tagged initial position, velocity and attitude `nullptr` if unknown.
	 * @param expected_dt The typical delta time between PVAs, in seconds.
	 * @param buffer_length Amount of data, in seconds, this class should keep in history. Max
	 * number of accessible buffer elements is `buffer_length/expected_dt + 2`.
	 */
	BufferedPva(std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> pva = nullptr,
	            double expected_dt                                                     = 1.0,
	            double buffer_length                                                   = 60.0);

	/**
	 * Constructor.
	 *
	 * @param pva Time-tagged initial position, velocity and attitude.
	 * @param expected_dt The typical delta time between PVAs, in seconds.
	 * @param buffer_length Amount of data, in seconds, this class should keep in history. Max
	 * number of accessible buffer elements is `buffer_length/expected_dt + 2`.
	 */
	BufferedPva(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva,
	            double expected_dt   = 1.0,
	            double buffer_length = 60.0);

	/**
	 * Buffers solution history, with resets. In other words, this is a sorted container where the
	 * earlier times are in the front and the later times are in the back.
	 */
	filtering::TimestampedDataSeries<aspn_xtensor::MeasurementPositionVelocityAttitude,
	                                 aspn_xtensor::TypeTimestamp>
	    pva_buf;

	/**
	 * Allows calc_pva access to the 'extra' value on the PVA buffer by not checking if in_range(),
	 * which normally excludes this value when the buffer is full. This is useful for
	 * implementations which wrap an inertial since it enables proper repropagation from the first
	 * 'time_span' time in the case the underlying Inertial is using high-order mechanization
	 * methods that require a 'prior' PVA when resetting, such as Simpsons.
	 *
	 * @param time Requested solution time.
	 *
	 * @return Solution at time, or `nullptr` if Time is not within the available history of
	 * solutions.
	 */
	std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> calc_pva_no_check(
	    const aspn_xtensor::TypeTimestamp& time) const;
};

}  // namespace inertial
}  // namespace navtk
