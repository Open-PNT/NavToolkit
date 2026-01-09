#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/MeasurementBufferBase.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A buffer for storing and accessing 1-dimensional measurements.
 */
class MeasurementBuffer : public MeasurementBufferBase<double, double> {
public:
	using MeasurementBufferBase::MeasurementBufferBase;
	/**
	 * Get all measurements between `t_0` and `t_1`, including the measurement at time before or
	 * equal to t_0, and the measurement at time after or equal t_1.
	 * @param t_0 First time.
	 * @param t_1 Second time.
	 * @return A nested pair containing a valid flag, and then a pair with the times corresponding
	 * to the measurements, and the measurements in the range.
	 */
	std::pair<bool, std::pair<std::vector<aspn_xtensor::TypeTimestamp>, Vector>>
	get_measurements_around(aspn_xtensor::TypeTimestamp const& t_0,
	                        aspn_xtensor::TypeTimestamp const& t_1) const;
	/**
	 * Return the average variance of measurements within a time interval.
	 * @param t_0 First time.
	 * @param t_1 Second time.
	 * @return A pair containing a valid flag and the average variance of all measurements that fall
	 * between or on the given time interval. If no measurements fall between or on the time
	 * interval, though measurements surround the time interval this will return the average of
	 * `get_variance(t_0)` and `get_variance(t_1)`.
	 */
	std::pair<bool, double> get_average_variance(aspn_xtensor::TypeTimestamp const& t_0,
	                                             aspn_xtensor::TypeTimestamp const& t_1) const;
};

}  // namespace filtering
}  // namespace navtk
