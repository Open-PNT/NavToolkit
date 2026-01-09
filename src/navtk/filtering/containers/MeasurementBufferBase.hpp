#pragma once

#include <vector>

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/interpolation.hpp>

namespace navtk {
namespace filtering {


/**
 * A base class for buffers that store and access measurements.
 */
template <typename TMeasurement, typename TCovariance>
class MeasurementBufferBase {
public:
	/**
	 * Buffer data map container
	 */
	using BufferContainer =
	    typename std::map<aspn_xtensor::TypeTimestamp, std::pair<TMeasurement, TCovariance>>;
	/**
	 * iterator for class
	 */
	using BufferIterator = typename BufferContainer::iterator;
	/**
	 * const iterator for class
	 */
	using ConstBufferIterator = typename BufferContainer::const_iterator;
	/**
	 * @return read/write iterator that points
	 * to the last pair in the map
	 */
	BufferIterator begin() { return data.begin(); }
	/**
	 * @return read/write iterator that points
	 * to the last pair in the map
	 */
	BufferIterator end() { return data.end(); }
	/**
	 * @return read only (constant) iterator that points
	 * to the first pair in the map
	 */
	ConstBufferIterator begin() const { return data.cbegin(); }
	/**
	 * @return read only (constant) iterator that points
	 * to the last pair in the map
	 */
	ConstBufferIterator end() const { return data.cend(); }
	/**
	 * MeasurementBufferBase destructor.
	 */
	virtual ~MeasurementBufferBase() = default;
	/**
	 * Check if there is any data in the buffer.
	 * @return Empty or not.
	 */
	bool is_empty() const { return data.empty(); };
	/**
	 * Add a measurement to the buffer. Sort the times and measurements if necessary.
	 * @param time Time at which the measurement was taken.
	 * @param measurement Measurement to store in buffer.
	 * @param covariance Covariance of measurement.
	 */
	void add_measurement(aspn_xtensor::TypeTimestamp const& time,
	                     TMeasurement const& measurement,
	                     TCovariance const& covariance) {
		auto last_time = get_last_time();
		if (last_time.first) {
			if (last_time.second >= time) {
				if (data.find(time) != data.end()) {
					spdlog::info(
					    "Received measurement at exact time already in buffer. Ignoring new "
					    "measurement.");
					return;
				} else {
					spdlog::info("Measurement out of order, sorting.");
				}
			}
		}
		data.insert({time, {measurement, covariance}});
	}
	/**
	 * Check whether the buffer contains a measurement at the given time, or can interpolate one
	 * from available data.
	 * @param time Time of interest.
	 * @return `true` if buffer contains time, or contains enough data to generate interpolated
	 * values for the given time.
	 */
	bool covers_time(aspn_xtensor::TypeTimestamp const& time) const {
		if (!data.empty()) {
			if (time >= data.begin()->first && time <= (--data.end())->first) {
				return true;
			}
		}
		return false;
	}
	/**
	 * Remove unecessary or used measurements in the buffer, making sure to leave some measurements
	 * in the buffer and not empty past a certain time.
	 * @param time Threshold time, no measurements past this time will be removed.
	 */
	void remove_old_measurements(aspn_xtensor::TypeTimestamp const& time) {
		if (!data.empty()) {
			auto itr = data.upper_bound(time);
			if (itr != data.begin()) {
				data.erase(data.begin(), --itr);
			}
		}
	}
	/**
	 * Get a measurement at time provided.
	 * Do linear interpolation if there is no measurement at that time, but there are available
	 * measurements around that time.
	 * @param time Time at which to return the measurement.
	 * @return Pair containing valid flag and interpolated measurement.
	 */
	std::pair<bool, TMeasurement> get_measurement(aspn_xtensor::TypeTimestamp const& time) const {
		if (covers_time(time)) {
			auto itr = data.lower_bound(time);
			if (itr->first == time) {
				return {true, itr->second.first};
			} else {
				const auto& second_measurement = itr->second.first;
				const auto& second_time        = itr->first;
				// (itr > data.begin()) is guaranteed by covers_time(time) check above
				--itr;
				const auto& first_measurement = itr->second.first;
				const auto& first_time        = itr->first;
				return {true,
				        utils::linear_interpolate(
				            first_time, first_measurement, second_time, second_measurement, time)};
			}
		} else {
			spdlog::info(
			    "Returning invalid measurement from get_measurement() because no data is available "
			    "at the requested time.");
			return {false, TMeasurement()};
		}
	}
	/**
	 * Get the covariance of measurements at the time provided.
	 * Do linear interpolation if there is no measurement at the requested time, but available
	 * measurements around that time.
	 * @param time Time of the requested covariance.
	 * @return Pair containing valid flag and interpolated covariance.
	 */
	std::pair<bool, TCovariance> get_covariance(aspn_xtensor::TypeTimestamp const& time) const {
		if (covers_time(time)) {
			auto itr = data.lower_bound(time);
			if (itr->first == time) {
				return {true, itr->second.second};
			} else {
				auto second_covariance = itr->second.second;
				auto second_time       = itr->first;
				--itr;
				auto first_covariance = itr->second.second;
				auto first_time       = itr->first;
				return {true,
				        utils::linear_interpolate(
				            first_time, first_covariance, second_time, second_covariance, time)};
			}
		} else {
			spdlog::info(
			    "Returning invalid covariance from get_covariance() because no data is available "
			    "at the requested time.");
			return {false, TCovariance()};
		}
	}
	/**
	 * Get the latest time in the buffer.
	 * @return Valid flag, and the latest time in the buffer. (Valid flag is false if buffer is
	 * empty.)
	 */
	std::pair<bool, aspn_xtensor::TypeTimestamp> get_last_time() const {
		if (!data.empty()) {
			return {true, (--data.end())->first};
		}
		return {false, aspn_xtensor::TypeTimestamp((int64_t)0)};
	}

	/**
	 * Get a vector of all the aspn_xtensor::TypeTimestamp instances in the buffer.
	 * @return std::vector of times in the buffer. If the buffer is empty, returns empty vector.
	 */
	std::vector<aspn_xtensor::TypeTimestamp> get_times() const {
		std::vector<aspn_xtensor::TypeTimestamp> times(data.size(),
		                                               aspn_xtensor::TypeTimestamp((int64_t)0));
		size_t idx = 0;
		for (const auto& measurement : data) times[idx++] = measurement.first;
		return times;
	}

	/**
	 * Empty contents of buffer entirely.
	 */
	void clear() { data.clear(); }

protected:
	/**
	 * stores the measurements.
	 */
	BufferContainer data;
};

}  // namespace filtering
}  // namespace navtk
