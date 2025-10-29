#include <navtk/factory.hpp>
#include <navtk/filtering/containers/MeasurementBuffer.hpp>
#include <navtk/utils/interpolation.hpp>

#include <spdlog/spdlog.h>
#include <xtensor/containers/xadapt.hpp>
#include <xtensor/generators/xbuilder.hpp>

using xt::vstack;
using xt::xtuple;

namespace navtk {
namespace filtering {

std::pair<bool, std::pair<std::vector<aspn_xtensor::TypeTimestamp>, Vector>>
MeasurementBuffer::get_measurements_around(aspn_xtensor::TypeTimestamp const& t_0,
                                           aspn_xtensor::TypeTimestamp const& t_1) const {
	if (covers_time(t_0) && covers_time(t_1)) {
		if (t_0 == t_1) {
			auto measurement = get_measurement(t_0);
			std::pair<std::vector<aspn_xtensor::TypeTimestamp>, Vector> ret(
			    {t_0, t_1}, {measurement.second, measurement.second});
			spdlog::info(
			    "MeasurementBuffer.get_measurements_around(): t_0 == t_1, returning the same value "
			    "for both times.");
			return {measurement.first, ret};
		}
		// Get the first measurement at time less or equal to t_0
		auto itr                                       = --data.upper_bound(t_0);
		std::vector<aspn_xtensor::TypeTimestamp> times = {itr->first};
		std::vector<double> measurements               = {itr->second.first};
		// Iterate until the measurement at time greater than or equal to t_1
		while (itr->first < t_1 && ++itr != data.end()) {
			times.push_back(itr->first);
			measurements.push_back(itr->second.first);
		}
		// Format into return type
		std::vector<std::size_t> shape = {times.size()};
		Vector ret_measurements        = xt::adapt(measurements, shape);
		return {true, {times, ret_measurements}};
	} else {
		std::pair<std::vector<aspn_xtensor::TypeTimestamp>, Vector> invalid_ret({t_0, t_1},
		                                                                        {0.0, 0.0});
		spdlog::info(
		    "No data is available at one or both of the requested times in "
		    "get_measurements_around().");
		return {false, invalid_ret};
	}
}

std::pair<bool, double> MeasurementBuffer::get_average_variance(
    aspn_xtensor::TypeTimestamp const& t_0, aspn_xtensor::TypeTimestamp const& t_1) const {
	if (covers_time(t_0) && covers_time(t_1)) {
		if (t_0 == t_1) {
			return get_covariance(t_0);
		}
		// Get iterator at time greater than or equal to t_0
		auto itr = data.upper_bound(t_0);
		// If iterator isn't at beginning, go back one and see if the previous value is at t_0
		if (itr != data.begin()) {
			--itr;
			if (itr->first != t_0) {
				++itr;
			}
		}
		double sum = 0;
		int cnt    = 0;
		while (itr != data.end() && itr->first <= t_1) {
			sum += itr->second.second;
			cnt += 1;
			++itr;
		}
		if (cnt == 0) {
			auto variance_0 = get_covariance(t_0);
			auto variance_1 = get_covariance(t_1);
			return {variance_0.first && variance_1.first,
			        (variance_0.second + variance_1.second) / 2.0};
		}
		return {true, sum / cnt};
	} else {
		spdlog::info(
		    "No data is available at one or both of the requested times in "
		    "get_average_variance().");
		return {false, 0.0};
	}
}

}  // namespace filtering
}  // namespace navtk
