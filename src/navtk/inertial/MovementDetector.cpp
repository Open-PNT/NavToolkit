#include <navtk/inertial/MovementDetector.hpp>

#include <map>
#include <tuple>
#include <utility>

#include <navtk/errors.hpp>

using aspn_xtensor::to_seconds;

namespace navtk {
namespace inertial {

void MovementDetector::add_plugin(const std::string& id,
                                  not_null<std::shared_ptr<MovementDetectorPlugin>> plugin,
                                  const double weight,
                                  const double stale_time) {
	if (mp.count(id) > 0) {
		log_or_throw<std::invalid_argument>("Cannot add plugin; one with id {} already exists", id);
		return;
	}
	if (weight <= 0) {
		log_or_throw<std::invalid_argument>(
		    "Cannot add plugin; weight for plugin {} must be positive; got {}", id, weight);
		return;
	}
	if (stale_time < 0) {
		log_or_throw<std::invalid_argument>(
		    "Cannot add plugin; stale_time for plugin {} must not be negative; got {}",
		    id,
		    stale_time);
		return;
	}
	mp.insert({id, FullPluginStat{MovementDetectorPluginStat{weight, stale_time}, plugin}});
	keys.clear();
	std::transform(
	    mp.begin(), mp.end(), std::back_inserter(keys), [](const auto& d) { return d.first; });
}

void MovementDetector::remove_plugin(const std::string& id) {
	if (mp.count(id) > 0) {
		mp.erase(id);
		keys.clear();
		std::transform(mp.cbegin(), mp.cend(), std::back_inserter(keys), [](const auto& d) {
			return d.first;
		});
	} else {
		log_or_throw<std::invalid_argument>("Cannot remove plugin with id {}", id);
	}
}

MovementStatus MovementDetector::get_status() {
	if (mp.empty()) {
		return MovementStatus::INVALID;
	}

	std::vector<aspn_xtensor::TypeTimestamp> times;
	for (auto k = mp.cbegin(); k != mp.cend(); k++) {
		times.push_back(k->second.plugin->get_time());
	}
	auto latest_time = *std::max_element(times.cbegin(), times.cend());

	std::map<MovementStatus, double> scores;
	for (auto k = mp.cbegin(); k != mp.cend(); k++) {
		auto status = k->second.plugin->get_status();
		if (status != MovementStatus::INVALID &&
		    (latest_time.get_elapsed_nsec() - k->second.plugin->get_time().get_elapsed_nsec()) *
		            1e-9 <=
		        k->second.stats.stale_time) {
			scores[status] += k->second.stats.weight;
		}
	}

	// All stale/INVALID
	if (scores.empty()) {
		return MovementStatus::INVALID;
	}

	auto max_ele =
	    std::max_element(scores.cbegin(), scores.cend(), [](const auto& lhs, const auto& rhs) {
		    return lhs.second < rhs.second;
	    });
	auto max_val = (*max_ele).second;

	// Have to check for ties
	auto num_max = std::count_if(scores.cbegin(), scores.cend(), [max_val](const auto& d) {
		return std::abs(d.second - max_val) / max_val < 1e-2;
	});
	if (num_max > 1) {
		return MovementStatus::POSSIBLY_MOVING;
	}
	return (*max_ele).first;
}

std::unordered_map<std::string, MovementDetectorPluginStat> MovementDetector::plugin_info() const {
	std::unordered_map<std::string, MovementDetectorPluginStat> out;
	for (auto k = mp.cbegin(); k != mp.cend(); k++) {
		out.insert({k->first, k->second.stats});
	}
	return out;
}

void MovementDetector::process(not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) {
	process(keys, data);
}

void MovementDetector::process(const std::vector<std::string>& ids,
                               not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) {
	for (auto id = ids.cbegin(); id != ids.cend(); id++) {
		if (mp.count(*id) > 0) {
			mp[*id].plugin->process(data);
		} else {
			log_or_throw<std::invalid_argument>("No plugin named {}", *id);
		}
	}
}

}  // namespace inertial
}  // namespace navtk
