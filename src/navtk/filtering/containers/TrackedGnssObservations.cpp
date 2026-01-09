#include <navtk/filtering/containers/TrackedGnssObservations.hpp>

#include <algorithm>

namespace navtk {
namespace filtering {

void TrackedGnssObservations::update(const aspn_xtensor::TypeTimestamp& time,
                                     const Prns& observation_prns) {
	for (auto prn : observation_prns) tracked_prns_time_tag.at(prn) = time;

	Prns obs_prns = observation_prns;
	std::sort(obs_prns.begin(), obs_prns.end());

	// set the added prns
	added_prns.resize(obs_prns.size());
	auto added_it = std::set_difference(obs_prns.begin(),
	                                    obs_prns.end(),
	                                    tracked_prns.begin(),
	                                    tracked_prns.end(),
	                                    added_prns.begin());
	added_prns.resize(added_it - added_prns.begin());

	// partition the tracked prns into tracking/not-tracking (separated by removed_it)
	auto removed_it =
	    std::partition(tracked_prns.begin(), tracked_prns.end(), [&time, this](uint16_t prn) {
		    return (time.get_elapsed_nsec() - tracked_prns_time_tag.at(prn).get_elapsed_nsec()) *
		               1e-9 <
		           reset_prn_time;
	    });

	// set the removed prns
	removed_prns.clear();
	removed_prns.insert(removed_prns.end(), removed_it, tracked_prns.end());

	// remove the removed prns from the tracked prns
	tracked_prns.resize(removed_it - tracked_prns.begin());

	// add the added prns to the tracked prns
	tracked_prns.insert(tracked_prns.end(), added_prns.begin(), added_prns.end());
	std::sort(tracked_prns.begin(), tracked_prns.end());
}

bool TrackedGnssObservations::changed() const {
	return added_prns.size() > 0 || removed_prns.size() > 0;
}

const TrackedGnssObservations::Prns& TrackedGnssObservations::tracked() const {
	return tracked_prns;
}

const TrackedGnssObservations::Prns& TrackedGnssObservations::added() const { return added_prns; }

const TrackedGnssObservations::Prns& TrackedGnssObservations::removed() const {
	return removed_prns;
}
}  // namespace filtering
}  // namespace navtk
