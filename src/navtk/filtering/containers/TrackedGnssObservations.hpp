#pragma once

#include <array>
#include <navtk/aspn.hpp>
#include <vector>

namespace navtk {
namespace filtering {

/**
 * Class used to keep track of a set of PRNs. Each time \p update is called the tracked
 * set is modified to reflect the PRNs in \p observation_prns. The class gives access to
 * the sets of PRNs that were added in the last update, removed in the last update, and
 * are currently being tracked (have been in \p observation_prns in the last
 * \p reset_prn_time seconds).
 */
class TrackedGnssObservations {
public:
	/** Default constructor */
	TrackedGnssObservations()
	    : tracked_prns_time_tag(33, aspn_xtensor::TypeTimestamp((int64_t)0)) {}

	/**
	 * Type declaration for a vector of PRNs.
	 */
	using Prns = std::vector<uint16_t>;

	/**
	 * Time limit in seconds for keep a PRN in tracking list when a measurement has not been
	 * received. If measurement is not received after `reset_prn_time`, the PRN is removed from
	 * the tracking list.
	 */
	double reset_prn_time = 5.0;

	/**
	 * Updates the lists based on the time and observations_prns.
	 *
	 * @param time Time of observation
	 * @param observation_prns Valid set of PRNs in observation
	 */
	void update(const aspn_xtensor::TypeTimestamp& time, const Prns& observation_prns);

	/**
	 * @return `true` if the list of tracked PRNs changed in the last update.
	 */
	bool changed() const;

	/**
	 * @return The list of PRNs being tracked as of the last update.
	 */
	const Prns& tracked() const;

	/**
	 * @return The list of PRNs added in the last update.
	 */
	const Prns& added() const;

	/**
	 * @return The list of PRNs removed in the last update.
	 */
	const Prns& removed() const;

private:
	/**
	 * The list of PRNs being tracked as of the last update.
	 */
	Prns tracked_prns;

	/**
	 * The list of PRNs added in the last update.
	 */
	Prns added_prns;

	/**
	 * The list of PRNs removed in the last update.
	 */
	Prns removed_prns;

	/**
	 * Tracks the time tag of the last measurement update for each PRN. The index in the
	 * Vector corresponds to the PRN number, i.e., PRN 32's last time tag will be located at
	 * track_current_prns_time_tag(32).
	 */
	std::vector<aspn_xtensor::TypeTimestamp> tracked_prns_time_tag;
};

}  // namespace filtering
}  // namespace navtk
