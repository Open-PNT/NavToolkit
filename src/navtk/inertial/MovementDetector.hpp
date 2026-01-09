#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <navtk/inertial/MovementDetectorPlugin.hpp>
#include <navtk/inertial/MovementStatus.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace inertial {

/**
 * Container to hold values related to MovementDetectorPlugins being queried as a collection.
 */
struct MovementDetectorPluginStat {
	/**
	 * Relative weight this plugin has compared to others.
	 */
	double weight = 0.0;

	/**
	 * Amount of time (in seconds) the related plugin can go without a measurement before its
	 * status is considered invalid.
	 */
	double stale_time = 0.0;
};

/**
 * Holds MovementDetectorPlugin objects and polls them to determine a MovementStatus consensus.
 */
class MovementDetector {

public:
	/**
	 * Add plugin to internal list of movement detection algorithm plugins.
	 *
	 * @param id Identifier for this plugin instance.
	 * @param plugin A movement-detecting plugin. It is recommended that the plugin gracefully
	 * handle unexpected data types to avoid errors in the process() function.
	 * @param weight Relative weight assigned to this plugin. Must be positive. See get_status() for
	 * how weights affect results.
	 * @param stale_time Amount of time (seconds) allowed to have elapsed between this plugin's
	 * MovementDetectorPlugin#get_time result and the latest MovementDetectorPlugin#get_time result
	 * from all plugins before treating the status from this plugin as temporarily invalid.
	 *
	 * @throw std::invalid_argument if `weight <= 0`, or `stale_time < 0`.
	 */
	void add_plugin(const std::string& id,
	                not_null<std::shared_ptr<MovementDetectorPlugin>> plugin,
	                const double weight     = 1.0,
	                const double stale_time = 1.0);

	/**
	 * Remove plugin by id/key.
	 *
	 * @param id The id of the plugin to be removed.
	 *
	 * @throw std::invalid_argument If no plugin tagged with \p id is present and
	 * GLOBAL_ERROR_MODE is ErrorMode::DIE.
	 *
	 */
	void remove_plugin(const std::string& id);

	// Formatting breaks in-docs link to other function version
	// clang-format off
	/**
	 * Attempt to process the provided data with all available plugins. If any plugin is likely to
	 * react poorly upon receiving unexpected data types or multiple data sources of the same data
	 * type need to be routed to specific plugins, use
	 * MovementDetector::process(const std::vector<std::string>&, not_null<std::shared_ptr<aspn_xtensor::AspnBase>>).
	 *
	 * @param data Data to process.
	 */
	// clang-format on
	void process(not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data);

	/**
	 * Process the provided data only with specified plugins.
	 *
	 * @param ids All ids for the plugins to pass \p data to.
	 * @param data Data for the specified algorithm.
	 *
	 * @throw std::invalid_argument If any of the plugins referenced by \p ids are not available,
	 * and the global error mode is ErrorMode::DIE (accesible with navtk::get_global_error_mode()).
	 */
	void process(const std::vector<std::string>& ids,
	             not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data);

	/**
	 * Poll all plugins and calculate the current status consensus.
	 *
	 * @return Estimated status. The returned status is determined by selecting the status that has
	 * the maximum weight assigned after removing plugins that are stale or return INVALID. In
	 * other words
	 * \f$ status = \max(\sum \omega_{MOVING}, \sum\omega_{NOT\_MOVING},
	 * \sum\omega_{POSSIBLY\_MOVING}) \f$. In the case of ties MovementStatus::POSSIBLY_MOVING is
	 * returned. MovementStatus::INVALID is returned only if all plugins return an INVALID status or
	 * are deemed stale, or no plugins have been supplied.
	 */
	MovementStatus get_status();

	/**
	 * Get information on existing plugins.
	 *
	 * @return Map keyed by plugin id containing info. See MovementDetectorPluginStat.
	 */
	std::unordered_map<std::string, MovementDetectorPluginStat> plugin_info() const;

private:
	// Container for storing plugin and assorted info
	struct FullPluginStat {
		MovementDetectorPluginStat stats;
		std::shared_ptr<MovementDetectorPlugin> plugin;
	};

	// Storage for plugins and related data.
	std::unordered_map<std::string, FullPluginStat> mp;

	// Extracted keys from plugin map to support no-id process() function
	std::vector<std::string> keys;
};

}  // namespace inertial
}  // namespace navtk
