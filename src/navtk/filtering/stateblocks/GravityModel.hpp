#pragma once

#include <memory>

namespace navtk {
namespace filtering {

struct EarthModel;

/**
 * Model used to calculate gravity for a given earth model at a given altitude.
 */
struct GravityModel {
	/**
	 * Calculates the force of gravity.
	 *
	 * @param earth_model An instance of EarthModel.
	 * @param alt_msl The altitude above mean sea level in meters for which the force of gravity
	 *                should be calculated.
	 *
	 * @return The magnitude of the force of gravity in m/s^2.
	 */
	virtual double calculate_gravity(const EarthModel &earth_model, double alt_msl) const = 0;

	virtual ~GravityModel() = default;
};

}  // namespace filtering
}  // namespace navtk
