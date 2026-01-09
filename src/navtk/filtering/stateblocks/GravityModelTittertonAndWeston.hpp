#pragma once

#include <navtk/filtering/stateblocks/GravityModel.hpp>

namespace navtk {
namespace filtering {

/**
 * A model for calculating gravity near-Earth based on Titterton and Weston.
 */
struct GravityModelTittertonAndWeston : GravityModel {
	/**
	 * Determine the Earth's local gravity magnitude.
	 *
	 * @param earth_model Earth model object. Uses filtering::EarthModel::sin_l,
	 * filtering::EarthModel::sin_2l, and filtering::EarthModel::r_zero during calculations.
	 * @param alt_msl MSL Altitude (m).
	 *
	 * @return Gravity magnitude (m/s^2).
	 */
	double calculate_gravity(const EarthModel &earth_model, double alt_msl) const override;
};
}  // namespace filtering
}  // namespace navtk
