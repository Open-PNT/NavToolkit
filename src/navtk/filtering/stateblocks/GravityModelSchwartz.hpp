#pragma once

#include <navtk/filtering/stateblocks/GravityModel.hpp>

namespace navtk {
namespace filtering {
/**
 * A model for calculating gravity near-Earth based on Schwartz.
 */
struct GravityModelSchwartz : GravityModel {
public:
	/**
	 * Determine the Earth's local gravity magnitude.
	 *
	 * @param earth_model Earth model object. Uses filtering::EarthModel::sin_l during calculations.
	 * @param alt_msl MSL Altitude (m).
	 *
	 * @return Gravity magnitude (m/s^2).
	 */
	double calculate_gravity(const EarthModel &earth_model, double alt_msl) const override;

private:
	constexpr static double A1 = 9.7803267715;
	constexpr static double A2 = 0.0052790414;
	constexpr static double A3 = 0.0000232718;
	constexpr static double A4 = -3.0876910891e-6;
	constexpr static double A5 = 4.3977311e-9;
	constexpr static double A6 = 7.211e-13;
};
}  // namespace filtering
}  // namespace navtk
