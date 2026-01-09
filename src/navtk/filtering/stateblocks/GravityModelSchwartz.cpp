#include <navtk/filtering/stateblocks/GravityModelSchwartz.hpp>

#include <navtk/filtering/stateblocks/EarthModel.hpp>

using std::pow;

namespace navtk {
namespace filtering {

double GravityModelSchwartz::calculate_gravity(const EarthModel &earth_model,
                                               double alt_msl) const {
	double sin2_l = pow(earth_model.sin_l, 2);
	double sin4_l = pow(sin2_l, 2);

	return A1 * (1.0 + A2 * sin2_l + A3 * sin4_l) + (A4 + A5 * sin2_l) * alt_msl +
	       A6 * pow(alt_msl, 2);
}

}  // namespace filtering
}  // namespace navtk
