#include <navtk/filtering/stateblocks/GravityModelTittertonAndWeston.hpp>

#include <navtk/filtering/stateblocks/EarthModel.hpp>

using std::pow;

namespace navtk {
namespace filtering {

double GravityModelTittertonAndWeston::calculate_gravity(const EarthModel &earth_model,
                                                         double alt_msl) const {
	double g0 = 9.780318 *
	            (1 + 5.3024e-3 * pow(earth_model.sin_l, 2) - 5.9e-6 * pow(earth_model.sin_2l, 2));
	double r = 1 + alt_msl / earth_model.r_zero;
	if (alt_msl >= 0.0)
		return g0 / pow(r, 2);
	else
		return g0 * r;
}

}  // namespace filtering
}  // namespace navtk
