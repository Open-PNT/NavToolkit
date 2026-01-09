#include <navtk/filtering/stateblocks/EarthModel.hpp>

#include <cmath>

#include <navtk/tensors.hpp>

using std::cos;
using std::pow;
using std::sin;
using std::sqrt;

namespace navtk {
namespace filtering {

EarthModel::EarthModel(Vector3 pos, Vector3 vel, const GravityModel& gravity) {
	lat     = pos[0];
	alt_msl = pos[2];
	v_n     = vel[0];
	v_e     = vel[1];

	// Update Earth parameters.
	sin_l  = sin(lat);
	cos_l  = cos(lat);
	tan_l  = sin_l / cos_l;
	sec_l  = 1 / cos_l;
	sin_2l = sin(2 * lat);

	// Take into account earth is elliptical
	double ell = 1.0 - ECC_SQUARE * pow(sin_l, 2);
	// Titterton Weston Second Edition pg49 Equations 3.83, 3.84
	r_n    = RAD_E * (1.0 - ECC_SQUARE) / pow(ell, 1.5);
	r_e    = RAD_E / sqrt(ell);
	r_zero = sqrt(r_n * r_e);

	// Used to convert degrees lat/lon into meters.
	lat_factor = r_n + alt_msl;
	lon_factor = cos_l * (r_e + alt_msl);

	// Calculate transport rate, `omega_en`, in
	// the navigation frame. NOTE: This is a rate!
	omega_en_n =
	    Vector3{v_e / (r_e + alt_msl), -v_n / (r_n + alt_msl), -v_e * tan_l / (r_e + alt_msl)};

	// Calculate earth rate, `omega_ie`, in the navigation frame.
	// NOTE: This is a rate!
	omega_ie_n = Vector3{OMEGA_E * cos_l, 0.0, -OMEGA_E * sin_l};

	// Calculate turn rate of navigation frame with respect to inertial frame, `omega_in_n`.
	// NOTE: This is a rate!
	omega_in_n = omega_ie_n + omega_en_n;

	g_n = Vector3{0.0, 0.0, gravity.calculate_gravity(*this, alt_msl)};
}

}  // namespace filtering
}  // namespace navtk
