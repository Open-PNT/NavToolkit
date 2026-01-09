#pragma once

#include <navtk/filtering/stateblocks/GravityModel.hpp>
#include <navtk/filtering/stateblocks/GravityModelSchwartz.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Contains constants describing the shape and movement of the planet Earth.
 *
 * References:
 *
 * Titterton, D. H., & Weston, J. L. (2004). Strapdown Inertial Navigation Technology. 2nd. London:
 * Institution of Electrical Engineers.
 */
struct EarthModel {
	/**
	 * Earth semi-major axis, (m).
	 */
	constexpr static double RAD_E = 6378137.0;

	/**
	 * Earth rate, (rad/s).
	 */
	constexpr static double OMEGA_E = 7.2921151467e-5;

	/**
	 * Flattening of Earth.
	 */
	constexpr static double F = 1 / 298.257223563;

	/**
	 * Eccentricity squared.
	 */
	constexpr static double ECC_SQUARE = F * (2 - F);

	/**
	 * Major eccentricity of the ellipsoid.
	 */
	const double ecc = sqrt(ECC_SQUARE);

	/**
	 * Maps the parameters to the states, calculating the remaining properties.
	 *
	 * @param pos The latitude-longitude-altitude position in rad-rad-meters.
	 * @param vel The north-east-down velocity in m/s.
	 * @param gravity A GravityModel.
	 */
	EarthModel(Vector3 pos, Vector3 vel, const GravityModel &gravity = GravityModelSchwartz());

	/**
	 * Latitude in radians.
	 */
	double lat;

	/**
	 * Altitude above mean sea level in meters.
	 */
	double alt_msl;

	/**
	 * Velocity in the North direction in m/s.
	 */
	double v_n;

	/**
	 * Velocity in the East direction in m/s.
	 */
	double v_e;

	/**
	 * The sine of #lat.
	 */
	double sin_l;

	/**
	 * The cosine of #lat.
	 */
	double cos_l;

	/**
	 * The tangent of #lat.
	 */
	double tan_l;

	/**
	 * The secant of #lat.
	 */
	double sec_l;

	/**
	 * The sine of two times #lat.
	 */
	double sin_2l;

	/**
	 * The meridian radius of curvature of the Earth in meters.
	 */
	double r_n;

	/**
	 * The transverse radius of curvature of the Earth in meters.
	 */
	double r_e;

	/**
	 * The mean radius of curvature of the Earth in meters.
	 */
	double r_zero;

	/**
	 * A conversion factor to convert between radians and meters in the North direction in m/rad.
	 * The change in meters is equal to the change in radians multiplied by #lat_factor. #lat_factor
	 * is a nonlinear function evaluated at a single point and is only valid for the given #lat and
	 * #alt_msl.
	 */
	double lat_factor;

	/**
	 * A conversion factor to convert between radians and meters in the East direction in m/rad. The
	 * change in meters is equal to the change in radians multiplied by #lon_factor. #lon_factor is
	 * a nonlinear function evaluated at a single point and is only valid for the given #lat and
	 * #alt_msl.
	 */
	double lon_factor;

	/**
	 * The transport rate, `omega_en`, in the navigation frame in rad/s.
	 */
	Vector3 omega_en_n;

	/**
	 * The earth rate, `omega_ie`, in the navigation frame in rad/s.
	 */
	Vector3 omega_ie_n;

	/**
	 * The turn rate of navigation frame with respect to inertial frame in rad/s.
	 */
	Vector3 omega_in_n;

	/**
	 * The NED force of gravity with respect to the navigation frame in m/s^2.
	 */
	Vector3 g_n;
};


}  // namespace filtering
}  // namespace navtk
