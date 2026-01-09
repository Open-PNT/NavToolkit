#include <navtk/navutils/gravity.hpp>

#include <functional>

#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/wgs84.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace navutils {

Vector3 calculate_gravity_titterton(double alt, double lat, double R0) {
	auto sin_lat  = sin(lat);
	auto sin2_lat = sin_lat * sin_lat;

	// Equations in this function from Titterton & Weston, Strapdown Inertial Navigation
	// Technology, 2nd Ed.

	// Eqn 3.89 - parameters come from 1967 Earth model
	auto g0  = 9.780318 * (1 + 5.3024e-3 * sin2_lat - 5.9e-6 * pow((sin(2 * lat)), 2));
	double g = 0.0;
	if (alt >= 0)
		// Eqn 3.91
		g = g0 / pow((1 + alt / R0), 2);
	else
		g = g0 * (1 + alt / R0);
	return Vector3{0.0, 0.0, g};
}

Vector3 calculate_gravity_schwartz(double alt, double lat) {
	// Apply gravity model from K.P. Schwartz ENGO 623 Course Notes (University of Calgary)
	// This is the GRS80 gravity model from
	// http://geoweb.mit.edu/~tah/12.221_2005/grs80_corr.pdf
	// combined with the height variation correction given in
	// https://archive.org/details/HeiskanenMoritz1967PhysicalGeodesy/page/n87
	// eq 2-123
	const double a1 = 9.7803267715;
	const double a2 = 0.0052790414;
	const double a3 = 0.0000232718;
	const double a4 = -3.0876910891E-6;
	const double a5 = 4.3977311E-9;
	const double a6 = 7.211E-13;
	auto sin_lat    = sin(lat);
	auto sin2_lat   = sin_lat * sin_lat;
	auto sin4_lat   = sin2_lat * sin2_lat;

	double g = 0.0;
	if (alt >= 0)
		g = a1 * (1 + a2 * sin2_lat + a3 * sin4_lat) + (a4 + a5 * sin2_lat) * alt + a6 * alt * alt;
	else {
		// Original does not account for negative altitudes- implement
		// linear scaling of gravity at altitude 0 as recommended in Savage
		// 5.4-2 and done elsewhere (T+W)
		auto g0  = a1 * (1 + a2 * sin2_lat + a3 * sin4_lat);
		auto R_N = meridian_radius(lat);
		auto R_E = transverse_radius(lat);
		auto R_0 = sqrt(R_N * R_E);
		g        = g0 * (1 + alt / R_0);
	}
	return Vector3{0.0, 0.0, g};
}

/*
 * Helper function for the savage n frame gravity model that calculates gravity
 * components perpendicular and parallel to the geocentric ellipsoidal position
 * vector.
 *
 * @param r Position vector from center of earth to vehicle location, in E frame.
 * If ellipsoidal altitude is less than 0, this should be the position vector
 * from the center of the earth to the point where r intersects the ellipsoidal
 * surface (see Rs variable, Savage, Vol 1 eq 5.2.1-4).
 * @param r2 r squared.
 * @param cos_phi Cosine of the angle from earth's positive polar rotation axis to R (kind of like
 * latitude measured from North pole spanning 0 to 180 degrees).
 *
 * @return Pair consisting of: gravitational mass attraction along R vector
 * (positive away from center of earth) and gravitational mass attraction component
 * perpendicular to R in the direction of the positive polar axis. These values are
 * similar to, but not exactly equal to gravity components in local level up and north;
 * see Savage vol 1 eq. 5.4-3 for relationship between the two.
 */
std::pair<double, double> gravitational_attraction(double r, double r2, double cos_phi) {
	// Equations in this function are from Savage, Strapdown Analytics, 2nd Ed.
	// With additional J4 expansion taken from Britting eq 4-28 and 4-29
	// Values of J terms from GRS80 model

	auto cos2_phi = cos_phi * cos_phi;

	auto r0_r  = SEMI_MAJOR_RADIUS / r;
	auto r0_r2 = r0_r * r0_r;

	// Table 5.6-1 in Savage; updated to include J4 term from GRS80
	const auto J2 = 1.08263e-3;
	const auto J3 = -2.5327E-6;
	const auto J4 = -2.3709122e-6;  // From GRS80; Britting gives -1.8e-6, but these values were
	                                // still being actively determined

	// Eqn 5.4-1 (a)
	auto temp1 = 1.5 * J2 * r0_r2 * (3.0 * cos2_phi - 1.0);
	auto temp2 = 2.0 * J3 * r0_r2 * r0_r * cos_phi * (5.0 * cos2_phi - 3.0);
	auto temp3 = 5.0 / 8.0 * J4 * r0_r2 * r0_r2 * (35 * cos2_phi * cos2_phi - 30 * cos2_phi + 3);
	auto gr    = -MU / r2 * (1.0 - temp1 - temp2 - temp3);

	// Eqn 5.4-1 (b)
	temp1            = 3.0 * MU * r0_r2 / r2;
	temp2            = J3 * r0_r * (5.0 * cos2_phi - 1) / 2.0;
	temp3            = 5.0 / 6.0 * J4 * r0_r2 * cos_phi * (7 * cos2_phi - 3.0);
	auto gphi_sinphi = temp1 * (J2 * cos_phi + temp2 + temp3);

	return {gr, gphi_sinphi};
}

Vector3 calculate_gravity_savage_n(const Matrix& C_n_to_e, double h) {
	const auto R0 = SEMI_MAJOR_RADIUS;

	// Equations in this function are from Savage, Strapdown Analytics, 2nd Ed.
	// Eqn 5.3-16
	auto u_up_ye  = C_n_to_e(1, 2);  // aka sin(lat)
	auto u_up_ye2 = u_up_ye * u_up_ye;

	// Eqn 5.1-10
	auto rs_prime = R0 / sqrt(1.0 + u_up_ye2 * (OMF2 - 1.0));

	// 5.2.1-4
	auto rs = rs_prime * sqrt(1.0 + u_up_ye2 * (OMF4 - 1.0));

	// 5.2.1-5
	auto r2 = rs * rs + 2.0 * h * R0 * R0 / rs_prime + h * h;
	auto r  = sqrt(r2);

	// Eqn 5.2.2-3
	auto cos_phi = u_up_ye * (OMF2 * rs_prime + h) / r;

	double gr          = 0.0;
	double gphi_sinphi = 0.0;
	if (h >= 0.0) {
		auto temp_res = gravitational_attraction(r, r2, cos_phi);
		gr            = temp_res.first;
		gphi_sinphi   = temp_res.second;
	} else {
		// Eqn 5.4-2
		auto temp_res     = gravitational_attraction(rs, rs * rs, cos_phi);
		auto grs          = temp_res.first;
		auto gphi_sinphis = temp_res.second;
		gr                = r * grs / rs;
		gphi_sinphi       = r * gphi_sinphis / rs;
	}

	// Eqn 5.2.2-5
	auto sinphi_sqrt = (rs_prime + h) / r;

	// Eqn 5.2.3-5
	auto sindl_sqrt = u_up_ye * (1.0 - OMF2) * rs_prime / r;
	auto cosdl      = ((1.0 - u_up_ye2 * (1.0 - OMF2)) * rs_prime + h) / r;

	// Eqn 5.4-4
	auto g_up         = gr * cosdl - gphi_sinphi * sinphi_sqrt * sindl_sqrt * (1.0 - u_up_ye2);
	auto g_north_sqrt = -gphi_sinphi * sinphi_sqrt * cosdl - gr * sindl_sqrt;

	// Plumb-bob gravity
	auto we2 = ROTATION_RATE * ROTATION_RATE;

	auto gp_north_sqrt = g_north_sqrt - (rs_prime + h) * we2 * u_up_ye;
	auto gp_up         = g_up + (rs_prime + h) * we2 * (1.0 - u_up_ye2);

	// Eqn 5.4.1-11
	auto gp_xn = gp_north_sqrt * C_n_to_e(1, 0);
	auto gp_yn = gp_north_sqrt * C_n_to_e(1, 1);
	auto gp_zn = gp_up;

	return Vector3{gp_xn, gp_yn, gp_zn};
}

Vector3 calculate_gravity_savage_ned(const Matrix& C_n_to_e, double h) {
	auto g = calculate_gravity_savage_n(C_n_to_e, h);

	// Convert from N to NED frame
	auto wander     = C_n_to_e_to_wander(C_n_to_e);
	auto C_ned_to_n = wander_to_C_ned_to_n(wander);
	return dot(xt::transpose(C_ned_to_n), g);
}

}  // namespace navutils
}  // namespace navtk
