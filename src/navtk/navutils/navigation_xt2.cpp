#include <navtk/navutils/navigation.hpp>

#include <cmath>

#include <xtensor/reducers/xnorm.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/wgs84.hpp>
#include <navtk/utils/ValidationContext.hpp>

using navtk::utils::ValidationContext;
using xt::transpose;
using xt::view;
using xt::xrange;

namespace navtk {
namespace navutils {

Matrix3 wander_to_C_enu_to_n(double wander) {
	auto cosa = cos(wander);
	auto sina = sin(wander);
	return Matrix3{{cosa, sina, 0}, {-sina, cosa, 0}, {0, 0, 1}};
}

Matrix3 wander_to_C_ned_to_n(double wander) {
	auto cosa = cos(wander);
	auto sina = sin(wander);
	return Matrix3{{sina, cosa, 0}, {cosa, -sina, 0}, {0, 0, -1}};
}

Matrix3 wander_to_C_ned_to_l(double wander) { return transpose(wander_to_C_enu_to_n(wander)); }

// 4.4.2.1-2 Checks out
Matrix3 lat_lon_wander_to_C_n_to_e(double lat, double lon, double wander) {
	Matrix3 C_n_to_e = zeros(3, 3);

	auto c_lon   = cos(lon);
	auto c_lat   = cos(lat);
	auto c_alpha = cos(wander);
	auto s_lon   = sin(lon);
	auto s_lat   = sin(lat);
	auto s_alpha = sin(wander);

	// From Section 3.2.3.3 Savage Vol. 1
	C_n_to_e(0, 0) = c_lon * c_alpha - s_lon * s_lat * s_alpha;
	C_n_to_e(0, 1) = -c_lon * s_alpha - s_lon * s_lat * c_alpha;
	C_n_to_e(0, 2) = s_lon * c_lat;

	C_n_to_e(1, 0) = c_lat * s_alpha;
	C_n_to_e(1, 1) = c_lat * c_alpha;
	C_n_to_e(1, 2) = s_lat;

	C_n_to_e(2, 0) = -s_lon * c_alpha - c_lon * s_lat * s_alpha;
	C_n_to_e(2, 1) = s_lon * s_alpha - c_lon * s_lat * c_alpha;
	C_n_to_e(2, 2) = c_lon * c_lat;

	return C_n_to_e;
}

Vector3 C_n_to_e_to_lat_lon_wander(const Matrix &C_n_to_e) {

	auto lat = atan2(C_n_to_e(1, 2), sqrt(pow(C_n_to_e(1, 0), 2) + pow(C_n_to_e(1, 1), 2)));

	// Lon and wander azimuth undefined at the poles, so set them to zero since they are arbitrary
	// Originally tested for elements == 0, but cos(PI/2) != 0... so switching to
	// an arbitrarily chosen 1/10 deg threshold
	// TODO PNTOS-257 Need a more well-defined range boundary for 'close to the poles'
	const double deg_by_10 = 0.00174533;
	double lon             = 0.0;
	auto c_last_el         = C_n_to_e(2, 2);
	if (c_last_el > deg_by_10 || c_last_el < -deg_by_10) lon = atan2(C_n_to_e(0, 2), c_last_el);

	auto wander = C_n_to_e_to_wander(C_n_to_e);

	return Vector3{lat, lon, wander};
}

// 4.4.2.1-3 Checks out
double C_n_to_e_to_wander(const Matrix3 &C_n_to_e) {
	// Assuming same latitude based checks apply here as in C_n_to_e_to_lat_lon_wander
	double deg_by_10 = 0.00174533;
	double wander    = 0.0;
	auto c_mid_el    = C_n_to_e(1, 1);
	if (c_mid_el > deg_by_10 || c_mid_el < -deg_by_10) wander = atan2(C_n_to_e(1, 0), c_mid_el);
	return wander;
}

std::pair<Matrix3, double> ecef_wander_to_C_n_to_e_h(const Vector3 &ecef_pos, double wander) {
	Vector3 llh      = ecef_to_llh(ecef_pos);
	Matrix3 C_n_to_e = lat_lon_wander_to_C_n_to_e(llh[0], llh[1], wander);
	return {C_n_to_e, llh[2]};
}

Vector3 C_n_to_e_h_to_llh(const Matrix3 &C_n_to_e, double h) {
	Vector3 ll = C_n_to_e_to_lat_lon_wander(C_n_to_e);
	ll[2]      = h;
	return ll;
}

Vector3 C_n_to_e_h_to_ecef(const Matrix3 &C_n_to_e, double h) {
	// Eqn 5.1-10 in Savage - slightly modified as u_U_p_E = CEN * [0; 0; 1];
	// in other words the last column of CEN
	auto rs_prime = SEMI_MAJOR_RADIUS / sqrt(1.0 + C_n_to_e(1, 2) * C_n_to_e(1, 2) * (OMF2 - 1.0));

	// Eqn 5.2.2-1 in Savage
	auto r_xe = C_n_to_e(0, 2) * (rs_prime + h);
	auto r_ye = C_n_to_e(1, 2) * (OMF2 * rs_prime + h);
	auto r_ze = C_n_to_e(2, 2) * (rs_prime + h);

	// Convert from E frame to ECEF frame
	return Vector3{r_ze, r_xe, r_ye};
}

Matrix3 C_ecef_to_e() { return Matrix3{{0, 1, 0}, {0, 0, 1}, {1, 0, 0}}; }

Matrix3 rot_vec_to_dcm(const Vector3 &phi) {

	/*
	// 'Normal' implementation. Long chains of xtensor related operations can cause large slowdowns,
	// especially w/ ASAN testing, so actual implementation does math 'manually' to achieve speed.
	 double phi_mag   = xt::norm_l2(phi)[0];
	 double phi_mag2  = phi_mag * phi_mag;
	 double phi_mag4  = phi_mag2 * phi_mag2;
	 double term1     = 1 - phi_mag2 / 6 + phi_mag4 / 120;
	 double term2     = 0.5 - phi_mag2 / 24 + phi_mag4 / 720;
	 auto phi_cross = skew(phi);
	 return eye(3) + term1 * phi_cross + term2 * dot(phi_cross, phi_cross);
	 */

	auto p0         = phi[0];
	auto p1         = phi[1];
	auto p2         = phi[2];
	double phi_mag  = sqrt(p0 * p0 + p1 * p1 + p2 * p2);
	double phi_mag2 = phi_mag * phi_mag;
	double phi_mag4 = phi_mag2 * phi_mag2;
	double t1       = 1 - phi_mag2 / 6 + phi_mag4 / 120;
	double t2       = 0.5 - phi_mag2 / 24 + phi_mag4 / 720;

	return {{1.0 + t2 * (-p2 * p2 - p1 * p1), -t1 * p2 + t2 * p1 * p0, t1 * p1 + t2 * p0 * p2},
	        {t1 * p2 + t2 * p0 * p1, 1.0 + t2 * (-p2 * p2 - p0 * p0), -t1 * p0 + t2 * p1 * p2},
	        {-t1 * p1 + t2 * p0 * p2, t1 * p0 + t2 * p1 * p2, 1.0 + t2 * (-p1 * p1 - p0 * p0)}};
}

}  // namespace navutils
}  // namespace navtk
