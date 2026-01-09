#include <memory>
#include <utility>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <navtk/navutils/derivatives.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/leverarms.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/quaternions.hpp>
#include <navtk/navutils/wgs84.hpp>
#include <navtk/not_null.hpp>

#include "binding_helpers.hpp"

namespace nav = navtk::navutils;

using namespace pybind11::literals;
using nav::GravModels;
using navtk::not_null;

void add_navutils_functions(pybind11::module& m) {
	m.doc() = "General utilities for frames, rotations, and other navigation quantities";

	NAMESPACE_FUNCTION(d_cen_wrt_lat, nav, "lla"_a)
	NAMESPACE_FUNCTION(d_cen_wrt_lon, nav, "lla"_a)
	NAMESPACE_FUNCTION(d_cns_wrt_r, nav, "rpy"_a)
	NAMESPACE_FUNCTION(d_cns_wrt_p, nav, "rpy"_a)
	NAMESPACE_FUNCTION(d_cns_wrt_y, nav, "rpy"_a)
	NAMESPACE_FUNCTION(d_llh_to_ecef_wrt_llh, nav, "llh"_a)
	NAMESPACE_FUNCTION(d_ecef_to_llh_wrt_ecef, nav, "ecef"_a)
	NAMESPACE_FUNCTION(d_cne_wrt_k, nav, "dk"_a, "llh"_a)
	NAMESPACE_FUNCTION(
	    d_dcm_to_rpy, nav, "a"_a, "dadx"_a, "dady"_a, "dadz"_a, "b"_a, "dbdx"_a, "dbdy"_a, "dbdz"_a)
	NAMESPACE_FUNCTION(d_rpy_tilt_corr_wrt_tilt, nav, "tilts"_a, "C_nav_to_platform"_a)
	NAMESPACE_FUNCTION(d_rpy_correct_dcm_with_tilt_wrt_tilt, nav, "tilts"_a, "C_nav_to_platform"_a)
	NAMESPACE_FUNCTION(d_quat_prop_wrt_r, nav, "q"_a, "r"_a)
	NAMESPACE_FUNCTION(d_quat_tilt_corr_wrt_tilt, nav, "q"_a)
	NAMESPACE_FUNCTION(d_llh_to_quat_en_wrt_llh, nav, "llh"_a)
	NAMESPACE_FUNCTION(d_quat_to_rpy_wrt_q, nav, "q"_a)
	NAMESPACE_FUNCTION(d_quat_norm_wrt_q, nav, "q"_a)
	NAMESPACE_FUNCTION(d_ortho_dcm_wrt_tilt, nav, "C_nav_to_platform"_a, "tilts"_a, "dtilt"_a)
	NAMESPACE_FUNCTION(d_rpy_to_dcm_wrt_r, nav, "rpy"_a)
	NAMESPACE_FUNCTION(d_rpy_to_dcm_wrt_p, nav, "rpy"_a)
	NAMESPACE_FUNCTION(d_rpy_to_dcm_wrt_y, nav, "rpy"_a)
	NAMESPACE_FUNCTION(d_sensor_to_platform_pos_wrt_q, nav, "q_s_to_n"_a, "l_ps_p"_a, "C_p_to_s"_a)
	NAMESPACE_FUNCTION(d_platform_to_sensor_pos_wrt_q, nav, "q_p_to_n"_a, "l_ps_p"_a)

	ENUM(GravModels)
	CHOICE(GravModels, TITTERTON)
	CHOICE(GravModels, SCHWARTZ)
	CHOICE(GravModels, SAVAGE);
	NAMESPACE_FUNCTION(calculate_gravity_titterton, nav, "alt"_a, "lat"_a, "R0"_a)
	NAMESPACE_FUNCTION(calculate_gravity_schwartz, nav, "alt"_a, "lat"_a)
	NAMESPACE_FUNCTION(calculate_gravity_savage_n, nav, "C_n_to_e"_a, "h"_a)
	NAMESPACE_FUNCTION(calculate_gravity_savage_ned, nav, "C_n_to_e"_a, "h"_a)

	NAMESPACE_FUNCTION(sensor_to_platform,
	                   nav,
	                   "sensor_pose"_a,
	                   "platform_to_sensor_in_platform"_a,
	                   "C_platform_to_sensor"_a,
	                   "C_k_to_j"_a = navtk::eye(3))
	NAMESPACE_FUNCTION(platform_to_sensor,
	                   nav,
	                   "platform_pose"_a,
	                   "platform_to_sensor_in_platform"_a,
	                   "C_platform_to_sensor"_a,
	                   "C_k_to_j"_a = navtk::eye(3))

	NAMESPACE_FUNCTION(skew, nav, "angles"_a)
	NAMESPACE_FUNCTION(ortho_dcm, nav, "dcm"_a)
	NAMESPACE_FUNCTION(wrap_to_pi, nav, "orig"_a)
	NAMESPACE_FUNCTION(wrap_to_2_pi, nav, "orig"_a)

	NAMESPACE_FUNCTION(axis_angle_to_dcm, nav, "axis"_a, "angle"_a)
	NAMESPACE_FUNCTION(calc_van_loan, nav, "F"_a, "G"_a, "Q"_a, "dt"_a)
	NAMESPACE_FUNCTION(correct_dcm_with_tilt, nav, "dcm"_a, "tilt"_a)
	NAMESPACE_FUNCTION(dcm_to_quat, nav, "dcm"_a)
	NAMESPACE_FUNCTION(dcm_to_rpy, nav, "dcm"_a)
	NAMESPACE_FUNCTION(delta_lat_to_north, nav, "delta_lat"_a, "approx_lat"_a, "altitude"_a)
	NAMESPACE_FUNCTION(delta_lon_to_east, nav, "delta_lon"_a, "approx_lat"_a, "altitude"_a)
	NAMESPACE_FUNCTION(discretize_first_order, nav, "f"_a, "q"_a, "dt"_a)
	NAMESPACE_FUNCTION(discretize_second_order, nav, "f"_a, "q"_a, "dt"_a)
	NAMESPACE_FUNCTION(discretize_van_loan, nav, "f"_a, "q"_a, "dt"_a)
	NAMESPACE_FUNCTION(east_to_delta_lon, nav, "east_distance"_a, "approx_lat"_a, "altitude"_a)
	NAMESPACE_FUNCTION(ecef_to_cen, nav, "Pe"_a)
	NAMESPACE_FUNCTION(ecef_to_llh, nav, "Pe"_a)
	NAMESPACE_FUNCTION(ecef_to_local_level, nav, "P0e"_a, "Pe"_a)
	NAMESPACE_FUNCTION(llh_to_cen, nav, "Pwgs"_a)
	NAMESPACE_FUNCTION(llh_to_ecef, nav, "Pwgs"_a)
	NAMESPACE_FUNCTION(local_level_to_ecef, nav, "P0e"_a, "Pn"_a)
	NAMESPACE_FUNCTION(meridian_radius, nav, "latitude"_a)
	NAMESPACE_FUNCTION(north_to_delta_lat, nav, "north_distance"_a, "approx_lat"_a, "altitude"_a)
	NAMESPACE_FUNCTION(quat_to_dcm, nav, "quat"_a)
	NAMESPACE_FUNCTION(quat_to_rpy, nav, "quat"_a)
	NAMESPACE_FUNCTION(rpy_to_dcm, nav, "rpy"_a)
	NAMESPACE_FUNCTION(rpy_to_quat, nav, "rpy"_a)
	NAMESPACE_FUNCTION(transverse_radius, nav, "latitude"_a)
	NAMESPACE_FUNCTION(wander_to_C_enu_to_n, nav, "wander"_a)
	NAMESPACE_FUNCTION(wander_to_C_ned_to_n, nav, "wander"_a)
	NAMESPACE_FUNCTION(wander_to_C_ned_to_l, nav, "wander"_a)
	NAMESPACE_FUNCTION(lat_lon_wander_to_C_n_to_e, nav, "lat"_a, "lon"_a, "wander"_a)
	NAMESPACE_FUNCTION(C_n_to_e_to_lat_lon_wander, nav, "C_n_to_e"_a)
	NAMESPACE_FUNCTION(C_n_to_e_to_wander, nav, "C_n_to_e"_a)
	NAMESPACE_FUNCTION(ecef_wander_to_C_n_to_e_h, nav, "ecef_pos"_a, "wander"_a)
	NAMESPACE_FUNCTION(C_n_to_e_h_to_llh, nav, "C_n_to_e"_a, "h"_a)
	NAMESPACE_FUNCTION(C_n_to_e_h_to_ecef, nav, "C_n_to_e"_a, "h"_a)
	NAMESPACE_FUNCTION_VOID(C_ecef_to_e, nav)
	NAMESPACE_FUNCTION(rot_vec_to_dcm, nav, "phi"_a)

	NAMESPACE_FUNCTION(calculate_gravity_titterton, nav, "alt"_a, "lat"_a, "R0"_a)
	NAMESPACE_FUNCTION(calculate_gravity_schwartz, nav, "alt"_a, "lat"_a)
	NAMESPACE_FUNCTION(calculate_gravity_savage_n, nav, "C_n_to_e"_a, "h"_a)
	NAMESPACE_FUNCTION(calculate_gravity_savage_ned, nav, "C_n_to_e"_a, "h"_a)

	NAMESPACE_FUNCTION(geoid_minus_ellipsoid,
	                   nav,
	                   "latitude"_a,
	                   "longitude"_a,
	                   "path"_a = std::string("ww15mgh.grd"))
	NAMESPACE_FUNCTION(hae_to_msl,
	                   nav,
	                   "hae"_a,
	                   "latitude"_a,
	                   "longitude"_a,
	                   "path"_a = std::string("ww15mgh.grd"))
	NAMESPACE_FUNCTION(msl_to_hae,
	                   nav,
	                   "msl"_a,
	                   "latitude"_a,
	                   "longitude"_a,
	                   "path"_a = std::string("ww15mgh.grd"))

	NAMESPACE_FUNCTION(quat_conj, nav, "q"_a)
	NAMESPACE_FUNCTION(quat_norm, nav, "q"_a)
	NAMESPACE_FUNCTION(quat_mult, nav, "q"_a, "p"_a)
	NAMESPACE_FUNCTION(quat_rot, nav, "q"_a, "r"_a)
	NAMESPACE_FUNCTION(quat_prop, nav, "q"_a, "r"_a)
	NAMESPACE_FUNCTION(llh_to_quat_en, nav, "llh"_a)
	NAMESPACE_FUNCTION(correct_quat_with_tilt, nav, "q"_a, "t"_a)

	m.attr("ECCENTRICITY_SQUARED") = nav::ECCENTRICITY_SQUARED;
	m.attr("ECCENTRICITY")         = nav::ECCENTRICITY;
	m.attr("EQUATORIAL_GRAVITY")   = nav::EQUATORIAL_GRAVITY;
	m.attr("POLAR_GRAVITY")        = nav::POLAR_GRAVITY;
	m.attr("FLATTENING")           = nav::FLATTENING;
	m.attr("SEMI_MAJOR_RADIUS")    = nav::SEMI_MAJOR_RADIUS;
	m.attr("SEMI_MINOR_RADIUS")    = nav::SEMI_MINOR_RADIUS;
	m.attr("MU")                   = nav::MU;
	m.attr("ROTATION_RATE")        = nav::ROTATION_RATE;
	m.attr("OMF2")                 = nav::OMF2;
	m.attr("OMF4")                 = nav::OMF4;
}
