#include <navtk/navutils/navigation.hpp>

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#include <navtk/geospatial/sources/GeoidUndulationSource.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/wgs84.hpp>

namespace navtk {
namespace navutils {


double delta_lat_to_north(double delta_lat, double approx_lat, double altitude) {
	return (meridian_radius(approx_lat) + altitude) * delta_lat;
}


double delta_lon_to_east(double delta_lon, double approx_lat, double altitude) {
	return (transverse_radius(approx_lat) + altitude) * delta_lon * cos(approx_lat);
}


double east_to_delta_lon(double east_distance, double approx_lat, double altitude) {
	return east_distance / ((transverse_radius(approx_lat) + altitude) * cos(approx_lat));
}


double meridian_radius(double latitude) {
	auto sin_lat = sin(latitude);
	return SEMI_MAJOR_RADIUS * (1 - ECCENTRICITY_SQUARED) /
	       pow(1 - ECCENTRICITY_SQUARED * sin_lat * sin_lat, 1.5);
}

double north_to_delta_lat(double north_distance, double approx_lat, double altitude) {
	return north_distance / (meridian_radius(approx_lat) + altitude);
}


double transverse_radius(double latitude) {
	auto sin_lat = sin(latitude);
	return SEMI_MAJOR_RADIUS / pow(1 - ECCENTRICITY_SQUARED * sin_lat * sin_lat, 0.5);
}


std::pair<bool, double> geoid_minus_ellipsoid(double latitude,
                                              double longitude,
                                              const std::string& path) {
	return navtk::geospatial::GeoidUndulationSource::get_shared(path)->lookup_datum(latitude,
	                                                                                longitude);
}

std::pair<bool, double> hae_to_msl(double hae,
                                   double latitude,
                                   double longitude,
                                   const std::string& path) {
	auto undulation = geoid_minus_ellipsoid(latitude, longitude, path);
	if (undulation.first) return {true, hae - undulation.second};
	return undulation;
}

std::pair<bool, double> msl_to_hae(double msl,
                                   double latitude,
                                   double longitude,
                                   const std::string& path) {
	auto undulation = geoid_minus_ellipsoid(latitude, longitude, path);
	if (undulation.first) return {true, msl + undulation.second};
	return undulation;
}

}  // namespace navutils
}  // namespace navtk
