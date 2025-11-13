#include <navtk/geospatial/GdalRaster.hpp>

#include <sstream>

#ifndef GDAL_INCLUDE_IN_SUBFOLDER
#	include <gdal_priv.h>
#	include <ogr_spatialref.h>
#else
#	include <gdal/gdal_priv.h>
#	include <gdal/ogr_spatialref.h>
#endif

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace geospatial {

using navtk::navutils::DEG2RAD;

GdalRaster::GdalRaster(const std::string& filename, const std::string& undulation_path)
    : filename(filename), undulation_path(undulation_path) {
	GDALAllRegister();
	GDALDataset* gdal_handle = (GDALDataset*)GDALOpen(filename.c_str(), GA_ReadOnly);

	if (gdal_handle != NULL) {
		valid = true;

		dataset = std::unique_ptr<GDALDataset, detail::DatasetDelete>{gdal_handle};
		dataset->GetGeoTransform(pixel_transform.data());
		wgs84_to_map_transformation = detail::create_wgs84_to_map_transformation(*dataset);
		map_to_wgs84_transformation = detail::create_map_to_wgs84_transformation(*dataset);

		auto raster_band    = dataset->GetRasterBand(1);
		this->no_data_value = raster_band->GetNoDataValue();

		size_x = dataset->GetRasterXSize();
		size_y = dataset->GetRasterYSize();
	} else
		spdlog::warn("Skipping {} because it is not a valid GDAL file.", filename);
}

bool GdalRaster::is_valid() const { return valid; }

int GdalRaster::get_width() const { return size_x; }

int GdalRaster::get_height() const { return size_y; }

std::pair<double, double> GdalRaster::wgs84_to_pixel(double latitude, double longitude) const {
	double x_geo = longitude;
	double y_geo = latitude;

	// Convert from wgs84 (lat/lon) to map coordinates (could be meters or other units, or remain as
	// lat/lon, in which case transform would not change x and y)
	wgs84_to_map_transformation->Transform(1, &x_geo, &y_geo);

	// Convert from map to pixel index using Cramer's rule on the two equations:
	// pixel_transform[1] * x_pixel + pixel_transform[2] * y_pixel = x_geo - pixel_transform[0]
	// pixel_transform[4] * x_pixel + pixel_transform[5] * y_pixel = y_geo - pixel_transform[3]

	// Cramer's rule solves equations in the form:
	// ax + by = e
	// cx + dy = f
	// **NOTE** substitution method is less clean and has negligible performance improvement,
	// linear algebra method is slightly cleaner but has significant performance drop

	// pixel_transform[0] = upper left x coordinate
	// pixel_transform[1] = pixel width (number of map units (e.g. meters) / pixel)
	// pixel_transform[2] = row rotation, will be 0 for north up images (virtually every dataset)
	// pixel_transform[3] = upper left y coordinate
	// pixel_transform[4] = column rotation, will be 0 for north up images (virtually every dataset)
	// pixel_transform[5] = pixel height

	double a = pixel_transform[1];
	double b = pixel_transform[2];
	double c = pixel_transform[4];
	double d = pixel_transform[5];
	double e = x_geo - pixel_transform[0];
	double f = y_geo - pixel_transform[3];

	double determinant = a * d - b * c;
	double x_pixel     = (e * d - b * f) / determinant;
	double y_pixel     = (a * f - e * c) / determinant;

	return {x_pixel, y_pixel};
}

std::pair<double, double> GdalRaster::pixel_to_wgs84(double x_pixel, double y_pixel) const {
	double x_geo = pixel_transform[0] + pixel_transform[1] * x_pixel + pixel_transform[2] * y_pixel;
	double y_geo = pixel_transform[3] + pixel_transform[4] * x_pixel + pixel_transform[5] * y_pixel;

	// Convert from map to wgs84 (lat/lon) coordinates
	map_to_wgs84_transformation->Transform(1, &x_geo, &y_geo);

	return {y_geo, x_geo};
}

void GdalRaster::scan_tile() {
	cached_tile = std::vector<double>(size_x * size_y, no_data_value);

	if (dataset->GetRasterBand(1)->RasterIO(
	        GF_Read, 0, 0, size_x, size_y, cached_tile.data(), size_x, size_y, GDT_Float64, 0, 0) !=
	    CPLE_None) {
		spdlog::warn("Unable to read tile.");
	}
}

double GdalRaster::read_pixel(size_t idx_x, size_t idx_y) {
	if (!cached) {
		scan_tile();
		cached = true;
	}

	if (idx_x >= size_x || idx_y >= size_y) {
		return no_data_value;
	}
	auto cache_index = idx_y * size_x + idx_x;
	return cached_tile[cache_index];
}

std::string GdalRaster::get_name() const { return filename; }

bool GdalRaster::is_valid_data(double data) const { return data != no_data_value; }

void GdalRaster::transform_tile(AspnMeasurementAltitudeReference prev_ref,
                                AspnMeasurementAltitudeReference new_ref) {
	std::pair<bool, double> (*transform)(double, double, double, const std::string&) = nullptr;
	if (prev_ref == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL &&
	    new_ref == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE) {
		transform = navtk::navutils::msl_to_hae;
	} else if (prev_ref == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE &&
	           new_ref == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL) {
		transform = navtk::navutils::hae_to_msl;
	} else {
		std::ostringstream error;
		error << "Invalid transformation. Cannot transform vertical reference from " << prev_ref
		      << " to " << new_ref << ".";
		log_or_throw<std::invalid_argument>("{}", error.str());
		return;
	}

	size_t idx = 0;
	for (size_t x = 0; x < size_x; x++) {
		for (size_t y = 0; y < size_y; y++) {
			// Note: for north-up tiles, latitude has one-to-one correspondence with a given
			// y_pixel offset (and longitude with x_pixel), but this is not the case for
			// non-north-up tiles, thus the need for a more complex pixel to wgs84 conversion.
			if (cached_tile[idx] != no_data_value) {
				auto coords    = pixel_to_wgs84(x, y);
				auto elevation = transform(cached_tile[idx],
				                           coords.first * DEG2RAD,
				                           coords.second * DEG2RAD,
				                           undulation_path);
				if (elevation.first) {
					cached_tile[idx] = elevation.second;
				} else {
					cached_tile[idx] = no_data_value;
				}
			}
			idx++;
		}
	}
}

void GdalRaster::unload() {
	if (cached) {
		cached_tile.clear();
		cached_tile.shrink_to_fit();
		cached = false;
	}
}
}  // namespace geospatial
}  // namespace navtk
