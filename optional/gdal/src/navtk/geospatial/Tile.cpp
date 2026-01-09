#include <navtk/geospatial/Tile.hpp>

#include <cmath>
#include <iostream>

#ifndef GDAL_INCLUDE_IN_SUBFOLDER
#	include <ogr_spatialref.h>
#else
#	include <gdal/ogr_spatialref.h>
#endif

#include <spdlog/spdlog.h>
#include <navtk/geospatial/ElevationInterpolator.hpp>

namespace navtk {
namespace geospatial {

Tile::Tile(std::shared_ptr<Raster> raster) : raster(raster) {}

std::pair<bool, double> Tile::lookup_datum(double latitude, double longitude) const {
	/* Offset is the decimal distance from top left of tile, convert to int to get pixel index.
	 Given the point (pixel_idx.x, pixel_idx.y) = (floor(tile_offset.first),
	 floor(tile_offset.second)), get the 4 closest pixel indices and interpolate to
	 find the elevation difference at (latitude, longitude)

	 (idx.x,idx.y)***********(idx.x + 1,idx.y)
	 *****************************************
	 *****************************************
	 *****************************************
	 *****************************************
	 *****************************************
	 ****************************file*************
	 **(offset.first, offset.sec)*************
	 *****************************************
	 *****************************************
	 *****************************************
	 *****************************************
	 *****************************************
	 (idx.x,idx.y + 1)***(idx.x + 1,idx.y + 1)
	*/
	auto tile_offset = raster->wgs84_to_pixel(latitude, longitude);
	auto pixel_idx =
	    Post({static_cast<int>(tile_offset.first), static_cast<int>(tile_offset.second)});


	// Get the 4 corners
	auto top_left     = raster->read_pixel(pixel_idx.x, pixel_idx.y);
	auto top_right    = raster->read_pixel(pixel_idx.x + 1, pixel_idx.y);
	auto bottom_left  = raster->read_pixel(pixel_idx.x, pixel_idx.y + 1);
	auto bottom_right = raster->read_pixel(pixel_idx.x + 1, pixel_idx.y + 1);

	select_best_available_pixels(top_left, top_right, bottom_left, bottom_right);

	// only need to check one of these here because if any of the others aren't no_data_value, then
	// the above call to select_best_available_pixels will have corrected the value of top_left.
	if (!raster->is_valid_data(top_left)) {
		return {false, 0.0};
	}

	auto elevation =
	    ElevationInterpolator(top_left, top_right, bottom_left, bottom_right)
	        .interpolate({tile_offset.first - pixel_idx.x, tile_offset.second - pixel_idx.y});

	return {true, elevation};
}

void Tile::select_best_available_pixels(double& top_left,
                                        double& top_right,
                                        double& bottom_left,
                                        double& bottom_right) const {

	double top_left_temp     = select_valid(top_left, top_right, bottom_left, bottom_right);
	double top_right_temp    = select_valid(top_right, top_left, bottom_right, bottom_left);
	double bottom_left_temp  = select_valid(bottom_left, bottom_right, top_left, top_right);
	double bottom_right_temp = select_valid(bottom_right, bottom_left, top_right, top_left);

	top_left     = top_left_temp;
	top_right    = top_right_temp;
	bottom_left  = bottom_left_temp;
	bottom_right = bottom_right_temp;
}

// select first pixel_idx (i.e. pixel) that has a valid elevation
double Tile::select_valid(double one, double two, double three, double four) const {
	return (raster->is_valid_data(one))     ? one
	       : (raster->is_valid_data(two))   ? two
	       : (raster->is_valid_data(three)) ? three
	                                        : four;
}

std::string Tile::get_filename() const { return raster->get_name(); }

bool Tile::contains(double latitude, double longitude) const {

	std::pair<double, double> tile_offset = raster->wgs84_to_pixel(latitude, longitude);

	return (tile_offset.first <= raster->get_width() && tile_offset.first >= 0 &&
	        tile_offset.second <= raster->get_height() && tile_offset.second >= 0);
}

void Tile::unload() { raster->unload(); }

const std::ostream& operator<<(std::ostream& os, const Tile& tile) {

	for (int y = 0; y < tile.raster->get_height(); ++y) {
		for (int x = 0; x < tile.raster->get_width(); ++x) {
			auto elevation = tile.raster->read_pixel(x, y);
			if (elevation == tile.raster->no_data_value) {
				os << "Point (" << x << ", " << y << ") could not be read." << std::endl;
			} else {
				os << elevation << " ";
			}
		}
		os << std::endl;
	}

	return os;
}

}  // namespace geospatial
}  // namespace navtk
