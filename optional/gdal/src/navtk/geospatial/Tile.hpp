#pragma once

#include <memory>
#include <ostream>
#include <utility>

#include <navtk/aspn.hpp>
#include <navtk/geospatial/GdalRaster.hpp>
#include <navtk/geospatial/Post.hpp>

namespace navtk {
namespace geospatial {

/**
 * This class represents a valid GDAL tile.
 */
class Tile {
public:
	/**
	 * Constructor
	 *
	 * @param raster the raster image for this GDAL tile.
	 */
	Tile(std::shared_ptr<Raster> raster);


	/**
	 * Gets the elevation from a GDAL tile.
	 *
	 * @param latitude latitude value in degrees
	 * @param longitude longitude value in degrees
	 * @return A `pair` showing whether an elevation was found (`.first`) and if `true`, the
	 * elevation in meters above mean sea level (`.second`).
	 */
	std::pair<bool, double> lookup_datum(double latitude, double longitude) const;

	/**
	 * Gets the filename of the raster tile that is being
	 * accessed.
	 *
	 * @return The filename.
	 */
	std::string get_filename() const;

	/**
	 * Check if the given coordinates are in the bounds of this tile
	 *
	 * @param latitude latitude value in degrees
	 * @param longitude longitude value in degrees
	 *
	 * @return `true` if the given coordinates are in the tile.
	 */
	bool contains(double latitude, double longitude) const;

	/**
	 * Remove the tile data from memory, if cached
	 */
	void unload();

	/**
	 * Overload insertion operator to print the tile's data
	 *
	 * @param os the `ostream` object
	 * @param tile the tile to print
	 *
	 * @return The output stream `os`.
	 */
	friend const std::ostream& operator<<(std::ostream& os, const Tile& tile);

private:
	void select_best_available_pixels(double& top_left,
	                                  double& top_right,
	                                  double& bottom_left,
	                                  double& bottom_right) const;
	double select_valid(double one, double two, double three, double four) const;

	std::shared_ptr<Raster> raster;
};
}  // namespace geospatial
}  // namespace navtk
