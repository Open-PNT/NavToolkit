#pragma once

#include <string>
#include <utility>
#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/geospatial/detail/custom_deleters.hpp>
#include <navtk/geospatial/detail/transformations.hpp>

namespace navtk {
namespace geospatial {

/**
 * Interface to an Elevation Reader based on a rectangular grid of pixels (i.e. posts).
 */
class Raster {
public:
	virtual ~Raster() = default;

	/**
	 * Datasets define a value which they use to denote invalid or missing data inside of a tile. If
	 * any elevation matches this value then it is invalid.
	 */
	double no_data_value;

	/**
	 * A unique pointer to a GDALDataset object with a custom deleter.
	 */
	std::unique_ptr<GDALDataset, detail::DatasetDelete> dataset;

	/**
	 * Returns the total number of pixels in each line.
	 *
	 * @return The number of pixels in the line.
	 */
	virtual int get_width() const = 0;

	/**
	 * Returns the total number of lines in the dataset object (i.e. in the file).
	 *
	 * @return The number of lines.
	 */
	virtual int get_height() const = 0;

	/**
	 * Transform the coordinates from wgs84 (lat/lon) to pixel offset.  The pixel offset is the
	 * decimal distance from the top left of the tile, must be converted to an int to get a specific
	 * pixel index.
	 *
	 * @param latitude latitude in degrees
	 * @param longitude longitude in degrees
	 *
	 * @return The converted coordinates as a pixel offset from the top left of the tile.
	 */
	virtual std::pair<double, double> wgs84_to_pixel(double latitude, double longitude) const = 0;

	/**
	 * Returns a single double representing the elevation at the given pixel index.
	 *
	 * @param idx_x index of pixel in the line, value should be between 0 and #get_width().
	 * @param idx_y index of line to read, value should be between 0 and #get_height().
	 *
	 * @return The elevation at the pixel index, or #no_data_value if the requested index is
	 * unavailable in the dataset or is out of bounds.
	 */
	virtual double read_pixel(size_t idx_x, size_t idx_y) = 0;

	/**
	 * Return a human-readable name for this object.
	 *
	 * @return The file name.
	 */
	virtual std::string get_name() const = 0;

	/**
	 * Compare the given elevation to the raster's no data value.
	 *
	 * @param data The elevation to evaluate.
	 *
	 * @return `true` if the elevation is valid and `false` if not.
	 */
	virtual bool is_valid_data(double data) const = 0;

	/**
	 * Remove data from memory, if cached
	 */
	virtual void unload() = 0;
};
}  // namespace geospatial
}  // namespace navtk
