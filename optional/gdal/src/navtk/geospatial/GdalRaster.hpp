#pragma once

#include <array>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <navtk/aspn.hpp>
#include <navtk/geospatial/Raster.hpp>

namespace navtk {
namespace geospatial {

/**
 * Interface to an GDAL File made up of a rectangular grid of pixels. Each pixel represents a post;
 * thus, pixel and post can be considered synonymous
 */
class GdalRaster : public Raster {
public:
	/**
	 * Constructor
	 *
	 * @param filename the file path
	 * @param undulation_path the path to the geoid undulation file for converting between HAE and
	 * MSL. The default path of this variable requires setting the NAVTK_DATA_DIR environment
	 * variable to the folder containing the undulation file.
	 */
	GdalRaster(const std::string& filename, const std::string& undulation_path = "ww15mgh.grd");

	/**
	 * Destructor
	 */
	~GdalRaster() = default;

	/**
	 * Scan the whole dataset tile to store in the cache.
	 */
	void scan_tile();

	/**
	 *
	 * @return Whether raster tile is associated with valid file.
	 */
	bool is_valid() const;

	/**
	 * Returns the total number of elements in each line.
	 *
	 * @return The number of elements in the line.
	 */
	int get_width() const override;

	/**
	 * Returns the total number of lines available.
	 *
	 * @return The number of lines.
	 */
	int get_height() const override;

	/**
	 * Transform the coordinates from wgs84 (lat/lon) to pixel offset.
	 *
	 * @param latitude Latitude in degrees
	 * @param longitude Longitude in degrees
	 *
	 * @return The converted coordinates as a pixel offset from the top left of the tile.
	 */
	std::pair<double, double> wgs84_to_pixel(double latitude, double longitude) const override;

	/**
	 * Transform the coordinates from pixel offset to wgs84 (lat/lon).
	 *
	 * This is the reverse of `wgs84_to_pixel()`.
	 *
	 * @param x_pixel column offset from top left of tile
	 * @param y_pixel row offset from top left of tile
	 *
	 * @return The converted coordinates in the form {latitude, longitude}.
	 */
	std::pair<double, double> pixel_to_wgs84(double x_pixel, double y_pixel) const;

	/**
	 * Returns a single double representing the elevation at the given pixel index.
	 *
	 * @param idx_x index of pixel in the line, value should be between 0 and #get_width().
	 * @param idx_y index of line to read, value should be between 0 and #get_height().
	 *
	 * @return The elevation at the pixel index, or #no_data_value if the requested index is
	 * unavailable in the dataset or is out of bounds.
	 */
	double read_pixel(size_t idx_x, size_t idx_y) override;

	/**
	 * Return a human-readable name for this object.
	 *
	 * @return The file name.
	 */
	std::string get_name() const override;

	/**
	 * Compare the given elevation to the raster's no data value (a unique value used by the dataset
	 * to denote invalid or missing data inside of a tile).
	 *
	 * @param data The elevation to evaluate.
	 *
	 * @return `true` if the elevation is valid and `false` if not.
	 */
	bool is_valid_data(double data) const override;

	/**
	 * Remove data from memory, if cached
	 */
	void unload() override;

private:
	GdalRaster(const GdalRaster&)            = delete;
	GdalRaster& operator=(const GdalRaster&) = delete;
	GdalRaster(GdalRaster&&)                 = delete;
	GdalRaster& operator=(GdalRaster&&)      = delete;

	void transform_tile(AspnMeasurementAltitudeReference prev_ref,
	                    AspnMeasurementAltitudeReference new_ref);

	std::string filename;
	bool valid = false;

	size_t size_x;
	size_t size_y;

	std::vector<double> cached_tile;
	bool cached = false;

	std::array<double, 6> pixel_transform;

	std::unique_ptr<OGRCoordinateTransformation, detail::TransformDelete>
	    wgs84_to_map_transformation;
	std::unique_ptr<OGRCoordinateTransformation, detail::TransformDelete>
	    map_to_wgs84_transformation;

	std::string undulation_path;
};
}  // namespace geospatial
}  // namespace navtk
