#pragma once

namespace navtk {
namespace geospatial {

/**
 * A two-dimensional index into a GDAL tile (i.e. the pixel index), where #y is the line and #x is
 * the element within the line.
 */
struct Post {
	/**
	 * Horizontal position within an individual line, corresponding to longitude.
	 */
	int x;

	/**
	 * Vertical (line) index, corresponding to latitude.
	 */
	int y;
};

}  // namespace geospatial
}  // namespace navtk
