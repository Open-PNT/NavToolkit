#pragma once

#include <utility>

namespace navtk {
namespace geospatial {

/**
 * This class uses four GDAL elevation posts and an offset from the top left post to calculate a
 * weighted elevation value for the given offset.
 */
class ElevationInterpolator {
public:
	/**
	 * Used to interpolate the elevation from GDAL given four adjacent posts.
	 *
	 * @param top_left top left post elevation
	 * @param top_right top right post elevation
	 * @param bottom_left bottom left post elevation
	 * @param bottom_right bottom right post elevation
	 */
	ElevationInterpolator(double top_left,
	                      double top_right,
	                      double bottom_left,
	                      double bottom_right);

	/**
	 * This method interpolates a GDAL elevation using 3 linear interpolations. One on the top
	 * posts, one on the bottom posts, and a third between the results of the first two.
	 *
	 * Interpolate the elevation given the first element of the `std::pair` is a percentage from
	 * `top_left` to `top_right` in the x direction and the second element is a percentage from
	 * `top_left` to `bottom_left` in the y direction.
	 *
	 * @param fractions A pair where the first is the distance from top left post to selected post
	 * in x direction (0 - 1) and the second is the distance from top left post to selected post in
	 * y direction (0 - 1)
	 * @return Interpolated elevation between the four posts.
	 */
	double interpolate(std::pair<double, double> fractions) const;

private:
	double top_left;
	double top_right;
	double bottom_left;
	double bottom_right;
};

}  // namespace geospatial
}  // namespace navtk
