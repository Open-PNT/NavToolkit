#include <navtk/geospatial/ElevationInterpolator.hpp>

#include <utility>

namespace navtk {
namespace geospatial {

ElevationInterpolator::ElevationInterpolator(double top_left,
                                             double top_right,
                                             double bottom_left,
                                             double bottom_right)
    : top_left(top_left),
      top_right(top_right),
      bottom_left(bottom_left),
      bottom_right(bottom_right) {}

double ElevationInterpolator::interpolate(std::pair<double, double> fractions) const {

	return (((top_left * (1 - fractions.first)) + (top_right * fractions.first)) *
	        (1 - fractions.second)) +
	       (((bottom_left * (1 - fractions.first)) + (bottom_right * fractions.first)) *
	        fractions.second);
}
}  // namespace geospatial
}  // namespace navtk
