#include <navtk/geospatial/providers/SimpleProvider.hpp>

#include <navtk/utils/interpolation.hpp>

namespace navtk {
namespace geospatial {

std::pair<bool, double> SimpleProvider::lookup_datum(double latitude, double longitude) const {
	for (auto source : sources) {
		auto elevation = source->lookup_datum(latitude, longitude);
		if (elevation.first) {
			return elevation;
		}
	}
	return {false, 0.0};
}

}  // namespace geospatial
}  // namespace navtk