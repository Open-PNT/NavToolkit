#include <navtk/geospatial/sources/ElevationSource.hpp>

namespace navtk {
namespace geospatial {

AspnMeasurementAltitudeReference ElevationSource::get_output_vertical_reference_frame() const {
	return output_reference;
}
}  // namespace geospatial
}  // namespace navtk
