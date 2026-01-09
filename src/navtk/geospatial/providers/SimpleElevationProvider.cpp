#include <navtk/geospatial/providers/SimpleElevationProvider.hpp>

#include <navtk/utils/interpolation.hpp>

namespace navtk {
namespace geospatial {

SimpleElevationProvider::SimpleElevationProvider(not_null<std::shared_ptr<ElevationSource>> src,
                                                 AspnMeasurementAltitudeReference out_ref)
    : SimpleElevationProvider(std::vector<not_null<std::shared_ptr<ElevationSource>>>{src},
                              out_ref) {}

SimpleElevationProvider::SimpleElevationProvider(
    std::vector<not_null<std::shared_ptr<ElevationSource>>> srcs,
    AspnMeasurementAltitudeReference out_ref)
    : SimpleProvider({srcs.begin(), srcs.end()}), output_reference(out_ref) {
	if (output_reference == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL) {
		spdlog::warn(
		    "ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL is not supported. Output for each source "
		    "will be in the current reference frame of the source.");
	} else {
		for (auto source : srcs) {
			source->set_output_vertical_reference_frame(output_reference);
		}
	}
}

void SimpleElevationProvider::add_source(not_null<std::shared_ptr<ElevationSource>> src) {
	if (output_reference != ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL) {
		src->set_output_vertical_reference_frame(output_reference);
	}
	sources.push_back(src);
}
}  // namespace geospatial
}  // namespace navtk
