#pragma once

#include <navtk/aspn.hpp>
#include <navtk/geospatial/providers/SimpleProvider.hpp>
#include <navtk/geospatial/sources/ElevationSource.hpp>

namespace navtk {
namespace geospatial {

/**
 * SimpleProvider used specifically for elevations. Allows user to specify output vertical reference
 * frame (e.g. get elevations in HAE or MSL).
 */
class SimpleElevationProvider : public SimpleProvider {
public:
	/**
	 * Constructor for a single elevation source
	 *
	 * @param src the source
	 * @param out_ref the desired vertical reference frame for all elevations returned by this class
	 */
	SimpleElevationProvider(
	    not_null<std::shared_ptr<ElevationSource>> src,
	    AspnMeasurementAltitudeReference out_ref = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE);

	/**
	 * Constructor for a vector of zero or more elevation sources
	 *
	 * @param srcs The vector of sources.
	 * @param out_ref the desired vertical reference frame for all elevations returned by this class
	 */
	SimpleElevationProvider(
	    std::vector<not_null<std::shared_ptr<ElevationSource>>> srcs = {},
	    AspnMeasurementAltitudeReference out_ref = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE);

	/**
	 * Adds an `ElevationSource` to the provider.
	 *
	 * @param src the source.
	 */
	void add_source(not_null<std::shared_ptr<ElevationSource>> src);

private:
	AspnMeasurementAltitudeReference output_reference;
};
}  // namespace geospatial
}  // namespace navtk
