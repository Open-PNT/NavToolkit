#pragma once

#include <navtk/geospatial/providers/SpatialMapDataProvider.hpp>

namespace navtk {
namespace geospatial {

/**
 * Container that aggregates data from one or more `SpatialMapDataSource`s and supplies a unified
 * `lookup_datum` function for reading that data.
 */
class SimpleProvider : public SpatialMapDataProvider {
public:
	using SpatialMapDataProvider::SpatialMapDataProvider;

	/**
	 * Iterates over the available `SpatialMapDataSource`s and queries them
	 * for an value at the given latitude and longitude. If a valid value is found in
	 * the SpatialMapDataSource databases, it will be returned. Otherwise sets the validity flag
	 * to `false`.
	 *
	 * @param latitude The latitude value in radians.
	 * @param longitude The longitude value in radians.
	 *
	 * @return A `pair` showing whether a value was found (`.first`) and if `true`, the value.
	 */
	std::pair<bool, double> lookup_datum(double latitude, double longitude) const override;
};
}  // namespace geospatial
}  // namespace navtk