#pragma once

#include <string>
#include <utility>

namespace navtk {
namespace geospatial {

/**
 * This class is the interface for all sources of spatial map data.
 *
 * Derive from this class if you're writing a class that reads or calculates spatial map data. Most
 * common implementations will likely be some form of elevation (but are not limited to elevations
 * -- e.g. Jamming Map).  Each instance of this class should correspond to a single path or
 * no path (e.g. constant undulation model).
 */
class SpatialMapDataSource {
public:
	virtual ~SpatialMapDataSource() = default;

	/**
	 * Return a value from the map for a given location. If no value is available, return a pair
	 * indicating the return value is invalid (i.e. {false, 0.0}).
	 *
	 * @param latitude Latitude (radians) of the location being queried for a data value.
	 * @param longitude Longitude (radians) of the location being queried for a data value.
	 * @return A `pair` showing whether a value was found (`.first`) and if `true`, the value
	 * in the type associated with the derived class (`.second`).
	 */
	virtual std::pair<bool, double> lookup_datum(double latitude, double longitude) const = 0;
};
}  // namespace geospatial
}  // namespace navtk
