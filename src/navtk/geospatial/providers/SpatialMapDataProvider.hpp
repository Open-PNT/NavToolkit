#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <navtk/geospatial/sources/SpatialMapDataSource.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace geospatial {

/**
 * Container that aggregates spatial map data from one or more `SpatialMapDataSource` and supplies a
 * unified `lookup_datum` function for reading that data. In general, consumers of spatial map data
 * should be written in terms of this class. Classes that supply spatial map data implement
 * `SpatialMapDataSource`.
 *
 * Possible implementations of this class:
 * - Prioritization of various sources (returning higher precision sources when available)
 * - Stitching sources together
 * - Interpolation between sources
 * - Extrapolation
 * - Averaging overlapping sources
 * - Conditional selection (i.e. a coastal region with underwater and above water)
 * - Spot filling (use X if available, or Y if not)
 */
class SpatialMapDataProvider {
public:
	/**
	 * Destructor
	 */
	virtual ~SpatialMapDataProvider() = default;

	/**
	 * Constructor for a single `SpatialMapDataSource`
	 *
	 * @param src the source
	 */
	SpatialMapDataProvider(not_null<std::shared_ptr<SpatialMapDataSource>> src);

	/**
	 * Constructor for a vector of zero or more `SpatialMapDataSource`s.
	 *
	 * @param srcs The vector of sources.
	 */
	SpatialMapDataProvider(std::vector<not_null<std::shared_ptr<SpatialMapDataSource>>> srcs = {});

	/**
	 * Adds a `SpatialMapDataSource` to the provider.
	 *
	 * @param src the source.
	 */
	void add_source(not_null<std::shared_ptr<SpatialMapDataSource>> src);

	/**
	 * Returns a pair containing a validity flag and data at the given latitude and longitude. If
	 * the flag is `false`, the data is invalid and should not be used.
	 *
	 * @param latitude The latitude value in radians.
	 * @param longitude The longitude value in radians.
	 *
	 * @return A `pair` showing whether a datum was found (`.first`) and if `true`, the
	 * datum value.
	 */
	virtual std::pair<bool, double> lookup_datum(double latitude, double longitude) const = 0;

protected:
	/**
	 * Store the SpatialMapDataSources used in the background by the provider
	 */
	std::vector<not_null<std::shared_ptr<SpatialMapDataSource>>> sources;
};
}  // namespace geospatial
}  // namespace navtk