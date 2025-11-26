#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/geospatial/TileStorage.hpp>
#include <navtk/geospatial/sources/ElevationSource.hpp>

namespace navtk {
namespace geospatial {
/**
 * This is an abstract class for reading data files that conform to MIL-PRF-89020B, using Geospatial
 * Data Abstraction Library (GDAL). All GDAL map data sources should extend from this class.
 *
 * To use this package GDAL must be installed and built with navtk.
 */
class GdalSource : public ElevationSource {
public:
	/**
	 * Current GDAL implementations
	 */
	enum class MapType {
		/**
		 * A TIFF file with georeferencing information embedded in it.
		 */
		GEOTIFF,
		/**
		 * An NGA standard of terrain elevation data.
		 */
		DTED
	};

	/**
	 * Constructor
	 *
	 * @param map_path to directory containing one or more GDAL data files. All GDAL files in this
	 * directory must have the same file format and vertical reference frame.
	 * @param type the gdal format used by the dataset.
	 * @param in_ref the vertical reference frame stored in the input directory. Defaults to
	 * ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE.
	 * @param out_ref the vertical reference frame for output elevations. Defaults to
	 * ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE.
	 * @param undulation_path the path to the geoid undulation file for converting between HAE and
	 * MSL. The default path of this variable requires setting the NAVTK_DATA_DIR environment
	 * variable to the folder containing the undulation file, or setting the
	 * NAVTK_GEOID_UNDULATION_PATH environment variable to the path of the file itself.
	 * @param num_tiles the max number of tiles to store in memory.
	 */
	GdalSource(const std::string& map_path,
	           MapType type,
	           AspnMeasurementAltitudeReference in_ref  = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE,
	           AspnMeasurementAltitudeReference out_ref = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE,
	           unsigned int num_tiles                   = 10,
	           const std::string& undulation_path       = "ww15mgh.grd");

	/**
	 * Gets the elevation at a given latitude and longitude.
	 *
	 * @param latitude_rad latitude value in radians
	 * @param longitude_rad longitude value in radians
	 * @return A `pair` showing whether a valid elevation was found (`.first`) and if `true`, the
	 * elevation in meters above either geoid -- aka mean sea level -- or ellipsoid (`.second`).
	 */
	std::pair<bool, double> lookup_datum(double latitude_rad, double longitude_rad) const override;

	/**
	 * @param new_ref the output vertical reference frame to change to. If `new_ref` is
	 * `ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL`, then this function will do nothing.`
	 */
	void set_output_vertical_reference_frame(AspnMeasurementAltitudeReference new_ref) override;

private:
	mutable TileStorage storage;

	std::vector<std::shared_ptr<Tile>> known_tiles;

	const MapType map_type;
	std::string undulation_path;
};
}  // namespace geospatial
}  // namespace navtk
