#pragma once

#include <fstream>
#include <memory>

#include <navtk/factory.hpp>
#include <navtk/geospatial/sources/SpatialMapDataSource.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace geospatial {

/**
 * Class used to read geoid undulation data from 'ww15mgh.grd', a standard undulation file available
 * from https://earth-info.gs.mil/
 *
 * Geoid height is the actual height above mean sea level (often abbreviated MSL).
 *
 * Ellipsoid height is the height above an ellipsoid model of the Earth (often abbreviated HAE).
 *
 * Geoid undulation is the geoid height relative to the ellipsoid height (i.e. geoid height minus
 * ellipsoid height).
 *
 *
 * The file begins with a header line containing the range of latitudes and longitudes covered by
 * the file, as well as the steps between each. Steps in current file version are 0.25 degrees, and
 * are unlikely to change. The file is then separated into 'records'. Each record contains all
 * undulation values at a given latitude for the entire span of longitudes. The first record
 * contains the undulations at latitude 90 deg, the second 89.75 deg, and so on to -90 deg. Each
 * record contains 1441 undulation values for longitudes from 0 to 360 deg. Each 'record' is
 * comprised of 9 'blocks', each preceded by an empty line, and then a single value corresponding to
 * a longitude of 360 on another line, also preceded by an empty line. Each 'block' is comprised of
 * 20 lines with 8 undulation values per line. Block 0 covers longitude 0 to 39.75 deg, Block 1 40
 * to 79.75 deg, and so on until 359.75 deg longitude. Each line in a block contains 8 values, with
 * each value being allowed a 9 character spacing, except the first (which has 10). The 10 character
 * limit also applies to the single values corresponding to lon = 360.
 */
class GeoidUndulationSource : public SpatialMapDataSource {
public:
	/**
	 * Gets singleton object, constructing new instance if necessary.
	 *
	 * Upon construction, adds the path to the Geoid Undulation data file, 'ww15mgh.grd'. Opens the
	 * file, reads past the 6 header values describing the range the file covers. Yields runtime
	 * error if the file is not found or if the header values are invalid.
	 *
	 * * @param path the path to the geoid undulation file for converting between HAE and
	 * MSL. The default path of this variable requires setting the NAVTK_DATA_DIR
	 * environment variable to the folder containing the undulation file, or setting the
	 * NAVTK_GEOID_UNDULATION_PATH environment variable to the path of the file itself.
	 *
	 * @return The singleton object.
	 */
	static std::shared_ptr<GeoidUndulationSource> get_shared(
	    const std::string& path = "ww15mgh.grd");

	/**
	 * Gets the geoid undulation at a given latitude and longitude.
	 *
	 * @param latitude Latitude value in radians, in range [-PI/2, PI/2]
	 * @param longitude Longitude value in radians
	 *
	 * @return A `pair` showing whether a height was found (`.first`) and if `true`, the geoid
	 * undulation in meters. If the provided latitude and longitude are in a valid range, then an
	 * undulation value will be returned; otherwise, `.first` will be `false`.
	 */
	std::pair<bool, double> lookup_datum(double latitude, double longitude) const override;

	/**
	 * Allows the user to change the size of a chunk stored in memory.
	 *
	 * @param size New chunk size. Must be greater than 0 to allow for interpolation. The size is
	 * the length of each dimension of the undulation Matrix.  This length includes the values at
	 * each corner. Thus, the total number of values stored = (size + 1)^2.
	 */
	void set_chunk_size(Size size);

	/**
	 * @return The current chunk size.
	 */
	Size get_chunk_size() const;

private:
	// private constructor because global instance is implemented via `get_shared()`.
	GeoidUndulationSource(const std::string& path);
#ifndef NAVTK_PYTHON_TENSOR
	GeoidUndulationSource(const GeoidUndulationSource&)             = delete;
	GeoidUndulationSource& operator=(const GeoidUndulationSource&)  = delete;
	GeoidUndulationSource(const GeoidUndulationSource&&)            = delete;
	GeoidUndulationSource& operator=(const GeoidUndulationSource&&) = delete;
#endif

	// Check if either latitude/longitude pair are outside of the current chunk of undulation
	// values read into memory.
	bool is_outside_cache_bounds(double lat, double lon) const;

	// Moves file pointer to the start of a new chunk of data and reads the chunk into memory.
	void init_coverage(double lat, double lon) const;

	// Set the bounds for a new square chunk to be pulled into memory. This chunk will be centered
	// around (lat, lon). Will set internal variables lat_coverage_min, lat_coverage_max,
	// lon_coverage_min, and lat_coverage_max to reflect the range of latitudes and longitudes that
	// the Matrix available_undulations contains undulation values for.
	void set_coverage_bounds(double lat, double lon) const;

	// Read in a 'box' of undulation data into memory.
	void read_chunk() const;

	// Skip a section of the file equivalent to a given number of records. This is called after
	// resetting the file pointer to the start of the file to move the pointer to the start of the
	// first record within the coverage bounds.
	void skip_records(size_t num) const;

	// Reads one double value from the file and returns it.
	double read_value() const;

	// Reads the value from a line in the file with a single element (such as occurs at the end of
	// each record). Required that the file pointer at the end of block 9 (the last block) in a
	// given record. This value is the same as the first value in a record, so no need to store it.
	void ignore_value() const;

	// Read an entire record (1440 values) starting at the current location of the file pointer. A
	// record consists of 9 blocks containing 20 lines with 8 values per line (for a total of 160
	// values per block). Each block is preceded by an empty line. A record is followed by an empty
	// line and an additional single value on its own line.
	void read_record(Size min_lon_idx, Size max_lon_idx, size_t row_idx) const;

	// Get the record number of a given latitude.  Ex: latitude 90 is the first record in the file,
	// so return 0; latitude 80 is the 40th record, so return 40.
	size_t get_lat_idx(double lat) const;

	// Get the element in the available_undulations Matrix corresponding to the given lat and lon.
	// Assumes lat and lon will be multiples of 0.25 and that the corresponding point will be found
	// in memory.
	double get_value(double lat, double lon) const;

	mutable std::shared_ptr<std::ifstream> infile;

	/* 'ww15mgh.grd' file attributes */
	const double min_lat, max_lat, min_lon, max_lon, lat_step, lon_step;

	/* Chunk attributes */
	// chunk_size is the length of a "side" of a chunk square (i.e. the number of latitude points or
	// the number of longitude points to cover). The latitude or longitude range that a
	// available_undulations spans in chunk_size/4. Ex: A chunk_size of 40 includes 41x41
	// latitude/longitude points and spans 10 degrees of latitude and 10 degrees of longitude.
	Size chunk_size = 40;  // total number of bytes = (chunk_size + 1)^2 * (# of bytes/value)

	// latitude coverage range is [lat_coverage_min, lat_coverage_max]
	mutable double lat_coverage_min = -100;
	mutable double lat_coverage_max = -100;
	// longitude coverage range is [lon_coverage_min, lon_coverage_max]
	mutable double lon_coverage_min = -1;
	mutable double lon_coverage_max = -1;

	// The chunk of read data stored in memory
	mutable Matrix available_undulations = zeros(chunk_size + 1, chunk_size + 1);
};
}  // namespace geospatial
}  // namespace navtk
