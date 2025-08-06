#include <navtk/geospatial/sources/GeoidUndulationSource.hpp>

#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <stdexcept>

#include <navtk/errors.hpp>
#include <navtk/geospatial/ElevationInterpolator.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/utils/ValidationContext.hpp>
#include <navtk/utils/data.hpp>

namespace {
constexpr navtk::Size VALUES_PER_RECORD = 1440;
// 20 lines/block * 9 blocks + 1 single + 11 empty lines
constexpr navtk::Size LINES_PER_RECORD = 191;
// to move file pointer past header
constexpr navtk::Size END_OF_HEADER = 73;
}  // namespace

namespace navtk {
namespace geospatial {

using navtk::navutils::RAD2DEG;

std::shared_ptr<GeoidUndulationSource> GeoidUndulationSource::get_shared(const std::string& path) {
#ifdef NAVTK_PYTHON_TENSOR
	return std::make_shared<GeoidUndulationSource>(GeoidUndulationSource(path));
#else
	static std::shared_ptr<GeoidUndulationSource> instance(new GeoidUndulationSource(path));
	return instance;
#endif
}

GeoidUndulationSource::GeoidUndulationSource(const std::string& path)
    : infile(utils::open_data_file(ErrorMode::DIE, "Geoid Undulation", path)),
      min_lat(read_value()),
      max_lat(read_value()),
      min_lon(read_value()),
      max_lon(read_value()),
      lat_step(read_value()),
      lon_step(read_value()) {
	// ww15mgh.grd file should always start with these values, specifying the range the file covers.
	// If these values aren't found, then the file could have been corrupted, so yield a runtime
	// error.
	if (min_lat != -90 || max_lat != 90 || min_lon != 0 || max_lon != 360 || lat_step != 0.25 ||
	    lon_step != 0.25) {
		log_or_throw<std::runtime_error>(
		    "Error reading header of file 'ww15mgh.grd.' File may have been modified");
	}
}

std::pair<bool, double> GeoidUndulationSource::lookup_datum(double latitude,
                                                            double longitude) const {
	if (!infile->is_open()) {
		log_or_throw<std::runtime_error>("File is not open.", latitude, longitude);
		return {false, 0.0};
	}

	if (navtk::utils::ValidationContext validation{}) {
		if (std::isnan(latitude)) {
			spdlog::warn("Latitude of {} (radians) is not a number", latitude);
			return {false, 0.0};
		}
		if (std::isnan(longitude)) {
			spdlog::warn("Longitude of {} (radians) is not a number", longitude);
			return {false, 0.0};
		}
	}

	longitude = navtk::navutils::wrap_to_2_pi(longitude);

	// Convert from radians to degrees.
	latitude *= RAD2DEG;
	longitude *= RAD2DEG;

	if (navtk::utils::ValidationContext validation{}) {
		if (latitude < min_lat || latitude > max_lat) {
			spdlog::warn(
			    "Latitude of {} degrees is outside range [{}, {}]", latitude, min_lat, max_lat);
			return {false, 0.0};
		}
	}

	if (is_outside_cache_bounds(latitude, longitude)) init_coverage(latitude, longitude);

	/* Given the point (latitude, longitude), get the corners from the file data and interpolate to
	 find the elevation difference at (latitude, longitude)

	 (T_lat,L_lon)**********(T_lat,R_lon)
	 ************************************
	 ************************************
	 ************************************
	 ************************************
	 ************************************
	 ************************************
	 ********(latitude, longitude)*******
	 ************************************
	 ************************************
	 ************************************
	 (B_lat,L_lon)**********(B_lat,R_lon)
	*/

	double top_lat    = ceil(latitude * 4) / 4;
	double bottom_lat = floor(latitude * 4) / 4;
	double left_lon   = floor(longitude * 4) / 4;
	double right_lon  = ceil(longitude * 4) / 4;

	double top_left_elevation     = get_value(top_lat, left_lon);
	double top_right_elevation    = get_value(top_lat, right_lon);
	double bottom_left_elevation  = get_value(bottom_lat, left_lon);
	double bottom_right_elevation = get_value(bottom_lat, right_lon);

	ElevationInterpolator interpolator{
	    top_left_elevation, top_right_elevation, bottom_left_elevation, bottom_right_elevation};
	double lon_fraction = (longitude - left_lon) / lon_step;
	double lat_fraction = (top_lat - latitude) / lat_step;

	double elevation = interpolator.interpolate({lon_fraction, lat_fraction});

	return {true, elevation};
}

bool GeoidUndulationSource::is_outside_cache_bounds(double lat, double lon) const {
	if (lon < lon_coverage_min && lon_coverage_max > max_lon) {
		lon += max_lon;
	}
	return (lat < lat_coverage_min || lat > lat_coverage_max || lon < lon_coverage_min ||
	        lon > lon_coverage_max);
}

void GeoidUndulationSource::init_coverage(double lat, double lon) const {
	set_coverage_bounds(lat, lon);
	infile->clear();
	infile->seekg(END_OF_HEADER);
	size_t records_to_skip = get_lat_idx(lat_coverage_max);
	skip_records(records_to_skip);
	read_chunk();
}

void GeoidUndulationSource::set_coverage_bounds(double lat, double lon) const {
	double rd = chunk_size / 8.0;
	// round bounds to nearest 0.25
	lat_coverage_min = floor((lat - rd) * 4 + 0.5) / 4;
	lat_coverage_max = floor((lat + rd) * 4 + 0.5) / 4;
	if (lat_coverage_min < min_lat) {
		lat_coverage_max += (min_lat - lat_coverage_min);
		lat_coverage_min = min_lat;
	} else if (lat_coverage_max > max_lat) {
		lat_coverage_min -= (lat_coverage_max - max_lat);
		lat_coverage_max = max_lat;
	}

	lon_coverage_min = floor((lon - rd) * 4 + 0.5) / 4;
	lon_coverage_max = floor((lon + rd) * 4 + 0.5) / 4;
	if (lon_coverage_min < 0) {
		lon_coverage_min += max_lon;
		lon_coverage_max += max_lon;
	}
}

void GeoidUndulationSource::read_chunk() const {
	size_t row_idx   = 0;
	Size min_lon_idx = (static_cast<Size>(lon_coverage_min * 4) % static_cast<Size>(max_lon * 4));
	Size max_lon_idx = (static_cast<Size>(lon_coverage_max * 4) % static_cast<Size>(max_lon * 4));
	while (row_idx <= chunk_size) {
		read_record(min_lon_idx, max_lon_idx, row_idx);
		row_idx++;
	}
}

void GeoidUndulationSource::skip_records(size_t num_records) const {
	for (size_t i = 0; i < num_records * LINES_PER_RECORD + 1; i++) {
		infile->ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
}

void GeoidUndulationSource::ignore_value() const {
	while (infile && std::isspace(infile->peek())) infile->get();
	while (infile && !std::isspace(infile->peek())) infile->get();
}

double GeoidUndulationSource::read_value() const {
	double out = 0.0;

#if defined(__SANITIZE_ADDRESS__) && (defined(__GLIBCXX__) || defined(__GLIBCPP__))
	// https://stackoverflow.com/q/65703206/2789327
	// GNU libstdc++'s istream double parsing implementation allocates and frees a buffer
	// approximately six times per value when reading this file, which causes ASAN's allocator to
	// blow a fuse on memory-constrained systems such as armv7. So, if we're compiling with
	// -fsanitize=address and using GNU libstdc++, we need to read the value into a buffer first and
	// then use strtod to parse the value of the buffer.
	std::string word;
	*infile >> word;
	out = std::strtod(word.c_str(), nullptr);
#else
	*infile >> out;
#endif

	return out;
}

void GeoidUndulationSource::read_record(Size min_lon_idx, Size max_lon_idx, size_t row_idx) const {
	bool keep_value = false;
	Size col_idx    = 0;
	if (max_lon_idx < min_lon_idx) {
		keep_value = true;
		col_idx    = VALUES_PER_RECORD - min_lon_idx;
	}

	for (Size lon_idx = 0; lon_idx < VALUES_PER_RECORD; lon_idx++) {
		if (lon_idx == min_lon_idx) {
			keep_value = true;
			col_idx    = 0;
		}

		if (keep_value)
			available_undulations(row_idx, col_idx++) = read_value();
		else
			ignore_value();

		// set [keep_value] to false after including value at [max_lon_idx]
		if (lon_idx == max_lon_idx) {
			keep_value = false;
			col_idx    = 0;
		}
	}
	ignore_value();
}

size_t GeoidUndulationSource::get_lat_idx(double lat) const { return (max_lat - lat) * 4; }

void GeoidUndulationSource::set_chunk_size(Size size) {
	if (size < 1 || size > 720) {
		log_or_throw<std::runtime_error>(
		    "Invalid chunk size {}. Size must be between 1 and 720. Keeping current cache size of "
		    "{}",
		    size,
		    chunk_size);
	} else {
		chunk_size            = size;
		available_undulations = zeros(chunk_size + 1, chunk_size + 1);
		lat_coverage_min      = -100;
		lat_coverage_max      = -100;
		lon_coverage_min      = -1;
		lon_coverage_max      = -1;
	}
}

Size GeoidUndulationSource::get_chunk_size() const { return chunk_size; }

double GeoidUndulationSource::get_value(double lat, double lon) const {
	if (lon < lon_coverage_min) {
		lon += max_lon;
	}
	size_t lat_idx = (lat_coverage_max - lat) * 4;
	size_t lon_idx = (lon - lon_coverage_min) * 4;
	return available_undulations(lat_idx, lon_idx);
}
}  // namespace geospatial
}  // namespace navtk
