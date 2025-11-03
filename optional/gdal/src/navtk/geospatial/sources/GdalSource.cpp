#include <navtk/geospatial/sources/GdalSource.hpp>

#include <iomanip>
#include <sstream>
#include <stdexcept>

#ifndef GDAL_INCLUDE_IN_SUBFOLDER
#	include <gdal_priv.h>
#	include <ogr_spatialref.h>
#else
#	include <gdal/gdal_priv.h>
#	include <gdal/ogr_spatialref.h>
#endif

#include <navtk/errors.hpp>
#include <navtk/fs/filesystem.hpp>
#include <navtk/geospatial/GdalRaster.hpp>
#include <navtk/geospatial/Tile.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace geospatial {

using navtk::navutils::RAD2DEG;

GdalSource::GdalSource(const std::string& map_path,
                       MapType type,
                       AspnMeasurementAltitudeReference in_ref,
                       AspnMeasurementAltitudeReference out_ref,
                       unsigned int num_tiles,
                       const std::string& undulation_path)
    : storage(num_tiles), map_type(type), undulation_path(undulation_path) {

	std::string extension;
	switch (map_type) {
	case MapType::GEOTIFF:
		extension = ".tif";
		break;
	case MapType::DTED:
		extension = ".dt";
		break;
	default:
		log_or_throw<std::invalid_argument>("Invalid map type.");
	}

	input_reference  = in_ref;
	output_reference = out_ref;
	if (in_ref == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL ||
	    out_ref == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL) {
		spdlog::warn(
		    "ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL is unsupported. Setting input and output "
		    "reference to HAE.");
		input_reference  = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE;
		output_reference = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE;
	}

	auto absolute_map_path = fs::absolute(map_path);

	if (!map_path.empty()) {

		if (map_path[map_path.size() - 1] != fs::path::preferred_separator) {
			absolute_map_path = fs::absolute(map_path + fs::path::preferred_separator);
		}
	}

	// By default constructs an end iterator which will cause no paths to be searched
	fs::recursive_directory_iterator file_search_iterator;

	try {
		file_search_iterator = fs::recursive_directory_iterator(absolute_map_path);
	} catch (fs::filesystem_error& e) {
		log_or_throw<std::invalid_argument>(e.what());
	}

	for (const auto& entry : file_search_iterator) {
		fs::path filename = fs::path(entry.path());
		// Use `find` instead of `compare` to find extensions like `dt2`
		if (filename.filename().string().at(0) != '.' &&
		    filename.extension().string().find(extension) != std::string::npos) {

			std::shared_ptr<GdalRaster> raster =
			    std::make_shared<GdalRaster>(filename, undulation_path);
			if (raster->is_valid()) known_tiles.push_back(std::make_shared<Tile>(raster));
		}
	}
	if (map_type == MapType::DTED) {
		// sort so that '.dt5' files are found before '.dt0'
		std::sort(known_tiles.begin(),
		          known_tiles.end(),
		          [](const std::shared_ptr<Tile> tile1, const std::shared_ptr<Tile> tile2) {
			          return tile1->get_filename().back() > tile2->get_filename().back();
		          });
	}

	if (known_tiles.empty()) {
		log_or_throw("GdalSource: No elevation files found in path {}",
		             fmt::streamed(absolute_map_path));
	}
}

void GdalSource::set_output_vertical_reference_frame(AspnMeasurementAltitudeReference new_ref) {
	if (new_ref == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL) {
		spdlog::warn(
		    "Setting output reference to ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL is not "
		    "supported.");
	} else {
		output_reference = new_ref;
	}
}

std::pair<bool, double> GdalSource::lookup_datum(double latitude_rad, double longitude_rad) const {
	double latitude_deg  = latitude_rad * RAD2DEG;
	double longitude_deg = longitude_rad * RAD2DEG;
	for (auto const& tile : known_tiles) {
		if (tile->contains(latitude_deg, longitude_deg)) {
			if (!storage.is_stored(tile->get_filename())) {
				// will clear oldest tile from memory when new tile is added
				storage.add_tile(tile);
			}
			auto elevation = tile->lookup_datum(latitude_deg, longitude_deg);
			if (elevation.first) {
				// TODO (#733): ideally, this conversion won't need to occur here because it would
				// happen at initialization
				if (input_reference == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL &&
				    output_reference == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE) {
					return navtk::navutils::msl_to_hae(
					    elevation.second, latitude_rad, longitude_rad, undulation_path);
				} else if (input_reference == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE &&
				           output_reference == ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL) {
					return navtk::navutils::hae_to_msl(
					    elevation.second, latitude_rad, longitude_rad, undulation_path);
				}
				return elevation;
			}
		}
	}

	spdlog::debug(
	    "GdalSource::lookup_datum failed!  {}/{} not in known tiles.", latitude_deg, longitude_deg);

	return {false, 0.0};
}
}  // namespace geospatial
}  // namespace navtk
