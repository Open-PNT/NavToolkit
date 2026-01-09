#include <string>

#include <gtest/gtest.h>
#include <spdlog_assert.hpp>

#include <navtk/errors.hpp>
#include <navtk/geospatial/GdalRaster.hpp>

namespace navtk {
namespace geospatial {

TEST(GdalRaster, read_pixel_out_of_bounds) {
	char raster_path[200];
	char* data_dir = std::getenv("NAVTK_DATA_DIR");
	if (data_dir == NULL) {
		log_or_throw<std::runtime_error>(
		    "Environment variable 'NAVTK_DATA_DIR' not set. Must be set to directory "
		    "containing GDAL files.");
	} else {
		strncpy(raster_path, data_dir, 188);
		strcat(raster_path, "/bogota.tif");
		GdalRaster raster{raster_path};

		// Pick a location that does not have coverage
		EXPECT_EQ(raster.read_pixel(0, 999), raster.no_data_value);
	}
}
}  // namespace geospatial
}  // namespace navtk
