#pragma once

#include <utility>

#include <gmock/gmock.h>

#include <navtk/aspn.hpp>
#include <navtk/geospatial/Raster.hpp>

namespace navtk {
namespace geospatial {

class MockRaster : public Raster {
public:
	MockRaster() { no_data_value = -32767; }
	bool is_valid_data(double data) const override { return data != no_data_value; }

	// For tile testing, since there is no actual dataset, we assume each tile is 1 degree latitude
	// by 1 degree longitude, converting the lat/lon coordinates accordingly to a pixel index based
	// on the specified size of the tile.
	std::pair<double, double> wgs84_to_pixel(double latitude, double longitude) const override {

		double x = longitude;
		double y = latitude;

		double tile_fraction_x = std::fabs(x - static_cast<int>(x));
		double tile_fraction_y = std::fabs(y - static_cast<int>(y));

		if (x < 0) {
			tile_fraction_x = 1 - tile_fraction_x;
		}
		if (y > 0) {
			tile_fraction_y = 1 - tile_fraction_y;
		}

		x = (get_width() - 1) * tile_fraction_x;
		y = (get_height() - 1) * tile_fraction_y;

		return {x, y};
	}

	MOCK_METHOD(int, get_width, (), (const, override));
	MOCK_METHOD(int, get_height, (), (const, override));
	MOCK_METHOD(double, read_pixel, (size_t idx_x, size_t idx_y), (override));
	MOCK_METHOD(std::string, get_name, (), (const, override));
	MOCK_METHOD(void, unload, (), (override));
};
}  // namespace geospatial
}  // namespace navtk
