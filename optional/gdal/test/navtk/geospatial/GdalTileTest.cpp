#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <navtk/geospatial/MockRaster.hpp>
#include <navtk/geospatial/Tile.hpp>

using testing::_;
using testing::Return;
using testing::SetArrayArgument;
using testing::Throw;

namespace navtk {
namespace geospatial {

class TileTest : public testing::Test {
protected:
	void SetUp() override {
		raster       = std::make_unique<MockRaster>();
		raw_raster   = raster.get();
		tile         = std::make_shared<Tile>(std::move(raster));
		top_left     = raw_raster->no_data_value;
		top_right    = raw_raster->no_data_value;
		bottom_left  = raw_raster->no_data_value;
		bottom_right = raw_raster->no_data_value;
	}

	static constexpr unsigned size        = 11;
	std::array<double, 6> pixel_transform = {0, 1, 0, 0, 0, 1};
	std::unique_ptr<MockRaster> raster;
	MockRaster* raw_raster;
	double top_left;
	double top_right;
	double bottom_left;
	double bottom_right;
	std::shared_ptr<Tile> tile;
};

TEST_F(TileTest, get_elevation_max_tile_all_zeros) {
	const double longitude = -84;
	const double latitude  = 45;
	const int offset_x     = 10;
	const int offset_y     = 10;

	// NOTE: pixel and post are synonymous
	EXPECT_CALL(*raw_raster, get_width()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, get_height()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y)).WillOnce(Return(top_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y)).WillOnce(Return(top_right));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y + 1)).WillOnce(Return(bottom_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y + 1)).WillOnce(Return(bottom_right));

	auto elevation = tile->lookup_datum(latitude, longitude);

	EXPECT_FALSE(elevation.first);
	EXPECT_NEAR(0, elevation.second, 0.00001);
}

TEST_F(TileTest, get_elevation_max_tile) {
	const double longitude = -84;
	const double latitude  = 45;
	const int offset_x     = 10;
	const int offset_y     = 10;

	top_left = 10;  // other corners are `#no_data_value` because they are out of bounds of tile

	EXPECT_CALL(*raw_raster, get_width()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, get_height()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y)).WillOnce(Return(top_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y)).WillOnce(Return(top_right));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y + 1)).WillOnce(Return(bottom_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y + 1)).WillOnce(Return(bottom_right));

	auto elevation = tile->lookup_datum(latitude, longitude);

	ASSERT_TRUE(elevation.first);
	EXPECT_NEAR(10, elevation.second, 0.01);
}

TEST_F(TileTest, get_elevation_mixed_values) {
	const double longitude = -84.25;
	const double latitude  = 45.71;
	const int offset_x     = 7;
	const int offset_y     = 2;

	top_left     = 7;
	top_right    = 8;
	bottom_left  = 7;
	bottom_right = 8;

	EXPECT_CALL(*raw_raster, get_width()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, get_height()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y)).WillOnce(Return(top_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y)).WillOnce(Return(top_right));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y + 1)).WillOnce(Return(bottom_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y + 1)).WillOnce(Return(bottom_right));

	auto elevation = tile->lookup_datum(latitude, longitude);
	// A longitude of -84.25 would be halfway between
	// -84.3 and -84.2 (7 and 8 respectively)

	ASSERT_TRUE(elevation.first);
	EXPECT_NEAR(7.5, elevation.second, 0.01);
}

TEST_F(TileTest, get_elevation_mixed_values2) {
	const double longitude = -84.38;
	const double latitude  = 45.14;
	const int offset_x     = 6;
	const int offset_y     = 8;

	top_left     = 1;
	top_right    = 34;
	bottom_left  = 11;
	bottom_right = 345;
	// top row   = {25, 54, 71, 3, 2, 6, 1, 34, 8, 5, 19};
	// bottom row = {2, 5, 7, 34, 12, 56, 11, 345, 86, 55, 198};

	EXPECT_CALL(*raw_raster, get_width()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, get_height()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y)).WillOnce(Return(top_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y)).WillOnce(Return(top_right));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y + 1)).WillOnce(Return(bottom_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y + 1)).WillOnce(Return(bottom_right));

	auto elevation = tile->lookup_datum(latitude, longitude);

	// between 11(45.1, -84.4) and 345(45.1, -84.3) on bottom, 1 (45.2, -84.4) and 34(45.2, -84.3)
	// on top. Starting top left value is one, normalized slope is (34 - 1) = 33 Requested x
	// coordinate is 0.02 deg east of left boundary, or 0.2 normalized Should be 1 + (34 - 1) * 0.2
	// = 7.6 for top 11 + (345 - 11) * 0.2 = 77.8 for bottom Starting from top line, requested
	// latitude is 0.06 deg south So we should have top + (bottom - top) * 0.6
	ASSERT_TRUE(elevation.first);
	EXPECT_NEAR(49.72, elevation.second, 1e-6);
}

TEST_F(TileTest, read_pixel_no_elevation_top_row) {
	const double longitude = -84.38;
	const double latitude  = 45.14;
	const int offset_x     = 6;
	const int offset_y     = 8;

	bottom_left  = 1;
	bottom_right = 34;

	EXPECT_CALL(*raw_raster, get_width()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, get_height()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y)).WillOnce(Return(top_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y)).WillOnce(Return(top_right));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y + 1)).WillOnce(Return(bottom_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y + 1)).WillOnce(Return(bottom_right));

	auto elevation = tile->lookup_datum(latitude, longitude);
	// No 'top row' available. Fall back to a 1D interp.
	ASSERT_TRUE(elevation.first);
	EXPECT_NEAR(7.6, elevation.second, 0.01);
}

TEST_F(TileTest, read_pixel_no_elevation_top_left) {
	const double longitude = -84.38;
	const double latitude  = 45.14;
	const int offset_x     = 6;
	const int offset_y     = 8;

	top_right    = 18;
	bottom_left  = 1;
	bottom_right = 34;

	EXPECT_CALL(*raw_raster, get_width()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, get_height()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y)).WillOnce(Return(top_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y)).WillOnce(Return(top_right));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y + 1)).WillOnce(Return(bottom_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y + 1)).WillOnce(Return(bottom_right));

	auto elevation = tile->lookup_datum(latitude, longitude);
	// Filled with top right pixel, 18
	ASSERT_TRUE(elevation.first);
	EXPECT_NEAR(11.76, elevation.second, 0.01);
}

TEST_F(TileTest, read_pixel_no_elevation_top_right) {
	const double longitude = -84.38;
	const double latitude  = 45.14;
	const int offset_x     = 6;
	const int offset_y     = 8;

	top_left     = 2;
	bottom_left  = 1;
	bottom_right = 34;

	EXPECT_CALL(*raw_raster, get_width()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, get_height()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y)).WillOnce(Return(top_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y)).WillOnce(Return(top_right));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y + 1)).WillOnce(Return(bottom_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y + 1)).WillOnce(Return(bottom_right));

	auto elevation = tile->lookup_datum(latitude, longitude);
	// Missing value filled w/ top left pixel, 2
	ASSERT_TRUE(elevation.first);
	EXPECT_NEAR(5.36, elevation.second, 0.01);
}

TEST_F(TileTest, read_pixel_no_elevation_bottom_left) {
	const double longitude = -84.38;
	const double latitude  = 45.14;
	const int offset_x     = 6;
	const int offset_y     = 8;

	top_left     = 2;
	top_right    = 25;
	bottom_right = 34;

	EXPECT_CALL(*raw_raster, get_width()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, get_height()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y)).WillOnce(Return(top_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y)).WillOnce(Return(top_right));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y + 1)).WillOnce(Return(bottom_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y + 1)).WillOnce(Return(bottom_right));

	auto elevation = tile->lookup_datum(latitude, longitude);
	// Filled with bottom right pixel, 34
	ASSERT_TRUE(elevation.first);
	EXPECT_NEAR(23.04, elevation.second, 0.01);
}


TEST_F(TileTest, read_pixel_no_elevation_bottom_right) {
	const double longitude = -84.38;
	const double latitude  = 45.14;
	const int offset_x     = 6;
	const int offset_y     = 8;

	top_left    = 2;
	top_right   = 25;
	bottom_left = 8;

	EXPECT_CALL(*raw_raster, get_width()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, get_height()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y)).WillOnce(Return(top_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y)).WillOnce(Return(top_right));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y + 1)).WillOnce(Return(bottom_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y + 1)).WillOnce(Return(bottom_right));

	auto elevation = tile->lookup_datum(latitude, longitude);
	// Takes bottom left pixel, 8
	ASSERT_TRUE(elevation.first);
	EXPECT_NEAR(7.44, elevation.second, 0.01);
}

TEST_F(TileTest, read_pixel_no_elevation_all) {
	const double longitude = -84.38;
	const double latitude  = 45.14;
	const int offset_x     = 6;
	const int offset_y     = 8;

	EXPECT_CALL(*raw_raster, get_width()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, get_height()).WillRepeatedly(Return(size));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y)).WillOnce(Return(top_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y)).WillOnce(Return(top_right));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x, offset_y + 1)).WillOnce(Return(bottom_left));
	EXPECT_CALL(*raw_raster, read_pixel(offset_x + 1, offset_y + 1)).WillOnce(Return(bottom_right));

	auto elevation = tile->lookup_datum(latitude, longitude);

	EXPECT_FALSE(elevation.first);
}
}  // namespace geospatial
}  // namespace navtk
