#include <gtest/gtest.h>

#include <error_mode_assert.hpp>
#include <navtk/geospatial/MockRaster.hpp>
#include <navtk/geospatial/Tile.hpp>
#include <navtk/geospatial/TileStorage.hpp>

using testing::Return;
using testing::Throw;

namespace navtk {
namespace geospatial {

class TileStorageTest : public testing::Test {
protected:
	void SetUp() override {
		raster_1     = std::make_unique<MockRaster>();
		raster_2     = std::make_unique<MockRaster>();
		raster_3     = std::make_unique<MockRaster>();
		raw_raster_1 = raster_1.get();
		raw_raster_2 = raster_2.get();
		raw_raster_3 = raster_3.get();
		tile_1       = std::make_shared<Tile>(std::move(raster_1));
		tile_2       = std::make_shared<Tile>(std::move(raster_2));
		tile_3       = std::make_shared<Tile>(std::move(raster_3));
	}

	std::unique_ptr<MockRaster> raster_1;
	std::unique_ptr<MockRaster> raster_2;
	std::unique_ptr<MockRaster> raster_3;
	MockRaster* raw_raster_1;
	MockRaster* raw_raster_2;
	MockRaster* raw_raster_3;
	std::shared_ptr<Tile> tile_1;
	std::shared_ptr<Tile> tile_2;
	std::shared_ptr<Tile> tile_3;
};

TEST_F(TileStorageTest, add_single_max_2) {
	TileStorage storage(2);
	storage.add_tile(tile_1);
	ASSERT_EQ(1, storage.get_size());
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, TileStorageTest, zero_max_size_throws_exception) {
	// construct shared_ptr instead of reference to avoid dangling pointer warning (see navtk#965)
	EXPECT_HONORS_MODE_EX(*std::make_shared<TileStorage>(0), "Tile storage", std::invalid_argument);
}

TEST_F(TileStorageTest, add_multiple_max_1) {

	EXPECT_CALL(*raw_raster_2, get_name()).WillRepeatedly(Return("pork.dt2"));

	TileStorage storage(1);
	storage.add_tile(tile_1);
	storage.add_tile(tile_2);

	ASSERT_EQ(1, storage.get_size());
	ASSERT_EQ(true, storage.is_stored("pork.dt2"));
}

TEST_F(TileStorageTest, add_multiple_remove_least_recently_used) {

	EXPECT_CALL(*raw_raster_1, get_name()).WillRepeatedly(Return("chicken.dt1"));
	EXPECT_CALL(*raw_raster_2, get_name()).WillRepeatedly(Return("pork.dt2"));
	EXPECT_CALL(*raw_raster_3, get_name()).WillRepeatedly(Return("beef.dt3"));

	TileStorage storage(2);
	storage.add_tile(tile_1);
	storage.add_tile(tile_2);
	auto tile = storage.get_tile("chicken.dt1");
	storage.add_tile(tile_3);

	ASSERT_EQ(2, storage.get_size());
	ASSERT_EQ(true, storage.is_stored("chicken.dt1"));
	ASSERT_EQ(true, storage.is_stored("beef.dt3"));
}
}  // namespace geospatial
}  // namespace navtk
