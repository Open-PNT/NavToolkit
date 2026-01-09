#pragma once

#include <deque>
#include <memory>
#include <string>

#include <navtk/geospatial/Tile.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace geospatial {

/**
 * This class holds a specified number of GDAL tiles in
 * memory. If the number of tiles added exceeds the `max_size`
 * attribute, the tile least recently accessed will be removed.
 */
class TileStorage final {
public:
	/**
	 * Constructor
	 *
	 * @param max_size capacity of the storage.
	 * @throw std::invalid_argument if `max_size` is 0 and the error mode is ErrorMode::DIE.
	 */
	explicit TileStorage(unsigned int max_size);

	/**
	 * Adds a tile to the storage.
	 *
	 * @param tile a shared pointer to the GDAL tile.
	 */
	void add_tile(not_null<std::shared_ptr<Tile>> tile);

	/**
	 * Returns the number of tiles currently stored.
	 *
	 * @return Number of tiles stored.
	 */
	size_t get_size() const;

	/**
	 * Checks to see if a tile with the given filename
	 * is currently stored.
	 *
	 * @param filename the filename of the tile
	 * @return `true` if found, `false` otherwise.
	 */
	bool is_stored(const std::string& filename) const;

	/**
	 * Gets the tile with the given filename
	 *
	 * @param filename filename of the tile
	 * @return A shared pointer to the GDAL tile, or `nullptr` if the filename is not found.
	 */
	std::shared_ptr<Tile> get_tile(const std::string& filename) const;

private:
	/**
	 * Stored tiles
	 */
	mutable std::deque<std::shared_ptr<Tile>> tiles;

	TileStorage(const TileStorage&)            = delete;
	TileStorage& operator=(const TileStorage&) = delete;

	size_t max_size;
};
}  // namespace geospatial
}  // namespace navtk
