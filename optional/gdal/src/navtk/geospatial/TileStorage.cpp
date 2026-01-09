#include <navtk/geospatial/TileStorage.hpp>

#include <stdexcept>

#include <navtk/errors.hpp>

namespace navtk {
namespace geospatial {

TileStorage::TileStorage(unsigned int max_size) : max_size(max_size) {
	if (max_size == 0) {
		log_or_throw<std::invalid_argument>("Tile storage size can not be zero.");
	}
}

void TileStorage::add_tile(not_null<std::shared_ptr<Tile>> tile) {
	if (tiles.size() >= max_size) {
		tiles.front()->unload();
		tiles.pop_front();
	}
	tiles.emplace_back(tile);
}

size_t TileStorage::get_size() const { return tiles.size(); }

bool TileStorage::is_stored(const std::string& filename) const {
	for (auto tile : tiles) {
		if (tile->get_filename() == filename) {
			return true;
		}
	}
	return false;
}

std::shared_ptr<Tile> TileStorage::get_tile(const std::string& filename) const {
	for (auto iter = tiles.rbegin(); iter != tiles.rend(); ++iter) {
		auto tile = *iter;
		if (tile->get_filename() == filename) {
			// Move the tile to top by removing it and adding it again.
			tiles.erase((iter + 1).base());
			tiles.emplace_back(tile);
			return tile;
		}
	}
	return nullptr;
}

}  // namespace geospatial
}  // namespace navtk
