#include <navtk/geospatial/providers/SpatialMapDataProvider.hpp>

#include <memory>
#include <utility>
#include <vector>

#include <navtk/not_null.hpp>

namespace navtk {
namespace geospatial {

SpatialMapDataProvider::SpatialMapDataProvider(
    not_null<std::shared_ptr<SpatialMapDataSource>> src) {
	sources.push_back(src);
}

SpatialMapDataProvider::SpatialMapDataProvider(
    std::vector<not_null<std::shared_ptr<SpatialMapDataSource>>> srcs)
    : sources(srcs) {
	if (sources.empty()) {
		spdlog::warn("SpatialMapDataProvider created without any sources.");
	}
}

void SpatialMapDataProvider::add_source(not_null<std::shared_ptr<SpatialMapDataSource>> src) {
	sources.push_back(src);
}

}  // namespace geospatial
}  // namespace navtk