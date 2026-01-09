#pragma once

namespace navtk {
namespace geospatial {
namespace detail {

/**
 * A custom deleter for a pointer to an OGRCoordinateTransformation object. Allows the use of this
 * object with a unique pointer instead of the raw pointer returned by GDAL.
 */
struct TransformDelete {
	/**
	 * \param ptr A pointer to an OGRCoordinateTransformation object.
	 */
	void operator()(void* ptr) const;
};

/**
 * A custom deleter for a pointer to a GDALDataset object. Allows the use of this object with a
 * unique pointer instead of the raw pointer returned by GDAL.
 */
struct DatasetDelete {
	/**
	 * \param ptr A pointer to a GDALDataset object.
	 */
	void operator()(void* ptr) const;
};

}  // namespace detail
}  // namespace geospatial
}  // namespace navtk
