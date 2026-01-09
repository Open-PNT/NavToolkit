#include <navtk/geospatial/detail/custom_deleters.hpp>

#ifndef GDAL_INCLUDE_IN_SUBFOLDER
#	include <gdal_priv.h>
#	include <ogr_spatialref.h>
#else
#	include <gdal/gdal_priv.h>
#	include <gdal/ogr_spatialref.h>
#endif

namespace navtk {
namespace geospatial {
namespace detail {

void TransformDelete::operator()(void* ptr) const { OCTDestroyCoordinateTransformation(ptr); }

void DatasetDelete::operator()(void* ptr) const { GDALClose((GDALDatasetH)ptr); }

}  // namespace detail
}  // namespace geospatial
}  // namespace navtk
