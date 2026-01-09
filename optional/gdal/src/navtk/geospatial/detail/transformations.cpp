#include <navtk/geospatial/detail/transformations.hpp>

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

OGRSpatialReference import_frame_from_dataset(GDALDataset& dataset) {
	OGRSpatialReference frame;

	// Get the projection.
#if GDAL_COMPUTE_VERSION(2, 3, 0) <= GDAL_VERSION_NUM
	frame.importFromWkt(dataset.GetProjectionRef());
#else
	const char* frame_ref_const = dataset.GetProjectionRef();

	// Make a copy in order to lose the const modifier.
	const uint16_t MAX_PROJECTION_REF_LENGTH = 1000;
	char* frame_ref_original                 = strndup(frame_ref_const, MAX_PROJECTION_REF_LENGTH);

	// Make a copy of the pointer so that the original can be freed after the copy has been
	// modified.
	char* frame_ref = frame_ref_original;
	frame.importFromWkt(&frame_ref);
	free(frame_ref_original);
#endif

	// In GDAL 3.0.0 the default axis mapping strategy was changed to be consistent with the
	// standard axis mapping in other frames. Traditionally GIS axes are [lat, lon] while WGS84 is
	// [lon, lat]. This is a breaking change across versions to the SetAxisMappingStrategy method
	// was added in 3.0.0 to help with migration. For more information, see:
	// https://trac.osgeo.org/gdal/wiki/rfc73_proj6_wkt2_srsbarn#Axisorderissues
#if GDAL_COMPUTE_VERSION(3, 0, 0) <= GDAL_VERSION_NUM
	frame.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
#endif

	return frame;
}

OGRSpatialReference import_frame_from_wgs84() {
	OGRSpatialReference frame;

	frame.SetWellKnownGeogCS("WGS84");
	// In GDAL 3.0.0 the default axis mapping strategy was changed to be consistent with the
	// standard axis mapping in other frames. Traditionally GIS axes are [lat, lon] while WGS84 is
	// [lon, lat]. This is a breaking change across versions to the SetAxisMappingStrategy method
	// was added in 3.0.0 to help with migration. For more information, see:
	// https://trac.osgeo.org/gdal/wiki/rfc73_proj6_wkt2_srsbarn#Axisorderissues
#if GDAL_COMPUTE_VERSION(3, 0, 0) <= GDAL_VERSION_NUM
	frame.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
#endif

	return frame;
}

std::unique_ptr<OGRCoordinateTransformation, TransformDelete> create_wgs84_to_map_transformation(
    GDALDataset& dataset) {
	// The source projection.
	auto source = import_frame_from_wgs84();

	// The target projection.
	auto target = import_frame_from_dataset(dataset);

	// Create the transform - this can be used repeatedly
	return std::unique_ptr<OGRCoordinateTransformation, TransformDelete>{
	    OGRCreateCoordinateTransformation(&source, &target)};
}

std::unique_ptr<OGRCoordinateTransformation, TransformDelete> create_map_to_wgs84_transformation(
    GDALDataset& dataset) {
	// The source projection.
	auto source = import_frame_from_dataset(dataset);

	// The target projection.
	auto target = import_frame_from_wgs84();

	// Create the transform - this can be used repeatedly
	return std::unique_ptr<OGRCoordinateTransformation, TransformDelete>{
	    OGRCreateCoordinateTransformation(&source, &target)};
}

}  // namespace detail
}  // namespace geospatial
}  // namespace navtk
