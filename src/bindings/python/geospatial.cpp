#include <memory>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <navtk/aspn.hpp>
#include <navtk/geospatial/ElevationInterpolator.hpp>
#include <navtk/geospatial/providers/SimpleElevationProvider.hpp>
#include <navtk/geospatial/providers/SimpleProvider.hpp>
#include <navtk/geospatial/providers/SpatialMapDataProvider.hpp>
#include <navtk/geospatial/sources/ElevationSource.hpp>
#include <navtk/geospatial/sources/GeoidUndulationSource.hpp>
#include <navtk/geospatial/sources/SpatialMapDataSource.hpp>

#ifdef NAVTK_GDAL_ENABLED
#	include <navtk/geospatial/sources/GdalSource.hpp>
#	include <navtk/geospatial/detail/custom_deleters.hpp>
#	include <navtk/geospatial/detail/transformations.hpp>
#	include <navtk/geospatial/Raster.hpp>
#	include <navtk/geospatial/Post.hpp>
#	include <navtk/geospatial/Tile.hpp>
#	include <navtk/geospatial/TileStorage.hpp>
#	include <navtk/geospatial/GdalRaster.hpp>
#	ifndef GDAL_INCLUDE_IN_SUBFOLDER
#		include <gdal_priv.h>
#		include <ogr_spatialref.h>
#	else
#		include <gdal/gdal_priv.h>
#		include <gdal/ogr_spatialref.h>
#	endif
#endif

#include "binding_helpers.hpp"

namespace geo = navtk::geospatial;
using namespace pybind11::literals;

using geo::ElevationInterpolator;
using geo::ElevationSource;
using geo::GeoidUndulationSource;
using geo::SimpleElevationProvider;
using geo::SimpleProvider;
using geo::SpatialMapDataProvider;
using geo::SpatialMapDataSource;
using navtk::not_null;

#ifdef NAVTK_GDAL_ENABLED
using geo::GdalRaster;
using geo::GdalSource;
using geo::Post;
using geo::Raster;
using geo::Tile;
using geo::TileStorage;
using geo::detail::DatasetDelete;
using geo::detail::TransformDelete;

template <class RasterBase = Raster>
class PyRaster : public RasterBase {
public:
	using RasterBase::RasterBase;

	int get_width() const override { PYBIND11_OVERRIDE_PURE(int, RasterBase, get_width, ); }

	int get_height() const override { PYBIND11_OVERRIDE_PURE(int, RasterBase, get_height, ); }

	std::pair<double, double> wgs84_to_pixel(double latitude, double longitude) const override {
		PYBIND11_OVERRIDE_PURE(
		    PARAMS(std::pair<double, double>), RasterBase, wgs84_to_pixel, latitude, longitude);
	}

	double read_pixel(size_t idx_x, size_t idx_y) override {
		PYBIND11_OVERRIDE_PURE(double, RasterBase, read_pixel, idx_x, idx_y);
	}

	std::string get_name() const override {
		PYBIND11_OVERRIDE_PURE(std::string, RasterBase, get_name, );
	}

	bool is_valid_data(double data) const override {
		PYBIND11_OVERRIDE_PURE(bool, RasterBase, is_valid_data, data);
	}

	void unload() override { PYBIND11_OVERRIDE_PURE(void, RasterBase, unload, ); }
};
#endif

template <class SpatialMapDataProviderBase = SpatialMapDataProvider>
class PySpatialMapDataProvider : public SpatialMapDataProviderBase {
public:
	using SpatialMapDataProviderBase::SpatialMapDataProviderBase;

	std::pair<bool, double> lookup_datum(double latitude, double longitude) const override {
		PYBIND11_OVERRIDE_PURE(PARAMS(std::pair<bool, double>),
		                       SpatialMapDataProviderBase,
		                       lookup_datum,
		                       latitude,
		                       longitude);
	}
};

template <class SpatialMapDataSourceBase = SpatialMapDataSource>
class PySpatialMapDataSource : public SpatialMapDataSourceBase {
public:
	std::pair<bool, double> lookup_datum(double latitude, double longitude) const override {
		PYBIND11_OVERRIDE_PURE(PARAMS(std::pair<bool, double>),
		                       SpatialMapDataSourceBase,
		                       lookup_datum,
		                       latitude,
		                       longitude);
	}
};

template <class ElevationSourceBase = ElevationSource>
class PyElevationSource : public PySpatialMapDataSource<ElevationSourceBase> {
public:
	AspnMeasurementAltitudeReference get_output_vertical_reference_frame() const override {
		PYBIND11_OVERRIDE(AspnMeasurementAltitudeReference,
		                  ElevationSourceBase,
		                  get_output_vertical_reference_frame, );
	}

	void set_output_vertical_reference_frame(AspnMeasurementAltitudeReference new_ref) override {
		PYBIND11_OVERRIDE_PURE(
		    void, ElevationSourceBase, set_output_vertical_reference_frame, new_ref);
	}
};

void add_geospatial_functions(pybind11::module &m) {
	m.doc() = "Classes and utilties for reading geographic spatial map data.";

	CLASS(ElevationInterpolator)
	CTOR(ElevationInterpolator,
	     PARAMS(double, double, double, double),
	     "top_left"_a,
	     "top_right"_a,
	     "bottom_left"_a,
	     "bottom_right"_a)
	METHOD(ElevationInterpolator, interpolate, "fractions"_a) CDOC(ElevationInterpolator);

	CLASS(SpatialMapDataProvider, PySpatialMapDataProvider<>)
	CTOR(SpatialMapDataProvider, std::shared_ptr<SpatialMapDataSource>, "src"_a)
	CTOR_OVERLOAD(SpatialMapDataProvider,
	              std::vector<not_null<std::shared_ptr<SpatialMapDataSource>>>,
	              _2,
	              "srcs"_a = std::vector<not_null<std::shared_ptr<SpatialMapDataSource>>>{})
	METHOD(SpatialMapDataProvider, add_source, "src"_a)
	METHOD(SpatialMapDataProvider, lookup_datum, "latitude"_a, "longitude"_a)
	CDOC(SpatialMapDataProvider);

	CLASS(SimpleProvider, SpatialMapDataProvider)
	// have to redefine constructor bindings because pybind classes don't inherit constructors
	CTOR_NODOC(std::shared_ptr<SpatialMapDataSource>, "src"_a)
	CTOR_NODOC(std::vector<not_null<std::shared_ptr<SpatialMapDataSource>>>,
	           "srcs"_a = std::vector<not_null<std::shared_ptr<SpatialMapDataSource>>>{})
	CDOC(SimpleProvider);

	CLASS(SimpleElevationProvider, SimpleProvider)
	CTOR(SimpleElevationProvider,
	     PARAMS(std::shared_ptr<ElevationSource>, AspnMeasurementAltitudeReference),
	     "src"_a,
	     "out_ref"_a = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE)
	CTOR_OVERLOAD(SimpleElevationProvider,
	              PARAMS(std::vector<not_null<std::shared_ptr<ElevationSource>>>,
	                     AspnMeasurementAltitudeReference),
	              _2,
	              "srcs"_a    = std::vector<not_null<std::shared_ptr<ElevationSource>>>{},
	              "out_ref"_a = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE)
	CDOC(SimpleElevationProvider);

	CLASS(SpatialMapDataSource, PySpatialMapDataSource<>)
	CTOR_NODOC_DEFAULT
	METHOD(SpatialMapDataSource, lookup_datum, "latitude"_a, "longitude"_a)
	CDOC(SpatialMapDataSource);

	CLASS(ElevationSource, SpatialMapDataSource, PyElevationSource<>)
	METHOD_VOID(ElevationSource, get_output_vertical_reference_frame)
	METHOD(ElevationSource, set_output_vertical_reference_frame, "new_ref"_a)
	CDOC(ElevationSource);

	// clang-format off
	CLASS(GeoidUndulationSource, SpatialMapDataSource)
	.def_static("get_shared",
	     &GeoidUndulationSource::get_shared,
	     PROCESS_DOC(GeoidUndulationSource_get_shared), "path"_a = std::string("ww15mgh.grd"))
	METHOD(GeoidUndulationSource, set_chunk_size, "size"_a)
	METHOD_VOID(GeoidUndulationSource, get_chunk_size)
	CDOC(GeoidUndulationSource);

#ifdef NAVTK_GDAL_ENABLED
// Redefine DOCFMT because docs extraction doesn't work on gdal.
// This prevents the helper macros from binding_helpers.hpp (included above)
// from trying to access docstrings for bindings declared in this file.
#	undef DOCFMT
#	define DOCFMT(NAME) docfmt("")

	NAMESPACE_FUNCTION(import_frame_from_dataset, geo::detail, "dataset"_a)
	NAMESPACE_FUNCTION_VOID(import_frame_from_wgs84, geo::detail)
	NAMESPACE_FUNCTION(create_wgs84_to_map_transformation, geo::detail, "dataset"_a)

	// clang-format off
	CLASS(TransformDelete)
	.def("__call__", &TransformDelete::operator())
	CDOC(TransformDelete);

	CLASS(DatasetDelete)
	.def("__call__", &DatasetDelete::operator())
	CDOC(DatasetDelete);

	auto gdal_source = CLASS(GdalSource, ElevationSource);
	gdal_source CTOR(GdalSource,
	                 PARAMS(const std::string &,
	                        GdalSource::MapType,
	                        AspnMeasurementAltitudeReference,
	                        AspnMeasurementAltitudeReference,
	                        unsigned int,
							const std::string &
							),
	                 "map_path"_a,
	                 "type"_a,
	                 "in_ref"_a = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE,
	                 "out_ref"_a = ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE,
	                 "num_tiles"_a = 10,
					 "undulation_path"_a = "ww15mgh.grd"
					 )
	CDOC(GdalSource);
	// clang-format on

	CLASS(Tile)
	CTOR(Tile, std::shared_ptr<Raster>, "raster"_a)
	METHOD(Tile, lookup_datum, "latitude"_a, "longitude"_a)
	METHOD_VOID(Tile, get_filename)
	METHOD(Tile, contains, "latitude"_a, "longitude"_a)
	METHOD_VOID(Tile, unload)
	REPR(Tile)
	CDOC(Tile);

	ENUM_SCOPED(MapType, GdalSource, gdal_source)
	CHOICE_SCOPED(MapType, GdalSource, GEOTIFF)
	CHOICE_SCOPED(MapType, GdalSource, DTED);

	CLASS(Raster, PyRaster<>)
	CTOR_NODOC_DEFAULT
	METHOD_VOID(Raster, get_width)
	METHOD_VOID(Raster, get_height)
	METHOD_VOID(Raster, get_name)
	METHOD(Raster, is_valid_data, "data"_a)
	METHOD(Raster, wgs84_to_pixel, "latitude"_a, "longitude"_a)
	METHOD(Raster, read_pixel, "idx_x"_a, "idx_y"_a)
	METHOD_VOID(Raster, unload)
	CDOC(Raster);

	CLASS(GdalRaster, Raster)
	CTOR(GdalRaster,
	     PARAMS(const std::string &, const std::string &),
	     "filename"_a,
	     "undulation_path"_a = "ww15mgh.grd")
	METHOD_VOID(GdalRaster, is_valid)
	METHOD_VOID(GdalRaster, scan_tile)
	CDOC(GdalRaster);

	CLASS(Post)
	CTOR_NODOC(PARAMS(int, int), "x"_a, "y"_a)
	FIELD(Post, x)
	FIELD(Post, y)
	CDOC(Post);

	CLASS(TileStorage)
	CTOR(TileStorage, unsigned int, "max_size"_a)
	METHOD(TileStorage, add_tile, "tile"_a)
	METHOD_VOID(TileStorage, get_size)
	METHOD(TileStorage, is_stored, "filename"_a)
	METHOD(TileStorage, get_tile, "filename"_a)
	CDOC(TileStorage);
#endif
}
