#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/errors.hpp>
#include <navtk/geospatial/providers/SimpleElevationProvider.hpp>
#include <navtk/geospatial/sources/GdalSource.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace geospatial {

using navtk::navutils::DEG2RAD;

// These tests demonstrate that the SimpleElevationProvider can properly abstract away any need to
// call the GdalSource methods. By adding multiple sources to the provider, the user can let the
// provider do all the interfacing between its collection of sources. These tests also verify that
// specifying different output vertical reference frames will result in different outputs.
class SimpleElevationProviderTest : public ::testing::Test {
public:
	char* map_path;
	std::vector<not_null<std::shared_ptr<ElevationSource>>> sources;

	SimpleElevationProviderTest() {
		map_path = getenv("NAVTK_DATA_DIR");

		if (map_path == NULL) {
			log_or_throw(
			    "Environment variable 'NAVTK_DATA_DIR' not set. Must be set to directory "
			    "containing GDAL files.");
		} else {
			sources.emplace_back(
			    std::make_shared<GdalSource>(map_path,
			                                 GdalSource::MapType::GEOTIFF,
			                                 ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE,
			                                 ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE));
			sources.emplace_back(
			    std::make_shared<GdalSource>(map_path,
			                                 GdalSource::MapType::DTED,
			                                 ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL,
			                                 ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL));
		}
	}

	void test_valid_queries(std::shared_ptr<SimpleElevationProvider> provider,
	                        const Matrix& valid_coordinates,
	                        const Vector& expected_elevations) {
		auto resulting_elevations = zeros(num_rows(valid_coordinates));
		for (Size ii = 0; ii < num_rows(valid_coordinates); ++ii) {
			auto elevation = provider->lookup_datum(valid_coordinates(ii, 0) * DEG2RAD,
			                                        valid_coordinates(ii, 1) * DEG2RAD);
			EXPECT_TRUE(elevation.first);
			resulting_elevations(ii) = elevation.second;
		}

		ASSERT_ALLCLOSE_EX(expected_elevations, resulting_elevations, 0.05, 0.0);
	}

	void test_invalid_queries(std::shared_ptr<SimpleElevationProvider> provider,
	                          const Matrix& invalid_coordinates) {
		for (Size ii = 0; ii < num_rows(invalid_coordinates); ++ii) {
			auto elevation = provider->lookup_datum(invalid_coordinates(ii, 0) * DEG2RAD,
			                                        invalid_coordinates(ii, 1) * DEG2RAD);
			EXPECT_FALSE(elevation.first);
		}
	}
};

// Check that the provider can read from both the GeoTIFF and DTED sources.
TEST_F(SimpleElevationProviderTest, compare_against_command_line_gdal) {
	// Construct provider with vector of sources. Since unsupported elevation reference frame of AGL
	// is specified, provides elevations in reference frame of sources.
	std::shared_ptr<SimpleElevationProvider> provider_unsupported_ref =
	    std::make_shared<SimpleElevationProvider>(sources, ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL);

	// Series of arbitrary coordinates inside the GeoTIFF and DTED test tiles. Each row is
	// {latitude, longitude} in degrees.
	const Matrix QUERY_COORDINATES   = {{-3.597411, -78.89943}, {30.5, -82}};
	const Vector EXPECTED_ELEVATIONS = {82, 25};

	test_valid_queries(provider_unsupported_ref, QUERY_COORDINATES, EXPECTED_ELEVATIONS);
}

// Test point that neither the GeoTIFF nor DTED source cover
TEST_F(SimpleElevationProviderTest, get_elevation_outside_of_tiles) {
	std::shared_ptr<SimpleElevationProvider> provider =
	    std::make_shared<SimpleElevationProvider>(sources);

	const Matrix QUERY_COORDINATES = {{0, 0}};

	test_invalid_queries(provider, QUERY_COORDINATES);
}

// Check the same points as before, but with the provider only outputting HAE elevations.
TEST_F(SimpleElevationProviderTest, get_elevation_hae_SLOW) {
	// provides elevations in HAE
	std::shared_ptr<SimpleElevationProvider> provider_hae =
	    std::make_shared<SimpleElevationProvider>(sources, ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE);

	const Matrix QUERY_COORDINATES   = {{-3.597411, -78.89943}, {30.5, -82}};
	const Vector EXPECTED_ELEVATIONS = {82, -4.5};

	test_valid_queries(provider_hae, QUERY_COORDINATES, EXPECTED_ELEVATIONS);
}

// Same test but with output in mean sea level (MSL) as the vertical reference frame.
TEST_F(SimpleElevationProviderTest, get_elevation_msl_SLOW) {
	// provides elevations in MSL
	std::shared_ptr<SimpleElevationProvider> provider_msl =
	    std::make_shared<SimpleElevationProvider>(sources, ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL);

	const Matrix QUERY_COORDINATES   = {{-3.597411, -78.89943}, {30.5, -82}};
	const Vector EXPECTED_ELEVATIONS = {64, 25};

	test_valid_queries(provider_msl, QUERY_COORDINATES, EXPECTED_ELEVATIONS);
}

// Test different ways of constructing provider. Other tests check that the provider can be
// constructed with a vector of sources, so that case is excluded from this test.
TEST_F(SimpleElevationProviderTest, constructors) {
	const Matrix QUERY_COORDINATES   = {{-3.597411, -78.89943}};
	const Vector EXPECTED_ELEVATIONS = {82};

	// Construct with no sources and add a source later.
	std::shared_ptr<SimpleElevationProvider> provider = std::make_shared<SimpleElevationProvider>();
	provider->add_source(std::make_shared<GdalSource>(
	    map_path, GdalSource::MapType::GEOTIFF, ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE));

	test_valid_queries(provider, QUERY_COORDINATES, EXPECTED_ELEVATIONS);

	// Construct with a single source
	provider = std::make_shared<SimpleElevationProvider>(std::make_shared<GdalSource>(
	    map_path, GdalSource::MapType::GEOTIFF, ASPN_MEASUREMENT_ALTITUDE_REFERENCE_HAE));

	test_valid_queries(provider, QUERY_COORDINATES, EXPECTED_ELEVATIONS);
}


TEST_F(SimpleElevationProviderTest, unsupported_reference_frame) {
	// init output reference to unsupported type
	EXPECT_WARN(
	    std::make_shared<SimpleElevationProvider>(sources, ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL),
	    "ASPN_MEASUREMENT_ALTITUDE_REFERENCE_AGL is not supported");
}
}  // namespace geospatial
}  // namespace navtk
