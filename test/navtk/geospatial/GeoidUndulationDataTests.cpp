#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/errors.hpp>
#include <navtk/geospatial/sources/GeoidUndulationSource.hpp>
#include <navtk/navutils/math.hpp>

namespace navtk {
namespace geospatial {

// This test asserts that the geoid undulation data class returns the expected MSL minus HAE values
// for a given set of latitude and longitude values using 'ww15mgh.grd,' a data file obtained from
// https://earth-info.gs.mil/

// If the latitude and longitude coordinates do not fall exactly on a point in the file,
// interpolation is used to determine the undulation. The primary error source in the results is
// this interpolation.

using navtk::navutils::DEG2RAD;

class GeoidUndulationDataTest : public ::testing::Test {
public:
	std::shared_ptr<GeoidUndulationSource> source = GeoidUndulationSource::get_shared();
	void test_valid_queries(const Matrix& query_coordinates, const Vector& expected_undulations) {
		auto resulting_undulations = zeros(num_rows(query_coordinates));
		for (Size i = 0; i < num_rows(query_coordinates); ++i) {
			auto undulation = source->lookup_datum(query_coordinates(i, 0) * DEG2RAD,
			                                       query_coordinates(i, 1) * DEG2RAD);
			EXPECT_TRUE(undulation.first);
			resulting_undulations(i) = undulation.second;
		}

		ASSERT_ALLCLOSE_EX(expected_undulations, resulting_undulations, 0.005, 0.0);
	}

	void test_invalid_queries(const Matrix& invalid_coordinates) {
		// Turn on error mode so that validation is not skipped.
		auto guard = ErrorModeLock(ErrorMode::DIE);
		for (Size i = 0; i < num_rows(invalid_coordinates); ++i) {
			auto undulation = EXPECT_WARN(source->lookup_datum(invalid_coordinates(i, 0) * DEG2RAD,
			                                                   invalid_coordinates(i, 1) * DEG2RAD),
			                              "outside range");
			EXPECT_FALSE(undulation.first);
		}
	}
};

TEST_F(GeoidUndulationDataTest, without_interpolation) {
	// 3 coordinates that fall right on a point in the ww15mgh.grd file
	// clang-format off
	const Matrix query_coordinates = {{90   ,  35},  // line 17, index 3 in ww15mgh.grd
	                                  {-3.75, -79},  // line 71775, index 4
	                                  {-3.5 , -79}}; // line 71584, index 4
	// clang-format on

	const Vector expected_undulations = {13.606, 17.860, 18.908};

	test_valid_queries(query_coordinates, expected_undulations);
}

TEST_F(GeoidUndulationDataTest, with_interpolation) {
	// Series of arbitrary coordinates. Each row is {latitude, longitude} in degrees. The first 3
	// coordinates were hand-picked, the rest were from a test file located at the same source from
	// which 'ww15mgh.grd' was obtained (see above)
	// clang-format off
    const Matrix query_coordinates = {{ -3.699006 , -78.90912  },
                                      { -3.612280 , -79.05017  },
                                      { -3.795235 , -79.10016  },
                                      { 38.6281550, 269.7791550},
                                      {-14.6212170, 305.0211140},
                                      { 46.8743190, 102.4487290},
                                      {-23.6174460, 133.8747120},
                                      { 38.6254730, 359.9995000},
                                      {  -.4667440,    .0023000}};
	// clang-format on

	const Vector expected_undulations = {
	    17.8499, 18.3865, 17.8108, -31.628, -2.969, -43.575, 15.871, 50.066, 17.329};

	test_valid_queries(query_coordinates, expected_undulations);
}

TEST_F(GeoidUndulationDataTest, outside_range) {
	// clang-format off
    const Matrix invalid_coordinates = {{        100      ,         180      },
                                        {         90.00001,         180      },
                                        {       -100      ,         180      },
                                        {        -90.00001,         180      },
                                        {10000000000      , 10000000000      },
                                        {-1000000000      , 10000000000      }};
	// clang-format on

	test_invalid_queries(invalid_coordinates);
}

TEST_F(GeoidUndulationDataTest, not_a_number) {
	// Turn on error mode so that validation is not skipped.
	auto guard      = ErrorModeLock(ErrorMode::DIE);
	auto undulation = EXPECT_WARN(source->lookup_datum(NAN, NAN), "not a number");
	EXPECT_FALSE(undulation.first);
}

TEST_F(GeoidUndulationDataTest, file_endpoints) {
	// clang-format off
	const Matrix query_coordinates = {{  90     ,    0     },
	                                  {  90     ,  360     },
	                                  { -90     ,    0     },
	                                  { -90     ,  360     },
	                                  {  90     , -360     },
	                                  { -90     , -360     },
	                                  {  89.9999,  359.9999},
	                                  {  89.9999,    0.0001},
	                                  { -89.9999, -359.9999},
 	                                  { -89.9999,   -0.0001}};
	// clang-format on

	const Vector expected_undulations = {
	    13.606, 13.606, -29.534, -29.534, 13.606, -29.534, 13.606, 13.606, -29.534, -29.534};

	test_valid_queries(query_coordinates, expected_undulations);
}

TEST_F(GeoidUndulationDataTest, inputs_of_zero) {
	const Matrix query_coordinates = {{0, 0}, {0, 360}, {0, -360}};

	const double expected_undulation = 17.162;

	test_valid_queries(query_coordinates,
	                   {expected_undulation, expected_undulation, expected_undulation});
}

TEST_F(GeoidUndulationDataTest, cache_endpoints_SLOW) {
	// clang-format off
	const Matrix query_coordinates    = {{ 90        ,  35        },
									 	 { 80        ,  30        },
										 { 80        ,  40        },
										 { 90        ,  30        },
										 { 90        ,  40        },
										 { 89.9      ,  39.9      },
										 { 80.05     ,  34.95     },
										 { 38.6281550, 269.7791550},
										 { 33.75     , 264.75     },
										 { 33.75     , 274.75     },
										 { 43.75     , 264.75     },
										 { 43.75     , 274.75     },
										 { 43.749    , 264.751    },
										 { 38.62     , 274.749    }};
	// clang-format on
	const Vector expected_undulations = {13.606,
	                                     24.270,
	                                     18.290,
	                                     13.606,
	                                     13.606,
	                                     13.643,
	                                     21.950,
	                                     -31.628,
	                                     -28.479,
	                                     -30.135,
	                                     -27.548,
	                                     -33.134,
	                                     -27.55,
	                                     -34.642};

	test_valid_queries(query_coordinates, expected_undulations);
}

TEST_F(GeoidUndulationDataTest, large_cache_SLOW) {
	// clang-format off
	const Matrix invalid_coordinates = {{        100      ,         180      },
										{         90.00001,         180      },
										{       -100      ,         180      },
										{        -90.00001,         180      },
										{10000000000      , 10000000000      },
										{-1000000000      , 10000000000      }};
	const Matrix valid_coordinates   = {{  90        ,   35        },
										{  -3.75     ,  -79        },
										{  -3.5      ,  -79        },
										{  -3.699006 ,  -78.90912  },
										{  -3.612280 ,  -79.05017  },
										{  -3.795235 ,  -79.10016  },
										{  38.6281550,  269.7791550},
										{ -14.6212170,  305.0211140},
										{  46.8743190,  102.4487290},
										{ -23.6174460,  133.8747120},
										{  38.6254730,  359.9995000},
										{   -.4667440,     .0023000},
										{  90        ,    0        },
										{  90        ,  360        },
										{ -90        ,    0        },
										{ -90        ,  360        },
										{  90        , -360        },
										{ -90        , -360        },
										{  89.9999   ,  359.9999   },
										{  89.9999   ,    0.0001   },
										{ -89.9999   , -359.9999   },
										{ -89.9999   ,   -0.0001   },
										{   0        ,    0        },
										{   0        ,  360        },
										{   0        , -360        },
										{  90        ,   35        },
										{  40        ,   10        },
										{  40        ,   60        },
										{  90        ,   10        },
										{  90        ,   60        },
										{  89.9      ,   59.9      },
										{  40.05     ,   59.95     },
										{  38.6281550,  269.7791550},
										{  13.75     ,  244.75     },
										{  63.75     ,  294.75     },
										{  13.75     ,  294.75     },
										{  63.75     ,  244.75     },
										{  63.749    ,  244.751    },
										{  13.751    ,  294.749    },
										{  45        ,  -500       },
										{  45        ,  -360.00001 },
										{  45        ,   500       },
										{  45        ,   360.00001 }};
	// clang-format on

	const Vector expected_undulations = {
	    13.606,  17.860,  18.908,  17.8499, 18.3865, 17.8108, -31.628, -2.969, -43.575,
	    15.871,  50.066,  17.329,  13.606,  13.606,  -29.534, -29.534, 13.606, -29.534,
	    13.606,  13.606,  -29.534, -29.534, 17.162,  17.162,  17.162,  13.606, 45.726,
	    -28.329, 13.606,  13.606,  13.606,  -28.23,  -31.609, -40.79,  -1.865, -37.194,
	    -23.401, -23.404, -37.196, -23.638, 47.14,   28.156,  47.14};

	source->set_chunk_size(200);

	test_invalid_queries(invalid_coordinates);

	test_valid_queries(valid_coordinates, expected_undulations);
}

TEST_F(GeoidUndulationDataTest, small_cache) {
	// clang-format off
	const Matrix invalid_coordinates = {{        100      ,         180      },
										{         90.00001,         180      },
										{       -100      ,         180      },
										{        -90.00001,         180      },
										{10000000000      , 10000000000      },
										{-1000000000      , 10000000000      }};
	const Matrix valid_coordinates   = {{  90        ,   35        },
										{  -3.75     ,  -79        },
										{  -3.5      ,  -79        },
										{  -3.699006 ,  -78.90912  },
										{  -3.612280 ,  -79.05017  },
										{  -3.795235 ,  -79.10016  },
										{  38.6281550,  269.7791550},
										{ -14.6212170,  305.0211140},
										{  46.8743190,  102.4487290},
										{ -23.6174460,  133.8747120},
										{  38.6254730,  359.9995000},
										{   -.4667440,     .0023000},
										{  90        ,    0        },
										{  90        ,  360        },
										{ -90        ,    0        },
										{ -90        ,  360        },
										{  90        , -360        },
										{ -90        , -360        },
										{  89.9999   ,  359.9999   },
										{  89.9999   ,    0.0001   },
										{ -89.9999   , -359.9999   },
										{ -89.9999   ,   -0.0001   },
										{   0        ,    0        },
										{   0        ,  360        },
										{   0        , -360        },
										{   45       , -500        },
										{   45       , -360.00001  },
										{   45       ,  500        },
										{   45       ,  360.00001  }};
	// clang-format on

	const Vector expected_undulations = {
	    13.606,  17.860,  18.908, 17.8499, 18.3865, 17.8108, -31.628, -2.969,  -43.575, 15.871,
	    50.066,  17.329,  13.606, 13.606,  -29.534, -29.534, 13.606,  -29.534, 13.606,  13.606,
	    -29.534, -29.534, 17.162, 17.162,  17.162,  -23.638, 47.14,   28.156,  47.14};

	source->set_chunk_size(1);

	test_invalid_queries(invalid_coordinates);

	test_valid_queries(valid_coordinates, expected_undulations);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, GeoidUndulationDataTest, invalid_cache_size) {
	test.source->set_chunk_size(40);
	EXPECT_HONORS_MODE(test.source->set_chunk_size(0), "Invalid chunk size");
	EXPECT_EQ(test.source->get_chunk_size(), 40);
	EXPECT_HONORS_MODE(test.source->set_chunk_size(1000), "Invalid chunk size");
	EXPECT_EQ(test.source->get_chunk_size(), 40);
	EXPECT_HONORS_MODE(test.source->set_chunk_size(-40), "Invalid chunk size");
	EXPECT_EQ(test.source->get_chunk_size(), 40);
}
}  // namespace geospatial
}  // namespace navtk
