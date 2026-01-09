#include <gtest/gtest.h>

#include <error_mode_assert.hpp>

#include <navtk/errors.hpp>
#include <navtk/utils/OutlierDetection.hpp>
#include <navtk/utils/OutlierDetectionSigma.hpp>
#include <navtk/utils/OutlierDetectionThreshold.hpp>

using navtk::ErrorMode;
using navtk::ErrorModeLock;
using navtk::utils::OutlierDetection;
using navtk::utils::OutlierDetectionSigma;
using navtk::utils::OutlierDetectionThreshold;

struct OutlierDetectionTests : public ::testing::Test {

	static const int PARAMETERS_COUNT = 5;
	static const size_t HISTORY_COUNT = 9;

	size_t history_size;
	navtk::Vector data;

	OutlierDetectionTests()
	    : ::testing::Test(),
	      history_size{HISTORY_COUNT},
	      data{
	          10.1, 10.0, 9.7, 11.3, 30.0, 9.9, 32.0, 9.9, 9.7, 6.8, 10.5, 9.8, 50.0, 100.0, 10.3} {
	}
};

TEST_F(OutlierDetectionTests, testBadInputBufferSize) {
	auto guard = ErrorModeLock(ErrorMode::DIE);

	EXPECT_THROW(OutlierDetectionSigma(4, -403.9), std::invalid_argument);
	EXPECT_THROW(OutlierDetectionSigma(4, sqrt(-5)), std::invalid_argument);

	EXPECT_THROW(OutlierDetectionThreshold(4, -403.9), std::invalid_argument);
	EXPECT_THROW(OutlierDetectionThreshold(4, sqrt(-5)), std::invalid_argument);
}

TEST_F(OutlierDetectionTests, ManualScaling) {

	OutlierDetectionThreshold outlier(history_size, 1.0);

	// expected results with manual scaling
	std::vector<bool> expected_results{false,
	                                   false,
	                                   false,
	                                   true,
	                                   true,
	                                   false,
	                                   true,
	                                   false,
	                                   false,
	                                   true,
	                                   false,
	                                   false,
	                                   true,
	                                   true,
	                                   false};

	bool result;
	for (size_t i = 0; i < data.size(); i++) {
		result = outlier.is_outlier(data[i]);

		EXPECT_EQ(result, expected_results[i]);
	}
}

TEST_F(OutlierDetectionTests, AutoScaling) {

	OutlierDetectionSigma outlier(history_size, 1.0);

	// expected results with auto scaling
	std::vector<bool> expected_results{false,
	                                   false,
	                                   false,
	                                   true,
	                                   true,
	                                   true,
	                                   true,
	                                   false,
	                                   true,
	                                   true,
	                                   true,
	                                   false,
	                                   true,
	                                   true,
	                                   false};

	bool result;
	for (size_t i = 0; i < data.size(); i++) {
		result = outlier.is_outlier(data[i]);

		EXPECT_EQ(result, expected_results[i]);
	}
}
