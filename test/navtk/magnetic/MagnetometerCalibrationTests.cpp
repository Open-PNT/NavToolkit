#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/magnetic/MagnetometerCalibrationCaruso2d.hpp>
#include <navtk/magnetic/MagnetometerCalibrationEllipse2d.hpp>
#include <navtk/magnetic/magnetic.hpp>

using namespace navtk::magnetic;
using navtk::Matrix;
using navtk::Vector;
using navtk::Vector4;
using navtk::zeros;

struct MagnetometerCalibrationTests : public ::testing::Test {
	double epsilon = 0.0000001;

	MagnetometerCalibrationCaruso2d caruso_calibration;
	MagnetometerCalibrationEllipse2d ellipse2d_calibration;

	// Samples from hmr2300 dataset located at
	// https://git.is4s.com/dayton/magnetometer-cal/-/tree/main/data/magtest_4.lcm/MAG
	// clang-format off
	Matrix mag{{-1.8906666666666666e-05,
	                                                         -1.894e-05,
	                                                         -1.896666666666667e-05,
	                                                         -1.888e-05,
	                                                         -1.894666666666667e-05,
	                                                         -1.8906666666666666e-05,
	                                                         -1.8933333333333334e-05,
	                                                         -1.8906666666666666e-05,
	                                                         -1.894666666666667e-05,
	                                                         -1.8953333333333334e-05},{-9.6e-07,
	                                                         -9.6e-07,
	                                                         -8.000000000000001e-07,
	                                                         -1.14e-06,
	                                                         -9.533333333333333e-07,
	                                                         -9.133333333333334e-07,
	                                                         -1.04e-06,
	                                                         -1.0333333333333333e-06,
	                                                         -9.2e-07,
	                                                         -8.866666666666667e-07}};
	std::vector<Vector> raw_mag_values = {
	    {-1.3166666666666665e-05, -1.726666666666667e-05},
	    {-1.1753333333333335e-05, -1.8353333333333333e-05},
	    {-1.0186666666666667e-05, -1.8413333333333335e-05},
	    {-9.033333333333334e-06, -1.8900000000000002e-05},
	    {-7.4666666666666675e-06, -1.9873333333333335e-05},
	    {-5.813333333333334e-06, -1.9953333333333334e-05},
	    {-4.486666666666667e-06, -2.0180000000000003e-05},
	    {-3.24e-06, -2.0546666666666668e-05},
	    {-1.6666666666666667e-06, -2.0580000000000003e-05},
	    {1.4666666666666668e-07, -1.994e-05}};
	// clang-format on
};

TEST_F(MagnetometerCalibrationTests, Caruso) {
	caruso_calibration.generate_calibration(mag);
	auto calib_params = caruso_calibration.get_calibration_params();
	ASSERT_ALLCLOSE_EX(Matrix({{3.9230769230767564, 0}, {0, 1}}), calib_params.first, 0, epsilon);
	ASSERT_ALLCLOSE_EX(Vector({1.8923333333333336e-05, 9.7e-07}), calib_params.second, 0, epsilon);
	std::vector<Vector> expected_mag = {{2.25838462e-05, -1.62966667e-05},
	                                    {2.81284615e-05, -1.73833333e-05},
	                                    {3.42746154e-05, -1.74433333e-05},
	                                    {3.87992308e-05, -1.79300000e-05},
	                                    {4.49453846e-05, -1.89033333e-05},
	                                    {5.14315385e-05, -1.89833333e-05},
	                                    {5.66361538e-05, -1.92100000e-05},
	                                    {6.15269231e-05, -1.95766667e-05},
	                                    {6.76992308e-05, -1.96100000e-05},
	                                    {7.48130769e-05, -1.89700000e-05}};

	for (size_t idx = 0; idx < raw_mag_values.size(); idx++) {
		auto uncalibrated = raw_mag_values[idx];

		auto calibrated = caruso_calibration.apply_calibration(uncalibrated);
		ASSERT_ALLCLOSE_EX(calibrated, expected_mag[idx], 0, epsilon);
	}
}

TEST_F(MagnetometerCalibrationTests, Ellipse2d) {
	ellipse2d_calibration.generate_calibration(mag);
	auto calib_params = ellipse2d_calibration.get_calibration_params();
	ASSERT_ALLCLOSE_EX(Matrix({{0.99338090, 0.08293417}, {0.082934173, 0.12723292}}),
	                   calib_params.first,
	                   0,
	                   0.00001);
	ASSERT_ALLCLOSE_EX(Vector({1.89208810e-05, 9.90317202e-07}), calib_params.second, 0, epsilon);

	std::vector<Vector> expected_mag = {{4.36626112e-06, -1.59366646e-06},
	                                    {5.68011767e-06, -1.61471260e-06},
	                                    {7.23143838e-06, -1.49241637e-06},
	                                    {8.33677639e-06, -1.45868565e-06},
	                                    {9.81235055e-06, -1.45259549e-06},
	                                    {1.14481055e-05, -1.32565629e-06},
	                                    {1.27471925e-05, -1.24446974e-06},
	                                    {1.39551981e-05, -1.18773055e-06},
	                                    {1.55153530e-05, -1.06148854e-06},
	                                    {1.73697615e-05, -8.29672176e-07}};


	for (size_t idx = 0; idx < raw_mag_values.size(); idx++) {
		auto uncalibrated = raw_mag_values[idx];

		auto calibrated = ellipse2d_calibration.apply_calibration(uncalibrated);
		ASSERT_ALLCLOSE_EX(calibrated, expected_mag[idx], 0, epsilon);
	}
}

TEST_F(MagnetometerCalibrationTests, CarusoAndEllipse2d) {
	ellipse2d_calibration = MagnetometerCalibrationEllipse2d(true);
	ellipse2d_calibration.generate_calibration(mag);
	auto calib_params = ellipse2d_calibration.get_calibration_params();
	ASSERT_ALLCLOSE_EX(Matrix({{6.25265747, 0.38106367}, {1.49494212, 0.70049795}}),
	                   calib_params.first,
	                   0,
	                   epsilon);
	ASSERT_ALLCLOSE_EX(Vector({1.89188319e-05, 1.00435283e-06}), calib_params.second, 0, epsilon);

	std::vector<Vector> expected_mag = {{2.97693423e-05, -2.79256330e-06},
	                                    {3.81923423e-05, -1.44091954e-06},
	                                    {4.79653086e-05, 8.59126571e-07},
	                                    {5.49912558e-05, 2.24238415e-06},
	                                    {6.44161839e-05, 3.90264213e-06},
	                                    {7.47234258e-05, 6.31823994e-06},
	                                    {8.29322436e-05, 8.14275029e-06},
	                                    {9.05874999e-05, 9.74959555e-06},
	                                    {0.00010041, 1.20782878e-05},
	                                    {0.00011199, 1.52374349e-05}};


	for (size_t idx = 0; idx < raw_mag_values.size(); idx++) {
		auto uncalibrated = raw_mag_values[idx];

		auto calibrated = ellipse2d_calibration.apply_calibration(uncalibrated);
		ASSERT_ALLCLOSE_EX(calibrated, expected_mag[idx], 0, epsilon);
	}
}

TEST_F(MagnetometerCalibrationTests, ConvertMagToHeading) {
	// Samples from hmr2300 dataset located at
	// https://git.is4s.com/dayton/magnetometer-cal/-/tree/main/data/magtest_4.lcm/MAG
	// First 5 are uncalibrated samples, last 5 are calibrated samples obtained by applying both
	// the Caruso and Ellipse2d calibration algorithms to magnetometer measurements from the
	// dataset.
	std::vector<Vector> mag_values = {{-1.8906666666666666e-05, -9.6e-07},
	                                  {-1.894e-05, -9.6e-07},
	                                  {-1.896666666666667e-05, -8.000000000000001e-07},
	                                  {-1.888e-05, -1.14e-06},
	                                  {-1.894666666666667e-05, -9.533333333333333e-07},
	                                  {8.22882561e-05, -4.83441445e-06},
	                                  {9.12655586e-05, -3.13389281e-06},
	                                  {9.96426571e-05, -1.73454607e-06},
	                                  {1.10379301e-04, 5.86529764e-07},
	                                  {1.23013590e-04, 4.13858760e-06}};

	Vector expected_headings = {3.09086,
	                            3.09095,
	                            3.099438,
	                            3.081285,
	                            3.091318,
	                            0.058682,
	                            0.034325,
	                            0.017406,
	                            -0.00531372,
	                            -0.0336307};

	Vector result_headings = zeros(mag_values.size());
	for (size_t idx = 0; idx < mag_values.size(); idx++) {
		result_headings(idx) = mag_to_heading(mag_values[idx](0), mag_values[idx](1), 0.0);
	}

	ASSERT_ALLCLOSE_EX(expected_headings, result_headings, 0, 0.0001);
}
