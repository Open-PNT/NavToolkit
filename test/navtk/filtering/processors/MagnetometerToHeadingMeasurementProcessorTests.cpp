#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/filtering/processors/MagnetometerToHeadingMeasurementProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/magnetic/MagnetometerCalibrationCaruso2d.hpp>
#include <navtk/magnetic/MagnetometerCalibrationEllipse2d.hpp>
#include <navtk/magnetic/magnetic.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

namespace navtk {
namespace filtering {
using aspn_xtensor::MeasurementMagneticField;
using navtk::magnetic::mag_to_heading;
using navtk::magnetic::MagnetometerCalibrationCaruso2d;
using navtk::magnetic::MagnetometerCalibrationEllipse2d;

using GenXhatPFunction =
    std::function<std::shared_ptr<EstimateWithCovariance>(const std::vector<std::string> &)>;

struct MagnetometerToHeadingMeasurementProcessorTests : public ::testing::Test {
	double epsilon = 0.000001;

	double mag_x = -1.8906666666666666e-05;
	double mag_y = -9.6e-07;

	std::shared_ptr<MagnetometerCalibrationCaruso2d> caruso_calib;

	std::string state_block_label = "mag_vsb";
	int num_states                = 1;
	double r_multiplier           = 1e-8;
	double r_provided             = 2e-8;
	// Matrix cov                    = eye(num_states) * r_multiplier;
	Matrix cov_provided = eye(num_states) * r_provided;
	Matrix cov          = eye(3) * r_multiplier;
	GenXhatPFunction dummy_xhat_p;
	std::shared_ptr<MagnetometerToHeadingMeasurementProcessor> mag_mp;
	std::shared_ptr<MagnetometerToHeadingMeasurementProcessor> mag_mp_dcm;
	std::shared_ptr<MagnetometerToHeadingMeasurementProcessor> mag_mp_heading;

	// expected results
	double m = mag_x * mag_x + mag_y * mag_y;
	Matrix pd{{-mag_y / m, mag_x / m}};
	Vector expected_z{-0.151765};
	Matrix expected_H{{1, 1}};
	Matrix expected_R =
	    dot(dot(pd, view(cov, xt::range(0, 2), xt::range(0, 2))), xt::transpose(pd));
	Matrix dcm{{0., 1., 0.}, {1., 0., 0.}, {0., 0., 1.}};
	Matrix cov_adjusted = dot(dcm, dot(cov, xt::transpose(dcm)));
	Matrix expected_r_dcm =
	    dot(dot(pd, view(cov_adjusted, xt::range(0, 2), xt::range(0, 2))), xt::transpose(pd));
	// used for magnetometer message
	aspn_xtensor::TypeHeader header{ASPN_MEASUREMENT_MAGNETIC_FIELD, 0, 0, 0, 0};
	aspn_xtensor::TypeTimestamp timestamp{navtk::utils::NANO_PER_SEC};

	MagnetometerToHeadingMeasurementProcessorTests() {
		std::shared_ptr<MagnetometerCalibrationCaruso2d> caruso_calib =
		    std::make_shared<MagnetometerCalibrationCaruso2d>();
		caruso_calib->set_calibration_params(
		    Matrix{{3.9230769230767564, 0, 0}, {0, 1, 0}, {0, 0, 1}},
		    Vector{1.8923333333333336e-05, 9.7e-07, 0});

		// calculate R internally
		mag_mp = std::make_shared<MagnetometerToHeadingMeasurementProcessor>(
		    "a", state_block_label, caruso_calib);
		// calculate R internally
		mag_mp_dcm = std::make_shared<MagnetometerToHeadingMeasurementProcessor>(
		    "a", state_block_label, caruso_calib, -1.0, 0.0, dcm);

		// user provides R
		mag_mp_heading = std::make_shared<MagnetometerToHeadingMeasurementProcessor>(
		    "a", state_block_label, caruso_calib, r_provided);

		dummy_xhat_p = [=, this](const std::vector<std::string> &) {
			return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
			                                                navtk::eye(num_states) * 0.5);
		};
	}
};

TEST_F(MagnetometerToHeadingMeasurementProcessorTests, wrong_meas_type) {
	auto meas_wrong = std::make_shared<navtk::filtering::GaussianVectorData>(
	    aspn_xtensor::TypeTimestamp(1), Vector{1.0}, Matrix{{2.0}});

	auto measurement_model = mag_mp->generate_model(meas_wrong, dummy_xhat_p);

	// Model should return nullptr if not passed MeasurementMagneticField
	ASSERT_EQ(measurement_model, nullptr);
}

TEST_F(MagnetometerToHeadingMeasurementProcessorTests, expected_model) {

	std::shared_ptr<MeasurementMagneticField> mag_message =
	    std::make_shared<MeasurementMagneticField>(header,
	                                               timestamp,
	                                               mag_x,
	                                               mag_y,
	                                               0,
	                                               cov,
	                                               ASPN_MEASUREMENT_MAGNETIC_FIELD_ERROR_MODEL_NONE,
	                                               Vector(),
	                                               std::vector<aspn_xtensor::TypeIntegrity>{});


	auto measurement_model = mag_mp->generate_model(mag_message, dummy_xhat_p);

	auto expected = std::make_shared<StandardMeasurementModel>(expected_z, expected_H, expected_R);

	EXPECT_NEAR(measurement_model->z(0), expected->z(0), epsilon);
	EXPECT_EQ(measurement_model->H, expected->H);
	EXPECT_NEAR(measurement_model->R(0, 0), expected->R(0, 0), epsilon);
}

TEST_F(MagnetometerToHeadingMeasurementProcessorTests, switch_axes) {

	std::shared_ptr<MeasurementMagneticField> mag_message =
	    std::make_shared<MeasurementMagneticField>(header,
	                                               timestamp,
	                                               mag_y,
	                                               mag_x,
	                                               0,
	                                               cov,
	                                               ASPN_MEASUREMENT_MAGNETIC_FIELD_ERROR_MODEL_NONE,
	                                               Vector(),
	                                               std::vector<aspn_xtensor::TypeIntegrity>{});

	auto measurement_model = mag_mp_dcm->generate_model(mag_message, dummy_xhat_p);

	auto expected =
	    std::make_shared<StandardMeasurementModel>(expected_z, expected_H, expected_r_dcm);

	EXPECT_NEAR(measurement_model->z(0), expected->z(0), epsilon);
	EXPECT_EQ(measurement_model->H, expected->H);
	EXPECT_NEAR(measurement_model->R(0, 0), expected->R(0, 0), epsilon);
}

TEST_F(MagnetometerToHeadingMeasurementProcessorTests, give_heading_var) {
	std::shared_ptr<MeasurementMagneticField> mag_message =
	    std::make_shared<MeasurementMagneticField>(header,
	                                               timestamp,
	                                               mag_x,
	                                               mag_y,
	                                               0,
	                                               cov,
	                                               ASPN_MEASUREMENT_MAGNETIC_FIELD_ERROR_MODEL_NONE,
	                                               Vector(),
	                                               std::vector<aspn_xtensor::TypeIntegrity>{});

	auto measurement_model = mag_mp_heading->generate_model(mag_message, dummy_xhat_p);
	EXPECT_NEAR(measurement_model->R(0, 0), cov_provided(0, 0), epsilon);
}
}  // namespace filtering
}  // namespace navtk
