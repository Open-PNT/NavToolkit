#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/filtering/processors/ZuptMeasurementProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/inertial/MovementDetector.hpp>
#include <navtk/inertial/MovementDetectorImu.hpp>
#include <navtk/inertial/MovementDetectorPos.hpp>
#include <navtk/inertial/MovementStatus.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

namespace navtk {
namespace filtering {
using aspn_xtensor::MeasurementImu;
using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::filtering::ZuptMeasurementProcessor;

using GenXhatPFunction = std::function<std::shared_ptr<navtk::filtering::EstimateWithCovariance>(
    const std::vector<std::string> &)>;

TEST(ZuptMeasurementProcessorTests, wrongMeasType) {

	std::string state_block_label("zupt_vsb");
	auto cov     = zeros(3, 3);
	auto zupt_mp = ZuptMeasurementProcessor("a", state_block_label, cov);

	int num_states                = 3;
	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};

	auto meas = std::make_shared<GaussianVectorData>(
	    aspn_xtensor::TypeTimestamp(1), Vector{1.0}, Matrix{{2.0}});

	auto measurement_model = zupt_mp.generate_model(meas, dummy_xhat_p);

	// Model should return nullptr if not passed MeasurementImu or Geo3d
	ASSERT_EQ(measurement_model, nullptr);
}

ERROR_MODE_SENSITIVE_TEST(TEST, ZuptMeasurementProcessorTests, testInvalidData) {

	std::string state_block_label = {"zupt_vsb"};
	auto cov                      = zeros(3, 3);
	auto zupt_mp                  = ZuptMeasurementProcessor("a", state_block_label, cov);

	int num_states                = 3;
	GenXhatPFunction dummy_xhat_p = [num_states](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};

	auto meas = std::make_shared<GaussianVectorData>(
	    aspn_xtensor::TypeTimestamp(1), Vector{1.0}, Matrix{{2.0}});
	// check for warning if wrong data is provided
	EXPECT_HONORS_MODE_EX(zupt_mp.generate_model(meas, dummy_xhat_p),
	                      "Measurement is not of correct type",
	                      std::invalid_argument);
}


TEST(ZuptMeasurementProcessorTests, expected_model) {
	std::string state_block_label = {"zupt_vsb"};
	auto cov                      = zeros(3, 3);
	Matrix expected_H             = zeros(3, 15);
	expected_H.at(0, 3)           = 1;
	expected_H.at(1, 4)           = 1;
	expected_H.at(2, 5)           = 1;
	Vector expected_z             = zeros(3);
	auto zupt_mp                  = ZuptMeasurementProcessor("a", state_block_label, cov);

	int num_states = 15;

	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};
	auto header     = TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0);
	auto timestamp1 = TypeTimestamp(11 * navtk::utils::NANO_PER_SEC);
	auto dlat       = navtk::navutils::north_to_delta_lat(0.0, 0.0, 0.0);
	auto dlon       = navtk::navutils::north_to_delta_lat(0.0, 0.0, 0.0);
	auto pos_message =
	    std::make_shared<MeasurementPosition>(header,
	                                          timestamp1,
	                                          ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
	                                          dlat,
	                                          dlon,
	                                          0,
	                                          cov,
	                                          ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
	                                          Vector(),
	                                          std::vector<aspn_xtensor::TypeIntegrity>{});
	auto timestamp2 = TypeTimestamp(12 * navtk::utils::NANO_PER_SEC);
	auto pos_message2 =
	    std::make_shared<MeasurementPosition>(header,
	                                          timestamp2,
	                                          ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
	                                          dlat,
	                                          dlon,
	                                          0,
	                                          zeros(3, 3),
	                                          ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
	                                          Vector(),
	                                          std::vector<aspn_xtensor::TypeIntegrity>{});

	auto measurement_model = zupt_mp.generate_model(pos_message, dummy_xhat_p);
	measurement_model      = zupt_mp.generate_model(pos_message2, dummy_xhat_p);

	auto expected = std::make_shared<StandardMeasurementModel>(expected_z, expected_H, cov);
	// If movement is detected, z should be 0, directly mapped to velocity states, with user
	// provided covariance
	EXPECT_EQ(measurement_model->z, expected->z);
	EXPECT_EQ(measurement_model->H, expected->H);
	EXPECT_EQ(measurement_model->R, expected->R);
}

TEST(ZuptMeasurementProcessorTests, detect_imu_stationary) {

	std::string state_block_label = {"zupt_vsb"};
	auto cov                      = zeros(3, 3);
	Matrix expected_H             = zeros(3, 15);
	expected_H.at(0, 3)           = 1;
	expected_H.at(1, 4)           = 1;
	expected_H.at(2, 5)           = 1;
	Vector expected_z             = zeros(3);
	auto zupt_mp                  = ZuptMeasurementProcessor("a", state_block_label, cov);

	int num_states = 15;

	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};
	auto measurement_model =
	    std::make_shared<StandardMeasurementModel>(expected_z, expected_H, cov);
	auto header = TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0);
	for (auto k = 0; k < 15; k++) {
		auto timestamp    = TypeTimestamp(k * navtk::utils::NANO_PER_SEC);
		measurement_model = zupt_mp.generate_model(
		    std::make_shared<MeasurementImu>(header,
		                                     timestamp,
		                                     ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED,
		                                     zeros(3),
		                                     zeros(3),
		                                     std::vector<aspn_xtensor::TypeIntegrity>{}),
		    dummy_xhat_p);
	}
	// if movement is not detected, then measurement_model is calculated, If not then measurement
	// model is nullptr
	ASSERT_NE(measurement_model, nullptr);
}

TEST(ZuptMeasurementProcessorTests, detect_pos_stationary) {
	std::string state_block_label = {"zupt_vsb"};
	auto cov                      = zeros(3, 3);
	Matrix expected_H             = zeros(3, 15);
	expected_H.at(0, 3)           = 1;
	expected_H.at(1, 4)           = 1;
	expected_H.at(2, 5)           = 1;
	Vector expected_z             = zeros(3);
	auto zupt_mp                  = ZuptMeasurementProcessor("a", state_block_label, cov);

	int num_states = 15;

	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};
	auto dlat = navtk::navutils::north_to_delta_lat(0.0, 0.0, 0.0);
	auto dlon = navtk::navutils::north_to_delta_lat(0.0, 0.0, 0.0);
	auto measurement_model =
	    std::make_shared<StandardMeasurementModel>(expected_z, expected_H, cov);
	auto header = TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0);

	for (auto k = 0; k < 2; k++) {
		auto timestamp = TypeTimestamp(k * navtk::utils::NANO_PER_SEC);
		measurement_model =
		    zupt_mp.generate_model(std::make_shared<MeasurementPosition>(
		                               header,
		                               timestamp,
		                               ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
		                               dlat,
		                               dlon,
		                               0,
		                               zeros(3, 3),
		                               ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
		                               Vector(),
		                               std::vector<aspn_xtensor::TypeIntegrity>{}),
		                           dummy_xhat_p);
	}
	// if movement is not detected, then measurement_model is calculated, If not then measurement
	// model is nullptr
	ASSERT_NE(measurement_model, nullptr);
}

TEST(ZuptMeasurementProcessorTests, detect_imu_movement) {

	std::string state_block_label = {"zupt_vsb"};
	auto cov                      = zeros(3, 3);
	Matrix expected_H             = zeros(3, 15);
	expected_H.at(0, 3)           = 1;
	expected_H.at(1, 4)           = 1;
	expected_H.at(2, 5)           = 1;
	Vector expected_z             = zeros(3);
	auto zupt_mp                  = ZuptMeasurementProcessor("a", state_block_label, cov);

	int num_states = 15;

	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};
	auto measurement_model =
	    std::make_shared<StandardMeasurementModel>(expected_z, expected_H, cov);
	auto header = TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0);
	for (auto k = 0; k < 15; k++) {
		auto timestamp    = TypeTimestamp(k * navtk::utils::NANO_PER_SEC);
		measurement_model = zupt_mp.generate_model(
		    std::make_shared<MeasurementImu>(header,
		                                     timestamp,
		                                     ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED,
		                                     ones(3) * k,
		                                     ones(3) * k,
		                                     std::vector<aspn_xtensor::TypeIntegrity>{}),
		    dummy_xhat_p);
	}
	// if movement is not detected, then measurement_model is calculated, If not then measurement
	// model is nullptr
	ASSERT_EQ(measurement_model, nullptr);
}

TEST(ZuptMeasurementProcessorTests, detect_pos_movement) {
	std::string state_block_label = {"zupt_vsb"};
	auto cov                      = zeros(3, 3);
	Matrix expected_H             = zeros(3, 15);
	expected_H.at(0, 3)           = 1;
	expected_H.at(1, 4)           = 1;
	expected_H.at(2, 5)           = 1;
	Vector expected_z             = zeros(3);
	auto zupt_mp                  = ZuptMeasurementProcessor("a", state_block_label, cov);

	int num_states = 15;

	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};
	auto dlat = navtk::navutils::north_to_delta_lat(10.0, 0.0, 0.0);
	auto dlon = navtk::navutils::north_to_delta_lat(10.0, 0.0, 0.0);
	auto measurement_model =
	    std::make_shared<StandardMeasurementModel>(expected_z, expected_H, cov);
	auto header = TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0);
	for (auto k = 0; k < 2; k++) {
		auto timestamp = TypeTimestamp(k * navtk::utils::NANO_PER_SEC);
		dlat           = navtk::navutils::north_to_delta_lat(10.0 * k, 0.0, 0.0);
		dlon           = navtk::navutils::north_to_delta_lat(10.0 * k, 0.0, 0.0);
		measurement_model =
		    zupt_mp.generate_model(std::make_shared<MeasurementPosition>(
		                               header,
		                               timestamp,
		                               ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
		                               dlat,
		                               dlon,
		                               0,
		                               zeros(3, 3),
		                               ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
		                               Vector(),
		                               std::vector<aspn_xtensor::TypeIntegrity>{}),
		                           dummy_xhat_p);
	}
	// if movement is not detected, then measurement_model is calculated, If not then measurement
	// model is nullptr
	ASSERT_EQ(measurement_model, nullptr);
}

}  // namespace filtering
}  // namespace navtk
