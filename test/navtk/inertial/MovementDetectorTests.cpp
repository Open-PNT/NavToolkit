#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <memory>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/inertial/MovementDetector.hpp>
#include <navtk/inertial/MovementDetectorImu.hpp>
#include <navtk/inertial/MovementDetectorPos.hpp>
#include <navtk/inertial/MovementStatus.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::MeasurementImu;
using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::eye;
using navtk::Matrix3;
using navtk::ones;
using navtk::Vector3;
using navtk::zeros;
using navtk::inertial::MovementDetector;
using navtk::inertial::MovementDetectorImu;
using navtk::inertial::MovementDetectorPos;
using navtk::inertial::MovementStatus;

std::shared_ptr<MeasurementImu> imu_gen(const aspn_xtensor::TypeTimestamp& t,
                                        const Vector3& delta_v     = ones(3),
                                        const Vector3& delta_theta = ones(3)) {
	auto header = TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0);
	return std::make_shared<MeasurementImu>(header,
	                                        t,
	                                        ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED,
	                                        delta_v,
	                                        delta_theta,
	                                        std::vector<aspn_xtensor::TypeIntegrity>{});
}

std::shared_ptr<MeasurementPosition> gen_pos(const aspn_xtensor::TypeTimestamp& t,
                                             const Vector3& delta_pos_m,
                                             const Matrix3& cov) {
	auto dlat   = navtk::navutils::north_to_delta_lat(delta_pos_m[0], 0.0, 0.0);
	auto dlon   = navtk::navutils::north_to_delta_lat(delta_pos_m[1], 0.0, 0.0);
	auto header = TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0);
	return std::make_shared<MeasurementPosition>(header,
	                                             t,
	                                             ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
	                                             dlat,
	                                             dlon,
	                                             -delta_pos_m[2],
	                                             cov,
	                                             ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
	                                             navtk::Vector(),
	                                             std::vector<aspn_xtensor::TypeIntegrity>{});
}

/* Test support class that always returns the same status */
class MovementDetectorDummy : public navtk::inertial::MovementDetectorPlugin {
public:
	MovementDetectorDummy(const MovementStatus ret) : MovementDetectorPlugin() {
		last_status = ret;
	}

	MovementStatus process(navtk::not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) override {
		auto imu    = std::dynamic_pointer_cast<MeasurementImu>(data);
		latest_time = imu->get_time_of_validity();
		return last_status;
	}
	aspn_xtensor::TypeTimestamp get_time() override { return latest_time; }

	aspn_xtensor::TypeTimestamp latest_time = aspn_xtensor::to_type_timestamp();
};

struct MovementDetectorTests : public ::testing::Test {};

ERROR_MODE_SENSITIVE_TEST(TEST_F, MovementDetectorTests, imu_constructor) {
	EXPECT_HONORS_MODE_EX(MovementDetectorImu(0), "window must", std::invalid_argument);
	EXPECT_HONORS_MODE_EX(MovementDetectorImu(1, -1.0), "calib_time must", std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, MovementDetectorTests, pos_order) {
	auto md = MovementDetectorPos();
	md.process(gen_pos(to_type_timestamp(2.0), zeros(3), eye(3)));
	EXPECT_HONORS_MODE_EX(md.process(gen_pos(to_type_timestamp(1.0), zeros(3), eye(3))),
	                      "out of order",
	                      std::invalid_argument);
	// Equivalent time also; dt==0 bad for speed calc
	EXPECT_HONORS_MODE_EX(md.process(gen_pos(to_type_timestamp(2.0), zeros(3), eye(3))),
	                      "out of order",
	                      std::invalid_argument);
}

TEST_F(MovementDetectorTests, pos_wrong_data) {
	auto md   = MovementDetectorPos();
	auto stat = md.process(imu_gen(to_type_timestamp(0, 0)));
	ASSERT_EQ(stat, MovementStatus::INVALID);
}

TEST_F(MovementDetectorTests, imu_wrong_data) {
	auto md   = MovementDetectorImu(1, 10.0);
	auto stat = md.process(gen_pos(to_type_timestamp(0, 0), zeros(3), zeros(3, 3)));
	ASSERT_EQ(stat, MovementStatus::INVALID);
}

TEST_F(MovementDetectorTests, position) {
	auto md    = MovementDetectorPos();
	auto stat1 = md.process(gen_pos(to_type_timestamp(1.0), zeros(3), eye(3)));
	ASSERT_EQ(stat1, MovementStatus::INVALID);
	auto stat2 = md.process(gen_pos(to_type_timestamp(2.0), zeros(3), eye(3)));
	ASSERT_EQ(stat2, MovementStatus::NOT_MOVING);
	auto stat3 = md.process(gen_pos(to_type_timestamp(3.0), ones(3) * 10.0, eye(3)));
	ASSERT_EQ(stat3, MovementStatus::MOVING);
	auto stat4 = md.process(gen_pos(to_type_timestamp(4.0), ones(3) * 12, eye(3) * 9));
	ASSERT_EQ(stat4, MovementStatus::POSSIBLY_MOVING);
	auto stat5 = md.process(gen_pos(to_type_timestamp(5.0), ones(3) * 12.1, eye(3) * 100.0));
	ASSERT_EQ(stat5, MovementStatus::NOT_MOVING);
}

TEST_F(MovementDetectorTests, imu_not_full) {
	auto md   = MovementDetectorImu(1, 10.0);
	auto stat = md.process(imu_gen(to_type_timestamp(), ones(3) * 1.1, ones(3) * 0.51));
	// Not enough stationary data
	ASSERT_EQ(stat, MovementStatus::INVALID);
	ASSERT_EQ(to_type_timestamp(0, 0), md.get_time());

	// Stationary requirement (10s) met; calc initial bias estimates
	stat = md.process(imu_gen(to_type_timestamp(10), ones(3) * 0.9, ones(3) * 0.49));
	ASSERT_EQ(stat, MovementStatus::INVALID);
	ASSERT_EQ(to_type_timestamp(0, 0), md.get_time());

	// A measurement that is very similar to stationary will be tagged as not moving
	stat = md.process(imu_gen(to_type_timestamp(11), ones(3) * 1.0, ones(3) * 0.50));
	ASSERT_EQ(stat, MovementStatus::NOT_MOVING);
	ASSERT_EQ(to_type_timestamp(11, 0), md.get_time());

	// While one outside will be moving
	stat =
	    md.process(imu_gen(to_type_timestamp(12), ones(3) * 1.1 * 100.0, ones(3) * 0.51 * 100.0));
	ASSERT_EQ(stat, MovementStatus::MOVING);
	ASSERT_EQ(to_type_timestamp(12, 0), md.get_time());
}

TEST_F(MovementDetectorTests, imu_multiple_measurement_window) {
	auto md   = MovementDetectorImu(2, 1.0);
	auto stat = md.process(imu_gen(to_type_timestamp(0, 0), ones(3) * 1.1, ones(3) * 0.51));
	// Not enough stationary data
	ASSERT_EQ(stat, MovementStatus::INVALID);
	ASSERT_EQ(to_type_timestamp(0, 0), md.get_time());

	// Stationary requirement (1.0s) met; calc initial bias estimates and clear measurement buffer
	stat = md.process(imu_gen(to_type_timestamp(1, 0), ones(3) * 0.9, ones(3) * 0.49));
	ASSERT_EQ(stat, MovementStatus::INVALID);
	ASSERT_EQ(to_type_timestamp(0, 0), md.get_time());

	// Buffer size still smaller than window, so status not updated
	stat = md.process(imu_gen(to_type_timestamp(2, 0), ones(3) * 1.0, ones(3) * 0.50));
	ASSERT_EQ(stat, MovementStatus::INVALID);
	ASSERT_EQ(to_type_timestamp(0, 0), md.get_time());

	// Window reached, update status
	stat = md.process(imu_gen(to_type_timestamp(3, 0), ones(3) * 1.1, ones(3) * 0.51));
	ASSERT_EQ(stat, MovementStatus::NOT_MOVING);
	ASSERT_EQ(to_type_timestamp(3, 0), md.get_time());

	// Measurements very different from stationary data should result in MOVING status.
	// Buffer size still smaller than window, so status not updated
	stat = md.process(imu_gen(to_type_timestamp(4, 0), ones(3) * 1.0 * 100, ones(3) * 0.50 * 100));
	ASSERT_EQ(stat, MovementStatus::NOT_MOVING);
	ASSERT_EQ(to_type_timestamp(3, 0), md.get_time());

	// Window reached, update status
	stat = md.process(imu_gen(to_type_timestamp(5, 0), ones(3) * 1.1 * 100, ones(3) * 0.51 * 100));
	ASSERT_EQ(stat, MovementStatus::MOVING);
	ASSERT_EQ(to_type_timestamp(5, 0), md.get_time());
}

TEST_F(MovementDetectorTests, imu_const_imu_not_move) {
	auto md   = MovementDetectorImu(1, 10.0);
	auto stat = MovementStatus::INVALID;
	for (auto k = 0; k < 15; k++) {
		stat = md.process(imu_gen(to_type_timestamp(k, 0), ones(3) * 1.1, ones(3) * 0.51));
	}
	ASSERT_EQ(stat, MovementStatus::NOT_MOVING);
}

TEST_F(MovementDetectorTests, no_plugins) {
	MovementDetector det;
	ASSERT_EQ(det.get_status(), MovementStatus::INVALID);
	ASSERT_TRUE(det.plugin_info().empty());
	det.process({"SOMETHING"}, imu_gen(to_type_timestamp(1, 0), ones(3), ones(3)));
	ASSERT_EQ(det.get_status(), MovementStatus::INVALID);
	det.remove_plugin("Does not exist, but that is okay");
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, MovementDetectorTests, missing_plugin) {
	MovementDetector det;
	EXPECT_HONORS_MODE_EX(
	    det.process({"something"}, imu_gen(to_type_timestamp(1, 0), ones(3), ones(3))),
	    "No plugin named",
	    std::invalid_argument);

	ASSERT_EQ(det.get_status(), MovementStatus::INVALID);

	EXPECT_HONORS_MODE_EX(det.remove_plugin("Madam Reginald Plinkington"),
	                      "Cannot remove plugin",
	                      std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, MovementDetectorTests, duplicate_plugins) {
	MovementDetector det;
	det.add_plugin("POS", std::make_shared<MovementDetectorPos>());
	ASSERT_EQ(det.plugin_info().size(), 1);
	EXPECT_HONORS_MODE_EX(det.add_plugin("POS", std::make_shared<MovementDetectorPos>()),
	                      "Cannot add plugin",
	                      std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, MovementDetectorTests, bad_add_args) {
	MovementDetector det;

	EXPECT_HONORS_MODE_EX(det.add_plugin("POS", std::make_shared<MovementDetectorPos>(), -1.0, 1.0),
	                      "weight",
	                      std::invalid_argument);
	EXPECT_HONORS_MODE_EX(det.add_plugin("POS", std::make_shared<MovementDetectorPos>(), 0.0, 1.0),
	                      "weight",
	                      std::invalid_argument);
	EXPECT_HONORS_MODE_EX(det.add_plugin("POS", std::make_shared<MovementDetectorPos>(), 1.0, -1.0),
	                      "stale",
	                      std::invalid_argument);
	// Zero stale time ok
	det.add_plugin("POS", std::make_shared<MovementDetectorPos>(), 1.0, 0.0);
	ASSERT_EQ(det.plugin_info().size(), 1);
}

TEST_F(MovementDetectorTests, data_to_multiple) {
	MovementDetector det;
	det.add_plugin("A", std::make_shared<MovementDetectorPos>(), 1.0, 0.0);
	det.add_plugin("B", std::make_shared<MovementDetectorPos>(), 1.0, 0.0);
	det.add_plugin("C", std::make_shared<MovementDetectorPos>(), 1.0, 0.0);
	det.add_plugin("D", std::make_shared<MovementDetectorPos>(), 1.0, 0.0);

	det.process(std::vector<std::string>{"A", "B", "C", "D"},
	            gen_pos(aspn_xtensor::TypeTimestamp((int64_t)0), ones(3) * 0.0, eye(3)));
	ASSERT_EQ(det.get_status(), MovementStatus::INVALID);
	// Only one with enough data
	det.process(std::vector<std::string>{"A"},
	            gen_pos(to_type_timestamp(1.0), ones(3) * 0.0, eye(3)));
	ASSERT_EQ(det.get_status(), MovementStatus::NOT_MOVING);
	// 2 outvote 1
	det.process(std::vector<std::string>{"B", "C"},
	            gen_pos(to_type_timestamp(1.0), ones(3) * 10.0, eye(3)));
	ASSERT_EQ(det.get_status(), MovementStatus::MOVING);
	// Now tied
	det.process(std::vector<std::string>{"D"},
	            gen_pos(to_type_timestamp(1.0), ones(3) * 0.0, eye(3)));
	ASSERT_EQ(det.get_status(), MovementStatus::POSSIBLY_MOVING);
	// B, C go stale
	det.process(std::vector<std::string>{"A", "D"},
	            gen_pos(to_type_timestamp(3.0), ones(3) * 0.0, eye(3)));
	ASSERT_EQ(det.get_status(), MovementStatus::NOT_MOVING);
	// No change
	det.process(std::vector<std::string>{},
	            gen_pos(to_type_timestamp(10.0), ones(3) * 11111.0, eye(3)));
	ASSERT_EQ(det.get_status(), MovementStatus::NOT_MOVING);

	// 2 calls of process with no-ids will pass data to all plugins, which for this plugin at least
	// will cause all to have the same status and return POSSIBLY_MOVING
	// Stale plugins see movement over their interval, A+D see no moevement, and we have a split
	// decision
	det.process(gen_pos(to_type_timestamp(11.0), ones(3) * 0.0, eye(3)));
	ASSERT_EQ(det.get_status(), MovementStatus::POSSIBLY_MOVING);
	det.process(gen_pos(to_type_timestamp(12.0), ones(3) * 0.0, eye(3)));
	ASSERT_EQ(det.get_status(), MovementStatus::NOT_MOVING);
}

TEST_F(MovementDetectorTests, equal_weight_results) {
	MovementDetector det;
	det.add_plugin("A", std::make_shared<MovementDetectorDummy>(MovementStatus::MOVING));
	det.add_plugin("B", std::make_shared<MovementDetectorDummy>(MovementStatus::MOVING));
	ASSERT_EQ(det.get_status(), MovementStatus::MOVING);
	det.add_plugin("C", std::make_shared<MovementDetectorDummy>(MovementStatus::NOT_MOVING));
	det.add_plugin("D", std::make_shared<MovementDetectorDummy>(MovementStatus::NOT_MOVING));
	ASSERT_EQ(det.get_status(), MovementStatus::POSSIBLY_MOVING);
	det.add_plugin("E", std::make_shared<MovementDetectorDummy>(MovementStatus::MOVING));
	ASSERT_EQ(det.get_status(), MovementStatus::MOVING);
}

TEST_F(MovementDetectorTests, skewed_weight_results) {
	MovementDetector det;
	det.add_plugin("A", std::make_shared<MovementDetectorDummy>(MovementStatus::MOVING));
	det.add_plugin("B", std::make_shared<MovementDetectorDummy>(MovementStatus::MOVING));
	ASSERT_EQ(det.get_status(), MovementStatus::MOVING);
	det.add_plugin("C", std::make_shared<MovementDetectorDummy>(MovementStatus::NOT_MOVING), 5.0);
	ASSERT_EQ(det.get_status(), MovementStatus::NOT_MOVING);
	det.add_plugin("E", std::make_shared<MovementDetectorDummy>(MovementStatus::MOVING), 3.0);
	ASSERT_EQ(det.get_status(), MovementStatus::POSSIBLY_MOVING);
}

TEST_F(MovementDetectorTests, exclude_invalids) {
	MovementDetector det;
	det.add_plugin("A", std::make_shared<MovementDetectorDummy>(MovementStatus::MOVING));
	det.add_plugin("B", std::make_shared<MovementDetectorDummy>(MovementStatus::INVALID));
	det.add_plugin("C", std::make_shared<MovementDetectorDummy>(MovementStatus::INVALID));
	det.add_plugin("D", std::make_shared<MovementDetectorDummy>(MovementStatus::INVALID));
	ASSERT_EQ(det.get_status(), MovementStatus::MOVING);
	det.remove_plugin("A");
	ASSERT_EQ(det.get_status(), MovementStatus::INVALID);
}

TEST_F(MovementDetectorTests, exclude_stale) {
	MovementDetector det;
	det.add_plugin(
	    "A", std::make_shared<MovementDetectorDummy>(MovementStatus::NOT_MOVING), 1.0, 1.0);
	det.add_plugin(
	    "B", std::make_shared<MovementDetectorDummy>(MovementStatus::NOT_MOVING), 1.0, 2.0);
	det.add_plugin("C", std::make_shared<MovementDetectorDummy>(MovementStatus::MOVING), 1.0, 3.0);
	auto m = imu_gen(to_type_timestamp(1.0));
	det.process({"A"}, m);
	det.process({"B"}, m);
	det.process({"C"}, m);
	ASSERT_EQ(det.get_status(), MovementStatus::NOT_MOVING);
	m = imu_gen(to_type_timestamp(2.1));
	det.process({"C"}, m);
	ASSERT_EQ(det.get_status(), MovementStatus::POSSIBLY_MOVING);
	m = imu_gen(to_type_timestamp(3.1));
	det.process({"C"}, m);
	ASSERT_EQ(det.get_status(), MovementStatus::MOVING);
}

TEST_F(MovementDetectorTests, wrong_type) {
	MovementDetector det;
	det.add_plugin("A", std::make_shared<MovementDetectorPos>());
	det.process({"A"}, imu_gen(to_type_timestamp(2.1)));
	ASSERT_EQ(det.get_status(), MovementStatus::INVALID);
	det.add_plugin("B", std::make_shared<MovementDetectorImu>());
	det.process({"B"}, gen_pos(to_type_timestamp(1.0), zeros(3), eye(3)));
	ASSERT_EQ(det.get_status(), MovementStatus::INVALID);
}

TEST_F(MovementDetectorTests, get_stats) {
	MovementDetector det;
	det.add_plugin(
	    "A", std::make_shared<MovementDetectorDummy>(MovementStatus::NOT_MOVING), 7.0, 1.0);
	det.add_plugin(
	    "B", std::make_shared<MovementDetectorDummy>(MovementStatus::NOT_MOVING), 6.0, 2.0);
	det.add_plugin("C", std::make_shared<MovementDetectorDummy>(MovementStatus::MOVING), 7.0, 3.0);
	auto info = det.plugin_info();
	ASSERT_EQ(info.size(), 3);
	ASSERT_EQ(info.count("A"), 1);
	ASSERT_EQ(info.count("B"), 1);
	ASSERT_EQ(info.count("C"), 1);
	ASSERT_EQ(info["A"].weight, 7.0);
	ASSERT_EQ(info["A"].stale_time, 1.0);
	ASSERT_EQ(info["B"].weight, 6.0);
	ASSERT_EQ(info["B"].stale_time, 2.0);
	ASSERT_EQ(info["C"].weight, 7.0);
	ASSERT_EQ(info["C"].stale_time, 3.0);
	ASSERT_EQ(det.get_status(), MovementStatus::NOT_MOVING);
	det.remove_plugin("B");
	info = det.plugin_info();
	ASSERT_EQ(info.size(), 2);
	ASSERT_EQ(info.count("A"), 1);
	ASSERT_EQ(info.count("B"), 0);
	ASSERT_EQ(info.count("C"), 1);
	ASSERT_EQ(info["A"].weight, 7.0);
	ASSERT_EQ(info["A"].stale_time, 1.0);
	ASSERT_EQ(info["C"].weight, 7.0);
	ASSERT_EQ(info["C"].stale_time, 3.0);
	ASSERT_EQ(det.get_status(), MovementStatus::POSSIBLY_MOVING);
	det.remove_plugin("A");
	info = det.plugin_info();
	ASSERT_EQ(info.size(), 1);
	ASSERT_EQ(info.count("A"), 0);
	ASSERT_EQ(info.count("C"), 1);
	ASSERT_EQ(info["C"].weight, 7.0);
	ASSERT_EQ(info["C"].stale_time, 3.0);
	ASSERT_EQ(det.get_status(), MovementStatus::MOVING);
	det.remove_plugin("C");
	info = det.plugin_info();
	ASSERT_EQ(info.size(), 0);
	ASSERT_EQ(det.get_status(), MovementStatus::INVALID);
}
