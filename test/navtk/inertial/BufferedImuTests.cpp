#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <inertial/BufferedPvaTests.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>
#include <test_data_generation.hpp>

#include <navtk/aspn.hpp>
#include <navtk/experimental/random.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/inertial/BufferedImu.hpp>
#include <navtk/inertial/ImuErrors.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/inertial/inertial_functions.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::MeasurementImu;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::Vector3;
using navtk::zeros;
using navtk::inertial::BufferedImu;
using navtk::inertial::calc_force_ned;
using navtk::inertial::calc_rot_rate;
using navtk::inertial::DcmIntegrationMethods;
using navtk::inertial::EarthModels;
using navtk::inertial::ImuErrors;
using navtk::inertial::Inertial;
using navtk::inertial::IntegrationMethods;
using navtk::inertial::MechanizationOptions;
using navtk::inertial::StandardPosVelAtt;
using navtk::navutils::GravModels;
using navtk::navutils::quat_to_dcm;
using navtk::navutils::rpy_to_dcm;
using navtk::testing::stationary_imu;
using std::make_shared;

namespace {
MeasurementImu imu_pack(aspn_xtensor::TypeTimestamp t, const Vector3& dv, const Vector3& dth) {
	TypeHeader head(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0);
	return MeasurementImu(head, t, ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED, dv, dth, {});
}
}  // namespace

// PNTOS-377 Try to speed tests w/ASAN up

struct BufferedImuTests : public BufferedPvaTests {

	double wander0 = 0.4;

	// Number of propagations 'bi' member automatically undergoes during the test setup phase
	navtk::Size base_prop = 14;

	// Remaining number of propagations until 'bi' hits the 'simpson's solution' test values,
	// barring any deviant actions
	// Individual tests can update this value to track how many props are 'left' when mechanizing
	// in chunks
	navtk::Size from_setup_to_simpson = 99 - base_prop;

	MechanizationOptions options;

	BufferedImu bi;
	ImuErrors imu_e;

	BufferedImuTests()
	    : BufferedPvaTests(),

	      options({GravModels::TITTERTON,
	               EarthModels::ELLIPTICAL,
	               DcmIntegrationMethods::SIXTH_ORDER,
	               IntegrationMethods::SIMPSONS_RULE}),
	      bi(BufferedImu(start_pva,
	                     make_shared<aspn_xtensor::MeasurementImu>(imu_pack(start_time, dv, dth)),
	                     dt,
	                     ImuErrors{},
	                     options,
	                     3.0)),
	      imu_e(Vector3{0.1, 0.2, 0.3},
	            Vector3{0.4, 0.5, 0.6},
	            Vector3{-0.1, -0.2, -0.3},
	            Vector3{-0.4, -0.5, -0.6},
	            TypeTimestamp((int64_t)0)) {}

	using BufferedPvaTests::assert_eq;

	void assert_eq(const ImuErrors& e1, const ImuErrors& e2) {
		ASSERT_EQ(e1.time_validity, e2.time_validity);
		ASSERT_ALLCLOSE(e1.accel_biases, e2.accel_biases);
		ASSERT_ALLCLOSE(e1.gyro_biases, e2.gyro_biases);
		ASSERT_ALLCLOSE(e1.accel_scale_factors, e2.accel_scale_factors);
		ASSERT_ALLCLOSE(e1.gyro_scale_factors, e2.gyro_scale_factors);
	}

	void assert_bi_results(const aspn_xtensor::TypeTimestamp& t,
	                       const MeasurementPositionVelocityAttitude& expected,
	                       const MeasurementPositionVelocityAttitude& reprop,
	                       bool has_been_reset = false) {

		assert_eq(*bi.calc_pva_no_reset_since(t, bi.time_span().first), reprop);
		assert_eq(*bi.calc_pva_no_reset_since(t, t), expected);
		assert_eq(*bi.calc_pva_no_reset_since(t, bi.time_span().second + 100.0), expected);
		if (has_been_reset) {
			ASSERT_EQ(bi.calc_pva_no_reset_since(t, aspn_xtensor::TypeTimestamp((int64_t)0)),
			          nullptr);
		} else {
			assert_eq(*bi.calc_pva_no_reset_since(t, aspn_xtensor::TypeTimestamp((int64_t)0)),
			          expected);
		}
	}

	void assert_bi_results(const aspn_xtensor::TypeTimestamp& t,
	                       const MeasurementPositionVelocityAttitude& expected,
	                       bool has_been_reset = false) {
		assert_bi_results(t, expected, expected, has_been_reset);
	}

	// expected is nullptr
	void assert_bi_results(const aspn_xtensor::TypeTimestamp& t) {
		ASSERT_EQ(bi.calc_pva_no_reset_since(t, bi.time_span().first), nullptr);
		ASSERT_EQ(bi.calc_pva_no_reset_since(t, aspn_xtensor::TypeTimestamp((int64_t)0)), nullptr);
		ASSERT_EQ(bi.calc_pva_no_reset_since(t, t), nullptr);
		ASSERT_EQ(bi.calc_pva_no_reset_since(t, bi.time_span().second + 100.0), nullptr);
	}

	BufferedImu baseline_buffered(double additional_vel_seed = 0.0, double buffer_size = 3.0) {
		return BufferedImu(
		    to_positionvelocityattitude(StandardPosVelAtt(
		        start_time, llh0, vned0 + navtk::ones(3) * additional_vel_seed, c_s_to_ned)),
		    make_shared<aspn_xtensor::MeasurementImu>(imu_pack(start_time, dv, dth)),
		    dt,
		    ImuErrors{},
		    options,
		    buffer_size);
	}

	// Propagates some ins steps number of times using default dt, dv, dth values
	void prop(BufferedImu& ins, navtk::Size steps) {
		for (navtk::Size k = 0; k < steps; k++) {
			ins.mechanize(imu_pack(ins.time_span().second + dt, dv, dth));
		}
	}

	virtual void SetUp() override { prop(bi, base_prop); }
};

TEST_F(BufferedImuTests, MechAtTime_SLOW) {
	prop(bi, from_setup_to_simpson);
	assert_eq(*bi.calc_pva(), simpsons_pva);
	assert_bi_results(bi.time_span().second, simpsons_pva);
}

TEST_F(BufferedImuTests, MechAtTimeAlt_SLOW) {
	for (navtk::Size k = 0; k < from_setup_to_simpson; k++) {
		bi.mechanize(bi.time_span().second + dt, dv, dth);
	}
	assert_eq(*bi.calc_pva(), simpsons_pva);
	assert_bi_results(bi.time_span().second, simpsons_pva);
}

TEST_F(BufferedImuTests, MechPrior_SLOW) {
	// Should be no-op w/ warn log
	prop(bi, from_setup_to_simpson);

	EXPECT_WARN(bi.mechanize(imu_pack(bi.time_span().second - 0.01, dv, dth)), "Suspicious");
	assert_eq(*bi.calc_pva(), simpsons_pva);
	assert_bi_results(bi.time_span().second, simpsons_pva);
}

TEST_F(BufferedImuTests, CheckRange) {
	ASSERT_EQ(bi.time_span().first, start_time);
	ASSERT_EQ(bi.time_span().second, start_time + dt * base_prop);
}

TEST_F(BufferedImuTests, SolutionBefore) {
	// Request solution before available time; nullptr
	assert_bi_results(bi.time_span().first - 0.1);
}

TEST_F(BufferedImuTests, SolutionAfter) {
	// Request solution after available times; nullptr
	assert_bi_results(bi.time_span().second + 0.1);
}

TEST_F(BufferedImuTests, SolutionAtStart) { assert_bi_results(bi.time_span().first, start_pva); }

TEST_F(BufferedImuTests, SolutionAtEnd_SLOW) {
	prop(bi, from_setup_to_simpson);
	assert_bi_results(bi.time_span().second, simpsons_pva);
}

TEST_F(BufferedImuTests, SolutionAtExactTime) { assert_bi_results(start_time + dt * 10, at10); }

TEST_F(BufferedImuTests, SolutionBetweenRecords) {
	assert_bi_results(start_time + dt * 10.7, at10point7);
}

TEST_F(BufferedImuTests, ResetBefore_SLOW) {
	prop(bi, from_setup_to_simpson);
	// Not enough stored imu history to repropagate
	auto res = EXPECT_WARN(
	    bi.reset(make_shared<MeasurementPositionVelocityAttitude>(to_positionvelocityattitude(
	        StandardPosVelAtt(bi.time_span().first - 1.0, llh0, vned0, c_s_to_ned)))),
	    "failed in_range");

	ASSERT_FALSE(res);

	assert_bi_results(bi.time_span().second, simpsons_pva);
}

TEST_F(BufferedImuTests, ResetAfter_SLOW) {
	prop(bi, from_setup_to_simpson);
	// No leaps into the future
	auto res = EXPECT_WARN(
	    bi.reset(make_shared<MeasurementPositionVelocityAttitude>(to_positionvelocityattitude(
	        StandardPosVelAtt(bi.time_span().second + 1.0, llh0, vned0, c_s_to_ned)))),
	    "failed in_range");

	ASSERT_FALSE(res);

	assert_bi_results(bi.time_span().second, simpsons_pva);
}

TEST_F(BufferedImuTests, ResetStart_SLOW) {
	prop(bi, from_setup_to_simpson);
	ASSERT_TRUE(
	    bi.reset(make_shared<MeasurementPositionVelocityAttitude>(to_positionvelocityattitude(
	        StandardPosVelAtt(bi.time_span().first, llh0, vned0, c_s_to_ned)))));
	// Same start point, same data, should repropagate the same. Technically been reset, but is the
	// only reset and at start point, so acts more like a re-initialization, and thus no 'true'
	// to assert_bi_results
	assert_bi_results(bi.time_span().second, simpsons_pva);
}

TEST_F(BufferedImuTests, ResetEnd_SLOW) {
	prop(bi, from_setup_to_simpson);
	ASSERT_TRUE(
	    bi.reset(make_shared<MeasurementPositionVelocityAttitude>(to_positionvelocityattitude(
	        StandardPosVelAtt(bi.time_span().second, llh0, vned0, c_s_to_ned)))));
	// Should be starting PVA, but time is unchanged
	start_pva.set_time_of_validity(bi.time_span().second);
	assert_bi_results(bi.time_span().second, start_pva, simpsons_pva, true);

	// Make sure reset pva pushed into history
	auto sol_at_end = bi.calc_pva();
	prop(bi, 1);
	auto should_be_same = bi.calc_pva(sol_at_end->get_time_of_validity());
	assert_eq(*sol_at_end, *should_be_same);
}

TEST_F(BufferedImuTests, ResetAtExactTime) {

	auto normal = baseline_buffered(1.0);
	prop(normal, base_prop);

	auto t_prev     = bi.time_span().first + dt * 9;
	auto t          = bi.time_span().first + dt * 10;
	auto reset_val  = normal.calc_pva(t);
	auto reset_prev = normal.calc_pva(t_prev);
	ASSERT_TRUE(bi.reset(reset_val, nullptr, reset_prev));
	assert_eq(*reset_val, *bi.calc_pva(t));

	assert_eq(*normal.calc_pva(bi.time_span().second), *bi.calc_pva());
}

TEST_F(BufferedImuTests, ResetBetweenRecords) {
	auto normal = baseline_buffered(1.0);
	prop(normal, base_prop);

	auto t_prev     = bi.time_span().first + dt * 9.7;
	auto t          = bi.time_span().first + dt * 10.7;
	auto reset_val  = normal.calc_pva(t);
	auto reset_prev = normal.calc_pva(t_prev);
	ASSERT_TRUE(bi.reset(reset_val, nullptr, reset_prev));
	assert_eq(*reset_val, *bi.calc_pva(t));

	assert_eq(*normal.calc_pva(bi.time_span().second), *bi.calc_pva());
}

TEST_F(BufferedImuTests, ResetBetweenRecordsSmallDeltaTime) {
	auto normal = baseline_buffered(1.0);
	prop(normal, base_prop);

	auto t_prev     = bi.time_span().first + dt * 9 - 1e-12;
	auto t          = bi.time_span().first + dt * 10 - 1e-12;
	auto reset_val  = normal.calc_pva(t);
	auto reset_prev = normal.calc_pva(t_prev);
	ASSERT_TRUE(bi.reset(reset_val, nullptr, reset_prev));
	assert_eq(*reset_val, *bi.calc_pva(t));
	assert_eq(*normal.calc_pva(bi.time_span().second), *bi.calc_pva());

	t_prev     = bi.time_span().first + dt * 9;
	t_prev     = t_prev - 1e-9;
	t          = bi.time_span().first + dt * 10;
	t          = t - 1e-9;
	reset_val  = normal.calc_pva(t);
	reset_prev = normal.calc_pva(t_prev);
	ASSERT_TRUE(bi.reset(reset_val, nullptr, reset_prev));
	assert_eq(*reset_val, *bi.calc_pva(t));
	assert_eq(*normal.calc_pva(bi.time_span().second), *bi.calc_pva());
}

TEST_F(BufferedImuTests, ResetBetweenRecordsMultiple_SLOW) {
	auto normal = baseline_buffered(1.0);
	prop(bi, base_prop);
	prop(normal, base_prop * 2);

	auto t_prev = bi.time_span().first + dt * 9.7;
	auto t      = bi.time_span().first + dt * 10.7;

	auto reset_val = make_shared<MeasurementPositionVelocityAttitude>(
	    to_positionvelocityattitude(StandardPosVelAtt(t, llh0, vned0, c_s_to_ned)));
	reset_val->set_time_of_validity(t + dt * 5);
	ASSERT_TRUE(bi.reset(reset_val));
	reset_val->set_time_of_validity(t - dt * 5);
	ASSERT_TRUE(bi.reset(reset_val));
	reset_val->set_time_of_validity(t + dt * 10);
	ASSERT_TRUE(bi.reset(reset_val));
	reset_val->set_time_of_validity(t - dt * 10);
	ASSERT_TRUE(bi.reset(reset_val));
	reset_val->set_time_of_validity(t + dt * 10);
	ASSERT_TRUE(bi.reset(reset_val));
	reset_val->set_time_of_validity(t);

	reset_val       = normal.calc_pva(t);
	auto reset_prev = normal.calc_pva(t_prev);
	ASSERT_TRUE(bi.reset(reset_val, nullptr, reset_prev));
	assert_eq(*reset_val, *bi.calc_pva(t));
	assert_eq(*normal.calc_pva(bi.time_span().second), *bi.calc_pva());
}

TEST_F(BufferedImuTests, ResetEarlierWipesLater_SLOW) {
	auto normal = baseline_buffered();
	prop(bi, base_prop * 9);
	prop(normal, base_prop * 10);

	auto t = bi.time_span().first + dt * 10.5;

	auto reset_val = make_shared<MeasurementPositionVelocityAttitude>(
	    to_positionvelocityattitude(StandardPosVelAtt(t, llh0, vned0, c_s_to_ned)));

	ASSERT_TRUE(bi.reset(reset_val, make_shared<ImuErrors>(imu_e)));

	reset_val->set_time_of_validity(t + dt * 3);
	ASSERT_TRUE(bi.reset(reset_val, make_shared<ImuErrors>(ImuErrors{})));

	reset_val->set_time_of_validity(t - dt * 8);
	ASSERT_TRUE(bi.reset(reset_val, make_shared<ImuErrors>(ImuErrors{})));
	ASSERT_TRUE(normal.reset(reset_val, make_shared<ImuErrors>(ImuErrors{})));

	assert_eq(*normal.calc_pva(bi.time_span().second), *bi.calc_pva());
	assert_eq(*normal.calc_pva(t + dt * 5), *bi.calc_pva(t + dt * 5));
	assert_eq(*normal.calc_pva(t + dt * 10), *bi.calc_pva(t + dt * 10));
	assert_eq(*normal.calc_pva_no_reset_since(bi.time_span().second, t + dt * 4),
	          *bi.calc_pva_no_reset_since(bi.time_span().second, t + dt * 4));
	assert_eq(*normal.calc_pva_no_reset_since(bi.time_span().second, t + dt),
	          *bi.calc_pva_no_reset_since(bi.time_span().second, t + dt));
	assert_eq(*normal.calc_pva_no_reset_since(bi.time_span().second, t - dt * 10),
	          *bi.calc_pva_no_reset_since(bi.time_span().second, t - dt * 10));
}

TEST_F(BufferedImuTests, NoResetSinceFullBufferAtStart_SLOW) {
	auto baby   = baseline_buffered(0.0, 0.5);
	auto biggie = baseline_buffered(0.0, 1.0);
	// Propagate far enough so that biggie has at least 1 pva record more than baby.
	for (navtk::Size k = 1; k < 27; k++) {
		auto imu = k % 2 == 0 ? imu_pack(baby.time_span().second + dt, dv, dth)
		                      : imu_pack(baby.time_span().second + dt, -dv, -dth);
		baby.mechanize(imu);
		biggie.mechanize(imu);
		// >= 1 reset is required, or it'll return solution as normal rather than repropagate
		if (k == 20) {
			auto reset = baby.calc_pva();
			baby.reset(reset);
			biggie.reset(reset);
		}
	}
	// Because biggie has access to 'prior' record that baby does not, the starting point of
	// the repropagation is slightly different
	auto baby_sol = *baby.calc_pva_no_reset_since(baby.time_span().second, baby.time_span().first);
	auto big_sol = *biggie.calc_pva_no_reset_since(baby.time_span().second, baby.time_span().first);

	assert_eq(baby_sol, big_sol);

	// Additional check because assert_eq has slightly nerfed altitude check
	ASSERT_FLOAT_EQ(baby_sol.get_p3(), big_sol.get_p3());
}

TEST_F(BufferedImuTests, ResetBetweenRecordsFullBuffer_SLOW) {
	auto baby = baseline_buffered(0.0, 1.0);
	prop(baby, 25);
	prop(bi, 25 - base_prop);

	auto r_t       = baby.time_span().first + dt * 10.7;
	auto reset_val = make_shared<MeasurementPositionVelocityAttitude>(
	    to_positionvelocityattitude(StandardPosVelAtt(r_t, llh0, vned0, c_s_to_ned)));
	ASSERT_TRUE(baby.reset(reset_val, make_shared<ImuErrors>(ImuErrors{})));

	assert_eq(*reset_val, *baby.calc_pva(r_t));

	auto r_normal = BufferedImu(*reset_val,
	                            make_shared<aspn_xtensor::MeasurementImu>(imu_pack(r_t, dv, dth)),
	                            dt,
	                            ImuErrors{},
	                            options);

	for (navtk::Size k = 1; k < 16; k++) {
		r_normal.mechanize(imu_pack(r_normal.time_span().second + dt, dv, dth));
	}

	// Solutions post-reset should match ins initialized at reset time/solution...
	assert_eq(*r_normal.calc_pva(baby.time_span().second), *baby.calc_pva());
	assert_eq(*r_normal.calc_pva(r_t + dt * 3), *baby.calc_pva(r_t + dt * 3));
	assert_eq(*r_normal.calc_pva(r_t + dt / 2.0), *baby.calc_pva(r_t + dt / 2.0));
	assert_eq(*r_normal.calc_pva(r_t), *baby.calc_pva(r_t));

	// Unless no_reset is called
	assert_eq(*bi.calc_pva(baby.time_span().second),
	          *baby.calc_pva_no_reset_since(baby.time_span().second, baby.time_span().first + dt));
	assert_eq(*bi.calc_pva(r_t), *baby.calc_pva_no_reset_since(r_t, baby.time_span().first + dt));
	assert_eq(*bi.calc_pva(r_t + dt * 3),
	          *baby.calc_pva_no_reset_since(r_t + dt * 3.0, baby.time_span().first + dt));
	assert_eq(*bi.calc_pva(r_t + dt / 2.0),
	          *baby.calc_pva_no_reset_since(r_t + dt / 2.0, baby.time_span().first + dt));

	// Pre-reset should match the no-reset inertial solution
	assert_eq(*bi.calc_pva(baby.time_span().first), *baby.calc_pva(baby.time_span().first));
	assert_eq(*bi.calc_pva(baby.time_span().first + dt * 2.5),
	          *baby.calc_pva(baby.time_span().first + dt * 2.5));
	assert_eq(*bi.calc_pva(baby.time_span().first + dt / 2.0),
	          *baby.calc_pva(baby.time_span().first + dt / 2.0));

	ASSERT_EQ(baby.calc_pva(baby.time_span().first - 0.5), nullptr);
	assert_eq(*baby.calc_pva(baby.time_span().first),
	          *baby.calc_pva_no_reset_since(baby.time_span().first, baby.time_span().first + dt));
	assert_eq(*baby.calc_pva(baby.time_span().first + dt / 2.0),
	          *baby.calc_pva_no_reset_since(baby.time_span().first + dt / 2.0,
	                                        baby.time_span().first + dt));

	// Since the requested solution time is within 1 dt prior to a reset, this one would
	// interpolate between the last good and the reset value for the first, reset and reprop for
	// the second resulting in a tiny diff. However, since requesting values from time prior to a
	// reset occurring should be unlikely, we'll just not worry about it for now
	// assert_eq(*baby.calc_pva(r_t - dt/2.0),
	//       *baby.calc_pva_no_reset_since(r_t - dt/2.0, baby.time_span().first + dt));

	// This one is outside the 1 dt threshold
	assert_eq(*baby.calc_pva(r_t - 1.5 * dt),
	          *baby.calc_pva_no_reset_since(r_t - 1.5 * dt, baby.time_span().first + dt));
}

TEST_F(BufferedImuTests, ForceRateBefore) {
	ASSERT_EQ(bi.calc_force_and_rate(bi.time_span().first - 1.0), nullptr);
}

TEST_F(BufferedImuTests, ForceRateAfter) {
	ASSERT_EQ(bi.calc_force_and_rate(bi.time_span().second + 1.0), nullptr);
}

TEST_F(BufferedImuTests, ForceRateStart) {
	auto expected_force = calc_force_ned(start.get_C_s_to_ned(), dt, dth, dv);
	auto expected_rate  = calc_rot_rate(start_pva, dt, dth);
	auto farr           = bi.calc_force_and_rate(bi.time_span().first);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, expected_force);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, expected_rate);
}

TEST_F(BufferedImuTests, ForceRateEnd_SLOW) {
	prop(bi, from_setup_to_simpson);
	auto expected_force = calc_force_ned(simpsons.get_C_s_to_ned(), dt, dth, dv);
	auto expected_rate  = calc_rot_rate(simpsons_pva, dt, dth);
	auto farr           = bi.calc_force_and_rate(bi.time_span().second);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, expected_force);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, expected_rate);
}

TEST_F(BufferedImuTests, ForceRateAtExactTime) {
	auto expected_force = calc_force_ned(quat_to_dcm(at10.get_quaternion()), dt, dth, dv);
	auto expected_rate  = calc_rot_rate(at10, dt, dth);
	auto farr           = bi.calc_force_and_rate(bi.time_span().first + dt * 10);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, expected_force);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, expected_rate);
}

TEST_F(BufferedImuTests, ForceRateBetweenRecords) {
	auto expected_force = calc_force_ned(quat_to_dcm(at10point7.get_quaternion()), dt, dth, dv);
	auto expected_rate  = calc_rot_rate(at10point7, dt, dth);
	auto farr           = bi.calc_force_and_rate(bi.time_span().first + dt * 10.7);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, expected_force);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, expected_rate);
}

TEST_F(BufferedImuTests, AvgForceRateBefore) {
	ASSERT_EQ(
	    bi.calc_force_and_rate(bi.time_span().first - 1.0,
	                           aspn_xtensor::to_type_timestamp(
	                               to_seconds(bi.time_span().second + bi.time_span().first) / 2.0)),
	    nullptr);
}

TEST_F(BufferedImuTests, AvgForceRateAfter) {
	ASSERT_EQ(
	    bi.calc_force_and_rate(aspn_xtensor::to_type_timestamp(
	                               to_seconds(bi.time_span().second + bi.time_span().first) / 2.0),
	                           bi.time_span().second + 1.0),
	    nullptr);
}

TEST_F(BufferedImuTests, AvgForceRateStart) {
	auto farr    = bi.calc_force_and_rate(bi.time_span().first);
	auto far_avg = bi.calc_force_and_rate(bi.time_span().first, bi.time_span().first);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, Vector3{far_avg->get_meas_accel()});
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, Vector3{far_avg->get_meas_gyro()});
}

TEST_F(BufferedImuTests, AvgForceRateEnd) {
	auto farr    = bi.calc_force_and_rate(bi.time_span().second);
	auto far_avg = bi.calc_force_and_rate(bi.time_span().second, bi.time_span().second);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, Vector3{far_avg->get_meas_accel()});
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, Vector3{far_avg->get_meas_gyro()});
}

TEST_F(BufferedImuTests, AvgForceRateAtExactTime) {
	auto farr = bi.calc_force_and_rate(bi.time_span().first + dt * 10);
	auto far_avg =
	    bi.calc_force_and_rate(bi.time_span().first + dt * 10, bi.time_span().first + dt * 10);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, Vector3{far_avg->get_meas_accel()});
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, Vector3{far_avg->get_meas_gyro()});
}

TEST_F(BufferedImuTests, AvgForceRateBetweenRecords) {
	auto farr = bi.calc_force_and_rate(bi.time_span().first + dt * 10.7);
	auto far_avg =
	    bi.calc_force_and_rate(bi.time_span().first + dt * 10.7, bi.time_span().first + dt * 10.7);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, Vector3{far_avg->get_meas_accel()});
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, Vector3{far_avg->get_meas_gyro()});
}

TEST_F(BufferedImuTests, AvgForceRateSuperClose) {
	auto t    = bi.time_span().first + dt * 10;
	auto farr = bi.calc_force_and_rate(t);
	std::vector<double> deltas({1e-15, 1e-14, 1e-13, 1e-12, 1e-11});
	for (auto d : deltas) {
		auto far_avg = bi.calc_force_and_rate(t - d, t + d);
		ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, Vector3{far_avg->get_meas_accel()});
		ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, Vector3{far_avg->get_meas_gyro()});
	}
}

TEST_F(BufferedImuTests, AvgForceRateAll_SLOW) {
	auto ins    = baseline_buffered();
	auto forces = zeros(3);
	auto rates  = zeros(3);
	for (navtk::Size k = 1; k < 20; k++) {
		ins.mechanize(imu_pack(start_time + k * dt, dv, dth));
		auto farr = ins.calc_force_and_rate(ins.time_span().second);
		forces += farr->get_meas_accel();
		rates += farr->get_meas_gyro();
	}
	auto avgs = ins.calc_force_and_rate(ins.time_span().first, ins.time_span().second);
	ASSERT_ALLCLOSE(forces / 19.0, Vector3{avgs->get_meas_accel()});
	ASSERT_ALLCLOSE(rates / 19.0, Vector3{avgs->get_meas_gyro()});
}

TEST_F(BufferedImuTests, ImuErrsBefore) {
	auto initial        = bi.get_imu_errors(bi.time_span().first);
	auto t              = bi.time_span().first - 1.0;
	imu_e.time_validity = t;

	auto res = EXPECT_WARN(
	    bi.reset(make_shared<MeasurementPositionVelocityAttitude>(
	                 to_positionvelocityattitude(StandardPosVelAtt(t, llh0, vned0, c_s_to_ned))),
	             make_shared<ImuErrors>(imu_e)),
	    "failed in_range");

	ASSERT_FALSE(res);

	assert_eq(*bi.get_imu_errors(bi.time_span().first), *initial);
}

TEST_F(BufferedImuTests, ImuErrsAfter) {
	auto initial        = bi.get_imu_errors(bi.time_span().second);
	auto t              = bi.time_span().second + 1.0;
	imu_e.time_validity = t;

	auto res = EXPECT_WARN(
	    bi.reset(make_shared<MeasurementPositionVelocityAttitude>(
	                 to_positionvelocityattitude(StandardPosVelAtt(t, llh0, vned0, c_s_to_ned))),
	             make_shared<ImuErrors>(imu_e)),
	    "failed in_range");

	ASSERT_FALSE(res);

	assert_eq(*bi.get_imu_errors(bi.time_span().second), *initial);
}

TEST_F(BufferedImuTests, ImuErrsStart) {
	auto initial        = bi.get_imu_errors(bi.time_span().first);
	auto t              = bi.time_span().first;
	imu_e.time_validity = t;
	ASSERT_TRUE(
	    bi.reset(make_shared<MeasurementPositionVelocityAttitude>(
	                 to_positionvelocityattitude(StandardPosVelAtt(t, llh0, vned0, c_s_to_ned))),
	             make_shared<ImuErrors>(imu_e)));
	assert_eq(*bi.get_imu_errors(bi.time_span().first), imu_e);
	assert_eq(*bi.get_imu_errors(bi.time_span().second), imu_e);
}

TEST_F(BufferedImuTests, ImuErrsEnd) {
	auto initial        = bi.get_imu_errors(bi.time_span().first);
	auto t              = bi.time_span().second;
	imu_e.time_validity = t;
	ASSERT_TRUE(
	    bi.reset(make_shared<MeasurementPositionVelocityAttitude>(
	                 to_positionvelocityattitude(StandardPosVelAtt(t, llh0, vned0, c_s_to_ned))),
	             make_shared<ImuErrors>(imu_e)));
	assert_eq(*bi.get_imu_errors(bi.time_span().first), *initial);
	assert_eq(*bi.get_imu_errors(bi.time_span().second), imu_e);
}

TEST_F(BufferedImuTests, ImuErrsAtExactTime) {
	auto initial        = bi.get_imu_errors(bi.time_span().first);
	auto t              = bi.time_span().first + dt * 10;
	imu_e.time_validity = t;
	ASSERT_TRUE(
	    bi.reset(make_shared<MeasurementPositionVelocityAttitude>(
	                 to_positionvelocityattitude(StandardPosVelAtt(t, llh0, vned0, c_s_to_ned))),
	             make_shared<ImuErrors>(imu_e)));
	assert_eq(*bi.get_imu_errors(bi.time_span().first + dt * 10), imu_e);
	assert_eq(*bi.get_imu_errors(bi.time_span().first + dt * 10 - 0.01), *initial);
	assert_eq(*bi.get_imu_errors(bi.time_span().first + dt * 10 + 0.01), imu_e);
	assert_eq(*bi.get_imu_errors(bi.time_span().first), *initial);
	assert_eq(*bi.get_imu_errors(bi.time_span().second), imu_e);
}

TEST_F(BufferedImuTests, ImuErrsBetweenRecords) {
	auto initial        = bi.get_imu_errors(bi.time_span().first);
	auto t              = bi.time_span().first + dt * 10.7;
	imu_e.time_validity = t;
	ASSERT_TRUE(
	    bi.reset(make_shared<MeasurementPositionVelocityAttitude>(
	                 to_positionvelocityattitude(StandardPosVelAtt(t, llh0, vned0, c_s_to_ned))),
	             make_shared<ImuErrors>(imu_e)));
	assert_eq(*bi.get_imu_errors(bi.time_span().first + dt * 10.7), imu_e);
	assert_eq(*bi.get_imu_errors(bi.time_span().first + dt * 10.7 - 0.01), *initial);
	assert_eq(*bi.get_imu_errors(bi.time_span().first + dt * 10.7 + 0.01), imu_e);
	assert_eq(*bi.get_imu_errors(bi.time_span().first), *initial);
	assert_eq(*bi.get_imu_errors(bi.time_span().second), imu_e);
}

TEST_F(BufferedImuTests, CompareSynced) {

	auto ins              = Inertial(make_shared<StandardPosVelAtt>(start));
	auto buf              = baseline_buffered();
	navtk::Size num_loops = 10;
	navtk::Vector x{
	    2.0, 4.5, -3, -0.2, 1.1, 0.3, 3e-3, 1e-4, -2e-6, 3e-5, 3e-4, -3e-6, 1e-5, 1e-4, -1e-6};
	std::vector<MeasurementPositionVelocityAttitude> res;
	for (navtk::Size k = 0; k < num_loops; ++k) {
		auto c_sol = to_positionvelocityattitude(*ins.get_solution());
		assert_eq(*buf.calc_pva(), c_sol);
		res.push_back(c_sol);
		auto t = ins.get_solution()->time_validity + dt;
		// Actual values don't matter, just need results to be the same
		auto imu = imu_pack(t, navtk::ones(3) * 1e-1, navtk::ones(3) * 1e-1);
		buf.mechanize(imu);
		ins.mechanize(imu.get_time_of_validity(), imu.get_meas_accel(), imu.get_meas_gyro());

		auto ins_time = buf.time_span().second;
		auto sol      = buf.calc_pva();
		auto corr_sol =
		    navtk::filtering::apply_error_states<navtk::filtering::Pinson15NedBlock>(*sol, x);

		auto curr_errs = buf.get_imu_errors(ins_time);
		curr_errs->accel_biases += xt::view(x, xt::range(9, 12));
		curr_errs->gyro_biases += xt::view(x, xt::range(12, 15));

		ins.set_accel_biases(curr_errs->accel_biases);
		ins.set_gyro_biases(curr_errs->gyro_biases);
		ins.reset(navtk::utils::to_standardposvelatt(corr_sol));

		buf.reset(make_shared<MeasurementPositionVelocityAttitude>(corr_sol), curr_errs);
	}

	std::for_each(res.begin(),
	              res.end(),
	              [&buf, this](const MeasurementPositionVelocityAttitude& stored_pva) {
		              assert_eq(*buf.calc_pva(stored_pva.get_time_of_validity()), stored_pva);
	              });
}

TEST_F(BufferedImuTests, CorrectTimesAfterMech) {
	auto buf = baseline_buffered();
	auto t   = buf.calc_pva()->get_time_of_validity();
	for (navtk::Size k = 0; k < 5; ++k) {
		t        = t + dt;  //+= not implemented
		auto imu = imu_pack(t, zeros(3), zeros(3));
		buf.mechanize(imu);
		ASSERT_EQ(t.get_elapsed_nsec(), buf.calc_pva()->get_time_of_validity().get_elapsed_nsec());
		ASSERT_EQ(t.get_elapsed_nsec(), buf.time_span().second.get_elapsed_nsec());
	}
}

TEST_F(BufferedImuTests, ComparePvaGet) {
	auto buf = baseline_buffered();
	for (navtk::Size k = 1; k < 5; ++k) {
		buf.mechanize(imu_pack(start_time + k * dt, zeros(3), zeros(3)));
		assert_eq(*buf.calc_pva(), *buf.calc_pva(buf.time_span().second));
	}
}

TEST_F(BufferedImuTests, DetectBadDt) {
	auto ins = baseline_buffered();
	for (navtk::Size k = 1; k < 20; k++) {
		ins.mechanize(
		    imu_pack(start_time + k * dt + navtk::experimental::rand_n() * 0.001, dv, dth));
	}
	// Not actually a test, but should generate a spdlog warning
	EXPECT_WARN(ins.mechanize(imu_pack(start_time + 101 * dt, dv, dth)), "Suspicious");
}

TEST_F(BufferedImuTests, JustInitialized) {
	auto expected_force       = calc_force_ned(c_s_to_ned, dt, dth, dv);
	auto expected_rate        = calc_rot_rate(start_pva, dt, dth);
	auto test_errors          = ImuErrors{};
	test_errors.time_validity = start_time;

	auto ins = baseline_buffered();

	assert_eq(*ins.calc_pva(), start_pva);
	assert_eq(*ins.calc_pva(start_time), start_pva);
	assert_eq(*ins.calc_pva_no_reset_since(start_time, start_time), start_pva);
	auto farr = ins.calc_force_and_rate(start_time);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, expected_force);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, expected_rate);
	ASSERT_EQ(ins.time_span().first, start_time);
	ASSERT_EQ(ins.time_span().second, start_time);
	assert_eq(*ins.get_imu_errors(start_time), test_errors);

	ins.reset(make_shared<MeasurementPositionVelocityAttitude>(start_pva));

	assert_eq(*ins.calc_pva(), start_pva);
	assert_eq(*ins.calc_pva(start_time), start_pva);
	assert_eq(*ins.calc_pva_no_reset_since(start_time, start_time), start_pva);
	farr = ins.calc_force_and_rate(start_time);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, expected_force);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, expected_rate);
	ASSERT_EQ(ins.time_span().first, start_time);
	ASSERT_EQ(ins.time_span().second, start_time);
	assert_eq(*ins.get_imu_errors(start_time), test_errors);
}

TEST_F(BufferedImuTests, vary_rate_SLOW) {
	auto mo       = MechanizationOptions{};
	mo.int_method = IntegrationMethods::SIMPSONS_RULE;
	mo.dcm_method = DcmIntegrationMethods::EXPONENTIAL;
	start_pva.set_v1(0);
	start_pva.set_v2(0);
	start_pva.set_v3(0);
	BufferedImu b1(start_pva, nullptr, 0.05, ImuErrors{}, mo);
	BufferedImu b2(start_pva, nullptr, 0.1, ImuErrors{}, mo);

	for (navtk::Size k = 0; k < 10; k++) {
		auto cur   = b1.calc_pva();
		auto meas1 = stationary_imu(cur, 0.05);
		b1.mechanize(meas1);
		cur        = b1.calc_pva();
		auto meas2 = stationary_imu(cur, 0.05);
		b1.mechanize(meas2);

		cur        = b2.calc_pva();
		auto meas3 = stationary_imu(cur, 0.1);
		b2.mechanize(meas3);
	}
	assert_eq(*b1.calc_pva(), *b2.calc_pva());
}

// Making a separate suite with a name ending in DeathTest causes the EXPECT_DEATHs below to run
// before other tests have a chance to start threads, per google recommendation
// https://github.com/google/googletest/blob/main/docs/advanced.md#death-tests-and-threads
struct BufferedImuDeathTest : BufferedImuTests {};

TEST_F(BufferedImuDeathTest, rule_5) {
	// Copy ctor
	BufferedImu copied = bi;

	prop(copied, 10);
	prop(bi, 10);
	auto exp_pva = *bi.calc_pva();
	assert_eq(exp_pva, *copied.calc_pva());

	// Move ctor
	auto moved = std::move(bi);
	assert_eq(*moved.calc_pva(), exp_pva);
	EXPECT_DEATH(bi.calc_pva(), "");

	// Move assign
	bi = std::move(moved);
	assert_eq(exp_pva, *bi.calc_pva());
	EXPECT_DEATH(moved.calc_pva(), "");

	// Copy assign
	copied = bi;
	prop(copied, 10);
	prop(bi, 10);
	assert_eq(*bi.calc_pva(), *copied.calc_pva());
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, BufferedImuTests, rejects_sampled) {
	auto imu = std::make_shared<MeasurementImu>(navtk::testing::stationary_imu(
	    std::make_shared<aspn_xtensor::MeasurementPositionVelocityAttitude>(test.start_pva), 0.1));
	imu->set_imu_type(ASPN_MEASUREMENT_IMU_IMU_TYPE_SAMPLED);
	EXPECT_HONORS_MODE_EX(test.bi.add_data(imu),
	                      "Only ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED currently supported",
	                      std::invalid_argument);
	EXPECT_HONORS_MODE_EX(test.bi.mechanize(*imu),
	                      "Only ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED currently supported",
	                      std::invalid_argument);
}
