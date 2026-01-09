#include <gtest/gtest.h>
#include <inertial/BufferedPvaTests.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/inertial/BufferedIns.hpp>
#include <navtk/inertial/inertial_functions.hpp>
#include <navtk/utils/conversions.hpp>
#include <navtk/utils/interpolation.hpp>

using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::TypeTimestamp;
using navtk::inertial::BufferedIns;
using navtk::inertial::calc_force_ned;
using navtk::inertial::calc_rot_rate;
using navtk::navutils::quat_to_dcm;
using navtk::utils::linear_interp_pva;

struct BufferedInsTests : public BufferedPvaTests {

	double r_tol = 0.05;
	double a_tol = 1e-6;

	BufferedIns bi;

	BufferedInsTests() : BufferedPvaTests(), bi(BufferedIns(start_pva, 10.0)) {}


	void assert_bi_results(const aspn_xtensor::TypeTimestamp& t,
	                       const MeasurementPositionVelocityAttitude& expected) {

		assert_eq(*bi.calc_pva(t), expected);
	}

	// expected is nullptr
	void assert_bi_results(const aspn_xtensor::TypeTimestamp& t) {
		ASSERT_EQ(bi.calc_pva(t), nullptr);
	}

	virtual void SetUp() override {
		bi.add_pva(at10);
		bi.add_pva(at10point7);
		bi.add_pva(simpsons_pva);
	}
};

TEST_F(BufferedInsTests, CheckRange) {
	ASSERT_EQ(bi.time_span().first, start_time);
	ASSERT_EQ(bi.time_span().second, simpsons_pva.get_time_of_validity());
}

TEST_F(BufferedInsTests, SolutionBefore) {
	// Request solution before available time; nullptr
	assert_bi_results(bi.time_span().first - 0.1);
}

TEST_F(BufferedInsTests, SolutionAfter) {
	// Request solution after available times; nullptr
	assert_bi_results(bi.time_span().second + 0.1);
}

TEST_F(BufferedInsTests, SolutionAtStartTime) { assert_bi_results(start_time, start_pva); }

TEST_F(BufferedInsTests, SolutionAtEndTime) { assert_bi_results(start_time + 1.98, simpsons_pva); }

TEST_F(BufferedInsTests, SolutionBetweenRecords) {
	assert_bi_results(start_time + 1.2,
	                  linear_interp_pva(at10point7, simpsons_pva, start_time + 1.2));
}

TEST_F(BufferedInsTests, ForceRateBefore) {
	ASSERT_EQ(bi.calc_force_and_rate(bi.time_span().first - 1.0), nullptr);
}

TEST_F(BufferedInsTests, ForceRateAfter) {
	ASSERT_EQ(bi.calc_force_and_rate(bi.time_span().second + 1.0), nullptr);
}

TEST_F(BufferedInsTests, ForceRateAtExactTime) {
	auto expected_force = calc_force_ned(quat_to_dcm(at10.get_quaternion()), dt, dth, dv);
	auto expected_rate  = calc_rot_rate(at10, dt, dth);
	auto farr           = bi.calc_force_and_rate(bi.time_span().first + dt * 10);
	ASSERT_ALLCLOSE_EX(Vector3{farr->get_meas_accel()}, expected_force, r_tol, a_tol);
	ASSERT_ALLCLOSE_EX(Vector3{farr->get_meas_gyro()}, expected_rate, r_tol, a_tol);
}

TEST_F(BufferedInsTests, ForceRateBetweenRecords) {
	auto expected_force = calc_force_ned(quat_to_dcm(at10point7.get_quaternion()), dt, dth, dv);
	auto expected_rate  = calc_rot_rate(at10point7, dt, dth);
	auto farr           = bi.calc_force_and_rate(bi.time_span().first + dt * 10.7);
	ASSERT_ALLCLOSE_EX(Vector3{farr->get_meas_accel()}, expected_force, r_tol, a_tol);
	ASSERT_ALLCLOSE_EX(Vector3{farr->get_meas_gyro()}, expected_rate, r_tol, a_tol);
}

TEST_F(BufferedInsTests, AvgForceRateBefore) {
	ASSERT_EQ(
	    bi.calc_force_and_rate(bi.time_span().first - 1.0,
	                           aspn_xtensor::to_type_timestamp(
	                               to_seconds(bi.time_span().second + bi.time_span().first) / 2.0)),
	    nullptr);
}

TEST_F(BufferedInsTests, AvgForceRateAfter) {
	ASSERT_EQ(
	    bi.calc_force_and_rate(aspn_xtensor::to_type_timestamp(
	                               to_seconds(bi.time_span().second + bi.time_span().first) / 2.0),
	                           bi.time_span().second + 1.0),
	    nullptr);
}

TEST_F(BufferedInsTests, ForceRateBeforeStart) {
	EXPECT_WARN(bi.calc_force_and_rate(bi.time_span().first - dt), "out of range");
	EXPECT_WARN(bi.calc_force_and_rate(bi.time_span().first - dt, bi.time_span().first - dt),
	            "out of range");
}

TEST_F(BufferedInsTests, ForceRateAfterEnd) {
	EXPECT_WARN(bi.calc_force_and_rate(bi.time_span().second + dt), "out of range");
	EXPECT_WARN(bi.calc_force_and_rate(bi.time_span().second + dt, bi.time_span().second + dt),
	            "out of range");
}

TEST_F(BufferedInsTests, ForceRateSinglePva) {
	BufferedIns bi2(BufferedIns(start_pva, 10.0));
	EXPECT_WARN(bi2.calc_force_and_rate(start_pva.get_time_of_validity()), "not two PVAs");
	EXPECT_WARN(
	    bi2.calc_force_and_rate(start_pva.get_time_of_validity(), start_pva.get_time_of_validity()),
	    "not two PVAs");
}

TEST_F(BufferedInsTests, AvgForceRateAtExactTime) {
	auto farr = bi.calc_force_and_rate(bi.time_span().first + dt * 10);
	auto far_avg =
	    bi.calc_force_and_rate(bi.time_span().first + dt * 10, bi.time_span().first + dt * 10);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, Vector3{far_avg->get_meas_accel()});
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, Vector3{far_avg->get_meas_gyro()});
}

TEST_F(BufferedInsTests, AvgForceRateBetweenRecords) {
	auto farr = bi.calc_force_and_rate(bi.time_span().first + dt * 10.7);
	auto far_avg =
	    bi.calc_force_and_rate(bi.time_span().first + dt * 10.7, bi.time_span().first + dt * 10.7);
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, Vector3{far_avg->get_meas_accel()});
	ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, Vector3{far_avg->get_meas_gyro()});
}

TEST_F(BufferedInsTests, AvgForceRateSuperClose) {
	auto t    = bi.time_span().first + dt * 10;
	auto farr = bi.calc_force_and_rate(t);
	std::vector<double> deltas({1e-15, 1e-14, 1e-13, 1e-12, 1e-11});
	for (auto d : deltas) {
		auto far_avg = bi.calc_force_and_rate(t - d, t + d);
		ASSERT_ALLCLOSE(Vector3{farr->get_meas_accel()}, Vector3{far_avg->get_meas_accel()});
		ASSERT_ALLCLOSE(Vector3{farr->get_meas_gyro()}, Vector3{far_avg->get_meas_gyro()});
	}
}

TEST_F(BufferedInsTests, AvgForceRateAll) {
	auto forces = zeros(3);
	auto rates  = zeros(3);

	std::vector<aspn_xtensor::TypeTimestamp> times = {start_pva.get_time_of_validity() + dt,
	                                                  at10.get_time_of_validity(),
	                                                  at10point7.get_time_of_validity(),
	                                                  simpsons_pva.get_time_of_validity() - dt};
	for (const aspn_xtensor::TypeTimestamp& time : times) {
		auto farr = bi.calc_force_and_rate(time);
		ASSERT_NE(farr, nullptr);
		forces += farr->get_meas_accel();
		rates += farr->get_meas_gyro();
	}
	auto avgs = bi.calc_force_and_rate(bi.time_span().first, bi.time_span().second);
	ASSERT_ALLCLOSE_EX(forces / times.size(), Vector3{avgs->get_meas_accel()}, r_tol, a_tol);
	ASSERT_ALLCLOSE_EX(rates / times.size(), Vector3{avgs->get_meas_gyro()}, r_tol, a_tol);
}


TEST_F(BufferedInsTests, ForceRateFirstPartialBuffer) {
	auto bsize = 5;
	auto cp    = start_pva;
	auto bpa   = BufferedIns(cp, 1.0, bsize);
	for (auto k = 0; k < bsize - 2; ++k) {
		cp.set_time_of_validity((cp.get_time_of_validity() + 1.0));
		bpa.add_data(std::make_shared<MeasurementPositionVelocityAttitude>(cp));
	}
	auto f_r = bpa.calc_force_and_rate(bpa.time_span().first);
	ASSERT_TRUE(f_r != nullptr);
}

TEST_F(BufferedInsTests, ForceRateFirstFullBuffer) {
	auto bsize = 5;
	auto cp    = start_pva;
	auto bpa   = BufferedIns(cp, 1.0, bsize);
	for (auto k = 0; k < bsize + 1; ++k) {
		cp.set_time_of_validity((cp.get_time_of_validity() + 1.0));
		bpa.add_data(std::make_shared<MeasurementPositionVelocityAttitude>(cp));
	}
	auto f_r = bpa.calc_force_and_rate(bpa.time_span().first);
	ASSERT_TRUE(f_r != nullptr);
}

// Making a separate suite with a name ending in DeathTest causes the EXPECT_DEATHs below to run
// before other tests have a chance to start threads, per google recommendation
// https://github.com/google/googletest/blob/main/docs/advanced.md#death-tests-and-threads
struct BufferedInsDeathTest : BufferedInsTests {};

TEST_F(BufferedInsDeathTest, rule_5) {
	// Copy ctor
	BufferedIns copied = bi;

	bi.add_pva(at10);
	bi.add_pva(at10point7);
	bi.add_pva(simpsons_pva);

	copied.add_pva(at10);
	copied.add_pva(at10point7);
	copied.add_pva(simpsons_pva);

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

	bi.add_pva(at10);
	bi.add_pva(at10point7);
	bi.add_pva(simpsons_pva);

	copied.add_pva(at10);
	copied.add_pva(at10point7);
	copied.add_pva(simpsons_pva);

	assert_eq(*bi.calc_pva(), *copied.calc_pva());
}
