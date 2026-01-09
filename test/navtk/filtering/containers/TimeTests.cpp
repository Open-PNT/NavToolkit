#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>

#include <navtk/aspn.hpp>

using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;

constexpr int NANO_PER_SEC = 1'000'000'000;

TEST(TimeTests, checkDoubleConversions) {
	// expect nanosecond precision with small numbers
	aspn_xtensor::TypeTimestamp t1 = aspn_xtensor::to_type_timestamp(1.2345678901);
	EXPECT_EQ(t1.get_elapsed_nsec(), 1234567890);
	EXPECT_NEAR(to_seconds(t1), 1.234567890, 1e-9);
	EXPECT_NEAR(to_seconds(to_type_timestamp(1, 234567890)),
	            to_seconds(to_type_timestamp(1.2345678901)),
	            1e-9);

	// expect nanosecond precision w/rounding
	aspn_xtensor::TypeTimestamp t2 = aspn_xtensor::to_type_timestamp(1.2345678909);
	EXPECT_EQ(t2.get_elapsed_nsec(), 1234567891);
	EXPECT_NEAR(to_seconds(t2), 1.234567891, 1e-9);

	// test case where nanoseconds round to the next second
	aspn_xtensor::TypeTimestamp t3 = aspn_xtensor::to_type_timestamp(1.9999999997);
	EXPECT_EQ(t3.get_elapsed_nsec(), 2000000000);
	EXPECT_NEAR(to_seconds(t3), 2.0, 1e-9);

	// test case where nanoseconds round to a zero second
	aspn_xtensor::TypeTimestamp t4 = aspn_xtensor::to_type_timestamp(0.9999999997);
	EXPECT_EQ(t4.get_elapsed_nsec(), 1000000000);
	EXPECT_NEAR(to_seconds(t4), 1.0, 1e-9);

	// test case tiny number
	aspn_xtensor::TypeTimestamp t5 = aspn_xtensor::to_type_timestamp(0.0000000004);
	EXPECT_EQ(t5.get_elapsed_nsec(), 0);
	EXPECT_NEAR(to_seconds(t5), 0.0, 1e-9);

	// expect less precision with large numbers
	aspn_xtensor::TypeTimestamp t6 = aspn_xtensor::to_type_timestamp(987654321.123456789);
	EXPECT_NEAR(t6.get_elapsed_nsec(), double(987654321123456789), 1e6);
	EXPECT_NEAR(to_seconds(t6), 987654321.123456789, 1e-6);
}

TEST(TimeTests, checkNegativeDoubleConversions) {
	// expect nanosecond precision with small numbers
	aspn_xtensor::TypeTimestamp t1 = aspn_xtensor::to_type_timestamp(-1.2345678901);
	EXPECT_EQ(t1.get_elapsed_nsec(), -1234567890);
	EXPECT_NEAR(to_seconds(t1), -1.234567890, 1e-9);

	// expect nanosecond precision w/rounding
	aspn_xtensor::TypeTimestamp t2 = aspn_xtensor::to_type_timestamp(-1.2345678909);
	EXPECT_EQ(t2.get_elapsed_nsec(), -1234567891);
	EXPECT_NEAR(to_seconds(t2), -1.234567891, 1e-9);

	// test case where nanoseconds round to the next second
	aspn_xtensor::TypeTimestamp t3 = aspn_xtensor::to_type_timestamp(-1.9999999997);
	EXPECT_EQ(t3.get_elapsed_nsec(), -2000000000);
	EXPECT_NEAR(to_seconds(t3), -2.0, 1e-9);

	// test case where nanoseconds round to a zero second
	aspn_xtensor::TypeTimestamp t4 = aspn_xtensor::to_type_timestamp(-0.9999999997);
	EXPECT_EQ(t4.get_elapsed_nsec(), -1000000000);
	EXPECT_NEAR(to_seconds(t4), -1.0, 1e-9);

	// test case tiny number
	aspn_xtensor::TypeTimestamp t5 = aspn_xtensor::to_type_timestamp(-0.0000000004);
	EXPECT_EQ(t5.get_elapsed_nsec(), 0);
	EXPECT_NEAR(to_seconds(t5), 0.0, 1e-9);

	// expect less precision with large numbers
	aspn_xtensor::TypeTimestamp t6 = aspn_xtensor::to_type_timestamp(-987654321.123456789);
	EXPECT_NEAR(t6.get_elapsed_nsec(), double(-987654321123456789), 1e6);
	EXPECT_NEAR(to_seconds(t6), -987654321.123456789, 1e-6);
}

TEST(TimeTests, Comparisons) {
	aspn_xtensor::TypeTimestamp t1 = aspn_xtensor::to_type_timestamp(1.2345);
	aspn_xtensor::TypeTimestamp t2 = aspn_xtensor::to_type_timestamp(1.2345);
	aspn_xtensor::TypeTimestamp t3 = aspn_xtensor::to_type_timestamp(2.3456);
	aspn_xtensor::TypeTimestamp t4 = aspn_xtensor::to_type_timestamp(2.3456);

	EXPECT_TRUE(t1 < t3);
	EXPECT_TRUE(t2 < t4);

	EXPECT_FALSE(t1 < t2);
	EXPECT_FALSE(t3 < t4);
}

TEST(TimeTests, TimeMinusTime) {
	aspn_xtensor::TypeTimestamp t1    = aspn_xtensor::to_type_timestamp(25.32);
	aspn_xtensor::TypeTimestamp t2    = aspn_xtensor::to_type_timestamp(12.06);
	aspn_xtensor::TypeTimestamp t_out = t1 - t2;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 13260000000);

	aspn_xtensor::TypeTimestamp t3 = aspn_xtensor::to_type_timestamp(3.55);
	t_out                          = t1 - t3;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 21770000000);

	aspn_xtensor::TypeTimestamp t4 = aspn_xtensor::to_type_timestamp(0.0);
	t_out                          = t1 - t4;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 25320000000);

	aspn_xtensor::TypeTimestamp t5 = aspn_xtensor::to_type_timestamp(32000.0);
	aspn_xtensor::TypeTimestamp t6 = aspn_xtensor::to_type_timestamp(0.000000001);
	t_out                          = t5 - t6;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 31999999999999);

	aspn_xtensor::TypeTimestamp t7 = to_type_timestamp(-5, 123456789);
	EXPECT_EQ(t7.get_elapsed_nsec(), -4876543211);

	aspn_xtensor::TypeTimestamp t8 = to_type_timestamp(0, -100);
	EXPECT_EQ(t8.get_elapsed_nsec(), -100);

	aspn_xtensor::TypeTimestamp t9  = aspn_xtensor::to_type_timestamp(1.2);
	aspn_xtensor::TypeTimestamp t10 = aspn_xtensor::to_type_timestamp(2.6);
	t_out                           = t9 - t10;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -1400000000);

	t_out = t10 - t9;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 1400000000);

	aspn_xtensor::TypeTimestamp t11 = to_type_timestamp(-123456789, -123456789);
	aspn_xtensor::TypeTimestamp t12 = to_type_timestamp(-372891086, -163801927);
	t_out                           = t11 - t12;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 249434297040345138);

	aspn_xtensor::TypeTimestamp t13 = to_type_timestamp(648292819, 472910649);
	t_out                           = t11 - t13;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -771749608596367438);

	aspn_xtensor::TypeTimestamp t14 = aspn_xtensor::to_type_timestamp(722.66);
	aspn_xtensor::TypeTimestamp t15 = aspn_xtensor::to_type_timestamp(30.5);
	t_out                           = t14 - t15;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 692160000000);

	t_out = t15 - t14;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -692160000000);

	aspn_xtensor::TypeTimestamp t16 = aspn_xtensor::to_type_timestamp(-0.4);
	aspn_xtensor::TypeTimestamp t17 = aspn_xtensor::to_type_timestamp(1044.4445);
	t_out                           = t16 - t17;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -1044844500000);

	aspn_xtensor::TypeTimestamp t18 = aspn_xtensor::to_type_timestamp(0.4);
	aspn_xtensor::TypeTimestamp t19 = aspn_xtensor::to_type_timestamp(-1044.4445);
	t_out                           = t19 - t18;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -1044844500000);

	t_out = t17 - t18;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 1044044500000);

	t_out = t16 - t18;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -800000000);

	auto t20 = to_type_timestamp(-3, 1300000000);
	EXPECT_EQ(t20.get_elapsed_nsec(), -1700000000);

	aspn_xtensor::TypeTimestamp t21 = to_type_timestamp(-3, -1300000000);
	EXPECT_EQ(t21.get_elapsed_nsec(), -4300000000);

	aspn_xtensor::TypeTimestamp t22 = to_type_timestamp(1, 800000000);
	t_out                           = t21 - t22;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -6100000000);
}

TEST(TimeTests, TimeMinusDouble) {
	aspn_xtensor::TypeTimestamp t1     = aspn_xtensor::to_type_timestamp(25.32);
	aspn_xtensor::TypeTimestamp dt_out = t1 - 13.2421;
	EXPECT_NEAR(to_seconds(dt_out), 12.0779, 1e-9);

	dt_out = t1 - 13.63279;
	EXPECT_NEAR(to_seconds(dt_out), 11.687210000, 1e-9);

	dt_out = t1 - 13.63279;
	EXPECT_NEAR(to_seconds(dt_out), 11.687210000, 1e-9);

	dt_out = t1 - 33.456;
	EXPECT_NEAR(to_seconds(dt_out), -8.136, 1e-9);

	aspn_xtensor::TypeTimestamp t2 = aspn_xtensor::to_type_timestamp(722.66);
	aspn_xtensor::TypeTimestamp t3 = aspn_xtensor::to_type_timestamp(30.5);
	aspn_xtensor::TypeTimestamp t4 = aspn_xtensor::to_type_timestamp(-0.4);
	aspn_xtensor::TypeTimestamp t5 = aspn_xtensor::to_type_timestamp(1044.4445);
	aspn_xtensor::TypeTimestamp t6 = aspn_xtensor::to_type_timestamp(0.4);
	aspn_xtensor::TypeTimestamp t7 = aspn_xtensor::to_type_timestamp(-1044.4445);
	dt_out                         = t2 - 30.5;
	EXPECT_NEAR(to_seconds(dt_out), 692.16, 1e-9);

	dt_out = t3 - 722.66;
	EXPECT_NEAR(to_seconds(dt_out), -692.16, 1e-9);

	dt_out = t4 - 1044.4445;
	EXPECT_NEAR(to_seconds(dt_out), -1044.8445, 1e-9);

	dt_out = t6 - 1044.4445;
	EXPECT_NEAR(to_seconds(dt_out), -1044.0445, 1e-9);

	dt_out = t5 - 0.4;
	EXPECT_NEAR(to_seconds(dt_out), 1044.0445, 1e-9);

	dt_out = t7 - 0.4;
	EXPECT_NEAR(to_seconds(dt_out), -1044.8445, 1e-9);
}

TEST(TimeTests, DoubleMinusTime) {
	aspn_xtensor::TypeTimestamp t1 = aspn_xtensor::to_type_timestamp(25.32);
	auto dt_out                    = 13.2421 - t1;
	EXPECT_EQ(dt_out, -12.0779);

	dt_out = 13.63279 - t1;
	EXPECT_EQ(dt_out, -11.68721);

	dt_out = 33.456 - t1;
	EXPECT_EQ(dt_out, 8.136);

	aspn_xtensor::TypeTimestamp t2 = aspn_xtensor::to_type_timestamp(722.66);
	aspn_xtensor::TypeTimestamp t3 = aspn_xtensor::to_type_timestamp(30.5);
	aspn_xtensor::TypeTimestamp t4 = aspn_xtensor::to_type_timestamp(1044.4445);
	aspn_xtensor::TypeTimestamp t5 = aspn_xtensor::to_type_timestamp(0.4);
	dt_out                         = 30.5 - t2;
	EXPECT_EQ(dt_out, -692.16);

	dt_out = 722.66 - t3;
	EXPECT_EQ(dt_out, 692.16);

	dt_out = -1044.4445 - t5;
	EXPECT_EQ(dt_out, -1044.8445);

	dt_out = 1044.4445 - t5;
	EXPECT_EQ(dt_out, 1044.0445);

	dt_out = -0.4 - t4;
	EXPECT_EQ(dt_out, -1044.8445);

	dt_out = 0.4 - t4;
	EXPECT_EQ(dt_out, -1044.0445);
}

TEST(TimeTests, TimePlusTime) {
	aspn_xtensor::TypeTimestamp t1    = aspn_xtensor::to_type_timestamp(25.32);
	aspn_xtensor::TypeTimestamp t2    = aspn_xtensor::to_type_timestamp(12.06);
	aspn_xtensor::TypeTimestamp t_out = t1 + t2;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 37380000000);

	aspn_xtensor::TypeTimestamp t3 = aspn_xtensor::to_type_timestamp(3.85);
	t_out                          = t1 + t3;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 29170000000);

	aspn_xtensor::TypeTimestamp t4 = aspn_xtensor::to_type_timestamp(0.0);
	t_out                          = t1 + t4;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 25320000000);

	aspn_xtensor::TypeTimestamp t5 = aspn_xtensor::to_type_timestamp(32000.999999999);
	aspn_xtensor::TypeTimestamp t6 = aspn_xtensor::to_type_timestamp(0.000000001);
	t_out                          = t5 + t6;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 32001000000000);

	aspn_xtensor::TypeTimestamp t7 = to_type_timestamp(0, 500000000);
	aspn_xtensor::TypeTimestamp t8 = to_type_timestamp(0, 600000000);
	t_out                          = t7 + t8;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 1100000000);

	aspn_xtensor::TypeTimestamp t9 = to_type_timestamp(0, 1537000000);
	EXPECT_EQ(t9.get_elapsed_nsec(), 1537000000);

	aspn_xtensor::TypeTimestamp t10 = to_type_timestamp(9223372036, 854775807);
	EXPECT_EQ(t10.get_elapsed_nsec(), 9223372036854775807);

	aspn_xtensor::TypeTimestamp t11 = to_type_timestamp(1847520957, 965332471);
	aspn_xtensor::TypeTimestamp t12 = to_type_timestamp(1840275927, 592750174);
	t_out                           = t11 + t12;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 3687796885558082645);

	aspn_xtensor::TypeTimestamp t14 = aspn_xtensor::to_type_timestamp(13.774);
	aspn_xtensor::TypeTimestamp t15 = aspn_xtensor::to_type_timestamp(0.00621);
	t_out                           = t14 + t15;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 13780210000);

	t_out = t15 + t14;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 13780210000);

	t14.set_elapsed_nsec(-1 * t14.get_elapsed_nsec());
	t_out = t14 + t15;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -13767790000);

	t_out = t15 + t14;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -13767790000);

	t15.set_elapsed_nsec(-1 * t15.get_elapsed_nsec());
	t_out = t14 + t15;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -13780210000);

	t_out = t15 + t14;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -13780210000);

	t14.set_elapsed_nsec(-1 * t14.get_elapsed_nsec());
	t_out = t14 + t15;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 13767790000);

	t_out = t15 + t14;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 13767790000);

	aspn_xtensor::TypeTimestamp t16 = aspn_xtensor::to_type_timestamp(24.302);
	aspn_xtensor::TypeTimestamp t17 = aspn_xtensor::to_type_timestamp(9.856);
	t_out                           = t16 + t17;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 34158000000);

	t_out = t17 + t16;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 34158000000);

	t16.set_elapsed_nsec(-1 * t16.get_elapsed_nsec());
	t_out = t16 + t17;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -14446000000);

	t_out = t17 + t16;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -14446000000);

	t17.set_elapsed_nsec(-1 * t17.get_elapsed_nsec());
	t_out = t16 + t17;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -34158000000);

	t_out = t17 + t16;
	EXPECT_EQ(t_out.get_elapsed_nsec(), -34158000000);

	t16.set_elapsed_nsec(-1 * t16.get_elapsed_nsec());
	t_out = t16 + t17;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 14446000000);

	t_out = t17 + t16;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 14446000000);

	aspn_xtensor::TypeTimestamp t18 = to_type_timestamp(0, 2'147'483'647);
	aspn_xtensor::TypeTimestamp t19 = to_type_timestamp(0, 2'147'483'647);
	t_out                           = t18 + t19;
	EXPECT_EQ(t_out.get_elapsed_nsec(), 4'294'967'294);
}

TEST(TimeTests, TimePlusDouble) {
	aspn_xtensor::TypeTimestamp t1     = aspn_xtensor::to_type_timestamp(25.32);
	aspn_xtensor::TypeTimestamp dt_out = t1 + 3.2;
	EXPECT_NEAR(to_seconds(dt_out), 28.52, 1e-9);

	aspn_xtensor::TypeTimestamp t2 = to_type_timestamp(2, 972039417);
	dt_out                         = t2 + 4.592750174;
	EXPECT_NEAR(to_seconds(dt_out), 7.564789591, 1e-9);

	dt_out = t2 + (-1.33478);
	EXPECT_NEAR(to_seconds(dt_out), 1.637259417, 1e-9);

	aspn_xtensor::TypeTimestamp t3 = to_type_timestamp(-7492, -845913000);
	dt_out                         = t3 + 3192.491043;
	EXPECT_NEAR(to_seconds(dt_out), -4300.35487, 1e-9);

	dt_out = t3 + 3192.923481;
	EXPECT_NEAR(to_seconds(dt_out), -4299.922432, 1e-9);

	dt_out = t3 + 8210.003226;
	EXPECT_NEAR(to_seconds(dt_out), 717.157313, 1e-9);

	aspn_xtensor::TypeTimestamp t4    = to_type_timestamp(111111, 111222333);
	aspn_xtensor::TypeTimestamp t_out = t4 + 222222.0;
	EXPECT_EQ(t_out, to_type_timestamp(333333, 111222333));
}

TEST(TimeTests, DoublePlusTime) {
	aspn_xtensor::TypeTimestamp t1     = aspn_xtensor::to_type_timestamp(25.32);
	aspn_xtensor::TypeTimestamp dt_out = 3.2 + t1;
	EXPECT_NEAR(to_seconds(dt_out), 28.52, 1e-9);

	aspn_xtensor::TypeTimestamp t2 = to_type_timestamp(2, 972039417);
	dt_out                         = 4.592750174 + t2;
	EXPECT_NEAR(to_seconds(dt_out), 7.564789591, 1e-9);

	dt_out = (-1.33478) + t2;
	EXPECT_NEAR(to_seconds(dt_out), 1.637259417, 1e-9);

	aspn_xtensor::TypeTimestamp t3 = to_type_timestamp(-7492, -845913000);
	dt_out                         = 3192.491043 + t3;
	EXPECT_NEAR(to_seconds(dt_out), -4300.35487, 1e-9);

	dt_out = 3192.923481 + t3;
	EXPECT_NEAR(to_seconds(dt_out), -4299.922432, 1e-9);

	dt_out = 8210.003226 + t3;
	EXPECT_NEAR(to_seconds(dt_out), 717.157313, 1e-9);
}

void _check_time_compare(const aspn_xtensor::TypeTimestamp& lower,
                         const aspn_xtensor::TypeTimestamp& higher,
                         int line,
                         bool expect_equal) {
	EXPECT_LE(lower, higher) << " at test line " << line;
	EXPECT_GE(higher, lower) << " at test line " << line;
	if (expect_equal) {
		EXPECT_EQ(lower, higher) << " at test line " << line;
		EXPECT_EQ(higher, lower) << " at test line " << line;
		EXPECT_FALSE(lower < higher) << " at test line " << line;
		EXPECT_FALSE(lower > higher) << " at test line " << line;
		EXPECT_FALSE(higher < lower) << " at test line " << line;
		EXPECT_FALSE(higher > lower) << " at test line " << line;
	} else {
		EXPECT_NE(lower, higher) << " at test line " << line;
		EXPECT_NE(higher, lower) << " at test line " << line;
		EXPECT_LT(lower, higher) << " at test line " << line;
		EXPECT_GT(higher, lower) << " at test line " << line;
	}
}

#define EXPECT_TIME_EQ(...) IGNORE_LOGS(_check_time_compare(__VA_ARGS__, __LINE__, true))
#define EXPECT_TIME_NE(...) IGNORE_LOGS(_check_time_compare(__VA_ARGS__, __LINE__, false))

TEST(TimeTests, ComparisonOperators) {
	// binary equality
	EXPECT_TIME_EQ(to_type_timestamp(), to_type_timestamp());
	EXPECT_TIME_EQ(to_type_timestamp(1, 0), to_type_timestamp(1, 0));
	EXPECT_TIME_EQ(to_type_timestamp(-1, 0), to_type_timestamp(-1, 0));
	EXPECT_TIME_EQ(to_type_timestamp(0, 1), to_type_timestamp(0, 1));

	// compatible (rectified) inequality
	EXPECT_TIME_NE(to_type_timestamp(), to_type_timestamp(1, 0));
	EXPECT_TIME_NE(to_type_timestamp(), to_type_timestamp(0, 1));
	EXPECT_TIME_NE(to_type_timestamp(0, -1), to_type_timestamp());
	EXPECT_TIME_NE(to_type_timestamp(-1, 0), to_type_timestamp());
	EXPECT_TIME_NE(to_type_timestamp(-1, -1), to_type_timestamp(-1, 0));

	// overflow equality
	EXPECT_TIME_EQ(to_type_timestamp(), to_type_timestamp(-1, NANO_PER_SEC));
	EXPECT_TIME_EQ(to_type_timestamp(), to_type_timestamp(1, -NANO_PER_SEC));
	EXPECT_TIME_EQ(to_type_timestamp(2, 0), to_type_timestamp(0, 2 * NANO_PER_SEC));
	EXPECT_TIME_EQ(to_type_timestamp(-2, 0), to_type_timestamp(0, -2 * NANO_PER_SEC));

	// overflow inequality
	EXPECT_TIME_NE(to_type_timestamp(), to_type_timestamp(-1, 1 + NANO_PER_SEC));
	EXPECT_TIME_NE(to_type_timestamp(), to_type_timestamp(1, 1 + -NANO_PER_SEC));
	EXPECT_TIME_NE(to_type_timestamp(), to_type_timestamp(0, +NANO_PER_SEC));
	EXPECT_TIME_NE(to_type_timestamp(), to_type_timestamp(2, -NANO_PER_SEC));
	EXPECT_TIME_NE(to_type_timestamp(2, 0), to_type_timestamp(0, 1 + 2 * NANO_PER_SEC));
	EXPECT_TIME_NE(to_type_timestamp(-2, 0), to_type_timestamp(0, 1 - 2 * NANO_PER_SEC));
}

TEST(TimeTests, ScalarComparisonOperators) {
	int int_t1    = 1234;
	double dbl_t1 = int_t1;
	// abbreviations: 'r' for rectified, 'h' for high nsec, 'l' for low nsec (wrap arounds)
	aspn_xtensor::TypeTimestamp t1r = to_type_timestamp(int_t1, 0);
	aspn_xtensor::TypeTimestamp t1h = to_type_timestamp(int_t1 - 1, NANO_PER_SEC);
	auto t1l                        = to_type_timestamp(int_t1 + 1, -NANO_PER_SEC);

	// t0 is 1ns before t1
	aspn_xtensor::TypeTimestamp t0r = to_type_timestamp(int_t1 - 1, NANO_PER_SEC - 1);
	aspn_xtensor::TypeTimestamp t0h = to_type_timestamp(int_t1 - 2, 2 * NANO_PER_SEC - 1);
	aspn_xtensor::TypeTimestamp t0l = to_type_timestamp(int_t1, -1);

	// generated the rest of this test with python:
	/*

	scalars = ['int', 'dbl']
	time_representations = ['r', 'h', 'l']
	ops = [('==', '!='), ('>=', '<'), ('<=', '>')]
	times = ['t0', 't1']
	cmp = [(0, 0, 1), # left < right
	       (1, 1, 1), # left = right
	       (0, 1, 0)] # left >
	def expect(cond):
	    return 'EXPECT_TRUE' if cond else 'EXPECT_FALSE'
	def gen_code():
	    for l in (0, 1):
	        for r in (0, 1):
	            for t, yn in zip(cmp[l - r + 1], ops):
	                for flip, op in enumerate(yn):
	                    for ts in time_representations:
	                        for s in scalars:
	                            e = expect(t != flip)
	                            if l:
	                                yield (f'{e}({s}_t{l} {op} t{r}{ts});',)
	                            if r:
	                                yield (f'{e}(t{l}{ts} {op} {s}_t{r});',)
	code = ["\t" + c[-1] for c in sorted(gen_code())]
	print(f'\t// ------- begin codegen ({len(code) + 2} lines)')
	print('\n'.join(code))
	print(f'\t// ------- end codegen')

	*/
	// ------- begin codegen (146 lines)
	EXPECT_FALSE(dbl_t1 != t1h);
	EXPECT_FALSE(dbl_t1 != t1l);
	EXPECT_FALSE(dbl_t1 != t1r);
	EXPECT_FALSE(dbl_t1 < t0h);
	EXPECT_FALSE(dbl_t1 < t0l);
	EXPECT_FALSE(dbl_t1 < t0r);
	EXPECT_FALSE(dbl_t1 < t1h);
	EXPECT_FALSE(dbl_t1 < t1l);
	EXPECT_FALSE(dbl_t1 < t1r);
	EXPECT_FALSE(dbl_t1 <= t0h);
	EXPECT_FALSE(dbl_t1 <= t0l);
	EXPECT_FALSE(dbl_t1 <= t0r);
	EXPECT_FALSE(dbl_t1 == t0h);
	EXPECT_FALSE(dbl_t1 == t0l);
	EXPECT_FALSE(dbl_t1 == t0r);
	EXPECT_FALSE(dbl_t1 > t1h);
	EXPECT_FALSE(dbl_t1 > t1l);
	EXPECT_FALSE(dbl_t1 > t1r);
	EXPECT_FALSE(int_t1 != t1h);
	EXPECT_FALSE(int_t1 != t1l);
	EXPECT_FALSE(int_t1 != t1r);
	EXPECT_FALSE(int_t1 < t0h);
	EXPECT_FALSE(int_t1 < t0l);
	EXPECT_FALSE(int_t1 < t0r);
	EXPECT_FALSE(int_t1 < t1h);
	EXPECT_FALSE(int_t1 < t1l);
	EXPECT_FALSE(int_t1 < t1r);
	EXPECT_FALSE(int_t1 <= t0h);
	EXPECT_FALSE(int_t1 <= t0l);
	EXPECT_FALSE(int_t1 <= t0r);
	EXPECT_FALSE(int_t1 == t0h);
	EXPECT_FALSE(int_t1 == t0l);
	EXPECT_FALSE(int_t1 == t0r);
	EXPECT_FALSE(int_t1 > t1h);
	EXPECT_FALSE(int_t1 > t1l);
	EXPECT_FALSE(int_t1 > t1r);
	EXPECT_FALSE(t0h == dbl_t1);
	EXPECT_FALSE(t0h == int_t1);
	EXPECT_FALSE(t0h > dbl_t1);
	EXPECT_FALSE(t0h > int_t1);
	EXPECT_FALSE(t0h >= dbl_t1);
	EXPECT_FALSE(t0h >= int_t1);
	EXPECT_FALSE(t0l == dbl_t1);
	EXPECT_FALSE(t0l == int_t1);
	EXPECT_FALSE(t0l > dbl_t1);
	EXPECT_FALSE(t0l > int_t1);
	EXPECT_FALSE(t0l >= dbl_t1);
	EXPECT_FALSE(t0l >= int_t1);
	EXPECT_FALSE(t0r == dbl_t1);
	EXPECT_FALSE(t0r == int_t1);
	EXPECT_FALSE(t0r > dbl_t1);
	EXPECT_FALSE(t0r > int_t1);
	EXPECT_FALSE(t0r >= dbl_t1);
	EXPECT_FALSE(t0r >= int_t1);
	EXPECT_FALSE(t1h != dbl_t1);
	EXPECT_FALSE(t1h != int_t1);
	EXPECT_FALSE(t1h < dbl_t1);
	EXPECT_FALSE(t1h < int_t1);
	EXPECT_FALSE(t1h > dbl_t1);
	EXPECT_FALSE(t1h > int_t1);
	EXPECT_FALSE(t1l != dbl_t1);
	EXPECT_FALSE(t1l != int_t1);
	EXPECT_FALSE(t1l < dbl_t1);
	EXPECT_FALSE(t1l < int_t1);
	EXPECT_FALSE(t1l > dbl_t1);
	EXPECT_FALSE(t1l > int_t1);
	EXPECT_FALSE(t1r != dbl_t1);
	EXPECT_FALSE(t1r != int_t1);
	EXPECT_FALSE(t1r < dbl_t1);
	EXPECT_FALSE(t1r < int_t1);
	EXPECT_FALSE(t1r > dbl_t1);
	EXPECT_FALSE(t1r > int_t1);
	EXPECT_TRUE(dbl_t1 != t0h);
	EXPECT_TRUE(dbl_t1 != t0l);
	EXPECT_TRUE(dbl_t1 != t0r);
	EXPECT_TRUE(dbl_t1 <= t1h);
	EXPECT_TRUE(dbl_t1 <= t1l);
	EXPECT_TRUE(dbl_t1 <= t1r);
	EXPECT_TRUE(dbl_t1 == t1h);
	EXPECT_TRUE(dbl_t1 == t1l);
	EXPECT_TRUE(dbl_t1 == t1r);
	EXPECT_TRUE(dbl_t1 > t0h);
	EXPECT_TRUE(dbl_t1 > t0l);
	EXPECT_TRUE(dbl_t1 > t0r);
	EXPECT_TRUE(dbl_t1 >= t0h);
	EXPECT_TRUE(dbl_t1 >= t0l);
	EXPECT_TRUE(dbl_t1 >= t0r);
	EXPECT_TRUE(dbl_t1 >= t1h);
	EXPECT_TRUE(dbl_t1 >= t1l);
	EXPECT_TRUE(dbl_t1 >= t1r);
	EXPECT_TRUE(int_t1 != t0h);
	EXPECT_TRUE(int_t1 != t0l);
	EXPECT_TRUE(int_t1 != t0r);
	EXPECT_TRUE(int_t1 <= t1h);
	EXPECT_TRUE(int_t1 <= t1l);
	EXPECT_TRUE(int_t1 <= t1r);
	EXPECT_TRUE(int_t1 == t1h);
	EXPECT_TRUE(int_t1 == t1l);
	EXPECT_TRUE(int_t1 == t1r);
	EXPECT_TRUE(int_t1 > t0h);
	EXPECT_TRUE(int_t1 > t0l);
	EXPECT_TRUE(int_t1 > t0r);
	EXPECT_TRUE(int_t1 >= t0h);
	EXPECT_TRUE(int_t1 >= t0l);
	EXPECT_TRUE(int_t1 >= t0r);
	EXPECT_TRUE(int_t1 >= t1h);
	EXPECT_TRUE(int_t1 >= t1l);
	EXPECT_TRUE(int_t1 >= t1r);
	EXPECT_TRUE(t0h != dbl_t1);
	EXPECT_TRUE(t0h != int_t1);
	EXPECT_TRUE(t0h < dbl_t1);
	EXPECT_TRUE(t0h < int_t1);
	EXPECT_TRUE(t0h <= dbl_t1);
	EXPECT_TRUE(t0h <= int_t1);
	EXPECT_TRUE(t0l != dbl_t1);
	EXPECT_TRUE(t0l != int_t1);
	EXPECT_TRUE(t0l < dbl_t1);
	EXPECT_TRUE(t0l < int_t1);
	EXPECT_TRUE(t0l <= dbl_t1);
	EXPECT_TRUE(t0l <= int_t1);
	EXPECT_TRUE(t0r != dbl_t1);
	EXPECT_TRUE(t0r != int_t1);
	EXPECT_TRUE(t0r < dbl_t1);
	EXPECT_TRUE(t0r < int_t1);
	EXPECT_TRUE(t0r <= dbl_t1);
	EXPECT_TRUE(t0r <= int_t1);
	EXPECT_TRUE(t1h <= dbl_t1);
	EXPECT_TRUE(t1h <= int_t1);
	EXPECT_TRUE(t1h == dbl_t1);
	EXPECT_TRUE(t1h == int_t1);
	EXPECT_TRUE(t1h >= dbl_t1);
	EXPECT_TRUE(t1h >= int_t1);
	EXPECT_TRUE(t1l <= dbl_t1);
	EXPECT_TRUE(t1l <= int_t1);
	EXPECT_TRUE(t1l == dbl_t1);
	EXPECT_TRUE(t1l == int_t1);
	EXPECT_TRUE(t1l >= dbl_t1);
	EXPECT_TRUE(t1l >= int_t1);
	EXPECT_TRUE(t1r <= dbl_t1);
	EXPECT_TRUE(t1r <= int_t1);
	EXPECT_TRUE(t1r == dbl_t1);
	EXPECT_TRUE(t1r == int_t1);
	EXPECT_TRUE(t1r >= dbl_t1);
	EXPECT_TRUE(t1r >= int_t1);
	// ------- end codegen
}
