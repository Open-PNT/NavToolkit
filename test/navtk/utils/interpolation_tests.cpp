#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/utils/interpolation.hpp>

using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::Matrix;
using navtk::Size;
using navtk::Vector3;
using navtk::Vector4;
using navtk::navutils::PI;
using navtk::navutils::rpy_to_quat;
using navtk::utils::cubic_spline_interpolate;
using navtk::utils::linear_extrapolate_pva;
using navtk::utils::linear_extrapolate_rpy;
using navtk::utils::linear_interp_pva;
using navtk::utils::linear_interp_rpy;
using navtk::utils::linear_interpolate;
using navtk::utils::quadratic_spline_interpolate;
using std::pair;
using std::vector;

namespace {
void assert_eq(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva1,
               const aspn_xtensor::MeasurementPositionVelocityAttitude& pva2) {
	ASSERT_EQ(pva1.get_time_of_validity().get_elapsed_nsec(),
	          pva2.get_time_of_validity().get_elapsed_nsec());
	ASSERT_FLOAT_EQ(pva1.get_p1(), pva2.get_p1());
	ASSERT_FLOAT_EQ(pva1.get_p2(), pva2.get_p2());
	ASSERT_FLOAT_EQ(pva1.get_p3(), pva2.get_p3());
	ASSERT_FLOAT_EQ(pva1.get_v1(), pva2.get_v1());
	ASSERT_FLOAT_EQ(pva1.get_v2(), pva2.get_v2());
	ASSERT_FLOAT_EQ(pva1.get_v3(), pva2.get_v3());
	ASSERT_ALLCLOSE(Vector4(pva1.get_quaternion()), Vector4(pva2.get_quaternion()));
}
void assert_eq(std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> pva1,
               std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> pva2) {
	ASSERT_EQ(pva1->get_time_of_validity().get_elapsed_nsec(),
	          pva2->get_time_of_validity().get_elapsed_nsec());
	ASSERT_FLOAT_EQ(pva1->get_p1(), pva2->get_p1());
	ASSERT_FLOAT_EQ(pva1->get_p2(), pva2->get_p2());
	ASSERT_FLOAT_EQ(pva1->get_p3(), pva2->get_p3());
	ASSERT_FLOAT_EQ(pva1->get_v1(), pva2->get_v1());
	ASSERT_FLOAT_EQ(pva1->get_v2(), pva2->get_v2());
	ASSERT_FLOAT_EQ(pva1->get_v3(), pva2->get_v3());
	ASSERT_ALLCLOSE(Vector4(pva1->get_quaternion()), Vector4(pva2->get_quaternion()));
}
}  // namespace

struct InterpolationTests : public ::testing::Test {

	InterpolationTests() : ::testing::Test() {}

	using interp_sig = std::function<pair<vector<Size>, vector<double>>(
	    const vector<double>&, const vector<double>&, const vector<double>&)>;

	// linear_interpolate overloaded, need to cast to pass into test_interp
	interp_sig linear_fun_ptr = static_cast<pair<vector<Size>, vector<double>> (*)(
	    const vector<double>&, const vector<double>&, const vector<double>&)>(&linear_interpolate);

	const double thresh = 1e-7;

	vector<double> lin_t = {1000.0, 1001.0, 1002.0, 1003.0, 1004.0, 1005.0};
	vector<double> lin_d = {5.0, 4.0, 3.0, 2.0, 1.0, 0.0};

	vector<double> quat_t = {0.0, 5.0, 10.0, 15.0, 20.0};
	vector<double> quat_d = {5.0, 15.0, 10.0, 7.5, 10.0};

	vector<double> cub_t = {0.0, 3.0, 6.0, 9.0, 12.0};
	vector<double> cub_d = {0.0, 27.0, 216.0, 729.0, 1728.0};

	TypeTimestamp t0 = to_type_timestamp();
	TypeTimestamp t1 = to_type_timestamp(1, 0);

	aspn_xtensor::MeasurementPositionVelocityAttitude pva0 =
	    create_pva(t0, navtk::zeros(3), navtk::zeros(3), navtk::zeros(3), navtk::zeros(9, 9));

	aspn_xtensor::MeasurementPositionVelocityAttitude pva1 =
	    create_pva(t1, navtk::zeros(3), navtk::zeros(3), navtk::zeros(3), navtk::zeros(9, 9));

	void test_interp(vector<double> source_time,
	                 vector<double> source_data,
	                 vector<double> interp_time,
	                 vector<double> expected,
	                 interp_sig fun) {

		auto val         = fun(source_time, source_data, interp_time);
		auto interp_data = val.second;
		ASSERT_TRUE(expected.size() == interp_data.size());
		for (Size k = 0; k < expected.size(); k++) {
			EXPECT_NEAR(interp_data[k], expected[k], thresh) << "where k=" << k;
		}
	}

	void test_endpoints(interp_sig fun) {
		test_interp(lin_t,
		            lin_d,
		            {*lin_t.begin(), *(lin_t.end() - 1)},
		            {*lin_d.begin(), *(lin_d.end() - 1)},
		            fun);
	}

	void test_no_extrap(interp_sig fun) {
		EXPECT_WARN(
		    test_interp(lin_t, lin_d, {*lin_t.begin() - 1.0, *(lin_t.end() - 1) + 1.0}, {}, fun),
		    "bounds");
	}

	void test_bad_size(interp_sig fun) {
		// source_time is 1 less than source_data
		EXPECT_HONORS_MODE(fun({1.0, 2.0, 3.0}, {1.0, 2.0, 3.0, 4.0}, {1.0}),
		                   "Exception Occurred: Source time and source data are not matching "
		                   "lengths for interpolation.");
		EXPECT_UB_OR_DIE(fun({1.0}, {1.0}, {1.0}),
		                 "Exception Occurred: Trying to interpolate with source data that is "
		                 "smaller that 2 data points.");
	}

	void test_rpy(const navtk::Vector3& mid_rpy, const navtk::Vector3& delta_rpy) {
		navtk::Matrix mid_dcm = xt::transpose(navtk::navutils::rpy_to_dcm(mid_rpy));
		auto tx               = xt::transpose(navtk::navutils::rpy_to_dcm(delta_rpy));
		auto start_dcm        = navtk::dot(xt::transpose(tx), mid_dcm);
		auto start_rpy        = navtk::navutils::dcm_to_rpy(xt::transpose(start_dcm));
		auto stop_dcm         = navtk::dot(tx, mid_dcm);
		auto stop_rpy         = navtk::navutils::dcm_to_rpy(xt::transpose(stop_dcm));
		auto calc_mid         = linear_interp_rpy(aspn_xtensor::TypeTimestamp((int64_t)0),
                                          start_rpy,
                                          to_type_timestamp(1.0),
                                          stop_rpy,
                                          to_type_timestamp(0.5));
		auto new_mid          = xt::transpose(navtk::navutils::rpy_to_dcm(calc_mid));
		ASSERT_ALLCLOSE(mid_dcm, new_mid);
	}

	MeasurementPositionVelocityAttitude create_pva(const TypeTimestamp& time,
	                                               const Vector3& pos,
	                                               const Vector3& vel,
	                                               const Vector3& rpy,
	                                               const Matrix& cov) {
		TypeHeader header(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0);
		return MeasurementPositionVelocityAttitude(
		    header,
		    time,
		    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
		    pos(0),
		    pos(1),
		    pos(2),
		    vel(0),
		    vel(1),
		    vel(2),
		    rpy_to_quat(rpy),
		    cov,
		    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
		    {},
		    {});
	}
};

// Linear Interpolation Tests
TEST_F(InterpolationTests, LinearInterpolate) {
	vector<double> interp_time = {1000.5, 1001.5, 1002.5, 1003.5, 1004.5};
	vector<double> expected    = {4.5, 3.5, 2.5, 1.5, 0.5};

	test_interp(lin_t, lin_d, interp_time, expected, linear_fun_ptr);
}

TEST_F(InterpolationTests, LinearInterpolateDuplicates) {
	vector<double> interp_time = {1000.5, 1001.5, 1001.5, 1001.5, 1002.5, 1003.5, 1004.5};
	vector<double> expected    = {4.5, 3.5, 2.5, 1.5, 0.5};

	EXPECT_WARN(test_interp(lin_t, lin_d, interp_time, expected, linear_fun_ptr),
	            "duplicate time tag");

	vector<double> source_time = {1000.0, 1001.0, 1001.0, 1002.0, 1003.0, 1004.0, 1005.0};
	vector<double> source_data = {5.0, 4.0, 4.0, 3.0, 2.0, 1.0, 0.0};
	interp_time                = {1000.5, 1001.5, 1002.5, 1003.5, 1004.5};

	test_interp(source_time, source_data, interp_time, expected, linear_fun_ptr);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, InterpolationTests, LinearInterpolateSourceLength) {
	test.test_bad_size(test.linear_fun_ptr);
}

TEST_F(InterpolationTests, LinearInterpolateUnusedData) {
	vector<double> interp_time = {900.0, 910.0, 1000.5, 1006.5, 1007.5, 1008.5};
	vector<double> expected    = {4.5};
	test_interp(lin_t, lin_d, interp_time, expected, linear_fun_ptr);

	interp_time = {998.5, 999.5, 1006.5, 1007.5, 1008.5};
	auto val    = EXPECT_WARN(linear_interpolate(lin_t, lin_d, interp_time), "bounds");
	ASSERT_TRUE(val.second.empty());
}

TEST_F(InterpolationTests, LinearInterpolateOutOfOrderData) {
	// out of order time source data
	vector<double> source_time = {1001.0, 1002.0, 1000.0, 1005.0, 1003.0, 1004.0};
	vector<double> source_data = {4.0, 3.0, 5.0, 0.0, 2.0, 1.0};
	vector<double> interp_time = {1000.5, 1001.5, 1002.5, 1003.5, 1004.5};
	vector<double> expected    = {4.5, 3.5, 2.5, 1.5, 0.5};

	test_interp(source_time, source_data, interp_time, expected, linear_fun_ptr);

	// out of order interp time tags
	interp_time = {1001.5, 1000.5, 1004.5, 1002.5, 1003.5};

	test_interp(lin_t, lin_d, interp_time, expected, linear_fun_ptr);
}

TEST_F(InterpolationTests, LinearInterpolateSharedTimeTags) {
	vector<double> interp_time = {100.5, 101.5, 102.5, 103.5, 104.5};

	auto res = EXPECT_WARN(linear_interpolate(lin_t, lin_d, interp_time), "bounds");
	vector<Size> exp_ind_rejected(interp_time.size());
	std::iota(exp_ind_rejected.begin(), exp_ind_rejected.end(), 0);
	ASSERT_TRUE(res.first == exp_ind_rejected);
	ASSERT_TRUE(res.second.empty());
}

// Quadratic Spline Interpolation Tests
TEST_F(InterpolationTests, QuadSplineInterpolate) {
	vector<double> interp_time = {2.5, 7.5, 12.5, 17.5};
	vector<double> expected    = {10.0, 16.25, 4.375, 11.875};

	test_interp(quat_t, quat_d, interp_time, expected, quadratic_spline_interpolate);
}

TEST_F(InterpolationTests, QuadSplineInterpolateDuplicates) {
	vector<double> interp_time = {2.5, 7.5, 12.5, 17.5};
	vector<double> expected    = {10.0, 16.25, 4.375, 11.875};

	test_interp(quat_t, quat_d, interp_time, expected, quadratic_spline_interpolate);

	interp_time = {2.5, 7.5, 12.5, 12.5, 17.5};

	EXPECT_WARN(test_interp(quat_t, quat_d, interp_time, expected, quadratic_spline_interpolate),
	            "duplicate");
}

TEST_F(InterpolationTests, QuadSplineInterpolateBigJump) {
	vector<double> interp_time = {2.5, 7.5, 17.5};
	vector<double> expected    = {10.0, 16.25, 11.875};

	test_interp(quat_t, quat_d, interp_time, expected, quadratic_spline_interpolate);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, InterpolationTests, QuadSplineInterpolateSourceLength) {
	test.test_bad_size(quadratic_spline_interpolate);
}

TEST_F(InterpolationTests, QuadSplineInterpolateUnusedData) {
	vector<double> interp_time = {-5.0, -2.5, 2.5, 7.5, 12.5, 17.5, 22.5, 27.5};
	vector<double> expected    = {10.0, 16.25, 4.375, 11.875};

	test_interp(quat_t, quat_d, interp_time, expected, quadratic_spline_interpolate);

	interp_time = {-5.0, -2.5, 22.5, 27.5};
	auto val    = EXPECT_WARN(quadratic_spline_interpolate(quat_t, quat_d, interp_time), "bounds");
	EXPECT_EQ(val.second.size(), 0);
}

TEST_F(InterpolationTests, QuadSplineInterpolateOutOfOrderData) {
	vector<double> interp_time = {2.5, 7.5, 12.5, 17.5};
	vector<double> expected    = {10.0, 16.25, 4.375, 11.875};

	test_interp(quat_t, quat_d, interp_time, expected, quadratic_spline_interpolate);

	// out of order interp time tags
	interp_time = {12.5, 2.5, 7.5, 17.5};

	test_interp(quat_t, quat_d, interp_time, expected, quadratic_spline_interpolate);
}

TEST_F(InterpolationTests, QuadSplineSharedTimeTags) {
	vector<double> interp_time = {-2.5, -7.5, -12.5, -17.5};

	auto res = EXPECT_WARN(quadratic_spline_interpolate(quat_t, quat_d, interp_time), "bounds");
	vector<Size> exp_ind_rejected(interp_time.size());
	std::iota(exp_ind_rejected.begin(), exp_ind_rejected.end(), 0);
	ASSERT_TRUE(res.first == exp_ind_rejected);
	ASSERT_TRUE(res.second.empty());
}

// Cubic Spline Interpolation Tests
TEST_F(InterpolationTests, CubicSplineInterpolate) {
	vector<double> interp_time = {2.0, 4.0, 7.5, 9.1, 10.2};
	vector<double> expected    = {7.2857142857142883,
	                              65.571428571428584,
	                              413.91964285714283,
	                              755.33378571428557,
	                              1086.3874285714285};

	test_interp(cub_t, cub_d, interp_time, expected, cubic_spline_interpolate);
}

TEST_F(InterpolationTests, CubicSplineInterpolateLong) {
	vector<double> source_time = {
	    0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
	vector<double> source_data{
	    0.0, 1.0, 4.0, 9.0, 16.0, 25.0, 36.0, 49.0, 64.0, 81.0, 100.0, 121.0, 144.0};
	vector<double> interp_time = {
	    0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5, 10.5, 11.5, 12.0};
	vector<double> expected{0.34150629,
	                        2.22548113,
	                        6.25656921,
	                        12.24824204,
	                        20.25046262,
	                        30.24990748,
	                        42.24990748,
	                        56.25046262,
	                        72.24824204,
	                        90.25656921,
	                        110.22548113,
	                        132.34150629,
	                        144.};

	test_interp(source_time, source_data, interp_time, expected, cubic_spline_interpolate);
}

TEST_F(InterpolationTests, CubicSpaced) {
	vector<double> source_time = {0.0, 1.0, 3.0, 4.4, 5.0, 7.2};
	vector<double> source_data{1.3, 4.4, 5.0, 7.0, 8.0, 9.0};
	vector<double> interp_time = {2.0, 3.6, 4.7};
	vector<double> expected{5.057338013669601, 5.618149858925487, 7.532643599261868};

	test_interp(source_time, source_data, interp_time, expected, cubic_spline_interpolate);
}

TEST_F(InterpolationTests, CubicSplineInterpolateDuplicates) {
	vector<double> source_time = {0.0, 3.0, 3.0, 6.0, 9.0, 12.0};
	vector<double> source_data = {0.0, 27.0, 27.0, 216.0, 729.0, 1728.0};
	vector<double> interp_time = {2.0, 4.0, 7.5, 9.1, 10.2};
	vector<double> expected    = {7.2857142857142883,
	                              65.571428571428584,
	                              413.91964285714283,
	                              755.33378571428557,
	                              1086.3874285714285};

	test_interp(source_time, source_data, interp_time, expected, cubic_spline_interpolate);

	interp_time = {2.0, 4.0, 4.0, 7.5, 9.1, 10.2};

	EXPECT_WARN(test_interp(cub_t, cub_d, interp_time, expected, cubic_spline_interpolate),
	            "duplicate.*\\[2\\]");
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, InterpolationTests, CubicSplineInterpolateSourceLength) {
	test.test_bad_size(cubic_spline_interpolate);
}

TEST_F(InterpolationTests, CubicSplineInterpolateUnusedData) {
	vector<double> interp_time = {-5.0, -2.0, 2.0, 4.0, 7.5, 9.1, 10.2, 13.0, 15.0};
	vector<double> expected    = {7.2857142857142883,
	                              65.571428571428584,
	                              413.91964285714283,
	                              755.33378571428557,
	                              1086.3874285714285};

	test_interp(cub_t, cub_d, interp_time, expected, cubic_spline_interpolate);

	interp_time = {-5.0, -2.0, 13.0, 15.0};
	auto val    = EXPECT_WARN(cubic_spline_interpolate(cub_t, cub_d, interp_time), "bounds");
	EXPECT_EQ(val.second.size(), 0);
}

TEST_F(InterpolationTests, CubicSplineInterpolateOutOfOrderData) {
	// out of order time source data
	vector<double> source_time = {3.0, 0.0, 9.0, 12.0, 6.0};
	vector<double> source_data = {27.0, 0.0, 729.0, 1728.0, 216.0};
	vector<double> interp_time = {2.0, 4.0, 7.5, 9.1, 10.2};
	vector<double> expected    = {7.2857142857142883,
	                              65.571428571428584,
	                              413.91964285714283,
	                              755.33378571428557,
	                              1086.3874285714285};

	test_interp(source_time, source_data, interp_time, expected, cubic_spline_interpolate);

	// out of order interp time tags
	interp_time = {9.1, 2.0, 7.5, 4.0, 10.2};
	test_interp(cub_t, cub_d, interp_time, expected, cubic_spline_interpolate);
}

TEST_F(InterpolationTests, CubicSplineInterpolateSharedTimeTags) {
	vector<double> interp_time = {-2.0, -4.0, -7.5, -9.1, -10.2};

	auto res = EXPECT_WARN(cubic_spline_interpolate(cub_t, cub_d, interp_time), "bounds");
	vector<Size> exp_ind_rejected(interp_time.size());
	std::iota(exp_ind_rejected.begin(), exp_ind_rejected.end(), 0);
	ASSERT_TRUE(res.first == exp_ind_rejected);
	ASSERT_TRUE(res.second.empty());
}

TEST_F(InterpolationTests, CubicSplineInterpolate3SourcePoints) {
	vector<double> interp_time = {2.0, 4.0, 5.0};
	vector<double> expected    = {18, 90, 153};

	EXPECT_WARN(test_interp({cub_t.begin(), cub_t.begin() + 3},
	                        {cub_d.begin(), cub_d.begin() + 3},
	                        interp_time,
	                        expected,
	                        cubic_spline_interpolate),
	            "at least 4");
}

TEST_F(InterpolationTests, CubicSplineInterpolate4SourcePoints) {
	vector<double> interp_time = {2.0, 4.0, 5.0};
	vector<double> expected    = {10.0, 59.6, 118.6};

	test_interp({cub_t.begin(), cub_t.begin() + 4},
	            {cub_d.begin(), cub_d.begin() + 4},
	            interp_time,
	            expected,
	            cubic_spline_interpolate);
}

double eval_cubic(double dt) {
	double a = 3.0;
	double b = -2.0;
	double c = 1.2;
	double d = 0.7;
	return a + b * dt + c * dt * dt + d * dt * dt * dt;
}

TEST_F(InterpolationTests, CubicSplineInterpolateNotChunked) {
	vector<double> source_time = {0.0, 3.0, 6.0, 9.0, 12.0, 15.0};
	vector<double> source_data;
	for (auto k = source_time.cbegin(); k < source_time.cend(); k++) {
		source_data.push_back(eval_cubic(*k));
	}
	vector<double> interp_time = {1.0, 3.0, 4.5, 5.9, 6.0, 7.8, 9.0, 11.5, 12.0, 13.0, 15.0};

	// Taken from scipy.interpolate.CubicSpline(source_time, source_data, bc_type="natural")
	// evaluated at interp_time
	vector<double> exp_data{4.08660287,
	                        26.7,
	                        81.29461722,
	                        176.61820574,
	                        185.4,
	                        394.72752919,
	                        592.5,
	                        1197.75358852,
	                        1361.4,
	                        1737.03779904,
	                        2605.5};

	auto cspline = cubic_spline_interpolate(source_time, source_data, interp_time);

	for (Size k = 0; k < exp_data.size(); k++) {
		ASSERT_TRUE(std::abs(exp_data[k] - cspline.second[k]) < 1e-6);
	}
}

TEST(Interpolation, LinearInterpolate) {
	vector<pair<double, double>> in_and_out = {{10.1, 8.3},
	                                           {10.2, 9.1},
	                                           {10.3, 9.9},
	                                           {10.4, 10.7},
	                                           {10.5, 11.5},
	                                           {10.6, 12.3},
	                                           {10.7, 13.1},
	                                           {10.8, 13.9},
	                                           {10.9, 14.7}};
	// Test Time/double interpolation
	std::for_each(in_and_out.begin(), in_and_out.end(), [](pair<double, double> d) {
		EXPECT_DOUBLE_EQ(linear_interpolate(10.0, 7.5, 11.0, 15.5, d.first), d.second);
		EXPECT_DOUBLE_EQ(linear_interpolate(to_type_timestamp(10.0),
		                                    7.5,
		                                    to_type_timestamp(11.0),
		                                    15.5,
		                                    to_type_timestamp(d.first)),
		                 d.second);
		EXPECT_DOUBLE_EQ(linear_interpolate(to_type_timestamp(10.0),
		                                    7.5,
		                                    to_type_timestamp(11.0),
		                                    15.5,
		                                    to_type_timestamp(d.first)),
		                 d.second);
	});
	// Test integers for x
	EXPECT_DOUBLE_EQ(linear_interpolate(10, 7.5, 12, 15.5, 11),
	                 linear_interpolate(10.0, 7.5, 12.0, 15.5, 11.0));
}

TEST(Interpolation, condition_source_data) {
	vector<double> time_source{1.0, 1.0, 3.0, 2.0, 4.0, 5.0, 4.0, 6.0};
	vector<double> data_source{1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8};
	vector<double> time_interp{0.5, 1.5, 2.5, 2.5, 3.0, 2.5, 7.0, 4.5};

	vector<double> fixed_time_source{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
	vector<double> fixed_data_source{1.1, 4.4, 3.3, 5.5, 6.6, 8.8};
	vector<double> expected_time_interp{1.5, 2.5, 3.0, 4.5};
	vector<Size> expected_ignored{0, 3, 4, 7};

	auto ignored = EXPECT_WARN(
	    navtk::utils::condition_source_data(time_source, data_source, time_interp), "duplicate");

	ASSERT_TRUE(time_source == fixed_time_source);
	ASSERT_TRUE(data_source == fixed_data_source);
	ASSERT_TRUE(time_interp == expected_time_interp);
	ASSERT_TRUE(ignored == expected_ignored);
}

TEST_F(InterpolationTests, LinearEndpoint) { test_endpoints(linear_fun_ptr); }

TEST_F(InterpolationTests, QuadEndpoint) { test_endpoints(quadratic_spline_interpolate); }

TEST_F(InterpolationTests, CubicEndpoint) { test_endpoints(cubic_spline_interpolate); }

TEST_F(InterpolationTests, LinearExtrap) { test_no_extrap(linear_fun_ptr); }

TEST_F(InterpolationTests, QuadExtrap) { test_no_extrap(quadratic_spline_interpolate); }

TEST_F(InterpolationTests, CubicExtrap) { test_no_extrap(cubic_spline_interpolate); }

TEST_F(InterpolationTests, LinearInterpolateRpy) {
	test_rpy(Vector3{PI / 2, 0.0, 0.0}, Vector3{0.1, 0.0, 0.0});
	test_rpy(Vector3{-PI / 2, 0.0, 0.0}, Vector3{0.1, 0.0, 0.0});
	test_rpy(Vector3{PI / 2, PI / 2, 0.0}, Vector3{0.1, 0.0, 0.0});
	test_rpy(Vector3{-PI / 2, -PI / 2, 0.0}, Vector3{0.1, 0.0, 0.0});
	test_rpy(Vector3{PI / 2, -PI / 2, 0.0}, Vector3{0.1, 0.0, 0.0});
	test_rpy(Vector3{-PI / 2, PI / 2, 0.0}, Vector3{0.1, 0.0, 0.0});
	test_rpy(Vector3{0.0, PI / 2, 0.0}, Vector3{0.1, 0.0, 0.0});
	test_rpy(Vector3{0.0, -PI / 2, 0.0}, Vector3{0.1, 0.0, 0.0});
}

TEST_F(InterpolationTests, LinearInterpolateRpyOutsideRange) {
	auto rpy1 = navtk::zeros(3);
	auto rpy2 = navtk::ones(3);
	auto rpy  = EXPECT_WARN(linear_interp_rpy(aspn_xtensor::TypeTimestamp((int64_t)0),
                                             navtk::zeros(3),
                                             to_type_timestamp(1.0),
                                             navtk::ones(3),
                                             to_type_timestamp(1.5)),
                           "after latest data point");
	ASSERT_ALLCLOSE(rpy, rpy2);
}

TEST_F(InterpolationTests, LinearExtrapolateRpyOutsideRange) {
	auto rpy1 = navtk::zeros(3);
	auto rpy2 = navtk::ones(3) * 0.01;

	auto rpy = EXPECT_NO_LOG(linear_extrapolate_rpy(aspn_xtensor::TypeTimestamp((int64_t)0),
	                                                rpy1,
	                                                to_type_timestamp(1.0),
	                                                rpy2,
	                                                to_type_timestamp(1.5)));
	ASSERT_ALLCLOSE_EX(rpy, rpy2 * 1.5, 0.05, 0.01);
}

TEST_F(InterpolationTests, InterpolateRpyVariety) {
	auto rpy0 = navtk::zeros(3);
	auto rpy1 = navtk::ones(3);

	// Equal or very near to endpoints
	ASSERT_ALLCLOSE(rpy0, linear_interp_rpy(t0, rpy0, t1, rpy1, t0));
	ASSERT_ALLCLOSE(rpy1, linear_interp_rpy(t0, rpy0, t1, rpy1, t1));
	ASSERT_ALLCLOSE(
	    rpy0, linear_interp_rpy(t0, rpy0, t1, rpy1, to_type_timestamp(std::nextafter(0.0, 1.0))));
	ASSERT_ALLCLOSE(
	    rpy0, linear_interp_rpy(t0, rpy0, t1, rpy1, to_type_timestamp(std::nextafter(0.0, -1.0))));
	ASSERT_ALLCLOSE(
	    rpy1, linear_interp_rpy(t0, rpy0, t1, rpy1, to_type_timestamp(std::nextafter(1.0, 2.0))));
	ASSERT_ALLCLOSE(
	    rpy1, linear_interp_rpy(t0, rpy0, t1, rpy1, to_type_timestamp(std::nextafter(1.0, -1.0))));

	// Switched order
	ASSERT_ALLCLOSE(rpy1, linear_interp_rpy(t1, rpy0, t0, rpy1, t0));
	ASSERT_ALLCLOSE(rpy0, linear_interp_rpy(t1, rpy0, t0, rpy1, t1));
	ASSERT_ALLCLOSE(linear_interp_rpy(t0, rpy0, t1, rpy1, to_type_timestamp(0.7)),
	                linear_interp_rpy(t1, rpy1, t0, rpy0, to_type_timestamp(0.7)));

	// Extrap
	ASSERT_ALLCLOSE(
	    rpy0,
	    EXPECT_WARN(linear_interp_rpy(t0, rpy0, t1, rpy1, to_type_timestamp(-1.0)), "earliest"));
	ASSERT_ALLCLOSE(
	    rpy1, EXPECT_WARN(linear_interp_rpy(t0, rpy0, t1, rpy1, to_type_timestamp(2.0)), "latest"));
	EXPECT_WARN(linear_interp_rpy(t0, rpy0, t1, rpy1, to_type_timestamp(-1.0)), "earliest");
	EXPECT_WARN(linear_interp_rpy(t0, rpy0, t1, rpy1, to_type_timestamp(2.0)), "latest");

	// Identical angles
	ASSERT_ALLCLOSE(rpy1, linear_interp_rpy(t0, rpy1, t1, rpy1, t0));
	ASSERT_ALLCLOSE(rpy1, linear_interp_rpy(t0, rpy1, t1, rpy1, t1));
	ASSERT_ALLCLOSE(rpy1, linear_interp_rpy(t0, rpy1, t1, rpy1, to_type_timestamp(0.5)));
	ASSERT_ALLCLOSE(
	    rpy1,
	    EXPECT_WARN(linear_interp_rpy(t0, rpy1, t1, rpy1, to_type_timestamp(-1.0)), "earliest"));
	ASSERT_ALLCLOSE(
	    rpy1, EXPECT_WARN(linear_interp_rpy(t0, rpy1, t1, rpy1, to_type_timestamp(2.0)), "latest"));
}


TEST_F(InterpolationTests, InterpolatePvaVariety) {
	// Equal or very near to endpoints
	assert_eq(pva0, linear_interp_pva(pva0, pva1, t0));
	assert_eq(pva1, linear_interp_pva(pva0, pva1, t1));
	assert_eq(pva0, linear_interp_pva(pva0, pva1, to_type_timestamp(std::nextafter(0.0, 1.0))));
	assert_eq(pva0, linear_interp_pva(pva0, pva1, to_type_timestamp(std::nextafter(0.0, -1.0))));
	assert_eq(pva1, linear_interp_pva(pva0, pva1, to_type_timestamp(std::nextafter(1.0, 2.0))));
	assert_eq(pva1, linear_interp_pva(pva0, pva1, to_type_timestamp(std::nextafter(1.0, -1.0))));

	// Switched order
	assert_eq(pva0, linear_interp_pva(pva1, pva0, t0));
	assert_eq(pva1, linear_interp_pva(pva1, pva0, t1));
	assert_eq(linear_interp_pva(pva0, pva1, to_type_timestamp(0.7)),
	          linear_interp_pva(pva1, pva0, to_type_timestamp(0.7)));

	// Extrap
	EXPECT_WARN(assert_eq(pva0, linear_interp_pva(pva0, pva1, to_type_timestamp(-1.0))),
	            "earliest pva");
	EXPECT_WARN(assert_eq(pva1, linear_interp_pva(pva0, pva1, to_type_timestamp(2.0))),
	            "latest pva");
	EXPECT_WARN(linear_interp_pva(pva0, pva1, to_type_timestamp(-1.0)), "earliest pva");
	EXPECT_WARN(linear_interp_pva(pva0, pva1, to_type_timestamp(2.0)), "latest pva");

	// Identical angles
	assert_eq(pva1, EXPECT_WARN(linear_interp_pva(pva1, pva1, t0), "earliest"));
	assert_eq(pva1, linear_interp_pva(pva1, pva1, t1));
	EXPECT_WARN(assert_eq(pva1, linear_interp_pva(pva1, pva1, to_type_timestamp(0.5))),
	            "earliest pva");
	EXPECT_WARN(assert_eq(pva1, linear_interp_pva(pva1, pva1, to_type_timestamp(-1.0))),
	            "earliest pva");
	EXPECT_WARN(assert_eq(pva1, linear_interp_pva(pva1, pva1, to_type_timestamp(2.0))),
	            "latest pva");
}


TEST_F(InterpolationTests, InterpolatePvaVarietyPtr) {
	// Equal or very near to endpoints
	auto pva0_ptr = std::make_shared<MeasurementPositionVelocityAttitude>(pva0);
	auto pva1_ptr = std::make_shared<MeasurementPositionVelocityAttitude>(pva1);
	assert_eq(pva0_ptr, linear_interp_pva(pva0_ptr, pva1_ptr, t0));
	assert_eq(pva1_ptr, linear_interp_pva(pva0_ptr, pva1_ptr, t1));
	assert_eq(pva0_ptr,
	          linear_interp_pva(pva0_ptr, pva1_ptr, to_type_timestamp(std::nextafter(0.0, 1.0))));
	assert_eq(pva0_ptr,
	          linear_interp_pva(pva0_ptr, pva1_ptr, to_type_timestamp(std::nextafter(0.0, -1.0))));
	assert_eq(pva1_ptr,
	          linear_interp_pva(pva0_ptr, pva1_ptr, to_type_timestamp(std::nextafter(1.0, 2.0))));
	assert_eq(pva1_ptr,
	          linear_interp_pva(pva0_ptr, pva1_ptr, to_type_timestamp(std::nextafter(1.0, -1.0))));

	// Switched order
	assert_eq(pva0_ptr, linear_interp_pva(pva1_ptr, pva0_ptr, t0));
	assert_eq(pva1_ptr, linear_interp_pva(pva1_ptr, pva0_ptr, t1));
	assert_eq(linear_interp_pva(pva0_ptr, pva1_ptr, to_type_timestamp(0.7)),
	          linear_interp_pva(pva1_ptr, pva0_ptr, to_type_timestamp(0.7)));

	// Extrap
	EXPECT_WARN(assert_eq(pva0_ptr, linear_interp_pva(pva0_ptr, pva1_ptr, to_type_timestamp(-1.0))),
	            "earliest pva");
	EXPECT_WARN(assert_eq(pva1_ptr, linear_interp_pva(pva0_ptr, pva1_ptr, to_type_timestamp(2.0))),
	            "latest pva");
	EXPECT_WARN(linear_interp_pva(pva0_ptr, pva1_ptr, to_type_timestamp(-1.0)), "earliest pva");
	EXPECT_WARN(linear_interp_pva(pva0_ptr, pva1_ptr, to_type_timestamp(2.0)), "latest pva");

	// Identical angles
	assert_eq(pva1_ptr, EXPECT_WARN(linear_interp_pva(pva1_ptr, pva1_ptr, t0), "earliest"));
	assert_eq(pva1_ptr, linear_interp_pva(pva1_ptr, pva1_ptr, t1));
	EXPECT_WARN(assert_eq(pva1_ptr, linear_interp_pva(pva1_ptr, pva1_ptr, to_type_timestamp(0.5))),
	            "earliest pva");
	EXPECT_WARN(assert_eq(pva1_ptr, linear_interp_pva(pva1_ptr, pva1_ptr, to_type_timestamp(-1.0))),
	            "earliest pva");
	EXPECT_WARN(assert_eq(pva1_ptr, linear_interp_pva(pva1_ptr, pva1_ptr, to_type_timestamp(2.0))),
	            "latest pva");
}

TEST_F(InterpolationTests, LinearInterpolatePvaOutsideRange) {

	auto pva = EXPECT_WARN(linear_interp_pva(pva0, pva1, to_type_timestamp(1.5)),
	                       "after latest pva point");
	ASSERT_ALLCLOSE(Vector4(pva.get_quaternion()), Vector4(pva1.get_quaternion()));
	ASSERT_DOUBLE_EQ(pva.get_p1(), pva1.get_p1());
	ASSERT_DOUBLE_EQ(pva.get_p2(), pva1.get_p2());
	ASSERT_DOUBLE_EQ(pva.get_p3(), pva1.get_p3());
	ASSERT_DOUBLE_EQ(pva.get_v1(), pva1.get_v1());
	ASSERT_DOUBLE_EQ(pva.get_v2(), pva1.get_v2());
	ASSERT_DOUBLE_EQ(pva.get_v3(), pva1.get_v3());
	ASSERT_EQ(pva.get_time_of_validity().get_elapsed_nsec(),
	          pva1.get_time_of_validity().get_elapsed_nsec());
}

TEST_F(InterpolationTests, LinearExtrapolatePvaOutsideRange) {
	auto time = to_type_timestamp(1.5);
	auto pva  = EXPECT_NO_LOG(linear_extrapolate_pva(pva0, pva1, time));
	ASSERT_EQ(pva.get_time_of_validity(), time);
}

TEST_F(InterpolationTests, LinearExtrapolatePvaOutsideRangePtr) {
	auto time = to_type_timestamp(1.5);
	auto pva  = EXPECT_NO_LOG(
        linear_extrapolate_pva(std::make_shared<MeasurementPositionVelocityAttitude>(pva0),
                               std::make_shared<MeasurementPositionVelocityAttitude>(pva1),
                               time));
	ASSERT_EQ(pva->get_time_of_validity(), time);
}
