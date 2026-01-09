#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>
#include <test_data_generation.hpp>

#include <navtk/aspn.hpp>
#include <navtk/experimental/random.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/inertial/AlignBase.hpp>
#include <navtk/inertial/CoarseDynamicAlignment.hpp>
#include <navtk/inertial/ManualAlignment.hpp>
#include <navtk/inertial/ManualHeadingAlignment.hpp>
#include <navtk/inertial/StaticAlignment.hpp>
#include <navtk/inertial/StaticWahbaAlignment.hpp>
#include <navtk/inertial/quaternion_static_alignment.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/wgs84.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::MeasurementAttitude3D;
using aspn_xtensor::MeasurementImu;
using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::MeasurementVelocity;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::dot;
using navtk::eye;
using navtk::Matrix;
using navtk::Matrix3;
using navtk::Vector;
using navtk::Vector3;
using navtk::zeros;
using navtk::filtering::ideal_imu_model;
using navtk::filtering::ImuModel;
using navtk::filtering::stim300_model;
using navtk::inertial::AlignBase;
using navtk::inertial::ManualAlignment;
using navtk::inertial::ManualHeadingAlignment;
using navtk::inertial::MotionNeeded;
using navtk::inertial::quaternion_static_alignment;
using navtk::inertial::StaticAlignment;
using navtk::inertial::StaticWahbaAlignment;
using navtk::navutils::dcm_to_rpy;
using navtk::navutils::PI;
using navtk::navutils::rpy_to_dcm;
using navtk::navutils::rpy_to_quat;
using navtk::testing::stationary_imu;

namespace {
Vector static_align_lambda_base(const Vector& x,
                                std::shared_ptr<MeasurementPositionVelocityAttitude> pva,
                                double dt) {
	auto imu       = stationary_imu(pva, dt);
	auto to_solve  = xt::linalg::outer(xt::view(x, xt::range(0, 3)), imu.get_meas_accel());
	auto to_solve2 = xt::linalg::outer(xt::view(x, xt::range(3, 6)), imu.get_meas_gyro());
	auto to_solve3 = to_solve + to_solve2;
	auto csn3      = navtk::solve_wahba_svd(to_solve3);
	return dcm_to_rpy(xt::transpose(csn3));
}

Vector manual_align_lambda_base(const Vector& x, const double heading) {
	Matrix3 cns =
	    quaternion_static_alignment(xt::view(x, xt::range(0, 3)), xt::view(x, xt::range(3, 6)));
	auto rpy_out = dcm_to_rpy(cns);
	rpy_out[2]   = heading;
	return rpy_out;
}
}  // namespace

struct AlignmentTests : public ::testing::Test {
	TypeTimestamp timestamp = TypeTimestamp((int64_t)0);
	TypeTimestamp ts10      = TypeTimestamp((int64_t)10e9);
	TypeHeader header       = TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0);
	Vector3 llh{0.7330383, -1.308997, 500};
	Vector3 rpy{1.0, 0.2, -0.3};
	Matrix3 Csn          = xt::transpose(rpy_to_dcm(rpy));
	ImuModel model       = stim300_model();
	double heading_sigma = navtk::navutils::PI / 180.0;
	double dt            = 0.1;
	std::shared_ptr<MeasurementPositionVelocityAttitude> pva =
	    create_pva(timestamp, llh, zeros(3), rpy, zeros(9, 9));

	AlignmentTests() : ::testing::Test() {}

	Vector tilt_in_sigma(const Matrix3& csn, const Matrix3& csn_align, const Matrix3& tilt_cov) {
		auto dcm_diff = navtk::dot(xt::transpose(csn_align), csn);
		auto rpy_diff = navtk::navutils::dcm_to_rpy(dcm_diff);
		// If no bias on imu error should be near 0. As we induce error through bias, check to see
		// if covariance appropriately covers the error spread (2 sigma should be good based on bias
		// settings)
		return xt::diagonal(navtk::chol(tilt_cov)) * 2.0 - xt::abs(rpy_diff);
	}

	void compare_w_monte(std::shared_ptr<AlignBase> align,
	                     std::function<Vector(const Vector&)>& fx,
	                     const navtk::filtering::ImuModel model = navtk::filtering::stim300_model(),
	                     std::shared_ptr<Matrix3> monte_res     = nullptr,
	                     bool with_bias                         = true) {

		pva = create_pva(timestamp, llh, zeros(3), rpy, zeros(9, 9));

		// Note that biases are normally at the accel/rate level in imu frame, not dv/dth
		// but perturbation based methods will be working on the integrated/rotated values
		Vector biases_sig                     = zeros(6);
		xt::view(biases_sig, xt::range(0, 3)) = model.accel_bias_initial_sigma * dt;
		xt::view(biases_sig, xt::range(3, 6)) = model.gyro_bias_initial_sigma * dt;

		auto imu = std::make_shared<MeasurementImu>(navtk::testing::stationary_imu(pva, dt));

		// vstack apparently doesn't work w/ fixed sized
		// https://github.com/xtensor-stack/xtensor/issues/2372
		// xt::vstack(xt::xtuple(imu.get_meas_accel(), imu.get_meas_gyro()))
		Vector est                     = zeros(6);
		xt::view(est, xt::range(0, 3)) = imu->get_meas_accel();
		xt::view(est, xt::range(3, 6)) = imu->get_meas_gyro();
		auto ec = navtk::filtering::EstimateWithCovariance(est, xt::diag(xt::pow(biases_sig, 2.0)));

		if (with_bias) {
			// Biasing imu meas will induce an error into the estimated dcm. Random draws would be
			// more appropriate; setting to sigma will skew the error a little to the high side
			auto dv_bias  = navtk::dot(xt::transpose(Csn), model.accel_bias_sigma * dt);
			auto dth_bias = navtk::dot(xt::transpose(Csn), model.gyro_bias_sigma * dt);
			imu->set_meas_accel(imu->get_meas_accel() + dv_bias);
			imu->set_meas_gyro(imu->get_meas_gyro() + dth_bias);
		}
		for (navtk::Size k = 0; k < 1500; k++) {
			auto new_time = imu->get_time_of_validity() + dt;
			auto ts       = new_time;
			imu           = create_imu(ts, imu->get_meas_accel(), imu->get_meas_gyro());

			auto res = align->process(imu);

			if (k % 50 == 0) {
				auto pos = create_pos(ts, llh, navtk::eye(3));
				align->process(pos);
			}
			if (res == AlignBase::AlignmentStatus::ALIGNED_GOOD) {
				break;
			}
		}

		auto aligned = align->get_computed_alignment();
		auto cov_gen = align->get_computed_covariance();

		ASSERT_TRUE(aligned.first);
		ASSERT_TRUE(cov_gen.first);

		auto monte_cov_sol = (monte_res == nullptr)
		                         ? navtk::filtering::monte_carlo_approx_rpy(ec, fx, 1000).covariance
		                         : (Matrix)*monte_res;

		auto cov_val = xt::view(cov_gen.second, xt::range(6, 9), xt::range(6, 9));

		ASSERT_ALLCLOSE_EX(llh, aligned.second.pos, 0, 1e-5);
		ASSERT_ALLCLOSE_EX(zeros(3), aligned.second.vel, 0, 1e-5);

		// Due to linearization etc, nailing the cov is unlikely, making proper testing difficult.
		if (with_bias) {
			auto df = tilt_in_sigma(Csn, aligned.second.rot_mat, cov_val);
			ASSERT_TRUE(xt::all(df > 0));
		}
		ASSERT_ALLCLOSE_EX(xt::diagonal(cov_val), xt::diagonal(monte_cov_sol), 0.1, 0.0);
	}

	std::shared_ptr<ManualHeadingAlignment> manual_heading_test_base(
	    const Vector3& new_rpy,
	    std::shared_ptr<Matrix3> monte_1000_res,
	    bool with_bias = true,
	    ImuModel mod   = stim300_model()) {
		rpy        = new_rpy;
		Csn        = xt::transpose(rpy_to_dcm(new_rpy));
		auto align = std::make_shared<ManualHeadingAlignment>(rpy[2], heading_sigma, mod, 30.0);

		std::function<Vector(const Vector&)> fx = [&, heading = rpy[2]](const Vector& x) {
			return manual_align_lambda_base(x, heading);
		};

		compare_w_monte(align, fx, mod, monte_1000_res, with_bias);

		return align;
	}

	std::shared_ptr<StaticWahbaAlignment> static_wahba_test_base(
	    const Vector3& new_rpy,
	    std::shared_ptr<Matrix3> monte_1000_res,
	    bool with_bias = true,
	    ImuModel mod   = stim300_model()) {
		rpy        = new_rpy;
		Csn        = xt::transpose(rpy_to_dcm(new_rpy));
		auto align = std::make_shared<StaticWahbaAlignment>(mod, 30.0);

		std::function<Vector(const Vector&)> fx = [&](const Vector& x) {
			;
			return static_align_lambda_base(x, pva, dt);
		};

		compare_w_monte(align, fx, mod, monte_1000_res, with_bias);

		return align;
	}

	std::shared_ptr<MeasurementPosition> create_pos(const TypeTimestamp& time,
	                                                const Vector3& pos,
	                                                const Matrix& cov) {
		TypeHeader header(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0);
		return std::make_shared<MeasurementPosition>(
		    header,
		    time,
		    ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
		    pos(0),
		    pos(1),
		    pos(2),
		    cov,
		    ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
		    Vector(),
		    std::vector<aspn_xtensor::TypeIntegrity>{});
	}

	std::shared_ptr<MeasurementVelocity> create_vel(const TypeTimestamp& time,
	                                                const Vector3& vel,
	                                                const Matrix& cov) {
		TypeHeader header(ASPN_MEASUREMENT_VELOCITY, 0, 0, 0, 0);
		return std::make_shared<MeasurementVelocity>(header,
		                                             time,
		                                             ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED,
		                                             vel(0),
		                                             vel(1),
		                                             vel(2),
		                                             cov,
		                                             ASPN_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE,
		                                             Vector(),
		                                             std::vector<aspn_xtensor::TypeIntegrity>{});
	}

	std::shared_ptr<MeasurementAttitude3D> create_att(const TypeTimestamp& time,
	                                                  const Vector3& rpy,
	                                                  const Matrix& cov) {
		TypeHeader header(ASPN_MEASUREMENT_ATTITUDE_3D, 0, 0, 0, 0);
		return std::make_shared<MeasurementAttitude3D>(
		    header,
		    time,
		    ASPN_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_NED,
		    rpy_to_quat(rpy),
		    cov,
		    ASPN_MEASUREMENT_ATTITUDE_3D_ERROR_MODEL_NONE,
		    Vector(),
		    std::vector<aspn_xtensor::TypeIntegrity>{});
	}

	std::shared_ptr<MeasurementImu> create_imu(const TypeTimestamp& time,
	                                           const Vector3& delta_v,
	                                           const Vector& delta_theta) {
		TypeHeader header(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0);
		return std::make_shared<MeasurementImu>(header,
		                                        time,
		                                        ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED,
		                                        delta_v,
		                                        delta_theta,
		                                        std::vector<aspn_xtensor::TypeIntegrity>{});
	}

	std::shared_ptr<MeasurementPositionVelocityAttitude> create_pva(const TypeTimestamp& time,
	                                                                const Vector3& pos,
	                                                                const Vector& vel,
	                                                                const Vector& rpy,
	                                                                const Matrix& cov) {
		TypeHeader header(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0);
		return std::make_shared<MeasurementPositionVelocityAttitude>(
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
		    Vector(),
		    std::vector<aspn_xtensor::TypeIntegrity>{});
	}
};

TEST_F(AlignmentTests, quaternion_static_alignment) {
	Matrix3 out = quaternion_static_alignment(Vector3{0.01, 0.04, 0.078}, Vector3{1, 0, 1});
	Matrix3 expected{{0.884677431118, -0.451052224440, 0.117888649570},
	                 {-0.452215631646, -0.768766573798, 0.452215631646},
	                 {-0.113344013361, -0.453376053445, -0.884083304218}};
	ASSERT_ALLCLOSE(expected, out);
}

TEST_F(AlignmentTests, quaternion_static_alignment_real_data) {
	Vector dv_avg{-0.003379397801268078, -0.001691702852560517, 0.09796068893096181};
	Vector dth_avg{-3.980171944325169e-08, 5.605518429850539e-07, 4.966340687436362e-07};

	Matrix3 cnb = quaternion_static_alignment(dv_avg, dth_avg);

	Vector dv_n  = dot(cnb, dv_avg);
	Vector dth_n = dot(cnb, dth_avg);

	Matrix3 expected_cnb{{-0.04035186409426, 0.99905963693649, 0.01586092394656},
	                     {0.99859071639466, 0.03977511435515, 0.03513575683601},
	                     {0.03447184640432, 0.01725636469117, -0.99925667857819}};
	EXPECT_ALLCLOSE(cnb, expected_cnb);

	Vector expected_dv_n{0, 0, -0.0980335593757};
	Vector expected_dth_n{0.00000056950787, 0, -0.00000048796386};
	EXPECT_ALLCLOSE(dv_n, expected_dv_n);
	EXPECT_ALLCLOSE(dth_n, expected_dth_n);
}

TEST_F(AlignmentTests, quaternion_static_alignment_bad_input) {
	// Very simplified substrings used here since error_mode_assert has issues with the symbols in
	// the full error messages
	EXPECT_UB_OR_DIE(quaternion_static_alignment(Vector3{0.01, 1e10, 0.078}, Vector3{1, 0, 1}),
	                 "accel_np",
	                 std::runtime_error);
	EXPECT_UB_OR_DIE(quaternion_static_alignment(Vector3{0.01, 0.04, 0.078}, Vector3{1, 1e10, 1}),
	                 "gy_n",
	                 std::runtime_error);
}

TEST_F(AlignmentTests, StaticAlignment_SLOW) {

	// Setup measurements
	TypeTimestamp timestamp_later(150 * navtk::utils::NANO_PER_SEC);

	auto pos       = create_pos(timestamp, llh, eye(3));
	auto pos_later = create_pos(timestamp_later, llh, eye(3));
	auto pva_later =
	    create_pva(timestamp_later, llh, zeros(3), zeros(3), xt::diag(xt::arange(0.0, 9.0, 1.0)));
	auto imu       = create_imu(timestamp, Vector3{1, 2, 3}, Vector3{3, 2, 1});
	auto imu_later = create_imu(timestamp_later, Vector3{1, 2, 3}, Vector3{3, 2, 1});
	StaticAlignment align;


	// Check start-up state
	EXPECT_FALSE(align.requires_dynamic());
	EXPECT_EQ(align.check_alignment_status(), AlignBase::AlignmentStatus::ALIGNING_COARSE);
	EXPECT_EQ(align.get_computed_alignment().first, false);

	// Should not be aligned after first measurement
	align.process(pos);
	auto answer = align.process(imu);
	EXPECT_EQ(answer, AlignBase::AlignmentStatus::ALIGNING_COARSE);
	EXPECT_EQ(align.get_computed_alignment().first, false);

	// Aligned state possible after 120 seconds
	align.process(pos_later);
	answer = align.process(imu_later);
	EXPECT_EQ(answer, AlignBase::AlignmentStatus::ALIGNED_GOOD);
	EXPECT_EQ(align.get_computed_alignment().first, true);

	// Static alignment should need only a single pos; time of solution should be based on last imu
	StaticAlignment align2;
	align2.process(imu);
	align2.process(pos);
	align2.process(imu_later);
	EXPECT_EQ(align2.get_computed_alignment().second.time, 150.0);

	// Order of pos/imu shouldn't matter
	StaticAlignment align3;
	align3.process(imu);
	align3.process(imu_later);
	align3.process(pos);
	EXPECT_EQ(align3.get_computed_alignment().second.time, 150.0);

	// PVA should work too
	StaticAlignment align4;
	align4.process(imu);
	align4.process(imu_later);
	align4.process(pva_later);
	auto aligned = align4.get_computed_alignment();
	EXPECT_TRUE(aligned.first);
	EXPECT_EQ(aligned.second.time, 150.0);
	auto cov = align4.get_computed_covariance();
	EXPECT_ALLCLOSE(xt::view(cov.second, xt::range(0, 3), xt::range(0, 3)),
	                xt::diag(xt::arange(0.0, 3.0, 1.0)));
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, AlignmentTests, static_align_nan_pos) {
	StaticAlignment align;
	TypeTimestamp timestamp_later(150 * navtk::utils::NANO_PER_SEC);
	auto imu       = test.create_imu(test.timestamp, Vector3{1, 2, 3}, Vector3{3, 2, 1});
	auto imu_later = test.create_imu(timestamp_later, Vector3{1, 2, 3}, Vector3{3, 2, 1});
	auto pos_later =
	    test.create_pos(timestamp_later, Vector{test.llh[0], NAN, test.llh[2]}, eye(2));
	auto pva_later = test.create_pva(timestamp_later,
	                                 Vector{test.llh[0], test.llh[1], NAN},
	                                 zeros(3),
	                                 zeros(3),
	                                 xt::diag(xt::arange(0.0, 8.0, 1.0)));
	align.process(imu);
	align.process(imu_later);
	EXPECT_HONORS_MODE_EX(
	    align.process(pva_later),
	    "Position data passed to StaticAlignment appears to contain NANs, ignoring.",
	    std::invalid_argument);
	EXPECT_FALSE(align.get_computed_alignment().first);
	EXPECT_HONORS_MODE_EX(
	    align.process(pos_later),
	    "Position data passed to StaticAlignment appears to contain NANs, ignoring.",
	    std::invalid_argument);
	EXPECT_FALSE(align.get_computed_alignment().first);
}


class BadAlignment : public AlignBase {
public:
	BadAlignment() : AlignBase(false, false, stim300_model()) {}
	AlignmentStatus process(std::shared_ptr<aspn_xtensor::TypeHeader>) override {
		return AlignmentStatus::ALIGNED_GOOD;
	};
	MotionNeeded motion_needed() const override { return MotionNeeded::ANY_MOTION; };
};

ERROR_MODE_SENSITIVE_TEST(TEST_F, AlignmentTests, bad_class) {
	BadAlignment align;
	EXPECT_HONORS_MODE_EX(
	    align.requires_dynamic(), "Alignment class must be flagged", std::runtime_error);
}

TEST_F(AlignmentTests, stationary_SLOW) {
	double d_lat_to_meter = navtk::navutils::delta_lat_to_north(1, llh[0], llh[1]);
	double d_lon_to_meter = navtk::navutils::delta_lon_to_east(1, llh[0], llh[1]);
	auto pos              = create_pos(timestamp, llh, navtk::eye(3));
	auto imu              = std::make_shared<MeasurementImu>(stationary_imu(pva, 1.0));

	StaticAlignment align;
	StaticWahbaAlignment align2;
	Vector3 sigma{4.0, 2.0, 5.0};
	Matrix3 cov = xt::diag(xt::pow(sigma, 2.0));
	Matrix3 noise_shape =
	    dot(xt::diag(Vector3{1.0 / d_lat_to_meter, 1.0 / d_lon_to_meter, 1.0}), xt::diag(sigma));

	for (navtk::Size k = 0; k < 200; k++) {
		auto ts    = TypeTimestamp(k * navtk::utils::NANO_PER_SEC);
		auto noise = dot(noise_shape, navtk::experimental::rand_n(3));
		auto pos   = create_pos(ts, llh + noise, cov);
		imu        = create_imu(ts, imu->get_meas_accel(), imu->get_meas_gyro());
		align.process(pos);
		align.process(imu);
		align2.process(pos);
		align2.process(imu);
		if (align.check_alignment_status() == AlignBase::AlignmentStatus::ALIGNED_GOOD) {
			break;
		}
	}

	auto aligned2 = align2.get_computed_alignment();
	ASSERT_TRUE(aligned2.first);
	ASSERT_ALLCLOSE(Csn, aligned2.second.rot_mat);
	ASSERT_ALLCLOSE(xt::view(llh, xt::range(0, 2)), xt::view(aligned2.second.pos, xt::range(0, 2)));
	// With the addition of random noise we now need to allow for a bit of deviation from the
	// nominal position
	ASSERT_ALLCLOSE_EX(xt::view(llh, 2), xt::view(aligned2.second.pos, 2), 0.0, 1.0);

	auto aligned = align.get_computed_alignment();
	ASSERT_TRUE(aligned.first);
	ASSERT_ALLCLOSE(xt::view(llh, xt::range(0, 2)), xt::view(aligned.second.pos, xt::range(0, 2)));
	ASSERT_ALLCLOSE_EX(xt::view(llh, 2), xt::view(aligned.second.pos, 2), 0.0, 1.0);
	ASSERT_ALLCLOSE_EX(Csn, aligned.second.rot_mat, 1e-3, 0);
}

TEST_F(AlignmentTests, stationary_biased_SLOW) {
	auto imu = std::make_shared<MeasurementImu>(stationary_imu(pva, 1.0));

	// Nothing special about the added values, just a recorded random draw
	imu->set_meas_gyro(imu->get_meas_gyro() + Vector3{1.345296584723281e-07,
	                                                  -1.463817811897227e-07,
	                                                  1.606501823830636e-07});
	imu->set_meas_accel(imu->get_meas_accel() +
	                    Vector3{0.000163711684234, -0.000214253388209, 0.000298595259558});

	StaticAlignment align;
	StaticWahbaAlignment align2;
	StaticWahbaAlignment align3;

	auto dt = navtk::utils::NANO_PER_SEC / 1.2;
	for (navtk::Size k = 0; k < 200; k++) {
		auto ts          = TypeTimestamp(k * dt);
		auto pos         = create_pos(ts, llh, navtk::eye(3));
		imu              = create_imu(ts, imu->get_meas_accel(), imu->get_meas_gyro());
		auto imu_sampled = create_imu(ts, imu->get_meas_accel() / dt, imu->get_meas_gyro() / dt);
		imu_sampled->set_imu_type(ASPN_MEASUREMENT_IMU_IMU_TYPE_SAMPLED);

		align.process(pos);
		align.process(imu);
		align2.process(pos);
		align2.process(imu);
		align3.process(pos);
		align3.process(imu_sampled);
		if (align.check_alignment_status() == AlignBase::AlignmentStatus::ALIGNED_GOOD) {
			break;
		}
	}

	auto aligned2 = align2.get_computed_alignment();
	ASSERT_TRUE(aligned2.first);
	ASSERT_ALLCLOSE_EX(Csn, aligned2.second.rot_mat, 3e-2, 0);
	ASSERT_ALLCLOSE(llh, aligned2.second.pos);
	auto cov = xt::view(align2.get_computed_covariance().second, xt::range(6, 9), xt::range(6, 9));
	auto df  = tilt_in_sigma(Csn, aligned2.second.rot_mat, cov);
	ASSERT_TRUE(xt::all(df > 0));

	auto aligned = align.get_computed_alignment();
	ASSERT_TRUE(aligned.first);
	ASSERT_ALLCLOSE_EX(Csn, aligned.second.rot_mat, 3e-2, 0);
	ASSERT_ALLCLOSE(llh, aligned.second.pos);
	cov = xt::view(align.get_computed_covariance().second, xt::range(6, 9), xt::range(6, 9));
	df  = tilt_in_sigma(Csn, aligned.second.rot_mat, cov);
	ASSERT_TRUE(xt::all(df > 0));

	auto aligned3 = align3.get_computed_alignment();
	ASSERT_TRUE(aligned3.first);
	ASSERT_ALLCLOSE_EX(Csn, aligned3.second.rot_mat, 3e-2, 0);
	ASSERT_ALLCLOSE(llh, aligned3.second.pos);
	cov = xt::view(align3.get_computed_covariance().second, xt::range(6, 9), xt::range(6, 9));
	df  = tilt_in_sigma(Csn, aligned3.second.rot_mat, cov);
	ASSERT_TRUE(xt::all(df > 0));
}

TEST_F(AlignmentTests, manual) {
	auto pos = create_pos(timestamp, llh, navtk::eye(3));

	ManualAlignment align(*pva);
	align.process(pos);
	auto aligned = align.get_computed_alignment();
	ASSERT_TRUE(aligned.first);
	ASSERT_ALLCLOSE_EX(Csn, aligned.second.rot_mat, 1e-4, 0);
	ASSERT_ALLCLOSE(llh, aligned.second.pos);
	ASSERT_EQ(aligned.second.time, 0.0);

	auto imu = std::make_shared<MeasurementImu>(stationary_imu(pva, 1.0));
	align.process(imu);
	aligned = align.get_computed_alignment();
	ASSERT_EQ(aligned.second.time, 1.0);

	timestamp = TypeTimestamp(1000 * navtk::utils::NANO_PER_SEC);
	pos       = create_pos(timestamp, llh, navtk::eye(3));
	align.process(pos);
	aligned = align.get_computed_alignment();
	ASSERT_EQ(aligned.second.time, 1.0);
}

void compare_aligned_solutions(const AlignBase& aligner,
                               const navtk::filtering::NavSolution& exp_sol,
                               const Matrix& exp_cov) {
	auto sol = aligner.get_computed_alignment();
	auto cov = aligner.get_computed_covariance();
	ASSERT_TRUE(sol.first);
	ASSERT_TRUE(cov.first);
	ASSERT_ALLCLOSE(sol.second.rot_mat, exp_sol.rot_mat);
	ASSERT_ALLCLOSE(sol.second.pos, exp_sol.pos);
	ASSERT_ALLCLOSE(sol.second.vel, exp_sol.vel);
	ASSERT_EQ(sol.second.time, exp_sol.time);
	ASSERT_ALLCLOSE(cov.second, exp_cov);
}

TEST_F(AlignmentTests, manual_track) {
	Matrix main_cov = xt::diag(Vector{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0});
	Matrix my_cov   = zeros(15, 15);
	xt::view(my_cov, xt::range(0, 9), xt::range(0, 9)) = main_cov;
	xt::view(my_cov, xt::range(9, 12), xt::range(9, 12)) =
	    xt::diag(xt::pow((Vector)model.accel_bias_initial_sigma, 2));
	xt::view(my_cov, xt::range(12, 15), xt::range(12, 15)) =
	    xt::diag(xt::pow((Vector)model.gyro_bias_initial_sigma, 2));
	Vector3 my_vel{0.2, -1.9, 3.3};
	pva        = create_pva(timestamp, llh, my_vel, rpy, main_cov);
	auto as_ns = navtk::utils::to_navsolution(*pva);

	ManualAlignment align(*pva);
	compare_aligned_solutions(align, as_ns, my_cov);

	// Should update both pos and time
	TypeTimestamp ts(1.5 * navtk::utils::NANO_PER_SEC);
	Vector3 meas_pos{-1.0, -1.2, 999.0};
	auto pos                                           = create_pos(ts, meas_pos, navtk::eye(3));
	as_ns.time                                         = ts;
	as_ns.pos                                          = meas_pos;
	xt::view(my_cov, xt::range(0, 3), xt::range(0, 3)) = navtk::eye(3);

	align.process(pos);
	compare_aligned_solutions(align, as_ns, my_cov);

	// Should update to imu time
	auto imu = std::make_shared<MeasurementImu>(stationary_imu(pva, 2.0));
	align.process(imu);
	as_ns.time = imu->get_time_of_validity();
	compare_aligned_solutions(align, as_ns, my_cov);

	// MeasurementImu received; should change pos but not time (assumes that the imu stream will
	// continue)
	ts        = TypeTimestamp(2.5 * 1e9);
	meas_pos  = {1.0, 1.2, -999.0};
	pos       = create_pos(ts, meas_pos, navtk::eye(3) * 2.0);
	as_ns.pos = meas_pos;
	xt::view(my_cov, xt::range(0, 3), xt::range(0, 3)) = navtk::eye(3) * 2.0;
	align.process(pos);
	compare_aligned_solutions(align, as_ns, my_cov);

	// Other supported data types- should still track imu time but update meas/cov
	Vector3 xyz{3.0, 2.0, 1.0};
	auto att      = create_att(ts, xyz, navtk::eye(3) * 0.1);
	as_ns.rot_mat = xt::transpose(navtk::navutils::rpy_to_dcm(xyz));
	xt::view(my_cov, xt::range(6, 9), xt::range(6, 9)) = navtk::eye(3) * 0.1;
	align.process(att);
	compare_aligned_solutions(align, as_ns, my_cov);

	auto vel                                           = create_vel(ts, xyz, navtk::eye(3) * 6.0);
	as_ns.vel                                          = xyz;
	xt::view(my_cov, xt::range(3, 6), xt::range(3, 6)) = navtk::eye(3) * 6.0;
	align.process(vel);
	compare_aligned_solutions(align, as_ns, my_cov);

	meas_pos = {0.1, 0.2, 0.3};
	auto pva =
	    create_pva(ts, meas_pos, Vector3{0.4, 0.5, 0.6}, Vector3{0.7, 0.8, 0.9}, navtk::eye(9));
	as_ns.pos     = meas_pos;
	as_ns.vel     = Vector3{0.4, 0.5, 0.6};
	as_ns.rot_mat = xt::transpose(navtk::navutils::rpy_to_dcm(Vector3{0.7, 0.8, 0.9}));
	xt::view(my_cov, xt::range(0, 9), xt::range(0, 9)) = navtk::eye(9);

	align.process(pva);
	compare_aligned_solutions(align, as_ns, my_cov);

	// Unsupported
	auto meas = std::make_shared<aspn_xtensor::TypeHeader>(ASPN_UNDEFINED, 0, 0, 0, 0);
	align.process(meas);
	compare_aligned_solutions(align, as_ns, my_cov);
}

void check_flagged_correctly(ManualAlignment& align, bool expect_good) {
	if (expect_good) {
		ASSERT_TRUE(align.get_computed_alignment().first);
		ASSERT_TRUE(align.check_alignment_status() == AlignBase::AlignmentStatus::ALIGNED_GOOD);
	} else {
		ASSERT_TRUE(!align.get_computed_alignment().first);
		ASSERT_TRUE(align.check_alignment_status() == AlignBase::AlignmentStatus::ALIGNING_COARSE);
	}
}

TEST_F(AlignmentTests, computed_cov_size) {
	std::vector<std::shared_ptr<AlignBase>> aligners{std::make_shared<ManualAlignment>(*pva),
	                                                 std::make_shared<ManualHeadingAlignment>(0.0),
	                                                 std::make_shared<StaticAlignment>(),
	                                                 std::make_shared<StaticWahbaAlignment>()};
	for (auto k = aligners.begin(); k < aligners.end(); k++) {
		auto cov_15 =
		    (*k)->get_computed_covariance(AlignBase::CovarianceFormat::PINSON15NEDBLOCK).second;
		ASSERT_TRUE(navtk::num_rows(cov_15) == 15);
		ASSERT_TRUE(navtk::num_cols(cov_15) == 15);
		auto cov_21 =
		    (*k)->get_computed_covariance(AlignBase::CovarianceFormat::PINSON21NEDBLOCK).second;
		ASSERT_TRUE(navtk::num_rows(cov_21) == 21);
		ASSERT_TRUE(navtk::num_cols(cov_21) == 21);
	}
}

TEST_F(AlignmentTests, not_good_until) {
	// bool args default to false
	ManualAlignment align_needs_none(*pva);
	ManualAlignment align_needs_imu(*pva, true);
	ManualAlignment align_needs_pos(*pva, false, true);
	ManualAlignment align_needs_vel(*pva, false, false, true);
	ManualAlignment align_needs_att(*pva, false, false, false, true);
	ManualAlignment align_needs_all(*pva, true, true, true, true);
	ManualAlignment align_needs_pva(*pva, false, true, true, true);
	ManualAlignment align_needs_time_pos(*pva, true, true);

	std::vector<AlignBase*> ta{&align_needs_none,
	                           &align_needs_imu,
	                           &align_needs_pos,
	                           &align_needs_vel,
	                           &align_needs_att,
	                           &align_needs_all,
	                           &align_needs_pva,
	                           &align_needs_time_pos};
	auto imu = std::make_shared<MeasurementImu>(stationary_imu(pva, 2.0));
	auto p   = create_pos(timestamp, llh, zeros(3, 3));
	auto v   = create_vel(timestamp, zeros(3), zeros(3, 3));
	auto a   = create_att(timestamp, zeros(3), zeros(3, 3));

	check_flagged_correctly(align_needs_none, true);
	check_flagged_correctly(align_needs_imu, false);
	check_flagged_correctly(align_needs_pos, false);
	check_flagged_correctly(align_needs_vel, false);
	check_flagged_correctly(align_needs_att, false);
	check_flagged_correctly(align_needs_all, false);
	check_flagged_correctly(align_needs_pva, false);
	check_flagged_correctly(align_needs_time_pos, false);

	for (auto ele = ta.cbegin(); ele != ta.cend(); ele++) {
		(*ele)->process(v);
	}

	check_flagged_correctly(align_needs_none, true);
	check_flagged_correctly(align_needs_imu, false);
	check_flagged_correctly(align_needs_pos, false);
	check_flagged_correctly(align_needs_vel, true);
	check_flagged_correctly(align_needs_att, false);
	check_flagged_correctly(align_needs_all, false);
	check_flagged_correctly(align_needs_pva, false);
	check_flagged_correctly(align_needs_time_pos, false);

	for (auto ele = ta.cbegin(); ele != ta.cend(); ele++) {
		(*ele)->process(a);
	}

	check_flagged_correctly(align_needs_none, true);
	check_flagged_correctly(align_needs_imu, false);
	check_flagged_correctly(align_needs_pos, false);
	check_flagged_correctly(align_needs_vel, true);
	check_flagged_correctly(align_needs_att, true);
	check_flagged_correctly(align_needs_all, false);
	check_flagged_correctly(align_needs_pva, false);
	check_flagged_correctly(align_needs_time_pos, false);

	for (auto ele = ta.cbegin(); ele != ta.cend(); ele++) {
		(*ele)->process(p);
	}

	check_flagged_correctly(align_needs_none, true);
	check_flagged_correctly(align_needs_imu, false);
	check_flagged_correctly(align_needs_pos, true);
	check_flagged_correctly(align_needs_vel, true);
	check_flagged_correctly(align_needs_att, true);
	check_flagged_correctly(align_needs_all, false);
	check_flagged_correctly(align_needs_pva, true);
	check_flagged_correctly(align_needs_time_pos, false);

	for (auto ele = ta.cbegin(); ele != ta.cend(); ele++) {
		(*ele)->process(imu);
	}

	check_flagged_correctly(align_needs_none, true);
	check_flagged_correctly(align_needs_imu, true);
	check_flagged_correctly(align_needs_pos, true);
	check_flagged_correctly(align_needs_vel, true);
	check_flagged_correctly(align_needs_att, true);
	check_flagged_correctly(align_needs_all, true);
	check_flagged_correctly(align_needs_pva, true);
	check_flagged_correctly(align_needs_time_pos, true);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, AlignmentTests, unsupported_type_in_manual) {
	ManualAlignment align(*test.pva);
	EXPECT_HONORS_MODE_EX(
	    align.process(std::make_shared<aspn_xtensor::TypeHeader>(ASPN_UNDEFINED, 0, 0, 0, 0)),
	    "Unsupported data type received",
	    std::runtime_error);
	auto as_ns                                           = navtk::utils::to_navsolution(*test.pva);
	auto full_cov                                        = zeros(15, 15);
	xt::view(full_cov, xt::range(0, 9), xt::range(0, 9)) = test.pva->get_covariance();
	xt::view(full_cov, xt::range(9, 12), xt::range(9, 12)) =
	    xt::diag(xt::pow((Vector)test.model.accel_bias_initial_sigma, 2));
	xt::view(full_cov, xt::range(12, 15), xt::range(12, 15)) =
	    xt::diag(xt::pow((Vector)test.model.gyro_bias_initial_sigma, 2));
	compare_aligned_solutions(align, as_ns, full_cov);
}

TEST_F(AlignmentTests, monte_carlo_cov_SLOW) {
	auto align = std::make_shared<StaticWahbaAlignment>(model, 30.0);

	std::function<Vector(const Vector&)> fx = [pva = pva, dt = dt](const Vector& x) {
		pva->set_quaternion(rpy_to_quat(zeros(3)));
		auto imu       = stationary_imu(pva, dt);
		auto to_solve  = xt::linalg::outer(xt::view(x, xt::range(0, 3)), imu.get_meas_accel());
		auto to_solve2 = xt::linalg::outer(xt::view(x, xt::range(3, 6)), imu.get_meas_gyro());
		auto to_solve3 = to_solve + to_solve2;
		auto csn3      = navtk::solve_wahba_svd(to_solve3);
		return dcm_to_rpy(xt::transpose(csn3));
	};

	auto monte_1000_res =
	    std::make_shared<Matrix3>(Matrix3{{6.067015e-07, -4.821618e-08, -9.978713e-07},
	                                      {-4.821618e-08, 6.052448e-07, -1.074004e-06},
	                                      {-9.978713e-07, -1.074004e-06, 2.019243e-03}});
	compare_w_monte(align, fx, model, monte_1000_res);
}

TEST_F(AlignmentTests, static_monte_carlo_cov_SLOW) {
	auto align                              = std::make_shared<StaticAlignment>(model, 30.0);
	std::function<Vector(const Vector&)> fx = [](const Vector& x) {
		Matrix3 cns =
		    quaternion_static_alignment(xt::view(x, xt::range(0, 3)), xt::view(x, xt::range(3, 6)));
		return dcm_to_rpy(cns);
	};

	auto monte_1000_res =
	    std::make_shared<Matrix3>(Matrix3{{5.512104e-07, -1.871912e-08, -1.594532e-06},
	                                      {-1.871912e-08, 5.733052e-07, 9.789871e-07},
	                                      {-1.594532e-06, 9.789871e-07, 1.974818e-03}});
	compare_w_monte(align, fx, model, monte_1000_res);
}

TEST_F(AlignmentTests, manual_heading_monte_carlo_cov5_no_bias_SLOW) {
	rpy                 = Vector3{PI, PI / 2.0, PI / 4.0};
	auto monte_1000_res = std::make_shared<Matrix3>(
	    Matrix3{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, std::pow(heading_sigma, 2.0)}});
	manual_heading_test_base(rpy, monte_1000_res, false, ideal_imu_model());
}

TEST_F(AlignmentTests, manual_heading_approx_level_cov_check_SLOW) {
	rpy = Vector3{0.1, 0.1, PI / 4.0};

	// Expected covariance block for tilt/accel bias states.
	Matrix pre_res{
	    {5.634247e-07, -5.717685e-23, 0.000000e+00, 3.885601e-06, 3.924521e-06, -1.962642e-09},
	    {-7.358074e-23, 5.634247e-07, 0.000000e+00, -3.885601e-06, 3.846678e-06, -7.777732e-07},
	    {0.000000e+00, 0.000000e+00, 3.046174e-04, 0.0, 0.0, 0.000000e+00},
	    {3.885601e-06, -3.885601e-06, 0.000000e+00, 5.413281e-05, 0.000000e+00, 0.000000e+00},
	    {3.924521e-06, 3.846678e-06, 0.000000e+00, 0.000000e+00, 5.413281e-05, 0.000000e+00},
	    {-1.962642e-09, -7.777732e-07, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.413281e-05}};

	auto monte_1000_res =
	    std::make_shared<Matrix3>(xt::view(pre_res, xt::range(0, 3), xt::range(0, 3)));
	auto aligner = manual_heading_test_base(rpy, monte_1000_res, false);

	// Verify that cross terms are being set correctly
	auto cov_block =
	    xt::view(aligner->get_computed_covariance().second, xt::range(6, 12), xt::range(6, 12));
	ASSERT_ALLCLOSE(cov_block, pre_res);
}

TEST_F(AlignmentTests, wahba_monte_carlo_cov5_no_bias_SLOW) {
	// Compare with quaternion based results. In absence of bias, wahba based method can
	// determine this dcm, while quaternion fails
	rpy                 = Vector3{PI, PI / 2.0, PI / 4.0};
	auto monte_1000_res = std::make_shared<Matrix3>(zeros(3, 3));
	static_wahba_test_base(rpy, monte_1000_res, false, ideal_imu_model());
}

// With a bias, quaternion-based manual alignment fails, missing heading by about 126 deg, while
// wahba is off by about 4.5 deg. Re-enable this test to produce the failure.
TEST_F(AlignmentTests, DISABLED_manual_heading_monte_carlo_cov5) {
	rpy = Vector3{PI, PI / 2.0, PI / 4.0};
	auto monte_1000_res =
	    std::make_shared<Matrix3>(Matrix3{{4.235302e-07, -4.219383e-07, 0.0},
	                                      {-4.219383e-07, 4.234638e-07, 0.0},
	                                      {0.0, 0.0, std::pow(heading_sigma, 2.0)}});
	manual_heading_test_base(rpy, monte_1000_res);
}

TEST_F(AlignmentTests, wahba_monte_carlo_cov5_SLOW) {
	rpy = Vector3{PI, PI / 2.0, PI / 4.0};
	auto monte_1000_res =
	    std::make_shared<Matrix3>(Matrix3{{5.368492e-07, 7.350237e-10, -7.888005e-07},
	                                      {7.350237e-10, 5.505237e-07, 1.230954e-06},
	                                      {-7.888005e-07, 1.230954e-06, 2.007988e-03}});
	static_wahba_test_base(rpy, monte_1000_res);
}

/*
 * TODO #581
 * The following tests are disabled due to inability to resolve heading properly in cases where
 * z is not approximately down facing. It's easy enough to get z in the proper direction (see
 * the first 5 lines or so of quaternion_static_alignment), but results in a 'shortest distance'
 * rotation that projects the x and y axes onto the horizontal plane but not aligned with NE.
 * The process can probably be made to work with more arbitrary DCMs, but for now the documentation
 * has been changed to mention limitations.
 */
TEST_F(AlignmentTests, DISABLED_manual_heading_monte_carlo_cov6) {
	rpy = Vector3{0.7, 0.7, 0.7};
	auto monte_1000_res =
	    std::make_shared<Matrix3>(Matrix3{{5.695099e-07, 1.996765e-08, 0.0},
	                                      {1.996765e-08, 5.610084e-07, 0.0},
	                                      {0.0, 0.0, std::pow(heading_sigma, 2.0)}});
	manual_heading_test_base(rpy, monte_1000_res);
}

TEST_F(AlignmentTests, DISABLED_manual_heading_monte_carlo_cov) {
	auto monte_1000_res = std::make_shared<Matrix3>(
	    Matrix3{{5.759883e-07, -1.436935e-08, 0.000000e+00},
	            {-1.436935e-08, 6.241068e-07, 0.000000e+00},
	            {0.000000e+00, 0.000000e+00, std::pow(heading_sigma, 2.0)}});
	manual_heading_test_base(rpy, monte_1000_res);
}

TEST_F(AlignmentTests, DISABLED_manual_heading_monte_carlo_cov2) {
	rpy = Vector3{PI, 0, PI / 4.0};
	auto monte_1000_res =
	    std::make_shared<Matrix3>(Matrix3{{5.915542e-07, -4.521149e-08, 0.0},
	                                      {-4.521149e-08, 5.720447e-07, 0.0},
	                                      {0.0, 0.0, std::pow(heading_sigma, 2.0)}});
	manual_heading_test_base(rpy, monte_1000_res);
}

TEST_F(AlignmentTests, DISABLED_manual_heading_monte_carlo_cov3) {
	rpy = Vector3{PI, 0, -PI / 4.0};
	auto monte_1000_res =
	    std::make_shared<Matrix3>(Matrix3{{5.720460e-07, 4.521164e-08, 0.0},
	                                      {4.521164e-08, 5.915531e-07, 0.0},
	                                      {0.0, 0.0, std::pow(heading_sigma, 2.0)}});
	manual_heading_test_base(rpy, monte_1000_res);
}

TEST_F(AlignmentTests, DISABLED_manual_heading_monte_carlo_cov4) {
	rpy = Vector3{-PI, 0, PI / 4.0};
	auto monte_1000_res =
	    std::make_shared<Matrix3>(Matrix3{{5.915542e-07, -4.521149e-08, 0.0},
	                                      {-4.521149e-08, 5.720447e-07, 0.0},
	                                      {0.0, 0.0, std::pow(heading_sigma, 2.0)}});
	manual_heading_test_base(rpy, monte_1000_res);
}

TEST_F(AlignmentTests, manual_heading_imu_errors_SLOW) {
	rpy          = Vector3{0, 0, 0};
	ImuModel mod = stim300_model();
	auto align   = std::make_shared<ManualHeadingAlignment>(rpy[2], heading_sigma, mod, 30.0);
	auto pva     = create_pva(timestamp, llh, zeros(3), rpy, zeros(9, 9));
	auto imu     = std::make_shared<MeasurementImu>(navtk::testing::stationary_imu(pva, dt));

	// Add a large bias to the z-axis accelerometer.
	Vector3 expected_biases{0, 0, 1e-1};
	imu->set_meas_accel(imu->get_meas_accel() + expected_biases * dt);

	for (navtk::Size k = 0; k < 1500; k++) {
		auto new_time = imu->get_time_of_validity() + dt;
		auto ts       = new_time;
		imu           = create_imu(ts, imu->get_meas_accel(), imu->get_meas_gyro());

		auto res = align->process(imu);

		if (k % 50 == 0) {
			auto pos = create_pos(ts, llh, navtk::eye(3));
			align->process(pos);
		}
		if (res == AlignBase::AlignmentStatus::ALIGNED_GOOD) {
			break;
		}
	}

	auto imu_errors       = align->get_imu_errors();
	Vector3 actual_biases = imu_errors.second.accel_biases;
	ASSERT_ALLCLOSE_EX(expected_biases, actual_biases, 0, 1e-6);
}

// Process a measurement that should not result in alignment triggering
bool triggers_align(std::shared_ptr<AlignBase> a, std::shared_ptr<aspn_xtensor::AspnBase> m) {
	a->process(m);
	return a->get_computed_alignment().first;
}

TEST_F(AlignmentTests, manual_pos_variations) {
	auto good_d = create_pos(ts10, llh, navtk::eye(3));
	auto bad_d  = create_pos(ts10, llh, navtk::eye(3));

	// Waiting on a valid position
	auto ma = std::make_shared<ManualAlignment>(*pva, false, true, false, false);

	// Various invalid configurations
	bad_d->set_reference_frame(ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_ECI);
	ASSERT_FALSE(triggers_align(ma, bad_d));
	bad_d->set_reference_frame(good_d->get_reference_frame());

	bad_d->set_term1(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));
	bad_d->set_term1(good_d->get_term1());

	bad_d->set_term2(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));
	bad_d->set_term2(good_d->get_term2());

	bad_d->set_term3(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	bad_d->set_term1(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	bad_d->set_term2(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	ASSERT_TRUE(triggers_align(ma, good_d));
	Matrix exp_cov                                      = pva->get_covariance();
	xt::view(exp_cov, xt::range(0, 3), xt::range(0, 3)) = good_d->get_covariance();
	auto aligned                                        = ma->get_computed_alignment();
	auto acov                                           = ma->get_computed_covariance();
	auto act_cov = xt::view(acov.second, xt::range(0, 9), xt::range(0, 9));
	ASSERT_ALLCLOSE_EX(Csn, aligned.second.rot_mat, 1e-4, 0);
	ASSERT_ALLCLOSE(zeros(3), aligned.second.vel);
	ASSERT_ALLCLOSE(llh, aligned.second.pos);
	ASSERT_TRUE(acov.first);
	ASSERT_ALLCLOSE(exp_cov, act_cov);
	ASSERT_EQ(aligned.second.time, ts10);
}

TEST_F(AlignmentTests, manual_vel_variations) {
	Vector3 mv{1.0, 2.0, 3.0};
	Matrix ned_cov{{0.1, -0.2, 0.3}, {-0.2, 0.7, 0.8}, {0.3, 0.8, -1.0}};
	auto good_d = create_vel(ts10, mv, ned_cov);
	auto bad_d  = create_vel(ts10, mv, ned_cov);

	// Waiting on a valid velocity
	auto ma = std::make_shared<ManualAlignment>(*pva, false, false, true, false);

	// Various invalid configurations
	bad_d->set_reference_frame(ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_ECI);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	bad_d->set_reference_frame(ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_SENSOR);
	ASSERT_FALSE(triggers_align(ma, bad_d));
	bad_d->set_reference_frame(good_d->get_reference_frame());

	bad_d->set_x(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));
	bad_d->set_x(good_d->get_x());

	bad_d->set_y(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));
	bad_d->set_y(good_d->get_y());

	bad_d->set_z(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	bad_d->set_x(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	bad_d->set_y(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	ASSERT_TRUE(triggers_align(ma, good_d));
	Matrix exp_cov                                      = pva->get_covariance();
	xt::view(exp_cov, xt::range(3, 6), xt::range(3, 6)) = good_d->get_covariance();
	auto aligned                                        = ma->get_computed_alignment();
	auto acov                                           = ma->get_computed_covariance();
	auto act_cov = xt::view(acov.second, xt::range(0, 9), xt::range(0, 9));
	ASSERT_ALLCLOSE_EX(Csn, aligned.second.rot_mat, 1e-4, 0);
	ASSERT_ALLCLOSE(mv, aligned.second.vel);
	ASSERT_ALLCLOSE(llh, aligned.second.pos);
	ASSERT_TRUE(acov.first);
	ASSERT_ALLCLOSE(exp_cov, act_cov);
	ASSERT_EQ(aligned.second.time, ts10);

	auto C_ned_to_ecef = navtk::navutils::llh_to_cen(llh);
	auto ecef_vel      = navtk::dot(C_ned_to_ecef, mv);
	auto ecef_cov    = navtk::dot(C_ned_to_ecef, navtk::dot(ned_cov, xt::transpose(C_ned_to_ecef)));
	auto good_d_ecef = create_vel(ts10, ecef_vel, ecef_cov);
	good_d_ecef->set_reference_frame(ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_ECEF);

	// Provide ecef velocity, but no position yet so can't convert/align
	// We could store off vel input and convert later but...not yet
	ma = std::make_shared<ManualAlignment>(*pva, false, true, true, false);

	ASSERT_FALSE(triggers_align(ma, good_d_ecef));
	auto good_pos = create_pos(ts10, llh, navtk::eye(3));
	ASSERT_FALSE(triggers_align(ma, good_pos));

	// Now that pos is available, can interpret ecef vel
	ASSERT_TRUE(triggers_align(ma, good_d_ecef));
	aligned = ma->get_computed_alignment();
	acov    = ma->get_computed_covariance();
	act_cov = xt::view(acov.second, xt::range(0, 9), xt::range(0, 9));
	xt::view(exp_cov, xt::range(0, 3), xt::range(0, 3)) = good_pos->get_covariance();
	ASSERT_ALLCLOSE_EX(Csn, aligned.second.rot_mat, 1e-4, 0);
	ASSERT_ALLCLOSE(mv, aligned.second.vel);
	ASSERT_ALLCLOSE(llh, aligned.second.pos);
	ASSERT_TRUE(acov.first);
	ASSERT_ALLCLOSE(exp_cov, act_cov);
	ASSERT_EQ(aligned.second.time, ts10);
}

TEST_F(AlignmentTests, manual_att_variations) {
	Vector3 rpy{1.0, 2.0, 3.0};
	auto C_platform_to_ned = navtk::navutils::rpy_to_dcm(rpy);
	Matrix ned_cov{{0.1, -0.2, 0.3}, {-0.2, 0.7, 0.8}, {0.3, 0.8, -1.0}};

	auto good_d = create_att(ts10, rpy, ned_cov);
	auto bad_d  = create_att(ts10, rpy, ned_cov);

	// Waiting on a valid velocity
	auto ma = std::make_shared<ManualAlignment>(*pva, false, false, false, true);

	// Various invalid configurations
	bad_d->set_reference_frame(ASPN_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_ECI);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	ASSERT_TRUE(triggers_align(ma, good_d));
	Matrix exp_cov                                      = pva->get_covariance();
	xt::view(exp_cov, xt::range(6, 9), xt::range(6, 9)) = good_d->get_tilt_error_covariance();
	auto aligned                                        = ma->get_computed_alignment();
	auto acov                                           = ma->get_computed_covariance();
	auto act_cov = xt::view(acov.second, xt::range(0, 9), xt::range(0, 9));
	ASSERT_ALLCLOSE_EX(xt::transpose(C_platform_to_ned), aligned.second.rot_mat, 1e-4, 0);
	ASSERT_ALLCLOSE(zeros(3), aligned.second.vel);
	ASSERT_ALLCLOSE(llh, aligned.second.pos);
	ASSERT_TRUE(acov.first);
	ASSERT_ALLCLOSE(exp_cov, act_cov);
	ASSERT_EQ(aligned.second.time, ts10);

	auto C_ned_to_ecef      = navtk::navutils::llh_to_cen(llh);
	auto C_platform_to_ecef = navtk::dot(C_ned_to_ecef, C_platform_to_ned);
	auto ecef_rpy           = navtk::navutils::dcm_to_rpy(C_platform_to_ecef);
	auto ecef_cov = navtk::dot(C_ned_to_ecef, navtk::dot(ned_cov, xt::transpose(C_ned_to_ecef)));
	auto good_att_ecef = create_att(ts10, ecef_rpy, ecef_cov);
	good_att_ecef->set_reference_frame(ASPN_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_ECEF);

	ma = std::make_shared<ManualAlignment>(*pva, false, false, false, true);
	ASSERT_TRUE(triggers_align(ma, good_att_ecef));
	aligned = ma->get_computed_alignment();
	acov    = ma->get_computed_covariance();
	act_cov = xt::view(acov.second, xt::range(0, 9), xt::range(0, 9));
	ASSERT_ALLCLOSE_EX(xt::transpose(C_platform_to_ned), aligned.second.rot_mat, 1e-4, 0);
	ASSERT_ALLCLOSE(zeros(3), aligned.second.vel);
	ASSERT_ALLCLOSE(llh, aligned.second.pos);
	ASSERT_TRUE(acov.first);
	ASSERT_ALLCLOSE(exp_cov, act_cov);
	ASSERT_EQ(aligned.second.time, ts10);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, AlignmentTests, manual_pva_variations) {
	// Tests that piecemeal pva data triggers alignement as expected
	// Junk for repeated covariance views
	auto p_range  = xt::range(0, 3);
	auto v_range  = xt::range(3, 6);
	auto a_range  = xt::range(6, 9);
	auto pv_range = xt::range(0, 6);
	auto va_range = xt::range(3, 9);
	auto pa_range = xt::keep(std::vector<navtk::Size>{0, 1, 2, 6, 7, 8});
	Vector3 mv{1.0, 2.0, 3.0};
	Vector3 rpy{1.0, 2.0, 3.0};
	auto C_platform_to_ned = navtk::navutils::rpy_to_dcm(rpy);
	Matrix cov             = xt::diag(Vector{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9});
	// Mark off-diagonals with at least one identifiable element
	cov(0, 3)   = 4.4;
	cov(3, 0)   = 4.4;
	cov(0, 7)   = 3.4;
	cov(7, 0)   = 3.4;
	cov(3, 7)   = 1.4;
	cov(7, 3)   = 1.4;
	auto good_d = test.create_pva(test.ts10, test.llh, mv, rpy, cov);
	auto bad_d  = test.create_pva(test.ts10, test.llh, mv, rpy, cov);

	// Waiting on all data
	auto ma = std::make_shared<ManualAlignment>(*test.pva, false, true, true, true);

	// Unsupported frames
	bad_d->set_reference_frame(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_ECI);
	ASSERT_FALSE(triggers_align(ma, bad_d));
	bad_d->set_reference_frame(good_d->get_reference_frame());

	// Position, should be rejected because 9x9 cov doesn't jive w/ nan elements
	bad_d->set_p1(NAN);
	bool res = EXPECT_HONORS_MODE_EX(
	    triggers_align(ma, bad_d),
	    "PVA covariance \\(size 9, 9\\) and data elements \\(8\\) do not appear to agree.",
	    std::invalid_argument);
	ASSERT_FALSE(res);

	// Match cov shape but position rejected due to presence of NAN; vel and attitude should be
	// accepted. Apparently once you create the cov due to c-backing the size is fixed; ie can't use
	// set_cov as here to make it a valid shape. Not sure if that is intended behavior
	// bad_d->set_covariance(xt::view(cov, xt::range(1, 9), xt::range(1, 9)));
	bad_d = test.create_pva(
	    test.ts10, test.llh, mv, rpy, xt::view(cov, xt::range(1, 9), xt::range(1, 9)));
	bad_d->set_p1(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));
	bad_d->set_p1(good_d->get_p1());

	bad_d->set_p2(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));
	bad_d->set_p2(good_d->get_p2());

	bad_d->set_p3(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	bad_d = test.create_pva(
	    test.ts10, test.llh, mv, rpy, xt::view(cov, xt::range(2, 9), xt::range(2, 9)));
	bad_d->set_p1(NAN);
	bad_d->set_p3(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	bad_d = test.create_pva(
	    test.ts10, Vector3{NAN, NAN, NAN}, mv, rpy, xt::view(cov, va_range, va_range));
	bad_d->set_covariance(xt::view(cov, va_range, va_range));
	ASSERT_FALSE(triggers_align(ma, bad_d));

	// Now provide a meas with a valid position; as velocity and attitude already supplied this
	// should trigger alignment
	auto pos_d = test.create_pva(test.ts10,
	                             test.llh,
	                             Vector3{NAN, NAN, NAN},
	                             Vector3{NAN, NAN, NAN},
	                             xt::view(cov, p_range, p_range));
	ASSERT_TRUE(triggers_align(ma, pos_d));

	Matrix exp_cov = test.pva->get_covariance();
	// Since pos was provided separately from vel/att those cross terms will not be included
	xt::view(exp_cov, p_range, p_range)   = xt::view(cov, p_range, p_range);
	xt::view(exp_cov, va_range, va_range) = xt::view(cov, va_range, va_range);
	auto aligned                          = ma->get_computed_alignment();
	auto acov                             = ma->get_computed_covariance();
	auto act_cov                          = xt::view(acov.second, xt::range(0, 9), xt::range(0, 9));
	ASSERT_ALLCLOSE_EX(xt::transpose(C_platform_to_ned), aligned.second.rot_mat, 1e-4, 0);
	ASSERT_ALLCLOSE(mv, aligned.second.vel);
	ASSERT_ALLCLOSE(test.llh, aligned.second.pos);
	ASSERT_TRUE(acov.first);
	ASSERT_ALLCLOSE(exp_cov, act_cov);
	ASSERT_EQ(aligned.second.time, test.ts10);

	// Repeat above except velocity
	bad_d = test.create_pva(test.ts10, test.llh, mv, rpy, cov);
	ma    = std::make_shared<ManualAlignment>(*test.pva, false, true, true, true);

	bad_d->set_v1(NAN);
	res = EXPECT_HONORS_MODE_EX(
	    triggers_align(ma, bad_d),
	    "PVA covariance \\(size 9, 9\\) and data elements \\(8\\) do not appear to agree.",
	    std::invalid_argument);
	ASSERT_FALSE(res);

	// Match cov shape but vel rejected due to presence of NAN; pos and attitude should be accepted
	auto kp = xt::keep(std::vector<navtk::Size>{0, 1, 2, 4, 5, 6, 7, 8});
	bad_d   = test.create_pva(test.ts10, test.llh, mv, rpy, xt::view(cov, kp, kp));
	bad_d->set_v1(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));
	bad_d->set_v1(good_d->get_v1());

	bad_d->set_v2(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));
	bad_d->set_v2(good_d->get_v2());

	bad_d->set_v3(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	kp    = xt::keep(std::vector<navtk::Size>{0, 1, 2, 5, 6, 7, 8});
	bad_d = test.create_pva(test.ts10, test.llh, mv, rpy, xt::view(cov, kp, kp));
	bad_d->set_v1(NAN);
	bad_d->set_v3(NAN);
	ASSERT_FALSE(triggers_align(ma, bad_d));

	bad_d = test.create_pva(
	    test.ts10, test.llh, Vector3{NAN, NAN, NAN}, rpy, xt::view(cov, pa_range, pa_range));
	ASSERT_FALSE(triggers_align(ma, bad_d));

	// Now provide a meas with a valid velocity; as pos and attitude already supplied this should
	// trigger alignment
	auto vel_d = test.create_pva(test.ts10,
	                             Vector3{NAN, NAN, NAN},
	                             mv,
	                             Vector3{NAN, NAN, NAN},
	                             xt::view(cov, v_range, v_range));
	ASSERT_TRUE(triggers_align(ma, vel_d));

	exp_cov = test.pva->get_covariance();
	// Since vel was provided separately from pos/att those cross terms will not be included
	xt::view(exp_cov, v_range, v_range)   = xt::view(cov, v_range, v_range);
	xt::view(exp_cov, pa_range, pa_range) = xt::view(cov, pa_range, pa_range);
	aligned                               = ma->get_computed_alignment();
	acov                                  = ma->get_computed_covariance();
	act_cov                               = xt::view(acov.second, xt::range(0, 9), xt::range(0, 9));
	ASSERT_ALLCLOSE_EX(xt::transpose(C_platform_to_ned), aligned.second.rot_mat, 1e-4, 0);
	ASSERT_ALLCLOSE(mv, aligned.second.vel);
	ASSERT_ALLCLOSE(test.llh, aligned.second.pos);
	ASSERT_TRUE(acov.first);
	ASSERT_ALLCLOSE(exp_cov, act_cov);
	ASSERT_EQ(aligned.second.time, test.ts10);

	// Repeat above except attitude
	bad_d = test.create_pva(test.ts10, test.llh, mv, rpy, cov);
	ma    = std::make_shared<ManualAlignment>(*test.pva, false, true, true, true);

	// Cov size mismatch
	bad_d->set_quaternion(navtk::Vector4{NAN, 0.0, 0.0, 0.0});
	res = EXPECT_HONORS_MODE_EX(
	    triggers_align(ma, bad_d),
	    "PVA covariance \\(size 9, 9\\) and data elements \\(6\\) do not appear to agree.",
	    std::invalid_argument);
	ASSERT_FALSE(res);
	bad_d->set_quaternion(navtk::Vector4{1.0, NAN, 0.0, 0.0});
	res = EXPECT_HONORS_MODE_EX(
	    triggers_align(ma, bad_d),
	    "PVA covariance \\(size 9, 9\\) and data elements \\(6\\) do not appear to agree.",
	    std::invalid_argument);
	ASSERT_FALSE(res);
	bad_d->set_quaternion(navtk::Vector4{1.0, 0.0, NAN, 0.0});
	res = EXPECT_HONORS_MODE_EX(
	    triggers_align(ma, bad_d),
	    "PVA covariance \\(size 9, 9\\) and data elements \\(6\\) do not appear to agree.",
	    std::invalid_argument);
	ASSERT_FALSE(res);
	bad_d->set_quaternion(navtk::Vector4{1.0, 0.0, 0.0, NAN});
	res = EXPECT_HONORS_MODE_EX(
	    triggers_align(ma, bad_d),
	    "PVA covariance \\(size 9, 9\\) and data elements \\(6\\) do not appear to agree.",
	    std::invalid_argument);
	ASSERT_FALSE(res);
	bad_d->set_quaternion(navtk::Vector4{NAN, NAN, NAN, NAN});
	res = EXPECT_HONORS_MODE_EX(
	    triggers_align(ma, bad_d),
	    "PVA covariance \\(size 9, 9\\) and data elements \\(6\\) do not appear to agree.",
	    std::invalid_argument);
	ASSERT_FALSE(res);

	// Attitude is all or nothing, no 1/2d stuff. This should allow position and velocity to be
	// used.
	bad_d = test.create_pva(test.ts10, test.llh, mv, rpy, xt::view(cov, pv_range, pv_range));
	bad_d->set_quaternion(navtk::Vector4{1.0, 0.0, 0.0, NAN});
	ASSERT_FALSE(triggers_align(ma, bad_d));

	// Now provide a meas with a valid attitude; as pos and attitude already supplied this should
	// trigger alignment
	auto att_d = test.create_pva(test.ts10,
	                             Vector3{NAN, NAN, NAN},
	                             Vector3{NAN, NAN, NAN},
	                             rpy,
	                             xt::view(cov, a_range, a_range));
	ASSERT_TRUE(triggers_align(ma, att_d));
	exp_cov = test.pva->get_covariance();
	// Since att was provided separately from pos/vel those cross terms will not be included
	xt::view(exp_cov, a_range, a_range)   = xt::view(cov, a_range, a_range);
	xt::view(exp_cov, pv_range, pv_range) = xt::view(cov, pv_range, pv_range);
	aligned                               = ma->get_computed_alignment();
	acov                                  = ma->get_computed_covariance();
	act_cov                               = xt::view(acov.second, xt::range(0, 9), xt::range(0, 9));
	ASSERT_ALLCLOSE_EX(xt::transpose(C_platform_to_ned), aligned.second.rot_mat, 1e-4, 0);
	ASSERT_ALLCLOSE(mv, aligned.second.vel);
	ASSERT_ALLCLOSE(test.llh, aligned.second.pos);
	ASSERT_TRUE(acov.first);
	ASSERT_ALLCLOSE(exp_cov, act_cov);
	ASSERT_EQ(aligned.second.time, test.ts10);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, AlignmentTests, manual_pva_init) {
	// Tests that various invalid combinations of initial pva and 'wait_for' flags are handled
	Vector3 mv{1.0, 2.0, 3.0};
	Vector3 rpy{1.0, 2.0, 3.0};
	auto C_platform_to_ned = navtk::navutils::rpy_to_dcm(rpy);
	Matrix cov             = xt::diag(Vector{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9});

	auto d = test.create_pva(test.ts10, test.llh, mv, rpy, navtk::eye(8));
	EXPECT_HONORS_MODE_EX(
	    std::make_shared<ManualAlignment>(*d, false, true, true, true),
	    "PVA covariance \\(size 8, 8\\) and data elements \\(9\\) do not appear to agree.",
	    std::invalid_argument);

	d = test.create_pva(test.ts10, Vector3{0, 0, NAN}, mv, rpy, navtk::eye(8));
	EXPECT_HONORS_MODE_EX(std::make_shared<ManualAlignment>(*d, false, false, false, false),
	                      "Supplied alignment position had invalid elements",
	                      std::invalid_argument);
	EXPECT_NO_THROW(std::make_shared<ManualAlignment>(*d, false, true, false, false));

	d = test.create_pva(test.ts10, test.llh, Vector3{0, 0, NAN}, rpy, navtk::eye(8));
	EXPECT_HONORS_MODE_EX(std::make_shared<ManualAlignment>(*d, false, false, false, false),
	                      "Supplied alignment velocity had invalid elements",
	                      std::invalid_argument);
	EXPECT_NO_THROW(std::make_shared<ManualAlignment>(*d, false, false, true, false));

	d = test.create_pva(test.ts10, test.llh, mv, Vector3{0, 0, NAN}, navtk::eye(6));
	EXPECT_HONORS_MODE_EX(std::make_shared<ManualAlignment>(*d, false, false, false, false),
	                      "Supplied alignment attitude had invalid elements",
	                      std::invalid_argument);
	EXPECT_NO_THROW(std::make_shared<ManualAlignment>(*d, false, false, false, true));

	d = test.create_pva(test.ts10, test.llh, mv, rpy, navtk::eye(9));
	d->set_reference_frame(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_ECI);
	EXPECT_HONORS_MODE_EX(std::make_shared<ManualAlignment>(*d, false, true, true, true),
	                      "Invalid initial PVA reference frame.",
	                      std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, AlignmentTests, rejects_sampled) {
	auto imu = std::make_shared<MeasurementImu>(navtk::testing::stationary_imu(test.pva, 0.1));
	imu->set_imu_type(ASPN_MEASUREMENT_IMU_IMU_TYPE_SAMPLED);
	ManualHeadingAlignment align(0.0);
	EXPECT_HONORS_MODE_EX(align.process(imu),
	                      "Only ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED currently supported",
	                      std::invalid_argument);
	navtk::inertial::CoarseDynamicAlignment align2;
	EXPECT_HONORS_MODE_EX(align2.process(imu),
	                      "Only ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED currently supported",
	                      std::invalid_argument);
}
