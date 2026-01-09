#include <memory>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <equality_checks.hpp>
#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/inertial/BasicInsAndFilter.hpp>
#include <navtk/inertial/ImuErrors.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::MeasurementImu;
using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::Matrix;
using navtk::Matrix3;
using navtk::Vector;
using navtk::Vector3;
using navtk::filtering::testing::verify_sol;
using navtk::inertial::BasicInsAndFilter;
using navtk::inertial::ImuErrors;
using navtk::utils::to_navsolution;

namespace {
MeasurementImu imu_pack(aspn_xtensor::TypeTimestamp t, const Vector3& dv, const Vector3& dth) {
	TypeTimestamp ts(t);
	TypeHeader head(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0);
	return MeasurementImu(head,
	                      ts,
	                      ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED,
	                      dv,
	                      dth,
	                      std::vector<aspn_xtensor::TypeIntegrity>{});
}
}  // namespace

struct BasicInsAndFilterTests : public ::testing::Test {
	Vector3 dv{1e-3, 2e-4, 3e-5};
	Vector3 dth{1e-6, 2e-5, 3e-6};
	aspn_xtensor::TypeTimestamp start_time = to_type_timestamp(3, 500000000);
	Matrix init_pva_cov = xt::diag(Vector{1.0, 1.0, 1.0, 0.2, 0.2, 0.2, 1e-4, 1e-4, 1e-4});
	MeasurementPositionVelocityAttitude pva{
	    TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0),
	    start_time,
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
	    0.123,
	    -0.987,
	    555.0,
	    2.0,
	    -3.0,
	    -1.2,
	    navtk::navutils::rpy_to_quat(Vector3{0.12, 0.5, -1.2}),
	    init_pva_cov,
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
	    {},
	    {}};
	BasicInsAndFilterTests() : ::testing::Test() {}

	void check_post_init(BasicInsAndFilter& baif,
	                     const navtk::filtering::ImuModel& mod,
	                     const ImuErrors& exp_errs) {
		auto def     = baif.calc_pva();
		auto at_time = baif.calc_pva(start_time);
		ASSERT_TRUE(def != nullptr);
		ASSERT_TRUE(at_time != nullptr);
		verify_sol(to_navsolution(*def), to_navsolution(*at_time));

		auto late  = baif.calc_pva(to_type_timestamp(100, 0));
		auto early = baif.calc_pva(to_type_timestamp(0, 0));
		ASSERT_TRUE(late == nullptr);
		ASSERT_TRUE(early == nullptr);

		auto cov = baif.get_pinson15_cov();
		ASSERT_ALLCLOSE(xt::view(cov, xt::range(0, 9), xt::range(0, 9)), init_pva_cov);
		Matrix imu_err_cov = xt::diag(xt::pow(Vector{mod.accel_bias_initial_sigma[0],
		                                             mod.accel_bias_initial_sigma[1],
		                                             mod.accel_bias_initial_sigma[2],
		                                             mod.gyro_bias_initial_sigma[0],
		                                             mod.gyro_bias_initial_sigma[1],
		                                             mod.gyro_bias_initial_sigma[2]},
		                                      2.0));
		ASSERT_ALLCLOSE(xt::view(cov, xt::range(9, 15), xt::range(9, 15)), imu_err_cov);

		auto imu_errs = baif.calc_imu_errors();
		ASSERT_ALLCLOSE(imu_errs.accel_biases, exp_errs.accel_biases);
		ASSERT_ALLCLOSE(imu_errs.gyro_biases, exp_errs.gyro_biases);
		ASSERT_ALLCLOSE(imu_errs.accel_scale_factors, exp_errs.accel_scale_factors);
		ASSERT_ALLCLOSE(imu_errs.gyro_scale_factors, exp_errs.gyro_scale_factors);
	}
};

TEST_F(BasicInsAndFilterTests, constructor_with_imu) {
	auto mod  = navtk::filtering::stim300_model();
	auto errs = ImuErrors{};
	auto imu  = std::make_shared<MeasurementImu>(imu_pack(pva.get_time_of_validity(), dv, dth));
	auto baif = BasicInsAndFilter(pva, mod, imu, errs, 0.01);
	check_post_init(baif, mod, errs);
}

TEST_F(BasicInsAndFilterTests, all_defaults) {
	auto mod  = navtk::filtering::stim300_model();
	auto errs = ImuErrors{};
	auto imu  = std::make_shared<MeasurementImu>(imu_pack(pva.get_time_of_validity(), dv, dth));
	auto baif = BasicInsAndFilter(pva);
	check_post_init(baif, mod, errs);
}

TEST_F(BasicInsAndFilterTests, with_imu_errs) {
	auto mod  = navtk::filtering::stim300_model();
	auto errs = ImuErrors{Vector3{0.1, 0.2, 0.3},
	                      Vector3{0.4, 0.5, 0.6},
	                      Vector3{0.7, 0.8, 0.9},
	                      Vector3{1.0, 1.1, 1.2}};
	auto imu  = std::make_shared<MeasurementImu>(imu_pack(pva.get_time_of_validity(), dv, dth));
	auto baif = BasicInsAndFilter(pva, mod, nullptr, errs);
	check_post_init(baif, mod, errs);
}

TEST_F(BasicInsAndFilterTests, alt_model) {
	auto mod  = navtk::filtering::hg1700_model();
	auto errs = ImuErrors{};
	auto imu  = std::make_shared<MeasurementImu>(imu_pack(pva.get_time_of_validity(), dv, dth));
	auto baif = BasicInsAndFilter(pva, mod, nullptr, errs);
	check_post_init(baif, mod, errs);
}

TEST_F(BasicInsAndFilterTests, test_mech_update_SLOW) {
	// An adapted version of BufferedImuTests.MechAtTime
	auto mod  = navtk::filtering::stim300_model();
	auto errs = ImuErrors{};
	auto imu  = std::make_shared<MeasurementImu>(imu_pack(pva.get_time_of_validity(), dv, dth));
	navtk::inertial::MechanizationOptions options{
	    navtk::navutils::GravModels::TITTERTON,
	    navtk::inertial::EarthModels::ELLIPTICAL,
	    navtk::inertial::DcmIntegrationMethods::SIXTH_ORDER,
	    navtk::inertial::IntegrationMethods::SIMPSONS_RULE};
	auto baif = BasicInsAndFilter(pva, mod, imu, errs, 0.02, options);

	for (navtk::Size k = 0; k < 99; k++) {
		baif.mechanize(imu_pack(start_time + (k + 1) * 0.02, dv, dth));
	}

	auto exp_out = navtk::filtering::NavSolution(
	    Vector3{0.12300063271845, -0.98700094984638, 538.4405566448743},
	    Vector3{2.05051724784581, -3.07384916791962, 18.12121135559916},
	    Matrix3{{0.31817279874302, -0.81694723126002, -0.48100238198643},
	            {0.94603954373241, 0.30648541455158, 0.10524197053321},
	            {0.06144307800396, -0.48853240629843, 0.87037970803648}},
	    start_time + 1.98);
	auto sol = baif.calc_pva();
	ASSERT_TRUE(sol != nullptr);
	verify_sol(exp_out, to_navsolution(*sol));

	// Do an update with ultra-accurate pos, verify tracks
	double new_pos1(0.1235);
	double new_pos2(-0.9875);
	double new_pos3(500.0);
	baif.update(MeasurementPosition(TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0),
	                                sol->get_time_of_validity(),
	                                ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
	                                new_pos1,
	                                new_pos2,
	                                new_pos3,
	                                navtk::eye(3) * 1e-10,
	                                ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
	                                {},
	                                {}));

	auto new_sol = baif.calc_pva();
	ASSERT_TRUE(new_sol != nullptr);
	ASSERT_TRUE(std::abs(new_pos1 - sol->get_p1()) > std::abs(new_pos1 - new_sol->get_p1()));
	ASSERT_TRUE(std::abs(new_pos2 - sol->get_p2()) > std::abs(new_pos2 - new_sol->get_p2()));
	ASSERT_TRUE(std::abs(new_pos3 - sol->get_p3()) > std::abs(new_pos3 - new_sol->get_p3()));
}
