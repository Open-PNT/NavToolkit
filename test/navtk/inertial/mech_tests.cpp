#include <stdexcept>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/inertial/Inertial.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/inertial/WanderPosVelAtt.hpp>
#include <navtk/inertial/mechanization_standard.hpp>
#include <navtk/inertial/mechanization_wander.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

using navtk::eye;
using navtk::Matrix3;
using navtk::Vector3;
using navtk::zeros;
using navtk::inertial::DcmIntegrationMethods;
using navtk::inertial::EarthModels;
using navtk::inertial::IntegrationMethods;
using navtk::inertial::mechanization_standard;
using navtk::inertial::mechanization_wander;
using navtk::inertial::MechanizationOptions;
using navtk::navutils::GravModels;

struct MechTests : public ::testing::Test {
	Matrix3 dummy_pos = navtk::navutils::lat_lon_wander_to_C_n_to_e(0.0, 0.0, 0.0);
	Matrix3 M         = eye(3);
	Vector3 v         = zeros(3);
	double d          = 0.0;


	GravModels grav_mod            = GravModels::TITTERTON;
	DcmIntegrationMethods dcm_meth = DcmIntegrationMethods::SIXTH_ORDER;
	IntegrationMethods i_meth      = IntegrationMethods::TRAPEZOIDAL;
	MechanizationOptions options{grav_mod, EarthModels::ELLIPTICAL, dcm_meth, i_meth};
};

ERROR_MODE_SENSITIVE_TEST(TEST_F, MechTests, cast_int_to_enum_fail_grav) {
	auto bad_grav_mod = static_cast<GravModels>(6);
	MechanizationOptions opt{bad_grav_mod, EarthModels::ELLIPTICAL, test.dcm_meth, test.i_meth};
	EXPECT_HONORS_MODE_EX(
	    mechanization_standard(test.v, test.v, 0.1, test.v, test.M, test.v, test.v, opt),
	    "Invalid GravModels enum value supplied.",
	    std::invalid_argument);
	EXPECT_HONORS_MODE_EX(
	    mechanization_wander(test.v, test.v, test.d, test.dummy_pos, test.d, test.v, test.M, opt),
	    "Unknown gravity model",
	    std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, MechTests, cast_int_to_enum_fail_dcm_meth) {
	auto bad_dcm_meth = static_cast<DcmIntegrationMethods>(6);
	MechanizationOptions opt{test.grav_mod, EarthModels::ELLIPTICAL, bad_dcm_meth, test.i_meth};
	EXPECT_HONORS_MODE_EX(
	    mechanization_standard(test.v, test.v, 0.1, test.v, test.M, test.v, test.v, opt),
	    "Unrecognized dcm_integration_method",
	    std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, MechTests, cast_int_to_enum_fail_int_meth) {
	auto bad_i_meth = static_cast<IntegrationMethods>(6);
	MechanizationOptions opt{test.grav_mod, EarthModels::ELLIPTICAL, test.dcm_meth, bad_i_meth};
	EXPECT_HONORS_MODE_EX(
	    mechanization_standard(test.v, test.v, 0.1, test.v, test.M, test.v, test.v, opt),
	    "Unrecognized integration_method",
	    std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, MechTests, cast_int_to_enum_fail_earth_mod) {
	auto bad_e = static_cast<EarthModels>(6);
	MechanizationOptions opt{test.grav_mod, bad_e, test.dcm_meth, test.i_meth};
	EXPECT_HONORS_MODE_EX(
	    mechanization_wander(test.v, test.v, test.d, test.dummy_pos, test.d, test.v, test.M, opt),
	    "Unrecognized earth_model",
	    std::invalid_argument);
}

TEST_F(MechTests, comparison) {
	auto mats  = mechanization_standard(v, v, 0.1, v, M, v, v, options);
	auto res   = navtk::inertial::StandardPosVelAtt(aspn_xtensor::TypeTimestamp((int64_t)0), mats);
	auto mats2 = mechanization_wander(v, v, 0.1, dummy_pos, d, v, M, options);
	auto res2  = navtk::inertial::WanderPosVelAtt(aspn_xtensor::TypeTimestamp((int64_t)0), mats2);
	ASSERT_ALLCLOSE(res.get_llh(), res2.get_llh());
	ASSERT_ALLCLOSE(res.get_C_n_to_e_h(0.0).first, res2.get_C_n_to_e_h().first);
	ASSERT_EQ(res.get_C_n_to_e_h(0.0).second, res2.get_C_n_to_e_h().second);
}

TEST_F(MechTests, earth_model_comp_eq) {
	// Short mechanization period should yield approx same results
	Vector3 non_zero_vel{1.0, 2.0, 3.0};
	auto mats = mechanization_wander(v, v, 0.1, dummy_pos, d, non_zero_vel, M, options);
	MechanizationOptions opt{grav_mod, EarthModels::SPHERICAL, dcm_meth, i_meth};
	auto mats2 = mechanization_wander(v, v, 0.1, dummy_pos, d, non_zero_vel, M, opt);

	auto res  = navtk::inertial::WanderPosVelAtt(aspn_xtensor::TypeTimestamp((int64_t)0), mats);
	auto res2 = navtk::inertial::WanderPosVelAtt(aspn_xtensor::TypeTimestamp((int64_t)0), mats2);

	ASSERT_ALLCLOSE(res.get_llh(), res2.get_llh());
	ASSERT_ALLCLOSE(res.get_C_n_to_e_h().first, res2.get_C_n_to_e_h().first);
	ASSERT_TRUE(std::abs(res.get_C_n_to_e_h().second - res2.get_C_n_to_e_h().second) < 1e-10);
}

TEST_F(MechTests, compare_vel_diff_grav_mod) {
	MechanizationOptions opt1{GravModels::TITTERTON, EarthModels::ELLIPTICAL, dcm_meth, i_meth};
	MechanizationOptions opt2{GravModels::SAVAGE, EarthModels::ELLIPTICAL, dcm_meth, i_meth};
	MechanizationOptions opt3{GravModels::SCHWARTZ, EarthModels::ELLIPTICAL, dcm_meth, i_meth};

	auto tw_tw   = mechanization_standard(v, v, 0.1, v, M, v, v, opt1);
	auto tw_sav  = mechanization_standard(v, v, 0.1, v, M, v, v, opt2);
	auto tw_sch  = mechanization_standard(v, v, 0.1, v, M, v, v, opt3);
	auto sav_tw  = mechanization_wander(v, v, 0.1, dummy_pos, d, v, M, opt1);
	auto sav_sav = mechanization_wander(v, v, 0.1, dummy_pos, d, v, M, opt2);
	auto sav_sch = mechanization_wander(v, v, 0.1, dummy_pos, d, v, M, opt3);

	// Only valid because of the 0 wander angle
	Matrix3 conv{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}};
	ASSERT_ALLCLOSE(std::get<1>(tw_tw), navtk::dot(conv, std::get<2>(sav_tw)));
	ASSERT_ALLCLOSE(std::get<1>(tw_sav), navtk::dot(conv, std::get<2>(sav_sav)));
	ASSERT_ALLCLOSE(std::get<1>(tw_sch), navtk::dot(conv, std::get<2>(sav_sch)));
	// Sav grav has a north component that the other models don't;
	// The diff at this location/dt value is large enough to bust
	// the ASSERT default thresh (this is ok)
	ASSERT_ALLCLOSE_EX(std::get<1>(tw_tw), std::get<1>(tw_sav), 0.0, 5e-6);
	ASSERT_ALLCLOSE(std::get<1>(tw_tw), std::get<1>(tw_sch));
}

TEST_F(MechTests, titterton_diff_int_methods) {
	auto grav    = navtk::navutils::calculate_gravity_schwartz(0.0, 0.0)[2];
	auto delta_d = 1.0 - grav * 0.1;
	// DV meas of approx 1 m/s change over dt of 0.1 (neglecting transport rate etc)
	Vector3 dv{1, 1, delta_d};
	MechanizationOptions opt1{
	    GravModels::TITTERTON, EarthModels::ELLIPTICAL, dcm_meth, IntegrationMethods::RECTANGULAR};
	MechanizationOptions opt2{
	    GravModels::TITTERTON, EarthModels::ELLIPTICAL, dcm_meth, IntegrationMethods::TRAPEZOIDAL};
	MechanizationOptions opt3{GravModels::TITTERTON,
	                          EarthModels::ELLIPTICAL,
	                          dcm_meth,
	                          IntegrationMethods::SIMPSONS_RULE};

	auto rec      = mechanization_standard(dv, v, 0.1, v, M, v, v, opt1);
	auto trap     = mechanization_standard(dv, v, 0.1, v, M, v, v, opt2);
	auto simpsons = mechanization_standard(dv, v, 0.1, v, M, v, v, opt3);

	ASSERT_ALLCLOSE(std::get<0>(rec), zeros(3));
	auto del_n = navtk::navutils::delta_lat_to_north(std::get<0>(trap)[0], 0.0, 0.0);
	auto del_e = navtk::navutils::delta_lon_to_east(std::get<0>(trap)[1], 0.0, 0.0);
	ASSERT_TRUE(std::abs(del_n - 0.05) < 1e-6);
	ASSERT_TRUE(std::abs(del_e - 0.05) < 1e-6);
	ASSERT_TRUE(std::abs(std::get<0>(trap)[2] + 0.05) < 1e-6);
}

TEST_F(MechTests, titterton_diff_dcm_methods) {
	Vector3 dth{1e-4, 2e-5, 2e-4};
	MechanizationOptions opt1{
	    GravModels::TITTERTON, EarthModels::ELLIPTICAL, DcmIntegrationMethods::FIRST_ORDER, i_meth};
	MechanizationOptions opt2{
	    GravModels::TITTERTON, EarthModels::ELLIPTICAL, DcmIntegrationMethods::SIXTH_ORDER, i_meth};

	auto rec  = mechanization_standard(v, dth, 0.1, v, M, v, v, opt1);
	auto trap = mechanization_standard(v, dth, 0.1, v, M, v, v, opt2);
	ASSERT_ALLCLOSE(std::get<2>(rec), std::get<2>(trap));
	ASSERT_TRUE(std::get<2>(rec) != std::get<2>(trap));
}

TEST_F(MechTests, mixed_doc_check) {
	auto pva = navtk::inertial::StandardPosVelAtt(aspn_xtensor::TypeTimestamp((int64_t)0),
	                                              Vector3{0.7, 1.4, 100.0},
	                                              Vector3{1.1, 2.2, 3.3},
	                                              navtk::navutils::rpy_to_dcm({0.3, -1.4, 2.1}));
	ASSERT_ALLCLOSE(pva.get_vn(),
	                navtk::dot(Matrix3{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}, pva.get_vned()));
	ASSERT_ALLCLOSE(pva.get_C_s_to_l(), pva.get_C_s_to_ned());
}

TEST_F(MechTests, wander_proper) {
	auto pva1 = navtk::inertial::StandardPosVelAtt();
	auto pva2 = navtk::inertial::WanderPosVelAtt();
	ASSERT_TRUE(!pva1.is_wander_capable());
	ASSERT_TRUE(pva2.is_wander_capable());
}
