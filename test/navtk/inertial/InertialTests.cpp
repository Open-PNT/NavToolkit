#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/inertial/AidingAltData.hpp>
#include <navtk/inertial/Inertial.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/MechanizationStandard.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/inertial/WanderPosVelAtt.hpp>
#include <navtk/inertial/mechanization_standard.hpp>
#include <navtk/inertial/mechanization_wander.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using navtk::eye;
using navtk::Matrix;
using navtk::Matrix3;
using navtk::not_null;
using navtk::Vector;
using navtk::Vector3;
using navtk::zeros;
using navtk::inertial::AidingAltData;
using navtk::inertial::DcmIntegrationMethods;
using navtk::inertial::EarthModels;
using navtk::inertial::Inertial;
using navtk::inertial::InertialPosVelAtt;
using navtk::inertial::IntegrationMethods;
using navtk::inertial::mechanization_standard;
using navtk::inertial::mechanization_wander;
using navtk::inertial::MechanizationOptions;
using navtk::inertial::StandardPosVelAtt;
using navtk::inertial::WanderPosVelAtt;
using navtk::navutils::GravModels;


// Convenient declarations for the very ugly casting of overloaded
// mech functions in Inertial ctor
const auto FULL_MECH_CAST = static_cast<not_null<std::shared_ptr<InertialPosVelAtt>> (*)(
    const Vector3&,
    const Vector3&,
    double,
    const not_null<std::shared_ptr<InertialPosVelAtt>>,
    const not_null<std::shared_ptr<InertialPosVelAtt>>,
    const MechanizationOptions&,
    AidingAltData*)>(mechanization_standard);

const auto SAV_MECH_CAST = static_cast<not_null<std::shared_ptr<InertialPosVelAtt>> (*)(
    const Vector3&,
    const Vector3&,
    double,
    const not_null<std::shared_ptr<InertialPosVelAtt>>,
    const not_null<std::shared_ptr<InertialPosVelAtt>>,
    const MechanizationOptions&,
    AidingAltData*)>(mechanization_wander);

struct InertialTests : public ::testing::Test {
	// Kotlin test results, single mech
	// InertialNED with all default args except gravModel = Titterton
	Vector3 llh_out_single;
	Vector3 vel_out_single;
	Matrix3 c_n_to_s_out_single;

	// Kotlin test results, (1 until 100) loop
	Vector3 llh_out;
	Vector3 vel_out;
	Matrix3 c_n_to_s_out;

	// Kotlin test results, Simpson's Rule (1 until 100) loop
	Vector3 llh_out_simpsons;
	Vector3 vel_out_simpsons;
	Matrix3 c_n_to_s_out_simpsons;

	// Altitude aided standard mechanization results (1 until 100) loop
	Vector3 llh_out_aiding;
	Vector3 vel_out_aiding;
	Matrix3 c_n_to_s_out_aiding;

	double lat0                    = 0.123;
	double lon0                    = -0.987;
	double alt0                    = 555.0;
	double vn0                     = 2.0;
	double ve0                     = -3.0;
	double vd0                     = -1.2;
	double r0                      = 0.12;
	double p0                      = 0.5;
	double y0                      = -1.2;
	aspn_xtensor::TypeTimestamp t0 = aspn_xtensor::TypeTimestamp((int64_t)0);
	double dt                      = 0.02;

	double lat_aiding = 0.694286716020978;
	double lon_aiding = -1.46781279728985;
	double alt_aiding = 10000;
	double vn_aiding  = 254.142879269771;
	double ve_aiding  = 137.093255500711;
	double vd_aiding  = -0.30372999745227;
	double r_aiding   = -0.790601114640107;
	double p_aiding   = 0.00946273254199924;
	double y_aiding   = 0.441402705397983;
	double t_aiding   = 0.0;

	double wander0 = 0.4;

	Vector3 dv;
	Vector3 dth;
	Vector3 dv_aiding;
	Vector3 dth_aiding;

	Vector3 llh0;
	Vector3 vned0;
	Vector3 rpy0;
	Matrix3 c_s_to_ned;
	Vector3 llh_aiding;
	Vector3 vned_aiding;
	Vector3 rpy_aiding;
	Matrix3 c_s_to_ned_aiding;

	StandardPosVelAtt single;
	StandardPosVelAtt hunnert;
	StandardPosVelAtt simpsons;
	StandardPosVelAtt aiding;
	StandardPosVelAtt start;
	StandardPosVelAtt start_aiding;
	not_null<std::shared_ptr<StandardPosVelAtt>> start_ref;
	not_null<std::shared_ptr<StandardPosVelAtt>> aiding_ref;

	MechanizationOptions options;
	MechanizationOptions options2;
	MechanizationOptions aiding_options;

	Inertial def_ins;

	InertialTests()
	    : ::testing::Test(),
	      llh_out_single({0.12300000631299, -0.98700000947867, 555.0220483588777}),
	      vel_out_single({2.00051011578071, -3.00077404344239, -1.00483588776564}),
	      c_n_to_s_out_single({{0.31800060926582, -0.81793122263078, -0.47944147458504},
	                           {0.94613233763856, 0.3062615913387, 0.10505920876002},
	                           {0.06090330189048, -0.48702397550469, 0.87126255233566}}),

	      // Kotlin test results, (1 until 100) loop
	      llh_out({0.12300063279791, -0.98700094996267, 538.2479946985551}),
	      vel_out({2.05051724784607, -3.07384916786599, 18.12121193416597}),
	      c_n_to_s_out({{0.31817279874302, -0.81694723126001, -0.48100238198645},
	                    {0.94603954373241, 0.30648541455159, 0.1052419705332},
	                    {0.06144307800397, -0.48853240629844, 0.87037970803647}}),

	      // Kotlin test results, Simpson's Rule (1 until 100) loop
	      llh_out_simpsons({0.12300063271845, -0.98700094984638, 538.4405566448743}),
	      vel_out_simpsons({2.05051724784581, -3.07384916791962, 18.12121135559916}),
	      c_n_to_s_out_simpsons({{0.31817279874302, -0.81694723126002, -0.48100238198643},
	                             {0.94603954373241, 0.30648541455158, 0.10524197053321},
	                             {0.06144307800396, -0.48853240629843, 0.87037970803648}}),

	      // Altitude aided standard mechanization results (1 until 100) loop
	      llh_out_aiding({6.9436569209925048e-01, -1.4677575798917921e+00, 1.0000586366534133e+04}),
	      vel_out_aiding({254.1420298115993, 137.09291864237875, -0.3060444845216707}),
	      c_n_to_s_out_aiding({{0.9041241793039201, 0.4271646641097235, -0.0094772429556258},
	                           {-0.306578841005398, 0.6331268804375272, -0.7107459233194602},
	                           {-0.2976052463344687, 0.6455080967760763, 0.7033849688120497}}),

	      dv({1e-3, 2e-4, 3e-5}),
	      dth({1e-6, 2e-5, 3e-6}),

	      dv_aiding({0.0018708596761269, 0.138054848625319, -0.137525750779905}),
	      dth_aiding({1.05175371607249e-06, 2.3386238062626e-07, -1.9692250114017e-06}),

	      llh0({lat0, lon0, alt0}),
	      vned0({vn0, ve0, vd0}),
	      rpy0({r0, p0, y0}),
	      c_s_to_ned(navtk::navutils::rpy_to_dcm(rpy0)),
	      llh_aiding({lat_aiding, lon_aiding, alt_aiding}),
	      vned_aiding({vn_aiding, ve_aiding, vd_aiding}),
	      rpy_aiding({r_aiding, p_aiding, y_aiding}),
	      c_s_to_ned_aiding(navtk::navutils::rpy_to_dcm(rpy_aiding)),

	      single({aspn_xtensor::to_type_timestamp(),
	              llh_out_single,
	              vel_out_single,
	              xt::transpose(c_n_to_s_out_single)}),
	      hunnert(
	          {aspn_xtensor::to_type_timestamp(), llh_out, vel_out, xt::transpose(c_n_to_s_out)}),
	      simpsons({aspn_xtensor::to_type_timestamp(),
	                llh_out_simpsons,
	                vel_out_simpsons,
	                xt::transpose(c_n_to_s_out_simpsons)}),
	      aiding({aspn_xtensor::to_type_timestamp(),
	              llh_out_aiding,
	              vel_out_aiding,
	              xt::transpose(c_n_to_s_out_aiding)}),
	      start({aspn_xtensor::to_type_timestamp(), llh0, vned0, c_s_to_ned}),
	      start_aiding(
	          {aspn_xtensor::to_type_timestamp(), llh_aiding, vned_aiding, c_s_to_ned_aiding}),
	      start_ref(std::make_shared<StandardPosVelAtt>(start)),
	      aiding_ref(std::make_shared<StandardPosVelAtt>(start_aiding)),
	      options({GravModels::TITTERTON,
	               EarthModels::ELLIPTICAL,
	               DcmIntegrationMethods::SIXTH_ORDER,
	               IntegrationMethods::TRAPEZOIDAL}),
	      options2({GravModels::TITTERTON,
	                EarthModels::ELLIPTICAL,
	                DcmIntegrationMethods::SIXTH_ORDER,
	                IntegrationMethods::SIMPSONS_RULE}),
	      aiding_options({GravModels::TITTERTON,
	                      EarthModels::ELLIPTICAL,
	                      DcmIntegrationMethods::SIXTH_ORDER,
	                      IntegrationMethods::TRAPEZOIDAL}),

	      def_ins(Inertial(start_ref, options, FULL_MECH_CAST)) {}

	virtual void SetUp() override { def_ins.reset(start); }
};

void verify_sol(not_null<std::shared_ptr<InertialPosVelAtt>> sol1,
                not_null<std::shared_ptr<InertialPosVelAtt>> sol2,
                double llh_thresh = 0.0,
                double vel_thresh = 0.0,
                double dcm_thresh = 0.0) {
	navtk::Vector llh_diff  = abs(sol1->get_llh() - sol2->get_llh());
	navtk::Vector vned_diff = abs(sol1->get_vned() - sol2->get_vned());
	navtk::Matrix dcm_diff  = abs(sol1->get_C_s_to_ned() - sol2->get_C_s_to_ned());
	ASSERT_TRUE(xt::all(xt::less_equal(llh_diff, llh_thresh)));
	ASSERT_TRUE(xt::all(xt::less_equal(vned_diff, vel_thresh)));
	ASSERT_TRUE(xt::all(xt::less_equal(dcm_diff, dcm_thresh)));
}

TEST_F(InertialTests, hundred_mechs) {
	auto ins = Inertial(start_ref, options, FULL_MECH_CAST);

	for (navtk::Size k = 1; k < 100; k++) {
		ins.mechanize(t0 + dt * k, dv, dth);
	}

	auto sol = ins.get_solution();
	verify_sol(std::make_shared<StandardPosVelAtt>(hunnert), sol, 1e-12, 1e-12, 1e-12);
}

TEST_F(InertialTests, single_mech_comp) {
	auto ins = Inertial(start_ref, options, FULL_MECH_CAST);
	auto ins2 =
	    Inertial(std::make_shared<navtk::inertial::MechanizationStandard>(), start_ref, options);
	ins.mechanize(t0 + dt, dv, dth);
	ins2.mechanize(t0 + dt, dv, dth);
	verify_sol(ins.get_solution(), ins2.get_solution(), 1e-14, 1e-14, 1e-14);
}

TEST_F(InertialTests, hundred_mechs_simpsons) {
	auto ins = Inertial(start_ref, options2, FULL_MECH_CAST);

	for (navtk::Size k = 1; k < 100; k++) {
		ins.mechanize(t0 + dt * k, dv, dth);
	}

	auto sol = ins.get_solution();
	verify_sol(std::make_shared<StandardPosVelAtt>(simpsons), sol, 1e-12, 1e-12, 1e-12);
}

TEST_F(InertialTests, hundred_mechs_aiding) {
	auto ins = Inertial(aiding_ref, aiding_options, FULL_MECH_CAST);
	AidingAltData aiding_alt_data;
	aiding_alt_data.aiding_alt           = alt_aiding;
	aiding_alt_data.integrated_alt_error = 0.0;

	for (navtk::Size k = 1; k < 100; k++) {
		ins.mechanize(t0 + dt * k, dv_aiding, dth_aiding, &aiding_alt_data);
	}

	auto sol = ins.get_solution();
	verify_sol(std::make_shared<StandardPosVelAtt>(aiding), sol, 1e-12, 1e-12, 1e-12);
}

TEST_F(InertialTests, hundred_mechs_wander_aiding_SLOW) {
	auto ins        = Inertial(aiding_ref, aiding_options, SAV_MECH_CAST);
	auto C_ned_to_l = navtk::navutils::wander_to_C_ned_to_l(wander0);
	auto C_n_to_s   = xt::transpose(navtk::navutils::rpy_to_dcm(rpy_aiding));
	auto C_ned_to_n = navtk::navutils::wander_to_C_ned_to_n(wander0);
	auto C_s_to_l   = navtk::dot(C_ned_to_l, xt::transpose(C_n_to_s));
	auto vn         = navtk::dot(C_ned_to_n, vned_aiding);
	auto C_n_to_e   = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat_aiding, lon_aiding, wander0);
	auto reset_pva  = WanderPosVelAtt(t0, C_n_to_e, alt_aiding, vn, C_s_to_l);
	ins.reset(reset_pva);

	AidingAltData aiding_alt_data;
	aiding_alt_data.aiding_alt           = alt_aiding;
	aiding_alt_data.integrated_alt_error = 0.0;

	for (navtk::Size k = 1; k < 100; k++) {
		ins.mechanize(t0 + dt * k, dv_aiding, dth_aiding, &aiding_alt_data);
	}

	auto sol = ins.get_solution();
	verify_sol(std::make_shared<StandardPosVelAtt>(aiding), sol, 6e-4, 6e-4, 1e-8);
}

TEST_F(InertialTests, single_mech) {
	auto ins = Inertial(start_ref, options, FULL_MECH_CAST);
	ins.mechanize(t0 + dt, dv, dth);
	auto sol = ins.get_solution();
	verify_sol(std::make_shared<StandardPosVelAtt>(single), sol, 1e-12, 1e-12, 1e-12);
}

TEST_F(InertialTests, single_mech_savage) {

	auto ins        = Inertial(start_ref, options, SAV_MECH_CAST);
	auto C_ned_to_l = navtk::navutils::wander_to_C_ned_to_l(wander0);
	auto C_n_to_s   = xt::transpose(navtk::navutils::rpy_to_dcm(rpy0));
	auto C_ned_to_n = navtk::navutils::wander_to_C_ned_to_n(wander0);
	auto C_s_to_l   = navtk::dot(C_ned_to_l, xt::transpose(C_n_to_s));
	auto vn         = navtk::dot(C_ned_to_n, vned0);
	auto C_n_to_e   = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat0, lon0, wander0);
	auto reset_pva  = WanderPosVelAtt(t0, C_n_to_e, alt0, vn, C_s_to_l);
	ins.reset(reset_pva);
	ins.mechanize(t0 + dt, dv, dth);
	auto sol = ins.get_solution();

	// Differences in the Savage mechanization and Titterton are small
	// and oddly, approx. linear for dcm elements and vel, with
	// dcm element error being something like 4.7e-12 * num_iterations,
	// and vel 8.5e-9 * iterations, and pos (alt, at least) 8.5e-11 * num_iter^2
	verify_sol(std::make_shared<StandardPosVelAtt>(single), sol, 8.5e-11, 8.5e-9, 4.8e-12);
}

TEST_F(InertialTests, no_mech_savage) {

	auto ins        = Inertial(start_ref, options, SAV_MECH_CAST);
	auto C_ned_to_l = navtk::navutils::wander_to_C_ned_to_l(wander0);
	auto C_n_to_s   = xt::transpose(navtk::navutils::rpy_to_dcm(rpy0));
	auto C_ned_to_n = navtk::navutils::wander_to_C_ned_to_n(wander0);
	auto C_s_to_l   = navtk::dot(C_ned_to_l, xt::transpose(C_n_to_s));
	auto vn         = navtk::dot(C_ned_to_n, vned0);
	auto C_n_to_e   = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat0, lon0, wander0);
	auto reset_pva  = WanderPosVelAtt(t0, C_n_to_e, alt0, vn, C_s_to_l);
	ins.reset(reset_pva);
	auto sol = ins.get_solution();
	verify_sol(start_ref, sol, 1e-15, 1e-15, 1e-15);
}

TEST_F(InertialTests, hundred_mech_savage_standard_SLOW) {

	auto ins        = Inertial(start_ref, options, SAV_MECH_CAST);
	auto C_ned_to_l = navtk::navutils::wander_to_C_ned_to_l(wander0);
	auto C_n_to_s   = xt::transpose(navtk::navutils::rpy_to_dcm(rpy0));
	auto C_ned_to_n = navtk::navutils::wander_to_C_ned_to_n(wander0);
	auto C_s_to_l   = navtk::dot(C_ned_to_l, xt::transpose(C_n_to_s));
	auto vn         = navtk::dot(C_ned_to_n, vned0);
	auto C_n_to_e   = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat0, lon0, wander0);
	auto reset_pva  = WanderPosVelAtt(t0, C_n_to_e, alt0, vn, C_s_to_l);
	ins.reset(reset_pva);

	for (navtk::Size k = 1; k < 100; k++) {
		ins.mechanize(t0 + dt * k, dv, dth);
	}

	auto sol = ins.get_solution();
	verify_sol(std::make_shared<StandardPosVelAtt>(hunnert),
	           sol,
	           8.5e-11 * pow(100, 2.0),
	           8.5e-9 * 100,
	           4.8e-12 * 100);
}

TEST_F(InertialTests, hundred_mech_savage_standard_expm_SLOW) {
	MechanizationOptions opt{GravModels::TITTERTON,
	                         EarthModels::ELLIPTICAL,
	                         DcmIntegrationMethods::EXPONENTIAL,
	                         IntegrationMethods::TRAPEZOIDAL};
	auto ins        = Inertial(start_ref, opt, SAV_MECH_CAST);
	auto C_ned_to_l = navtk::navutils::wander_to_C_ned_to_l(wander0);
	auto C_n_to_s   = xt::transpose(navtk::navutils::rpy_to_dcm(rpy0));
	auto C_ned_to_n = navtk::navutils::wander_to_C_ned_to_n(wander0);
	auto C_s_to_l   = navtk::dot(C_ned_to_l, xt::transpose(C_n_to_s));
	auto vn         = navtk::dot(C_ned_to_n, vned0);
	auto C_n_to_e   = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat0, lon0, wander0);
	auto reset_pva  = WanderPosVelAtt(t0, C_n_to_e, alt0, vn, C_s_to_l);
	ins.reset(reset_pva);

	for (navtk::Size k = 1; k < 100; k++) {
		ins.mechanize(t0 + dt * k, dv, dth);
	}

	auto sol = ins.get_solution();
	verify_sol(std::make_shared<StandardPosVelAtt>(hunnert),
	           sol,
	           8.5e-11 * pow(100, 2.0),
	           8.5e-9 * 100,
	           4.8e-12 * 100);
}

TEST_F(InertialTests, tiny_dt) {
	const auto sec     = 1'234'567'890;
	const auto nsec    = 123'456'789;
	const auto tiny_dt = 1e-9;

	// Convert time to a double, a lossy conversion.
	start_ref->time_validity = aspn_xtensor::to_type_timestamp(sec, nsec);
	auto ins                 = Inertial(start_ref, options, FULL_MECH_CAST);

	// Mechanize with a tiny change in time. If the addition is lossless this will result in a small
	// mechanization. If it's lossy, it will result in NaN's.
	ins.mechanize(aspn_xtensor::to_type_timestamp(sec, nsec) + tiny_dt, dv, dth);
	auto sol = ins.get_solution();

	// Verify that solution has been mechanized by no more than a tiny bit and is not full of NaN's.
	verify_sol(start_ref, sol, 1e-7, 1e-3, 1e-4);
}

TEST_F(InertialTests, diffs_after_10_min_SLOW) {

	auto ins = Inertial(start_ref, options, SAV_MECH_CAST);
	auto ins_w_class =
	    Inertial(std::make_shared<navtk::inertial::MechanizationStandard>(), start_ref, options);
	auto C_ned_to_l = navtk::navutils::wander_to_C_ned_to_l(wander0);
	auto C_n_to_s   = xt::transpose(navtk::navutils::rpy_to_dcm(rpy0));
	auto C_ned_to_n = navtk::navutils::wander_to_C_ned_to_n(wander0);
	auto C_s_to_l   = navtk::dot(C_ned_to_l, xt::transpose(C_n_to_s));
	auto vn         = navtk::dot(C_ned_to_n, vned0);
	auto C_n_to_e   = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat0, lon0, wander0);

	auto reset_pva = WanderPosVelAtt(t0, C_n_to_e, alt0, vn, C_s_to_l);
	ins.reset(reset_pva);

	for (navtk::Size k = 1; k < 3000; k++) {
		def_ins.mechanize(t0 + dt * k, dv, dth);
		ins.mechanize(t0 + dt * k, dv, dth);
		ins_w_class.mechanize(t0 + dt * k, dv, dth);
	}

	auto sol1 = ins.get_solution();
	auto sol2 = def_ins.get_solution();
	auto sol3 = ins_w_class.get_solution();
	verify_sol(sol1, sol2, 8.5e-11 * pow(3000, 2.0), 8.5e-9 * 3000, 4.8e-12 * 3000);
	verify_sol(sol2, sol3, 1e-10, 1e-12, 1e-12);
}

TEST_F(InertialTests, init_dcm_vs_rpy_SLOW) {
	auto ins = Inertial(start_ref, options, FULL_MECH_CAST);
	ins.reset(start);

	for (navtk::Size k = 1; k < 100; k++) {
		ins.mechanize(t0 + dt * k, dv, dth);
		def_ins.mechanize(t0 + dt * k, dv, dth);
	}

	auto sol1 = ins.get_solution();
	auto sol2 = def_ins.get_solution();
	verify_sol(sol1, sol2);
}

TEST_F(InertialTests, reset_all_SLOW) {
	for (navtk::Size k = 1; k < 100; k++) {
		def_ins.mechanize(t0 + dt * k, dv, dth);
	}
	auto sol1 = def_ins.get_solution();

	def_ins.reset(start);
	for (navtk::Size k = 1; k < 100; k++) {
		def_ins.mechanize(t0 + dt * k, dv, dth);
	}
	auto sol2 = def_ins.get_solution();
	verify_sol(sol1, sol2);
}

TEST_F(InertialTests, get_and_set_gyro) {
	auto ins = Inertial(start_ref, options, FULL_MECH_CAST);
	ins.reset(start);

	Vector3 scale{1e-1, 1e-2, 2e-1};
	Vector3 bias{3e-4, 4e-5, 5e-2};
	def_ins.set_gyro_biases(bias);
	def_ins.set_gyro_scale_factors(scale);

	def_ins.mechanize(t0 + dt, dv, dth);
	ins.mechanize(t0 + dt, dv, dth);

	auto sol1 = def_ins.get_solution();
	auto sol2 = ins.get_solution();
	ASSERT_TRUE(sol1->get_C_s_to_ned() != sol2->get_C_s_to_ned());
	ASSERT_TRUE(def_ins.get_gyro_biases() == bias);
	ASSERT_TRUE(def_ins.get_gyro_scale_factors() == scale);
}

TEST_F(InertialTests, get_and_set_accel) {
	auto ins = Inertial(start_ref, options, FULL_MECH_CAST);
	ins.reset(start);

	Vector3 scale{1e-1, 1e-2, 2e-1};
	Vector3 bias{3e-4, 4e-5, 5e-2};
	def_ins.set_accel_biases(bias);
	def_ins.set_accel_scale_factors(scale);

	def_ins.mechanize(t0 + dt, dv, dth);
	ins.mechanize(t0 + dt, dv, dth);

	auto sol1 = def_ins.get_solution();
	auto sol2 = ins.get_solution();
	ASSERT_TRUE(sol1->get_vned() != sol2->get_vned());
	ASSERT_TRUE(def_ins.get_accel_biases() == bias);
	ASSERT_TRUE(def_ins.get_accel_scale_factors() == scale);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, InertialTests, invalid_GravModel_throws) {
	auto bad_arg = static_cast<GravModels>(999);
	MechanizationOptions opt{};
	opt.grav_model = bad_arg;
	auto ins       = Inertial(test.start_ref, opt, FULL_MECH_CAST);
	ins.reset(test.start);
	EXPECT_HONORS_MODE_EX(ins.mechanize(test.t0 + test.dt, test.dv, test.dth),
	                      "Invalid GravModels enum value supplied.",
	                      std::invalid_argument);
}

TEST_F(InertialTests, diff_settings) {
	MechanizationOptions stm{GravModels::SCHWARTZ,
	                         EarthModels::SPHERICAL,
	                         DcmIntegrationMethods::FIRST_ORDER,
	                         IntegrationMethods::RECTANGULAR};

	const auto full_mech_cast_dup = static_cast<not_null<std::shared_ptr<InertialPosVelAtt>> (*)(
	    const Vector3&,
	    const Vector3&,
	    double,
	    const not_null<std::shared_ptr<InertialPosVelAtt>>,
	    const not_null<std::shared_ptr<InertialPosVelAtt>>,
	    const MechanizationOptions&,
	    AidingAltData*)>(mechanization_standard);

	auto ins0 = Inertial(start_ref, options, FULL_MECH_CAST);
	auto ins1 = Inertial(start_ref, stm, full_mech_cast_dup);
	ins0.mechanize(t0 + dt, dv, dth);
	ins1.mechanize(t0 + dt, dv, dth);
}

TEST_F(InertialTests, clone) {
	double rtol = 1e-14;
	double atol = 1e-14;

	auto ins = Inertial(start_ref, options, FULL_MECH_CAST);
	ins.set_accel_scale_factors({1e-2, 2e-2, 3e-2});
	ins.set_gyro_scale_factors({-5e-2, 6e-2, -7e-2});
	ins.set_accel_biases({-0.4, -0.5, -0.6});
	ins.set_gyro_biases({0.1, 0.2, 0.3});

	auto cln = Inertial(ins);

	ASSERT_ALLCLOSE_EX(ins.get_accel_scale_factors(), cln.get_accel_scale_factors(), rtol, atol);
	ASSERT_ALLCLOSE_EX(ins.get_gyro_scale_factors(), cln.get_gyro_scale_factors(), rtol, atol);
	ASSERT_ALLCLOSE_EX(ins.get_accel_biases(), cln.get_accel_biases(), rtol, atol);
	ASSERT_ALLCLOSE_EX(ins.get_gyro_biases(), cln.get_gyro_biases(), rtol, atol);

	auto sol1 = ins.get_solution();
	auto sol2 = cln.get_solution();

	ASSERT_EQ(sol1->is_wander_capable(), sol2->is_wander_capable());
	ASSERT_ALLCLOSE_EX(sol1->get_llh(), sol2->get_llh(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1->get_vned(), sol2->get_vned(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1->get_C_s_to_ned(), sol2->get_C_s_to_ned(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1->get_C_s_to_l(), sol2->get_C_s_to_l(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1->get_vn(), sol2->get_vn(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1->get_C_n_to_e_h().first, sol2->get_C_n_to_e_h().first, rtol, atol);

	ins.set_accel_scale_factors(zeros(3));
	ins.set_gyro_scale_factors(zeros(3));
	ins.set_accel_biases(zeros(3));
	ins.set_gyro_biases(zeros(3));

	ASSERT_FALSE(xt::allclose(
	    (Vector)ins.get_accel_scale_factors(), (Vector)cln.get_accel_scale_factors(), rtol, atol));
	ASSERT_FALSE(xt::allclose(
	    (Vector)ins.get_gyro_scale_factors(), (Vector)cln.get_gyro_scale_factors(), rtol, atol));
	ASSERT_FALSE(
	    xt::allclose((Vector)ins.get_accel_biases(), (Vector)cln.get_accel_biases(), rtol, atol));
	ASSERT_FALSE(
	    xt::allclose((Vector)ins.get_gyro_biases(), (Vector)cln.get_gyro_biases(), rtol, atol));

	cln.set_accel_scale_factors(zeros(3));
	cln.set_gyro_scale_factors(zeros(3));
	cln.set_accel_biases(zeros(3));
	cln.set_gyro_biases(zeros(3));

	ins.mechanize(t0 + dt, dv, dth);
	sol1 = ins.get_solution();

	ASSERT_FALSE(xt::allclose(sol1->get_llh(), (Vector)sol2->get_llh(), rtol, atol));
	ASSERT_FALSE(xt::allclose((Vector)sol1->get_vned(), (Vector)sol2->get_vned(), rtol, atol));
	ASSERT_FALSE(
	    xt::allclose((Matrix)sol1->get_C_s_to_ned(), (Matrix)sol2->get_C_s_to_ned(), rtol, atol));
	ASSERT_FALSE(
	    xt::allclose((Matrix)sol1->get_C_s_to_l(), (Matrix)sol2->get_C_s_to_l(), rtol, atol));
	ASSERT_FALSE(xt::allclose((Vector)sol1->get_vn(), (Vector)sol2->get_vn(), rtol, atol));
	ASSERT_FALSE(xt::allclose(
	    (Matrix)sol1->get_C_n_to_e_h().first, (Matrix)sol2->get_C_n_to_e_h().first, rtol, atol));

	cln.mechanize(t0 + dt, dv, dth);
	sol2 = cln.get_solution();

	ASSERT_ALLCLOSE_EX(sol1->get_llh(), sol2->get_llh(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1->get_vned(), sol2->get_vned(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1->get_C_s_to_ned(), sol2->get_C_s_to_ned(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1->get_C_s_to_l(), sol2->get_C_s_to_l(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1->get_vn(), sol2->get_vn(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1->get_C_n_to_e_h().first, sol2->get_C_n_to_e_h().first, rtol, atol);
}

TEST_F(InertialTests, copy_constructor_is_deep) {
	auto ins      = Inertial(start_ref, options, FULL_MECH_CAST);
	auto the_copy = ins;
	// If a true deep-copy occurred, these pointers will be different.
	ASSERT_NE(ins.get_solution(), the_copy.get_solution());
}

TEST_F(InertialTests, clone_tests) {
	auto cl = start.clone();
	verify_sol(start_ref, cl);
	auto wpa     = WanderPosVelAtt(aspn_xtensor::to_type_timestamp(12.4),
                               xt::transpose(navtk::navutils::rpy_to_dcm({0.2, 1.0, 3.0})),
                               101.0,
	                               {1.2, 3.3, -4.4},
                               xt::transpose(navtk::navutils::rpy_to_dcm({0.4, 0.6, 0.0})));
	auto w_clone = wpa.clone();
	verify_sol(std::make_shared<WanderPosVelAtt>(wpa), w_clone);
}

TEST_F(InertialTests, rule_5) {
	// Copy ctor
	Inertial copied = def_ins;
	ASSERT_NE(def_ins.get_solution(), copied.get_solution());
	verify_sol(def_ins.get_solution(), copied.get_solution());

	auto keep = def_ins.get_solution().get();

	// Move ctor
	auto moved = std::move(def_ins);
	EXPECT_UB_OR_DIE(def_ins.get_solution().get(), "", std::runtime_error);
	ASSERT_NE(nullptr, moved.get_solution().get());
	ASSERT_EQ(keep, moved.get_solution().get());

	// Move assign
	def_ins = std::move(moved);
	ASSERT_EQ(keep, def_ins.get_solution().get());
	EXPECT_UB_OR_DIE(moved.get_solution().get(), "", std::runtime_error);

	// Copy assign
	copied = def_ins;
	ASSERT_NE(keep, copied.get_solution().get());
	verify_sol(def_ins.get_solution(), copied.get_solution());
}
