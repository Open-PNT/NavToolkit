#include <gtest/gtest.h>
#include <tensor_assert.hpp>

#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/wgs84.hpp>
#include <navtk/tensors.hpp>

using navtk::dot;
using navtk::eye;
using navtk::Matrix3;
using navtk::Vector3;
using navtk::zeros;
using navtk::navutils::PI;

TEST(lat_lon_wander_to_C_n_to_e, invertible) {
	double lat    = 1.4;
	double lon    = -0.8;
	double wander = 0.3;
	double h      = 128398.0;
	Vector3 llw_exp{lat, lon, wander};
	Vector3 llh_exp{lat, lon, h};
	auto ecef_exp = navtk::navutils::llh_to_ecef(llh_exp);
	auto C_n_to_e = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat, lon, wander);
	auto llw      = navtk::navutils::C_n_to_e_to_lat_lon_wander(C_n_to_e);
	auto llh      = navtk::navutils::C_n_to_e_h_to_llh(C_n_to_e, h);
	auto ecef     = navtk::navutils::C_n_to_e_h_to_ecef(C_n_to_e, h);
	ASSERT_ALLCLOSE(llw_exp, llw);
	ASSERT_ALLCLOSE(llh_exp, llh);
	ASSERT_ALLCLOSE(ecef_exp, ecef);
}

TEST(wander_to_C_enu_to_n, confirm_docs) {

	auto C_enu_to_n0 = navtk::navutils::wander_to_C_enu_to_n(0.0);
	ASSERT_ALLCLOSE(C_enu_to_n0, eye(3));

	Matrix3 pi2_exp{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}};
	auto C_enu_to_n_pi2 = navtk::navutils::wander_to_C_enu_to_n(PI / 2.0);
	ASSERT_ALLCLOSE(C_enu_to_n_pi2, pi2_exp);
}

TEST(wander_to_C_ned_to_n, confirm_docs) {
	auto C_ned_to_n0 = navtk::navutils::wander_to_C_ned_to_n(0.0);
	Matrix3 C_enu_to_ned{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}};
	ASSERT_ALLCLOSE(C_ned_to_n0, C_enu_to_ned);
}

TEST(wander_to_C_ned_to_n, transpose_combo) {
	auto C_ned_to_n = navtk::navutils::wander_to_C_ned_to_n(1.2334);
	auto C_l_to_ned = xt::transpose(navtk::navutils::wander_to_C_ned_to_l(1.2334));
	auto C_l_to_n   = dot(C_ned_to_n, C_l_to_ned);
	Matrix3 C_enu_to_ned{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}};
	ASSERT_ALLCLOSE(C_l_to_n, C_enu_to_ned);
}

TEST(wander_to_C_ned_to_l, confirm_docs) {
	auto C_ned_to_l0 = navtk::navutils::wander_to_C_ned_to_l(0.0);
	ASSERT_ALLCLOSE(C_ned_to_l0, eye(3));
}

TEST(wander_to_C_ned_to_l, rpyToDcmEq) {
	// Since NED is the base frame for our rpy_to_dcm function, this
	// function should be equivalent to that one when the L frame =
	// our body/sensor frame and wander = -heading (heading is right
	// handed about down, while wander is right handed about up)
	auto C_ned_to_l = navtk::navutils::wander_to_C_ned_to_l(-1.321);
	auto rpy_eq     = xt::transpose(navtk::navutils::rpy_to_dcm({0, 0, 1.321}));
	ASSERT_ALLCLOSE(C_ned_to_l, rpy_eq);
}

TEST(lat_lon_wander_to_C_n_to_e, compare_with_old) {
	double lat    = 1.4;
	double lon    = -0.8;
	double wander = 0.4;
	Vector3 ll{lat, lon, 0.0};

	// Now have conflicting 'earth frame' definitions- older navutils use
	// X out equator/meridian, y 'left', z through north pole, while Savage
	// used Z out equator/meridian, Y out north pole, and X 'left'. This
	// DCM rotates from our previous definition to the Savage definition.
	Matrix3 C_e_to_E{{0, 1, 0}, {0, 0, 1}, {1, 0, 0}};

	// NED to old Earth frame then to Savage Earth frame
	auto C_n_to_e = navtk::navutils::llh_to_cen(ll);
	auto C_n_to_E = dot(navtk::navutils::C_ecef_to_e(), C_n_to_e);

	// Savage N(enu frame w/ wander offset) to Savage Earth
	auto C_N_to_E = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat, lon, wander);

	// Get NED frame to Savage Earth using 'his' eq
	auto C_ned_to_N     = navtk::navutils::wander_to_C_ned_to_n(wander);
	auto C_ned_to_E_new = dot(C_N_to_E, C_ned_to_N);

	ASSERT_ALLCLOSE(C_n_to_E, C_ned_to_E_new);
}

void wander_testing(const Vector3& input, const Vector3& exp) {
	auto C_N_to_E = navtk::navutils::lat_lon_wander_to_C_n_to_e(input[0], input[1], input[2]);
	// This function kicks to C_NtoE_to_wander, so getting it as well
	auto output = navtk::navutils::C_n_to_e_to_lat_lon_wander(C_N_to_E);
	ASSERT_ALLCLOSE(exp, output);
}

TEST(C_n_to_e_to_wander, extract_wander) {
	double lat    = 1.2;
	double lon    = -0.6;
	double wander = 0.4;

	wander_testing(Vector3{lat, lon, wander}, Vector3{lat, lon, wander});
	wander_testing(Vector3{PI / 2, lon, wander}, Vector3{PI / 2, 0.0, 0.0});
	wander_testing(Vector3{-PI / 2, lon, wander}, Vector3{-PI / 2, 0.0, 0.0});
	wander_testing(Vector3{PI / 2, 0.0, wander}, Vector3{PI / 2, 0.0, 0.0});
	wander_testing(Vector3{-PI / 2, 0.0, wander}, Vector3{-PI / 2, 0.0, 0.0});
	wander_testing(Vector3{PI / 2, lon, 0.0}, Vector3{PI / 2, 0.0, 0.0});
	wander_testing(Vector3{-PI / 2, lon, 0.0}, Vector3{-PI / 2, 0.0, 0.0});
	wander_testing(Vector3{PI / 2, 0.0, 0.0}, Vector3{PI / 2, 0.0, 0.0});
	wander_testing(Vector3{-PI / 2, 0.0, 0.0}, Vector3{-PI / 2, 0.0, 0.0});
	wander_testing(Vector3{lat, 0.0, wander}, Vector3{lat, 0.0, wander});
	wander_testing(Vector3{lat, lon, 0.0}, Vector3{lat, lon, 0.0});
	wander_testing(Vector3{0.0, 0.0, wander}, Vector3{0.0, 0.0, wander});
	wander_testing(Vector3{0.0, lon, 0.0}, Vector3{0.0, lon, 0.0});
	wander_testing(Vector3{lat, 0.0, 0.0}, Vector3{lat, 0.0, 0.0});
}

TEST(ecef_wander_to_C_n_to_e_h, verify_docs) {
	// Verify that the ECEF position and the E frame referenced in the
	// function return value are in fact not the same frame, which is a
	// little weird
	Vector3 ecef_pos{1000000, 2000000, 3000000};

	// Get old Cen dcm, and rotate so that it references Savage N frame
	// rather than NED. We'll leave wander as 0 for simplicity (which makes N = ENU)
	auto C_ned_to_ecef = navtk::navutils::ecef_to_cen(ecef_pos);
	Matrix3 C_enu_to_ned{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}};
	auto C_n_to_ecef = dot(C_ned_to_ecef, C_enu_to_ned);

	auto pr       = navtk::navutils::ecef_wander_to_C_n_to_e_h(ecef_pos, 0.0);
	auto C_n_to_e = pr.first;

	// Now have to rotate to align the earth frames
	ASSERT_ALLCLOSE(dot(navtk::navutils::C_ecef_to_e(), C_n_to_ecef), C_n_to_e);
}

TEST(ecef_wander_to_C_n_to_e_h, various_heights) {
	Vector3 ecef_pos{1000000, 2000000, 3000000};
	Vector3 ecef_pole{0, 0, navtk::navutils::SEMI_MINOR_RADIUS};
	Vector3 ecef_eq1{navtk::navutils::SEMI_MAJOR_RADIUS, 0, 0};
	Vector3 ecef_eq2{0, navtk::navutils::SEMI_MAJOR_RADIUS, 0};
	Vector3 ecef_should_be_1000{0, 0, navtk::navutils::SEMI_MINOR_RADIUS + 1000.0};

	// Wander angle should have no effect on heights
	double wander = 0.7534;
	auto pr1      = navtk::navutils::ecef_wander_to_C_n_to_e_h(ecef_pos, wander);
	auto pr2      = navtk::navutils::ecef_wander_to_C_n_to_e_h(ecef_pole, wander);
	auto pr3      = navtk::navutils::ecef_wander_to_C_n_to_e_h(ecef_eq1, wander);
	auto pr4      = navtk::navutils::ecef_wander_to_C_n_to_e_h(ecef_eq2, wander);
	auto pr5      = navtk::navutils::ecef_wander_to_C_n_to_e_h(ecef_should_be_1000, wander);

	auto llh = navtk::navutils::ecef_to_llh(ecef_pos);
	ASSERT_EQ(pr1.second, llh[2]);

	// Polar radius derived values have a little squishiness (about 4e-5)
	// which is acceptable, since ecef_to_llh iteratively drills down to
	// a 1e-4 threshold and stops
	ASSERT_TRUE(std::abs(pr2.second) < 1e-4);
	ASSERT_EQ(pr3.second, 0.0);
	ASSERT_EQ(pr4.second, 0.0);
	ASSERT_TRUE(std::abs(pr5.second - 1000) < 1e-4);
}

TEST(ecef_wander_to_C_n_to_e_h, dcm_at_pole_eq) {
	Vector3 ecef_pole{0, 0, navtk::navutils::SEMI_MINOR_RADIUS};
	Vector3 ecef_eq1{navtk::navutils::SEMI_MAJOR_RADIUS, 0, 0};
	Vector3 ecef_eq2{0, navtk::navutils::SEMI_MAJOR_RADIUS, 0};
	Vector3 ecef_should_be_1000{0, 0, navtk::navutils::SEMI_MINOR_RADIUS + 1000.0};

	Matrix3 exp_ecef_x = eye(3);
	Matrix3 exp_ecef_y{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
	// This one is tricky, but with wander as 0 it should be as the ECEFX version 'slid up' the
	// meridian
	Matrix3 exp_ecef_z{{1, 0, 0}, {0, 0, 1}, {0, -1, 0}};
	auto pr1 = navtk::navutils::ecef_wander_to_C_n_to_e_h(ecef_pole, 0.0);
	auto pr2 = navtk::navutils::ecef_wander_to_C_n_to_e_h(ecef_eq1, 0.0);
	auto pr3 = navtk::navutils::ecef_wander_to_C_n_to_e_h(ecef_eq2, 0.0);
	auto pr4 = navtk::navutils::ecef_wander_to_C_n_to_e_h(ecef_should_be_1000, 0.0);

	ASSERT_ALLCLOSE(pr1.first, exp_ecef_z);
	ASSERT_ALLCLOSE(pr2.first, exp_ecef_x);
	ASSERT_ALLCLOSE(pr3.first, exp_ecef_y);
	ASSERT_ALLCLOSE(pr4.first, exp_ecef_z);

	// And just for fun, a non 0 wander
	auto pr5 = navtk::navutils::ecef_wander_to_C_n_to_e_h(ecef_eq1, PI / 2);
	Matrix3 exp{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
	ASSERT_ALLCLOSE(pr5.first, exp);
}
