#include <gtest/gtest.h>
#include <tensor_assert.hpp>

#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/wgs84.hpp>
#include <navtk/tensors.hpp>

using navtk::dot;
using navtk::eye;
using navtk::Matrix3;
using navtk::Vector3;
using navtk::zeros;
using navtk::navutils::calculate_gravity_savage_n;
using navtk::navutils::calculate_gravity_savage_ned;
using navtk::navutils::calculate_gravity_schwartz;
using navtk::navutils::calculate_gravity_titterton;
using navtk::navutils::PI;
using navtk::navutils::SEMI_MAJOR_RADIUS;
using std::abs;

/**
 * A short brief/rant on gravity modelling differences and testing parameters.
 *
 * Currently have 3 gravity models available. Of these, the 'Schwartz' model
 * is probably the most accurate for navigation use. It is based on GRS80
 * geoid (which is what WGS84 was originally based on)
 * Schwartz method uses parameters from the 1967 geoid.
 * Both of these models assume ellipsoid shape only (only even harmonic
 * terms included). This results in a gravity model that is symmetric about
 * the equator and invariant to longitude.
 * The Savage model is more accurate in  the sense that it includes
 * 'pear shaped' earth terms (odd harmonics), but less accurate in the sense
 * that decent values for these terms are hard to come by.
 * For a more direct comparison between models one can set the J3 term to 0.
 * http://articles.adsabs.harvard.edu/cgi-bin/nph-iarticle_query?1965IAUS...21...21K&classic=YES
 */
struct GravTests : public ::testing::Test {
	// TODO: Bring into a test at some point
	double grs80_lat45 = 9.806199203;
	// The 'Savage' gravity model is fundamentally different from the others
	// in the way it models the shape of the earth so we must allow for a
	// larger difference between it and the others. Models agree best at the equator
	// (order of e-6) and worst at poles (e-5). Setting the J3 term to 0
	// in the savage implementation makes the Savage and Schwartz models
	// nearly equivalent
	double savage_threshold = 1.2e-4;
};

void run_grav_test(
    double lat, double alt, double wander, double lon = 0.0, double savage_threshold = 1.2e-4) {
	// Values taken from https://nga-rescue.is4s.us/egm-readme.pdf.
	std::pair<double, double> max_ll{10.854 * PI / 180.0, 286.271 * PI};
	double max_def = 966.334e-5;

	std::pair<double, double> min_ll{35.896 * PI / 180.0, 74.771 * PI};
	double min_def = -385.543e-5;

	auto C_n_to_e = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat, lon, wander);

	auto C_ned_to_n = navtk::navutils::wander_to_C_ned_to_n(wander);

	auto r_n = navtk::navutils::meridian_radius(lat);
	auto r_e = navtk::navutils::transverse_radius(lat);
	auto r_0 = sqrt(r_n * r_e);

	auto t_v      = calculate_gravity_titterton(alt, lat, r_0);
	auto sc_v     = calculate_gravity_schwartz(alt, lat);
	auto savn_v   = calculate_gravity_savage_n(C_n_to_e, alt);
	auto savned_v = calculate_gravity_savage_ned(C_n_to_e, alt);

	// In all cases, the magnitudes along the vertical should be pretty
	// close (only small modelling differences).
	// Only the savage versions attempt to calculate deflections (which
	// should be small)

	// Savage models differ quite a bit (up to 0.06 m/s^2) from other 2 when latitude is non-zero
	// while schwartz vs titterton maintains agreement in < 10 micro-g range
	ASSERT_EQ(savned_v[2], -savn_v[2]);

	ASSERT_ALLCLOSE_EX(savn_v, dot(C_ned_to_n, savned_v), 0.0, 1e-14);

	ASSERT_TRUE(abs(t_v[2] - sc_v[2]) < 5e-5);
	ASSERT_TRUE(abs(t_v[2] + savn_v[2]) < savage_threshold);
	ASSERT_TRUE(abs(t_v[2] - savned_v[2]) < savage_threshold);
	ASSERT_TRUE(savn_v[0] <= max_def && savn_v[0] >= min_def);
	ASSERT_TRUE(savn_v[1] <= max_def && savn_v[0] >= min_def);
	ASSERT_TRUE(savned_v[0] <= max_def && savn_v[0] >= min_def);
	ASSERT_TRUE(savned_v[1] <= max_def && savn_v[0] >= min_def);
}

TEST_F(GravTests, all0) { run_grav_test(0.0, 0.0, 0.0, 0.0); }
TEST_F(GravTests, nPole) { run_grav_test(PI / 2.0, 0.0, 0.0, 0.0); }
TEST_F(GravTests, sPole) { run_grav_test(-PI / 2.0, 0.0, 0.0, 0.0); }
TEST_F(GravTests, nMidLat) { run_grav_test(PI / 3.0, 0.0, 0.0, 0.0); }
TEST_F(GravTests, sMidLat) { run_grav_test(-PI / 3.0, 0.0, 0.0, 0.0); }

TEST_F(GravTests, all0Alt) { run_grav_test(0.0, 1000.0, 0.0, 0.0); }
TEST_F(GravTests, nPoleAlt) { run_grav_test(PI / 2.0, 1000.0, 0.0, 0.0); }
TEST_F(GravTests, sPoleAlt) { run_grav_test(-PI / 2.0, 1000.0, 0.0, 0.0); }
TEST_F(GravTests, nMidLatAlt) { run_grav_test(PI / 3.0, 1000.0, 0.0, 0.0); }
TEST_F(GravTests, sMidLatAlt) { run_grav_test(-PI / 3.0, 1000.0, 0.0, 0.0); }

TEST_F(GravTests, all0NegAlt) { run_grav_test(0.0, -1000.0, 0.0, 0.0); }
TEST_F(GravTests, nPoleNegAlt) { run_grav_test(PI / 2.0, -1000.0, 0.0, 0.0); }
TEST_F(GravTests, sPoleNegAlt) { run_grav_test(-PI / 2.0, -1000.0, 0.0, 0.0); }
TEST_F(GravTests, nMidLatNegAlt) { run_grav_test(PI / 3.0, -1000.0, 0.0, 0.0); }
TEST_F(GravTests, sMidLatNegAlt) { run_grav_test(-PI / 3.0, -1000.0, 0.0, 0.0); }

TEST_F(GravTests, all0Wander) { run_grav_test(0.0, 0.0, 1.0, 0.0); }
TEST_F(GravTests, nPoleWander) { run_grav_test(PI / 2.0, 0.0, 1.0, 0.0); }
TEST_F(GravTests, sPoleWander) { run_grav_test(-PI / 2.0, 0.0, 1.0, 0.0); }
TEST_F(GravTests, nMidLatWander) { run_grav_test(PI / 3.0, 0.0, 1.0, 0.0); }
TEST_F(GravTests, sMidLatWander) { run_grav_test(-PI / 3.0, 0.0, 1.0, 0.0); }

TEST_F(GravTests, all0Lon) { run_grav_test(0.0, 0.0, 0.0, 1.0); }
TEST_F(GravTests, nPoleLon) { run_grav_test(PI / 2.0, 0.0, 0.0, 1.0); }
TEST_F(GravTests, sPoleLon) { run_grav_test(-PI / 2.0, 0.0, 0.0, 1.0); }
TEST_F(GravTests, nMidLatLon) { run_grav_test(PI / 3.0, 0.0, 0.0, 1.0); }
TEST_F(GravTests, sMidLatLon) { run_grav_test(-PI / 3.0, 0.0, 0.0, 1.0); }

TEST_F(GravTests, symmetric) {
	// First order models (Titterton, Schwartz) are symmetric about the equator
	double lat = PI / 3.0;
	double alt = 1000.0;

	auto r_n        = navtk::navutils::meridian_radius(lat);
	auto r_e        = navtk::navutils::transverse_radius(lat);
	auto r_0        = sqrt(r_n * r_e);
	auto t_d_plus   = calculate_gravity_titterton(alt, lat, r_0)[2];
	auto t_d_minus  = calculate_gravity_titterton(alt, -lat, r_0)[2];
	auto sc_d_plus  = calculate_gravity_schwartz(alt, lat)[2];
	auto sc_d_minus = calculate_gravity_schwartz(alt, -lat)[2];
	ASSERT_EQ(t_d_plus, t_d_minus);
	ASSERT_EQ(sc_d_plus, sc_d_minus);
}

TEST_F(GravTests, comp_alt) {
	// Negative altitudes should produce lower gravity than at surface
	double lat    = PI / 3.0;
	double wander = 0.0;
	double alt    = 1000.0;
	double lon    = 0.0;

	auto C_n_to_e   = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat, lon, wander);
	auto C_ned_to_n = navtk::navutils::wander_to_C_ned_to_n(wander);

	auto r_n = navtk::navutils::meridian_radius(lat);
	auto r_e = navtk::navutils::transverse_radius(lat);
	auto r_0 = sqrt(r_n * r_e);

	auto t_d_plus       = calculate_gravity_titterton(0, lat, r_0)[2];
	auto t_d_minus      = calculate_gravity_titterton(-alt, lat, r_0)[2];
	auto sc_d_plus      = calculate_gravity_schwartz(0, lat)[2];
	auto sc_d_minus     = calculate_gravity_schwartz(-alt, lat)[2];
	auto savn_d_plus    = calculate_gravity_savage_n(C_n_to_e, 0)[2];
	auto savn_d_minus   = calculate_gravity_savage_n(C_n_to_e, -alt)[2];
	auto savned_d_plus  = calculate_gravity_savage_ned(C_n_to_e, 0)[2];
	auto savned_d_minus = calculate_gravity_savage_ned(C_n_to_e, -alt)[2];
	ASSERT_TRUE(abs(t_d_plus) > abs(t_d_minus));
	ASSERT_TRUE(abs(sc_d_plus) > abs(sc_d_minus));
	ASSERT_TRUE(abs(savn_d_plus) > abs(savn_d_minus));
	ASSERT_TRUE(abs(savned_d_plus) > abs(savned_d_minus));

	// All scale down for neg altitudes using the same basic assumption,
	// so use approx bounds derived from Titterton version
	auto frac_lost = abs(alt / r_0) * t_d_plus;
	auto q_minus   = 0.75 * frac_lost;
	auto q_plus    = 1.25 * frac_lost;
	auto d1        = abs(t_d_plus - t_d_minus);
	auto d2        = abs(sc_d_plus - sc_d_minus);
	auto d3        = abs(savn_d_plus - savn_d_minus);
	auto d4        = abs(savned_d_plus - savned_d_minus);
	ASSERT_TRUE(d1 > q_minus && d1 < q_plus);
	ASSERT_TRUE(d2 > q_minus && d2 < q_plus);
	ASSERT_TRUE(d3 > q_minus && d3 < q_plus);
	ASSERT_TRUE(d4 > q_minus && d4 < q_plus);
}

TEST_F(GravTests, com_polar) {
	// WGS84 has defined equatorial and polar gravity values, so compare against those
	double lat    = PI / 2.0;
	double wander = 0.0;
	double alt    = 0.0;
	double lon    = 0.0;

	auto C_n_to_e_eq = navtk::navutils::lat_lon_wander_to_C_n_to_e(0.0, lon, wander);
	auto C_n_to_e_n  = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat, lon, wander);
	auto C_n_to_e_s  = navtk::navutils::lat_lon_wander_to_C_n_to_e(-lat, lon, wander);

	auto C_ned_to_n = navtk::navutils::wander_to_C_ned_to_n(wander);

	auto r_n_eq = navtk::navutils::meridian_radius(0.0);
	auto r_e_eq = navtk::navutils::transverse_radius(0.0);
	auto r_0_eq = sqrt(r_n_eq * r_e_eq);

	auto r_n_n = navtk::navutils::meridian_radius(lat);
	auto r_e_n = navtk::navutils::transverse_radius(lat);
	auto r_0_n = sqrt(r_n_n * r_e_n);

	auto r_n_s = navtk::navutils::meridian_radius(-lat);
	auto r_e_s = navtk::navutils::transverse_radius(-lat);
	auto r_0_s = sqrt(r_n_s * r_e_s);

	auto t_d_n  = calculate_gravity_titterton(alt, lat, r_0_n)[2];
	auto t_d_s  = calculate_gravity_titterton(alt, -lat, r_0_s)[2];
	auto t_d_eq = calculate_gravity_titterton(alt, 0.0, r_0_eq)[2];

	auto sc_d_n  = calculate_gravity_schwartz(alt, lat)[2];
	auto sc_d_s  = calculate_gravity_schwartz(alt, -lat)[2];
	auto sc_d_eq = calculate_gravity_schwartz(alt, 0.0)[2];

	auto savn_d_n  = calculate_gravity_savage_n(C_n_to_e_n, alt)[2];
	auto savn_d_eq = calculate_gravity_savage_n(C_n_to_e_eq, alt)[2];

	auto savned_d_n  = calculate_gravity_savage_ned(C_n_to_e_n, alt)[2];
	auto savned_d_eq = calculate_gravity_savage_ned(C_n_to_e_eq, alt)[2];

	// These are not valid for all models
	auto eg = navtk::navutils::EQUATORIAL_GRAVITY;
	auto pg = navtk::navutils::POLAR_GRAVITY;
	// http://geoweb.mit.edu/~tah/12.221_2005/grs80_corr.pdf
	auto grs80_eg = 9.7803267715;
	auto grs80_pg = 9.8321863685;
	// https://en.wikipedia.org/wiki/Normal_gravity_formula
	auto grs67_eg_from_wiki = 9.780318;

	// T+W use old grav model
	// Assertions: Based on the fact that there are 3 different models to
	// compare
	ASSERT_TRUE(abs(t_d_eq - grs67_eg_from_wiki) < 2e-6);
	ASSERT_TRUE(abs(sc_d_eq - eg) < 2e-6);
	ASSERT_TRUE(abs(savn_d_eq + grs80_eg) < 2e-6);  // Opposite vertical def
	ASSERT_TRUE(abs(savned_d_eq - grs80_eg) < 2e-6);


	ASSERT_TRUE(abs(t_d_n - pg) < 1e-5);  // No numer for this yet, needs to be calculated
	ASSERT_TRUE(abs(sc_d_n - pg) < 2e-6);
	ASSERT_TRUE(abs(savn_d_n + grs80_pg) < savage_threshold);  // Opposite vertical def
	ASSERT_TRUE(abs(savned_d_n - grs80_pg) < savage_threshold);

	// These are symmetric about eq- savage are not
	ASSERT_EQ(t_d_n, t_d_s);
	ASSERT_EQ(sc_d_n, sc_d_s);
}

TEST_F(GravTests, wander_invariant_ned) {
	// Prove that the NED version of Savage gravity is invariant to
	// wander angle (since the conversion from N to NED frame removes it)
	// so we can safely use it in non-wander mechanizations by setting
	// wander = 0
	double lat    = PI / 2.106;
	double wander = 0.6;
	double alt    = 1234.0;
	double lon    = 1.1;

	auto C_n_to_e0 = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat, lon, 0.0);
	auto C_n_to_e  = navtk::navutils::lat_lon_wander_to_C_n_to_e(lat, lon, wander);

	auto savn0 = calculate_gravity_savage_ned(C_n_to_e0, alt);
	auto savn  = calculate_gravity_savage_ned(C_n_to_e, alt);

	ASSERT_ALLCLOSE(savn0, savn);
}
