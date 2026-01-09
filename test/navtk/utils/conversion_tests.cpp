#include <gtest/gtest.h>
#include <equality_checks.hpp>
#include <tensor_assert.hpp>

#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>
#include <navtk/utils/human_readable.hpp>

using navtk::filtering::testing::verify_sol;
using navtk::utils::to_navsolution;
using navtk::utils::to_positionvelocityattitude;
using navtk::utils::to_standardposvelatt;
using navtk::utils::to_vector_pva;

struct ConversionTests : public ::testing::Test {
	aspn_xtensor::TypeTimestamp t = aspn_xtensor::to_type_timestamp(123.456789123);
	navtk::Vector3 pos;
	navtk::Vector3 vel;
	navtk::Matrix3 C_s_to_ned;
	navtk::inertial::StandardPosVelAtt spva;
	navtk::filtering::NavSolution ns;
	navtk::Vector v;
	navtk::Vector3 rpy;
	ConversionTests()
	    : ::testing::Test(),
	      pos({0.12300063279791, -0.98700094996267, 538.2479946985551}),
	      vel({2.05051724784607, -3.07384916786599, 18.12121193416597}),
	      C_s_to_ned({{0.31817279874302, -0.81694723126001, -0.48100238198645},
	                  {0.94603954373241, 0.30648541455159, 0.1052419705332},
	                  {0.06144307800397, -0.48853240629844, 0.87037970803647}}),
	      spva({t, pos, vel, C_s_to_ned}),
	      ns(pos, vel, C_s_to_ned, t),
	      rpy(navtk::navutils::dcm_to_rpy(ns.rot_mat)) {
		v = {123.456789123,
		     0.12300063279791,
		     -0.98700094996267,
		     538.2479946985551,
		     2.05051724784607,
		     -3.07384916786599,
		     18.12121193416597,
		     rpy[0],
		     rpy[1],
		     rpy[2]};
	}
};

TEST_F(ConversionTests, spva1) {
	auto cvt = to_standardposvelatt(to_navsolution(spva));
	verify_sol(spva, cvt, 0.0, 1e-14);
}

TEST_F(ConversionTests, spva2) {
	auto cvt = to_standardposvelatt(to_positionvelocityattitude(spva));
	verify_sol(spva, cvt, 0.0, 1e-14);
}

TEST_F(ConversionTests, spva3) {
	auto cvt = to_standardposvelatt(to_vector_pva(spva));
	verify_sol(spva, cvt, 0.0, 1e-14);
}

TEST_F(ConversionTests, ns1) {
	auto cvt = to_navsolution(to_positionvelocityattitude(ns));
	verify_sol(ns, cvt, 0.0, 1e-14);
}

TEST_F(ConversionTests, ns2) {
	auto cvt = to_navsolution(to_standardposvelatt(ns));
	verify_sol(ns, cvt, 0.0, 1e-14);
}

TEST_F(ConversionTests, ns3) {
	auto cvt = to_navsolution(to_vector_pva(ns));
	verify_sol(ns, cvt, 0.0, 1e-14);
}

TEST_F(ConversionTests, all) {
	auto cvt = to_standardposvelatt(to_navsolution(to_vector_pva(to_positionvelocityattitude(
	    to_standardposvelatt(to_positionvelocityattitude(to_navsolution(spva)))))));
	verify_sol(spva, cvt, 0.0, 1e-14);
}

TEST_F(ConversionTests, partial_pva_to_pos) {
	aspn_xtensor::TypeHeader header(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0);
	auto cov = xt::diag(xt::arange(0.0, 8.0, 1.0));
	auto pva = aspn_xtensor::MeasurementPositionVelocityAttitude(
	    header,
	    t,
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
	    pos[0],
	    NAN,
	    pos[2],
	    vel[0],
	    vel[1],
	    vel[2],
	    navtk::navutils::rpy_to_quat(rpy),
	    cov,
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
	    navtk::Vector(),
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	auto as_pos = navtk::utils::to_position(pva);
	EXPECT_EQ(as_pos.get_reference_frame(), ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC);
	EXPECT_DOUBLE_EQ(as_pos.get_term1(), pos[0]);
	EXPECT_TRUE(std::isnan(as_pos.get_term2()));
	EXPECT_DOUBLE_EQ(as_pos.get_term3(), pos[2]);
	navtk::Matrix ex_cov = as_pos.get_covariance();
	EXPECT_ALLCLOSE(ex_cov, xt::diag(xt::arange(0.0, 2.0, 1.0)));

	pva.set_reference_frame(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_ECI);
	as_pos = navtk::utils::to_position(pva);
	EXPECT_EQ(as_pos.get_reference_frame(), ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_ECI);
}
