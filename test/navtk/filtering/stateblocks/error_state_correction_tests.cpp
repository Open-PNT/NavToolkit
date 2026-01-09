#include <gtest/gtest.h>
#include <equality_checks.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/Pinson21NedBlock.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::MeasurementPositionVelocityAttitude;
using navtk::filtering::apply_error_states;
using navtk::filtering::NavSolution;
using navtk::filtering::Pinson15NedBlock;
using navtk::filtering::Pinson21NedBlock;
using navtk::filtering::testing::verify_sol;
using navtk::inertial::StandardPosVelAtt;

namespace {

NavSolution manual_correction(const NavSolution& ns, navtk::Vector x) {
	auto lat      = ns.pos[0] + navtk::navutils::north_to_delta_lat(x[0], ns.pos[0], ns.pos[2]);
	auto lon      = ns.pos[1] + navtk::navutils::east_to_delta_lon(x[1], ns.pos[0], ns.pos[2]);
	auto alt      = ns.pos[2] - x[2];
	auto v        = ns.vel + xt::view(x, xt::range(3, 6));
	auto C_n_to_s = transpose(navtk::navutils::correct_dcm_with_tilt(xt::transpose(ns.rot_mat),
	                                                                 xt::view(x, xt::range(6, 9))));
	return NavSolution({lat, lon, alt}, v, C_n_to_s, ns.time);
}

}  // namespace

struct ErrorStateCorrectionTests : public ::testing::Test {
	double lat                     = 0.7;
	double lon                     = -1.4;
	double alt                     = 123.45;
	double vn                      = 1.1;
	double ve                      = 2.1;
	double vd                      = -0.8;
	double r                       = 1.3;
	double p                       = 1.1;
	double y                       = 2.4;
	aspn_xtensor::TypeTimestamp ts = aspn_xtensor::to_type_timestamp(1234.72518);
	navtk::Vector x15;
	navtk::Vector x21;
	NavSolution ns_raw;
	StandardPosVelAtt spva_raw;
	MeasurementPositionVelocityAttitude pva_raw;
	NavSolution ns_cor;

	ErrorStateCorrectionTests()
	    : ::testing::Test(),
	      x15({0.1, -3.2, 1.1, 4.4, -0.06, -9.9, 1e-3, 2e-3, 3e-4, 0, 0, 0, 0, 0, 0}),
	      x21({x15[0], x15[1], x15[2], x15[3], x15[4], x15[5], x15[6], x15[7], x15[8], 0, 0,
	           0,      0,      0,      0,      0,      0,      0,      0,      0,      0}),
	      ns_raw({lat, lon, alt},
	             {vn, ve, vd},
	             xt::transpose(navtk::navutils::rpy_to_dcm({r, p, y})),
	             ts),
	      spva_raw(navtk::utils::to_standardposvelatt(ns_raw)),
	      pva_raw(navtk::utils::to_positionvelocityattitude(ns_raw)),
	      ns_cor(manual_correction(ns_raw, x15)) {}
};

TEST_F(ErrorStateCorrectionTests, UnimplementedType) {
	auto cor = EXPECT_ERROR(apply_error_states<NavSolution>(ns_raw, x15), "no correction applied");
	verify_sol(ns_raw, cor, 0.0, 1e-14);
}

TEST_F(ErrorStateCorrectionTests, WrongSize) {
	auto cor = EXPECT_WARN(apply_error_states<Pinson15NedBlock>(ns_raw, navtk::zeros(3)),
	                       "too few elements");
	verify_sol(ns_raw, cor, 0.0, 1e-14);
}

TEST_F(ErrorStateCorrectionTests, NavSolutionPinson) {
	auto cor = apply_error_states<Pinson15NedBlock>(ns_raw, x15);
	verify_sol(ns_cor, cor, 0.0, 1e-14);

	auto cor21 = apply_error_states<Pinson21NedBlock>(ns_raw, x21);
	verify_sol(ns_cor, cor21, 0.0, 1e-14);
}

TEST_F(ErrorStateCorrectionTests, StandardPosVelAttPinson) {
	auto cor = apply_error_states<Pinson15NedBlock>(spva_raw, x15);
	verify_sol(ns_cor, navtk::utils::to_navsolution(cor), 0.0, 1e-14);

	auto cor21 = apply_error_states<Pinson21NedBlock>(spva_raw, x21);
	verify_sol(ns_cor, navtk::utils::to_navsolution(cor21), 0.0, 1e-14);
}

TEST_F(ErrorStateCorrectionTests, PositionVelocityAttitudePinson) {
	auto cor = apply_error_states<Pinson15NedBlock>(pva_raw, x15);
	verify_sol(ns_cor, navtk::utils::to_navsolution(cor), 0.0, 1e-14);

	auto cor21 = apply_error_states<Pinson21NedBlock>(pva_raw, x21);
	verify_sol(ns_cor, navtk::utils::to_navsolution(cor21), 0.0, 1e-14);
}

TEST_F(ErrorStateCorrectionTests, PositionVelocityAttitudeKeepsAdditions) {
	auto cov     = navtk::ones(9, 9) * 4;
	auto dev_id  = 2;
	auto seq_num = 10;
	pva_raw.set_device_id(dev_id);
	pva_raw.set_sequence_id(seq_num);
	pva_raw.set_covariance(cov);
	auto cor = apply_error_states<Pinson15NedBlock>(pva_raw, x15);
	ASSERT_ALLCLOSE(cov, navtk::Matrix(cor.get_covariance()));
	ASSERT_EQ(dev_id, cor.get_device_id());
	ASSERT_EQ(seq_num, cor.get_sequence_id());

	auto cor21 = apply_error_states<Pinson21NedBlock>(pva_raw, x21);
	ASSERT_ALLCLOSE(cov, navtk::Matrix(cor21.get_covariance()));
	ASSERT_EQ(dev_id, cor21.get_device_id());
	ASSERT_EQ(seq_num, cor21.get_sequence_id());
}
