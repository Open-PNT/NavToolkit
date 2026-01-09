#include <gtest/gtest.h>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/inertial/BufferedPva.hpp>
#include <navtk/inertial/ImuErrors.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::Matrix;
using navtk::Matrix3;
using navtk::Vector3;
using navtk::Vector4;
using navtk::zeros;
using navtk::inertial::BufferedPva;
using navtk::inertial::ImuErrors;
using navtk::inertial::StandardPosVelAtt;
using navtk::navutils::rpy_to_dcm;
using navtk::navutils::rpy_to_quat;
using navtk::utils::to_positionvelocityattitude;

struct BufferedPvaTests : public ::testing::Test {

	Vector3 llh_out_simpsons;
	Vector3 vel_out_simpsons;
	Matrix3 c_n_to_s_out_simpsons;
	aspn_xtensor::TypeTimestamp start_time = aspn_xtensor::to_type_timestamp(3.5);

	double lat0 = 0.123;
	double lon0 = -0.987;
	double alt0 = 555.0;
	double vn0  = 2.0;
	double ve0  = -3.0;
	double vd0  = -1.2;
	double r0   = 0.12;
	double p0   = 0.5;
	double y0   = -1.2;
	double t0   = 0.0;
	double dt   = 0.02;

	Vector3 dv;
	Vector3 dth;

	Vector3 llh0;
	Vector3 vned0;
	Vector3 rpy0;
	Matrix3 c_s_to_ned;

	StandardPosVelAtt simpsons;
	MeasurementPositionVelocityAttitude simpsons_pva;
	MeasurementPositionVelocityAttitude at10;
	MeasurementPositionVelocityAttitude at10point7;
	StandardPosVelAtt start;
	MeasurementPositionVelocityAttitude start_pva;

	BufferedPvaTests()
	    : ::testing::Test(),

	      // Kotlin test results, Simpson's Rule (1 until 100) loop
	      llh_out_simpsons({0.12300063271845, -0.98700094984638, 538.4405566448743}),
	      vel_out_simpsons({2.05051724784581, -3.07384916791962, 18.12121135559916}),
	      c_n_to_s_out_simpsons({{0.31817279874302, -0.81694723126002, -0.48100238198643},
	                             {0.94603954373241, 0.30648541455158, 0.10524197053321},
	                             {0.06144307800396, -0.48853240629843, 0.87037970803648}}),

	      dv({1e-3, 2e-4, 3e-5}),
	      dth({1e-6, 2e-5, 3e-6}),

	      llh0({lat0, lon0, alt0}),
	      vned0({vn0, ve0, vd0}),
	      rpy0({r0, p0, y0}),
	      c_s_to_ned(rpy_to_dcm(rpy0)),

	      simpsons({start_time + 1.98,
	                llh_out_simpsons,
	                vel_out_simpsons,
	                xt::transpose(c_n_to_s_out_simpsons)}),
	      simpsons_pva(to_positionvelocityattitude(simpsons)),
	      at10(create_pva((start_time + dt * 10),
	                      {0.123000063194546, -0.987000094884677, 555.063701787553},
	                      {2.00510130382739, -3.00771463454115, 0.751640472366012},
	                      {0.1200333538052327, 0.5001815898987083, -1.1999398738709726},
	                      zeros(9, 9))),
	      at10point7(create_pva((start_time + dt * 10.7),
	                            {0.123000067624288, -0.987000101535863, 555.053315435747},
	                            {2.0054584074238, -3.00825245275297, 0.888255254635688},
	                            {0.1200356891187274, 0.5001943011475822, -1.199935664603395},
	                            zeros(9, 9))),
	      start({start_time, llh0, vned0, c_s_to_ned}),
	      start_pva(to_positionvelocityattitude(start)) {}

	void assert_eq(const MeasurementPositionVelocityAttitude& pva1,
	               const MeasurementPositionVelocityAttitude& pva2) {
		ASSERT_EQ(pva1.get_time_of_validity().get_elapsed_nsec(),
		          pva2.get_time_of_validity().get_elapsed_nsec());
		ASSERT_FLOAT_EQ(pva1.get_p1(), pva2.get_p1());
		ASSERT_FLOAT_EQ(pva1.get_p2(), pva2.get_p2());
		/*
		 * Accepting a small amount of error here, which arises when a reset is performed between
		 * two imu measurements using a pva pulled from another measurement history; as the reset
		 * PVA returned from get_pva(t) is a linearly interpolated solution, the two inertials are
		 * only approximately equal, and thus diverge over time. This is not an issue when the
		 * reset occurs on an exact measurement (compare ResetAtExactTime vs ResetBetweenRecords).
		 */
		ASSERT_NEAR(pva1.get_p3(), pva2.get_p3(), 1e-2);
		Vector3 vel1 = {pva1.get_v1(), pva1.get_v2(), pva1.get_v3()};
		Vector3 vel2 = {pva2.get_v1(), pva2.get_v2(), pva2.get_v3()};
		ASSERT_ALLCLOSE(vel1, vel2);
		ASSERT_ALLCLOSE(Vector4{pva1.get_quaternion()}, Vector4{pva2.get_quaternion()});
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
		    navtk::Vector(),
		    std::vector<aspn_xtensor::TypeIntegrity>{});
	}
};
