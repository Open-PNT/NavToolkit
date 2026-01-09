#!/usr/bin/env python3

import unittest
from numpy import zeros
from numpy.testing import assert_allclose
from navtk.inertial import BufferedImu, ImuErrors
from navtk.navutils import rpy_to_quat
from aspn23_xtensor import (
    MeasurementPositionVelocityAttitude,
    TypeHeader,
    TypeTimestamp,
    AspnMessageType,
    AspnMeasurementPositionVelocityAttitudeReferenceFrame as Frame,
    AspnMeasurementPositionVelocityAttitudeErrorModel as ErrorModel,
)

# Alias enum values to fit within formatting constraints.
GEODETIC = (
    Frame.ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC
)
NO_ERROR_MODEL = (
    ErrorModel.ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE
)


def pva_asserts(pva1, pva2):
    assert_allclose(pva1.get_p1(), pva2.get_p1(), 1e-14, 1e-14)
    assert_allclose(pva1.get_p2(), pva2.get_p2(), 1e-14, 1e-14)
    assert_allclose(pva1.get_p3(), pva2.get_p3(), 1e-14, 1e-14)
    assert_allclose(pva1.get_v1(), pva2.get_v1(), 1e-14, 1e-14)
    assert_allclose(pva1.get_v2(), pva2.get_v2(), 1e-14, 1e-14)
    assert_allclose(pva1.get_v3(), pva2.get_v3(), 1e-14, 1e-14)
    assert_allclose(pva1.get_quaternion(), pva2.get_quaternion(), 1e-14, 1e-14)


class BufferedImuTesting(unittest.TestCase):
    def setUp(self):
        lat0 = 0.123
        lon0 = -0.987
        alt0 = 555.0
        vn0 = 2.0
        ve0 = -3.0
        vd0 = -1.2
        r0 = 0.12
        p0 = 0.5
        y0 = -1.2
        self.t0 = TypeTimestamp(0)
        self.dt = 0.02
        self.dv = [1e-3, 2e-4, 3e-5]
        self.dth = [1e-6, 2e-5, 3e-6]
        self.llh0 = [lat0, lon0, alt0]
        self.vned0 = [vn0, ve0, vd0]
        self.rpy0 = [r0, p0, y0]
        self.pva = MeasurementPositionVelocityAttitude(
            TypeHeader(
                AspnMessageType.ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE,
                0,
                0,
                0,
                0,
            ),
            TypeTimestamp(0),
            GEODETIC,
            lat0,
            lon0,
            alt0,
            self.vned0[0],
            self.vned0[1],
            self.vned0[2],
            rpy_to_quat(self.rpy0),
            zeros((9, 9)),
            NO_ERROR_MODEL,
            [],
            [],
        )
        self.imu_errors = ImuErrors()
        self.imu_errors.accel_biases = [0.01, 0.02, 0.03]
        self.imu_errors.gyro_biases = [0.01, 0.02, 0.03]
        self.imu_errors2 = ImuErrors()
        self.imu_errors2.accel_biases = [-0.03, 0.01, 0.004]
        self.imu_errors2.gyro_biases = [-0.001, -0.002, 0.04]
        self.ins = BufferedImu(self.pva, None, self.dt, self.imu_errors)
        self.ins2 = BufferedImu(self.pva, None, self.dt, self.imu_errors2)
        self.loops = 99
        self.repeat_loops = 25

    def test_resets_no_imu_error(self):
        for k in range(0, self.loops):
            self.ins.mechanize(self.t0 + self.dt * (k + 1), self.dv, self.dth)
        pva_out = self.ins.calc_pva()
        self.t0 = pva_out.get_time_of_validity()
        reset_time = self.t0 - self.repeat_loops * self.dt
        new_ts = reset_time
        reset_pva = MeasurementPositionVelocityAttitude(
            TypeHeader(
                AspnMessageType.ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE,
                0,
                0,
                0,
                0,
            ),
            new_ts,
            GEODETIC,
            self.pva.get_p1(),
            self.pva.get_p2(),
            self.pva.get_p3(),
            self.pva.get_v1(),
            self.pva.get_v2(),
            self.pva.get_v3(),
            self.pva.get_quaternion(),
            zeros((9, 9)),
            NO_ERROR_MODEL,
            [],
            [],
        )
        # Do a reset in the past. ins will re-prop from this reset, through any
        # data it has received back up to its current solution time
        self.ins.reset(reset_pva)
        # Propagate again to get back to same total number of mechs from
        # starting pva as before, removing the number of mechs the ins
        # automatically did after reset. Note that this may only be done in
        # this way because dv/dth are constant.
        for k in range(0, self.loops - self.repeat_loops):
            self.ins.mechanize(self.t0 + self.dt * (k + 1), self.dv, self.dth)
        pva_out2 = self.ins.calc_pva()
        pva_asserts(pva_out, pva_out2)

    def test_resets(self):
        for k in range(0, self.loops):
            self.ins.mechanize(self.t0 + self.dt * (k + 1), self.dv, self.dth)
            self.ins2.mechanize(self.t0 + self.dt * (k + 1), self.dv, self.dth)
        pva_out = self.ins2.calc_pva()
        self.t0 = pva_out.get_time_of_validity()
        reset_time = self.t0 - self.repeat_loops * self.dt
        new_ts = reset_time
        reset_pva = MeasurementPositionVelocityAttitude(
            TypeHeader(
                AspnMessageType.ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE,
                0,
                0,
                0,
                0,
            ),
            new_ts,
            GEODETIC,
            self.pva.get_p1(),
            self.pva.get_p2(),
            self.pva.get_p3(),
            self.pva.get_v1(),
            self.pva.get_v2(),
            self.pva.get_v3(),
            self.pva.get_quaternion(),
            zeros((9, 9)),
            NO_ERROR_MODEL,
            [],
            [],
        )
        self.ins.reset(reset_pva, self.imu_errors2)
        for k in range(0, self.loops - self.repeat_loops):
            self.ins.mechanize(self.t0 + self.dt * (k + 1), self.dv, self.dth)
        pva_out2 = self.ins.calc_pva()
        pva_asserts(pva_out, pva_out2)


if __name__ == '__main__':
    unittest.main()
