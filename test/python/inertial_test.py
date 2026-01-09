#!/usr/bin/env python3

import unittest
from numpy import transpose, zeros
from numpy.testing import assert_allclose
from navtk.navutils import GravModels, rpy_to_dcm
from navtk.inertial import (
    Inertial,
    mechanization_wander,
    mechanization_standard,
    StandardPosVelAtt,
    MechanizationOptions,
    EarthModels,
    DcmIntegrationMethods,
    IntegrationMethods,
    InertialPosVelAtt,
    MechanizationStandard,
)
from aspn23_xtensor import TypeTimestamp


class DummyPosVelAtt(InertialPosVelAtt):
    def is_wander_capable(self):
        return True

    def get_llh(self):
        return zeros([3])

    def get_vned(self):
        return zeros([3])

    def get_C_s_to_ned(self):
        return zeros([3, 3])

    def get_C_n_to_e_h(self):
        return zeros([3, 3]), 0.0

    def get_vn(self):
        return zeros([3])

    def get_C_s_to_l(self):
        return zeros([3, 3])


class InertialTesting(unittest.TestCase):
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
        self.wander0 = 0.4

        self.dv = [1e-3, 2e-4, 3e-5]
        self.dth = [1e-6, 2e-5, 3e-6]

        self.llh0 = [lat0, lon0, alt0]
        self.vned0 = [vn0, ve0, vd0]
        self.rpy0 = [r0, p0, y0]
        self.c_s_to_ned = rpy_to_dcm(self.rpy0)

        self.options = MechanizationOptions(
            GravModels.TITTERTON,
            EarthModels.ELLIPTICAL,
            DcmIntegrationMethods.SIXTH_ORDER,
            IntegrationMethods.TRAPEZOIDAL,
        )
        self.start_pos = StandardPosVelAtt(
            self.t0, self.llh0, self.vned0, self.c_s_to_ned
        )
        self.ins = Inertial(
            self.start_pos, self.options, mechanization_standard
        )

    # Duplicate of C++ InertialTests.hunder_mechs_SLOW
    def test_hundred_mechs(self):
        llh_out = [0.12300063279791, -0.98700094996267, 538.2479946985551]
        vel_out = [2.05051724784607, -3.07384916786599, 18.12121193416597]
        c_n_to_s_out = [
            [0.31817279874302, -0.81694723126001, -0.48100238198645],
            [0.94603954373241, 0.30648541455159, 0.1052419705332],
            [0.06144307800397, -0.48853240629844, 0.87037970803647],
        ]
        # Testing bindings of alternative Inertial ctor
        ins = Inertial(MechanizationStandard(), self.start_pos, self.options)
        for k in range(0, 99):
            self.ins.mechanize(self.t0 + self.dt * (k + 1), self.dv, self.dth)
            ins.mechanize(self.t0 + self.dt * (k + 1), self.dv, self.dth)

        assert_allclose(
            self.ins.get_solution().get_llh(), llh_out, 1e-14, 1e-14
        )
        assert_allclose(
            self.ins.get_solution().get_vned(), vel_out, 1e-14, 1e-14
        )
        assert_allclose(
            self.ins.get_solution().get_C_s_to_ned(),
            transpose(c_n_to_s_out),
            1e-14,
            1e-14,
        )

        assert_allclose(ins.get_solution().get_llh(), llh_out, 1e-14, 1e-14)
        assert_allclose(ins.get_solution().get_vned(), vel_out, 1e-14, 1e-14)
        assert_allclose(
            ins.get_solution().get_C_s_to_ned(),
            transpose(c_n_to_s_out),
            1e-14,
            1e-14,
        )

    def test_mech_one(self):
        ins2 = Inertial(MechanizationStandard(), self.start_pos, self.options)

        self.ins.mechanize(self.t0 + self.dt, self.dv, self.dth)
        ins2.mechanize(self.t0 + self.dt, self.dv, self.dth)

        assert_allclose(
            ins2.get_solution().get_llh(),
            self.ins.get_solution().get_llh(),
            1e-14,
            1e-14,
        )
        assert_allclose(
            ins2.get_solution().get_vned(),
            self.ins.get_solution().get_vned(),
            1e-14,
            1e-14,
        )
        assert_allclose(
            ins2.get_solution().get_C_s_to_ned(),
            self.ins.get_solution().get_C_s_to_ned(),
            1e-14,
            1e-14,
        )

    def test_object_slicing_python_reference(self):
        # Test that python retains a reference to the DummyPosVelAtt object
        # when calling `target.mechanize`. This function will fail with a
        # "Tried to call pure virtual function" error if python doesn't retain
        # a reference to this object.
        target = Inertial(DummyPosVelAtt(), self.options, mechanization_wander)
        target.mechanize(self.t0 + self.dt, self.dv, self.dth)


if __name__ == '__main__':
    unittest.main()
