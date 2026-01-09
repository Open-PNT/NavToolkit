#!/usr/bin/env python3

import unittest
from math import isclose
from numpy import zeros, eye
from navtk.gnssutils import CAL1
from navtk.filtering import (
    StandardFusionEngine,
    hg1700_model,
    Pinson15NedBlock,
    COMPENSATED_CRYSTAL_CLOCK,
    ClockBiasesStateBlock,
    SinglePointPseudorangeProcessor,
    PairedPva,
    NavSolution,
)
from navtk.utils import NANO_PER_SEC, to_inertial_aux
from aspn23_xtensor import (
    TypeTimestamp,
    TypeHeader,
    MeasurementSatnav,
    TypeSatnavObs,
    MetadataGpsLnavEphemeris,
    AspnMessageType,
    TypeSatnavSatelliteSystem,
    TypeSatnavSignalDescriptor,
    AspnTypeSatnavObsPseudorangeRateType as RateType,
    AspnTypeSatnavObsIonoCorrectionSource as IonoCorrectionSource,
    AspnTypeSatnavSatelliteSystemSatelliteSystem as SatelliteSystem,
    AspnTypeSatnavSignalDescriptorSignalDescriptor as SignalDescriptor,
    TypeSatnavTime,
    AspnTypeSatnavTimeTimeReference as TimeReference,
    TypeMetadataheader,
    TypeSatnavClock,
    TypeKeplerOrbit,
)

# Alias enum values to fit within formatting constraints.
GPS_SYSTEM = (
    SatelliteSystem.ASPN_TYPE_SATNAV_SATELLITE_SYSTEM_SATELLITE_SYSTEM_SYS_GPS
)
L1C = SignalDescriptor.ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1C
DOPPLER = RateType.ASPN_TYPE_SATNAV_OBS_PSEUDORANGE_RATE_TYPE_PSR_RATE_DOPPLER
IONO_CORRECTION_UNKNOWN = (
    IonoCorrectionSource.ASPN_TYPE_SATNAV_OBS_IONO_CORRECTION_SOURCE_UNKNOWN
)
TIME_GPS = TimeReference.ASPN_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GPS


class SinglePointTesting(unittest.TestCase):
    def setUp(self):
        pass

    def test_SinglePoint(self):
        engine = StandardFusionEngine()
        model = hg1700_model()
        pins = Pinson15NedBlock("pins", model)
        engine.add_state_block(pins)

        clock_mod = COMPENSATED_CRYSTAL_CLOCK
        clock_bias = ClockBiasesStateBlock("clock", clock_mod)
        engine.add_state_block(clock_bias)

        gps = SinglePointPseudorangeProcessor(
            "gps", "pins", "clock", CAL1, 1.0, 1.0, 1.0, engine
        )
        engine.add_measurement_processor(gps)
        ts = TypeTimestamp(10123000000)
        ns = NavSolution(zeros(3), zeros(3), eye(3), ts)
        engine.give_state_block_aux_data("pins", to_inertial_aux(ns, zeros(3)))
        head = TypeHeader(AspnMessageType.ASPN_MEASUREMENT_SATNAV, 0, 0, 0, 0)
        sat_sys = TypeSatnavSatelliteSystem(GPS_SYSTEM)
        sig_desc = TypeSatnavSignalDescriptor(L1C)
        obs = TypeSatnavObs(
            sat_sys,
            sig_desc,
            1,
            0,
            0,
            0,
            DOPPLER,
            0,
            0,
            0,
            0,
            0,
            0,
            IONO_CORRECTION_UNKNOWN,
            False,
            False,
            False,
            [],
        )

        satnav_time = TypeSatnavTime(0, 0, TIME_GPS)
        gnss = MeasurementSatnav(head, ts, satnav_time, 1, [obs], [])
        # provide a measurement, check SB added according to naming scheme
        paired = PairedPva(gnss, ns)

        info = TypeMetadataheader(head, "", 0, 0, 0)
        clock = TypeSatnavClock(0, 0, 0, 0)
        orbit = TypeKeplerOrbit(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

        eph = MetadataGpsLnavEphemeris(info, ts, 0, 1, clock, orbit, 0, 0, 0)
        aux = [eph]
        engine.give_measurement_processor_aux_data("gps", aux)
        engine.update("gps", paired)

        names = engine.get_state_block_names_list()
        assert names[2] == "pr_bias_sv_1"

        assert isclose(
            gnss.get_time_of_validity().get_elapsed_nsec() / NANO_PER_SEC,
            10.123,
        )


if __name__ == '__main__':
    unittest.main()
