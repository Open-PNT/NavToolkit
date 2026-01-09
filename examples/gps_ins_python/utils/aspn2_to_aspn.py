from aspn23_xtensor import (
    TypeHeader,
    AspnMessageType,
    TypeTimestamp,
    MeasurementPositionVelocityAttitude,
    MeasurementPosition,
    MeasurementImu,
    AspnMeasurementImuImuType,
    AspnMeasurementPositionReferenceFrame,
    AspnMeasurementPositionErrorModel,
)
import numpy as np
from navtk.utils import NANO_PER_SEC

MAX_16_BIT = 65536


def lcm_header_to_aspn(header, type) -> TypeHeader:
    seq_num = header.seq_num
    while seq_num >= MAX_16_BIT:
        seq_num -= MAX_16_BIT
    return TypeHeader(type, 0, 0, 0, seq_num)


def lcm_header_to_aspn_time(header) -> TypeTimestamp:
    time = header.timestamp_valid
    return TypeTimestamp(int(time.sec * NANO_PER_SEC + time.nsec))


pos_ref_frame = AspnMeasurementPositionReferenceFrame
pos_error_model = AspnMeasurementPositionErrorModel


def lcm_geodeticposition3d_to_aspn(geo3d):
    return MeasurementPosition(
        lcm_header_to_aspn(
            geo3d.header, AspnMessageType.ASPN_MEASUREMENT_POSITION
        ),
        lcm_header_to_aspn_time(geo3d.header),
        pos_ref_frame.ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
        geo3d.position.latitude,
        geo3d.position.longitude,
        geo3d.position.altitude,
        np.array(geo3d.covariance),
        pos_error_model.ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
        [],
        [],
    )


IMU_INTEGRATED = (
    AspnMeasurementImuImuType.ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED
)


def lcm_imu_to_aspn(imu):
    return MeasurementImu(
        lcm_header_to_aspn(imu.header, AspnMessageType.ASPN_MEASUREMENT_IMU),
        lcm_header_to_aspn_time(imu.header),
        IMU_INTEGRATED,
        np.array(imu.delta_v),
        np.array(imu.delta_theta),
        [],
    )


def lcm_positionvelocityattitude_to_aspn(pva):
    return MeasurementPositionVelocityAttitude(
        lcm_header_to_aspn(
            pva.header,
            AspnMessageType.ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE,
        ),
        [pva.position.latitude, pva.position.longitude, pva.position.altitude],
        np.array(pva.velocity),
        np.array(pva.attitude),
        np.array(pva.covariance),
    )
