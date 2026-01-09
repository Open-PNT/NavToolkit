#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>

namespace navtk {
/**
 * NavToolkit namespace for general utilities.
 */
namespace utils {

/**
 * Can be used to convert between seconds and nanoseconds. It is bigger than necessary to avoid
 * overflows when multiplying.
 */
constexpr uint64_t NANO_PER_SEC = 1'000'000'000;

/**
 * The number of seconds in a week.
 */
constexpr double SECONDS_PER_WEEK = (60 * 60 * 24 * 7);

/**
 * Converts a position, velocity and attitude from ASPN representation to standard representation.
 *
 * @param pva Solution to convert.
 *
 * @return Converted solution.
 */
filtering::NavSolution to_navsolution(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva);

/**
 * Converts a position, velocity and attitude from inertial representation to standard
 * representation.
 *
 * @param pva Solution to convert.
 *
 * @return Converted solution.
 */
filtering::NavSolution to_navsolution(const inertial::InertialPosVelAtt& pva);

/**
 * Converts a position, velocity and attitude from vector representation to standard representation.
 *
 * @param pva A 10-element Vector containing time in seconds; latitude and longitude in radians;
 * height above WGS-84 ellipsoid; north, east and down velocity in m/s; and roll, pitch and yaw
 * angles in radians.
 *
 * @return Converted solution.
 */
filtering::NavSolution to_navsolution(const Vector& pva);

/**
 * Converts a position, velocity and attitude from standard representation to ASPN. Header
 * device_id and seq_num are set to "" and 0, respectively.
 *
 * @param pva Solution to convert.
 *
 * @return Converted solution.
 */
aspn_xtensor::MeasurementPositionVelocityAttitude to_positionvelocityattitude(
    const filtering::NavSolution& pva);

/**
 * Converts a position, velocity and attitude from inertial representation to ASPN. Header
 * `device_id` and `seq_num` are set to "" and 0, respectively.
 *
 * @param pva Solution to convert.
 *
 * @return Converted solution.
 */
aspn_xtensor::MeasurementPositionVelocityAttitude to_positionvelocityattitude(
    const inertial::InertialPosVelAtt& pva);

/**
 * Converts a position, velocity and attitude from vector representation to ASPN. Header `device_id`
 * and `seq_num` are set to "" and 0, respectively.
 *
 * @param pva A 10-element Vector containing time in seconds; latitude and longitude in radians;
 * height above WGS-84 ellipsoid; north, east and down velocity in m/s; and roll, pitch and yaw
 * angles in radians.
 *
 * @return Converted solution.
 */
aspn_xtensor::MeasurementPositionVelocityAttitude to_positionvelocityattitude(const Vector& pva);

/**
 * Converts a position, velocity and attitude from inertial representation to ASPN, storing the
 * result in /p storage. Header `device_id` and `seq_num` are set to "" and 0, respectively.
 * Does not modify the covariance of /p storage.
 *
 * @param pva Solution to convert.
 * @param storage Container to hold the output.
 */
void to_positionvelocityattitude(const inertial::InertialPosVelAtt& pva,
                                 aspn_xtensor::MeasurementPositionVelocityAttitude& storage);

/**
 * Converts a position, velocity and attitude from standard to inertial representation.
 *
 * @param pva Solution to convert.
 *
 * @return Converted solution.
 */
inertial::StandardPosVelAtt to_standardposvelatt(const filtering::NavSolution& pva);

/**
 * Converts a position, velocity and attitude from ASPN to inertial representation.
 *
 * @param pva Solution to convert.
 *
 * @return Converted solution.
 */
inertial::StandardPosVelAtt to_standardposvelatt(
    const aspn_xtensor::MeasurementPositionVelocityAttitude& pva);

/**
 * Converts a position, velocity and attitude from vector to inertial representation.
 *
 * @param pva A 10-element Vector containing time in seconds; latitude and longitude in radians;
 * height above WGS-84 ellipsoid; north, east and down velocity in m/s; and roll, pitch and yaw
 * angles in radians.
 *
 * @return Converted solution.
 */
inertial::StandardPosVelAtt to_standardposvelatt(const Vector& pva);

/**
 * Converts a position, velocity and attitude from standard to vector representation.
 *
 * @param pva Solution to convert.
 *
 * @return A 10-element Vector containing time in seconds; latitude and longitude in radians;
 * height above WGS-84 ellipsoid; north, east and down velocity in m/s; and roll, pitch and yaw
 * angles in radians.
 */
Vector to_vector_pva(const filtering::NavSolution& pva);

/**
 * Converts a position, velocity and attitude from ASPN representation to vector representation.
 *
 * @param pva Solution to convert.
 *
 * @return A 10-element Vector containing time in seconds; latitude and longitude in radians;
 * height above WGS-84 ellipsoid; north, east and down velocity in m/s; and roll, pitch and yaw
 * angles in radians.
 */
Vector to_vector_pva(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva);

/**
 * Converts a position, velocity and attitude from inertial representation to vector representation.
 *
 * @param pva Solution to convert.
 *
 * @return A 10-element Vector containing time in seconds; latitude and longitude in radians;
 * height above WGS-84 ellipsoid; north, east and down velocity in m/s; and roll, pitch and yaw
 * angles in radians.
 */
Vector to_vector_pva(const inertial::InertialPosVelAtt& pva);

/**
 * Reduces a position, velocity and attitude ASPN representation to a geodetic position 3D ASPN
 * representation using the position from the pva.
 *
 * @param pva Solution to reduce.
 *
 * @return
 */
aspn_xtensor::MeasurementPosition to_position(
    const aspn_xtensor::MeasurementPositionVelocityAttitude& pva);

/**
 * Extracts a vector of position from an ASPN measurement. Does not check frame or for NaN
 * components.
 *
 * @param pva An ASPN measurement to extract vector position from.
 *
 * @return A vector containing the p1, p2, and p3 components.
 */
Vector3 extract_pos(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva);

/**
 * Extracts a vector of velocity from an ASPN measurement. Does not check frame or for NaN
 * components.
 *
 * @param pva An ASPN measurement to extract vector position from.
 *
 * @return A vector containing the v1, v2, and v3 components.
 */
Vector3 extract_vel(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva);

/**
 * Extracts a vector of position from an ASPN measurement. Does not check frame or for NaN
 * components.
 *
 * @param pos An ASPN measurement to extract vector position from.
 *
 * @return A vector containing the p1, p2, and p3 components.
 */
Vector3 extract_pos(const aspn_xtensor::MeasurementPosition& pos);

/**
 * Extracts a vector of velocity from an ASPN measurement. Does not check frame or for NaN
 * components.
 *
 * @param vel An ASPN measurement to extract vector position from.
 *
 * @return A vector containing the x, y, and z components.
 */
Vector3 extract_vel(const aspn_xtensor::MeasurementVelocity& vel);

/**
 * Bundle forces and rates into an aspn_xtensor::MeasurementImu
 *
 * @param time Timestamp of forces and rates
 * @param forces Forces to bundle into aspn_xtensor::MeasurementImu
 * @param rates Rates to bundle into aspn_xtensor::MeasurementImu
 *
 * @return The bundled aspn_xtensor::MeasurementImu.
 */
aspn_xtensor::MeasurementImu to_imu(
    aspn_xtensor::TypeTimestamp time = aspn_xtensor::to_type_timestamp(),
    const Vector& forces             = {NAN, NAN, NAN},
    const Vector& rates              = {NAN, NAN, NAN});
/**
 * Bundle PVA and inertial forces and rates into vector.
 *
 * @param nav_sol NavSolution to convert to
 * @param forces Forces to bundle into aspn_xtensor::MeasurementImu
 * @param rates Rates to bundle into aspn_xtensor::MeasurementImu
 *
 * @return A vector containing the bundled aspn_xtensor::MeasurementPositionVelocityAttitude and
 * aspn_xtensor::MeasurementImu.
 */
AspnBaseVector to_inertial_aux(const filtering::NavSolution& nav_sol,
                               const Vector& forces,
                               const Vector& rates = zeros(3));

/**
 * Produces the appropriate refrence frame for an aspn::xtensor::MeasurementPosition derived from a
 * aspn::xtensor::MeasurementPositionVelocityAttitude.
 *
 * @param r Source reference frame
 *
 * @return The matching MeasurementPosition reference frame. \p r is not recognized will default
 * to ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC.
 *
 * @throw std::invalid_argument if error mode is DIE and \p r is not recongnized.
 */
AspnMeasurementPositionReferenceFrame convert_pva_to_pos_ref_frame(
    AspnMeasurementPositionVelocityAttitudeReferenceFrame r);
}  // namespace utils
}  // namespace navtk
