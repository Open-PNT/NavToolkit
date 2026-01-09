#pragma once

#include <datasources/lcm/messages/aspn/altitude.hpp>
#include <datasources/lcm/messages/aspn/deltaposition1d.hpp>
#include <datasources/lcm/messages/aspn/deltaposition3d.hpp>
#include <datasources/lcm/messages/aspn/directiontoknownfeature3d.hpp>
#include <datasources/lcm/messages/aspn/geodeticposition2d.hpp>
#include <datasources/lcm/messages/aspn/geodeticposition3d.hpp>
#include <datasources/lcm/messages/aspn/gnss.hpp>
#include <datasources/lcm/messages/aspn/gpsephemeris.hpp>
#include <datasources/lcm/messages/aspn/imu.hpp>
#include <datasources/lcm/messages/aspn/overhausermagnetometer.hpp>
#include <datasources/lcm/messages/aspn/positionvelocity.hpp>
#include <datasources/lcm/messages/aspn/positionvelocityattitude.hpp>
#include <datasources/lcm/messages/aspn/rangetoknownfeature.hpp>
#include <datasources/lcm/messages/aspn/types/header.hpp>
#include <datasources/lcm/messages/aspn/velocity1d.hpp>
#include <datasources/lcm/messages/aspn/velocity2d.hpp>
#include <datasources/lcm/messages/aspn/velocity3d.hpp>
#include <gps_ins/utils/results.hpp>
#include <lcm/lcm-cpp.hpp>
#include <navtk/aspn.hpp>

namespace navtk {
namespace exampleutils {

/**
 * Takes in the current truth message data of time valid, LLH, NED velocity, and attitude and stores
 * this data to a struct of type TruthResults.
 * @param truth struct containing the truth message data (time of validity, LLH, NED velocity, and
 * attitude (RPY))
 * @param truth_raw LCM type positionvelocityattitude that holds the truth data message
 */
void store_truth_lcm(TruthResults &truth,
                     const datasources::lcm::messages::aspn::positionvelocityattitude &truth_raw);

/**
 * Converts from lcm geodeticposition3d to ASPN MeasurementPosition type.
 * @param t lcm geodeticposition3d type object
 * @return ASPN MeasurementPosition type object
 */
aspn_xtensor::MeasurementPosition to_aspn(
    const datasources::lcm::messages::aspn::geodeticposition3d &t);

/**
 * Converts from gnss to ASPN Gnss type.
 * @param t lcm gnss type object
 * @return ASPN Gnss type object
 */
aspn_xtensor::MeasurementSatnav to_aspn(const datasources::lcm::messages::aspn::gnss &t);

/**
 * Converts from gnss_obs to ASPN GnssObs type.
 * @param t lcm gnss_obs type object
 * @return ASPN GnssObs type object
 */
aspn_xtensor::TypeSatnavObs to_aspn(
    const datasources::lcm::messages::aspn::nested::gnss::gnss_obs &t);

/**
 * Converts from gpsephemeris to ASPN GpsEphemeris type.
 * @param t lcm gpsephemeris type object
 * @return ASPN GpsEphemeris type object
 */
aspn_xtensor::MetadataGpsLnavEphemeris to_aspn(
    const datasources::lcm::messages::aspn::gpsephemeris &t);

/**
 * Converts from lcm imu type to the ASPN Imu type.
 * @param t lcm imu type object
 * @return ASPN Imu type object
 */
aspn_xtensor::MeasurementImu to_aspn(const datasources::lcm::messages::aspn::imu &t);

/**
 * Converts from lcm positionvelocityattitude type to the ASPN
 * MeasurementPositionVelocityAttitude type.
 * @param t lcm positionvelocityattitude type object
 * @return ASPN MeasurementPositionVelocityAttitude type object
 */
aspn_xtensor::MeasurementPositionVelocityAttitude to_aspn(
    const datasources::lcm::messages::aspn::positionvelocityattitude &t);

}  // namespace exampleutils
}  // namespace navtk
