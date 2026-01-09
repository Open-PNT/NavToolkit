#pragma once

#include <navtk/aspn.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace testing {

/**
 * Simulate an IMU measurement assuming fixed position and attitude. Uses
 * navutils::calculate_gravity_schwartz for gravity values.
 *
 * @param pva Position, velocity (assumed 0) and attitude of the sensor.
 * @param dt Delta time (seconds) over which the measurement is simulated.
 *
 * @return An Imu measurement in the IMU sensor frame containing only the effects of gravity and
 * Earth rotation.
 */
aspn_xtensor::MeasurementImu stationary_imu(
    not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> pva, double dt);

/**
 * Generates a set of observations containing all possible permutations of the given data.
 * Generated observations always have `LLI` `signal_strength` and `lock_count` set to 0.
 * The parameters of each observation are encoded into the measurement in the following way:
 *
 * 1 plus the integer value of AttributeValue -> X.00
 * e.g. SIG_P (P(Y)) = 1, SIG_C (C/A) = 2
 *
 * 1 plus the integer value of TypeValue -> 0.X0
 * e.g. OBS_C (pr) = 1, OBS_L (cp) = 2, OBS_D (dop) = 3
 *
 * 1 plus the integer value of BandValue -> 0.0X
 * e.g. BAND1 (L1) = 1, BAND2 (L2) = 2
 *
 * SatelliteSystemValue is not encoded into the measurement.
 *
 * @param prns_to_use List of PRN values to use for generation.
 * @param sat_sys_to_use List of satellite system values to use for generation.
 * @param descriptors_to_use List of descriptor values to use for generation.
 *
 * @return A set of observations containing all possible permutations of the given data.
 */
std::vector<aspn_xtensor::TypeSatnavObs> generate_gnss_observations(
    const std::vector<int32_t>& prns_to_use,
    const std::vector<aspn_xtensor::TypeSatnavSatelliteSystem>& sat_sys_to_use,
    const std::vector<AspnTypeSatnavSignalDescriptorSignalDescriptor>& descriptors_to_use);

}  // namespace testing
}  // namespace navtk
