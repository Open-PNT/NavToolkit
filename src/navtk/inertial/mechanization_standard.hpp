#pragma once

#include <memory>

#include <navtk/inertial/AidingAltData.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Perform a single mechanization of delta velocity and delta rotation
 * measurements using NED-frame-referenced attitude and velocity and
 * WGS84 polar coordinates. Based on equations found in Titterton and Weston,
 * Strapdown Inertial Navigation Technology. Not for use near poles.
 *
 * @param dv_s Vector3 of delta velocity measurements in the inertial
 * sensor frame, m/s.
 * @param dth_s Vector3 of delta rotations in the inertial sensor
 * frame, rad.
 * @param dt Time over which `dv_s` and `dth_s` were collected (and thus
 * time to mechanize over), seconds.
 * @param llh0 Latitude, longitude and ellipsoidal altitude at the start
 * of mechanization, in radians, radians, meters respectively.
 * @param C_s_to_n0 DCM that rotates from the inertial sensor frame to
 * the NED frame.
 * @param v_ned0 NED frame velocity measurements at the start of
 * mechanization, m/s.
 * @param v_ned_prev NED frame velocity measurement
 * @param mech_options Mechanization options to use. This function
 * only honors the `grav_model`, `dcm_method` and `int_method ` fields.
 * @param aiding_alt_data Container holding an external altitude measurement in m HAE, and the
 * accumulated error resulting from the aiding the altitude in the inertial's mechanization.
 *
 * @return A 3-element tuple consisting of the post-mechanization values
 * of position, velocity and attitude (in same format as `llh0`, `v_ned0`
 * and `C_s_to_n0`, respectively).
 */
std::tuple<Vector3, Vector3, Matrix3> mechanization_standard(
    const Vector3& dv_s,
    const Vector3& dth_s,
    double dt,
    const Vector3& llh0,
    const Matrix3& C_s_to_n0,
    const Vector3& v_ned0,
    const Vector3& v_ned_prev,
    const MechanizationOptions& mech_options = MechanizationOptions{},
    AidingAltData* aiding_alt_data           = nullptr);

/**
 * Perform a single mechanization of delta velocity and delta rotation
 * measurements using NED-frame-referenced attitude and velocity and
 * WGS84 polar coordinates. Based on equations found in Titterton and Weston,
 * Strapdown Inertial Navigation Technology. Not for use near poles.
 *
 * @param dv_s Vector3 of delta velocity measurements in the inertial
 * sensor frame, m/s.
 * @param dth_s Vector3 of delta rotations in the inertial sensor
 * frame, rad.
 * @param dt Time over which `dv_s` and `dth_s` were collected (and thus
 * time to mechanize over), seconds.
 * @param pva Position, velocity and attitude data at the start of
 * the mechanization interval.
 * @param old_pva Position, velocity, and attitude data from the start of the
 * previous mechanization interval.
 * @param mech_options Mechanization options to use. This function
 * only honors the `grav_model`, `dcm_method` and `int_method ` fields.
 * @param aiding_alt_data Container holding an external altitude measurement in m HAE, the
 * accumulated error resulting from the aiding the altitude in the inertial's mechanization, and
 * additional options for the aiding algorithm.
 *
 * @return Non-null `shared_ptr` to a StandardPosVelAtt instance that contains the
 * post-mechanization PVA values.
 */
not_null<std::shared_ptr<InertialPosVelAtt>> mechanization_standard(
    const Vector3& dv_s,
    const Vector3& dth_s,
    double dt,
    const not_null<std::shared_ptr<InertialPosVelAtt>> pva,
    const not_null<std::shared_ptr<InertialPosVelAtt>> old_pva,
    const MechanizationOptions& mech_options = MechanizationOptions{},
    AidingAltData* aiding_alt_data           = nullptr);

}  // namespace inertial
}  // namespace navtk
