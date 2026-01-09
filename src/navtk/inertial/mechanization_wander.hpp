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
 * Performs a single mechanization in the wander azimuth frame. Based on
 * equations found in Strapdown Analytics by Paul Savage, primarily
 * Vol 1, Ch. 7.
 *
 * @param dv_s Vector3 of delta velocity measurements in the
 * inertial sensor frame, m/s.
 * @param dth_s Vector3 of delta rotation measurements in the
 * inertial sensor frame, rad.
 * @param dt Delta time over which `dv_s` and `dth_s` are valid,
 * seconds.
 * @param C_n_to_e_0 DCM that rotates from Savage defined 'N' frame to
 * the Savage defined 'E' frame. Encodes position/wander angle at the
 * starting point of mechanization.
 * @param h0 Height above ellipsoid at the start of mechanization, meters.
 * @param v_n_0 Vector3 of X, Y, Z velocities in the Savage defined 'N'
 * frame, m/s.
 * @param C_s_to_l_0 DCM that rotates from the inertial sensor frame to
 * the Savage defined 'L' frame at the starting point of mechanization.
 * @param mech_options Mechanization options to use. This function
 * only honors the `grav_model` and `earth_model` fields.
 * @param aiding_alt_data Container holding an external altitude measurement in m HAE, the
 * accumulated error resulting from the aiding the altitude in the inertial's mechanization, and
 * additional options for the aiding algorithm.
 *
 * @return Tuple consisting of the post-mechanization position matrix,
 * height, velocity and orientation DCM, following the same format as
 * `C_n_to_e_0`, `h0`, `v_n_0`, `C_s_to_l_0` respectively.
 */
std::tuple<Matrix3, double, Vector3, Matrix3> mechanization_wander(
    const Vector3& dv_s,
    const Vector3& dth_s,
    double dt,
    const Matrix3& C_n_to_e_0,
    double h0,
    const Vector3& v_n_0,
    const Matrix3& C_s_to_l_0,
    const MechanizationOptions& mech_options = MechanizationOptions{},
    AidingAltData* aiding_alt_data           = nullptr);

/**
 * Perform a single mechanization of delta velocity and delta rotation
 * measurements in the wander azimuth frame. Based on equations found in
 * Strapdown Analytics by Paul Savage, primarily Vol 1, Ch. 7.
 *
 * @param dv_s Vector3 of delta velocity measurements in the inertial
 * sensor frame, m/s.
 * @param dth_s Vector3 of delta rotations in the inertial sensor
 * frame, rad.
 * @param dt Time over which `dv_s` and `dth_s` were collected (and thus
 * time to mechanize over), seconds.
 * @param pva Position, velocity and attitude data at the start of
 * the mechanization interval. Ideally an instance that incorporates
 * knowledge of the wander frame (such as WanderPosVelAtt);
 * otherwise mechanization is performed assuming a starting wander angle
 * of 0.
 * @param mech_options Mechanization options to use. This function
 * only honors the `grav_model` and `earth_model` fields.
 * @param aiding_alt_data Container holding an external altitude measurement in m HAE, and the
 * accumulated error resulting from the aiding the altitude in the inertial's mechanization.
 *
 * @return Non-null `shared_ptr` to a WanderPosVelAtt instance that contains the
 * post-mechanization PVA values.
 */
not_null<std::shared_ptr<InertialPosVelAtt>> mechanization_wander(
    const Vector3& dv_s,
    const Vector3& dth_s,
    double dt,
    const not_null<std::shared_ptr<InertialPosVelAtt>> pva,
    const not_null<std::shared_ptr<InertialPosVelAtt>>,
    const MechanizationOptions& mech_options = MechanizationOptions{},
    AidingAltData* aiding_alt_data           = nullptr);
}  // namespace inertial
}  // namespace navtk
