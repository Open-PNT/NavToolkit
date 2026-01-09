#pragma once

#include <navtk/tensors.hpp>

namespace navtk {
namespace navutils {

/**
 * Partial derivative of NED to ECEF DCM wrt latitude.
 *
 * @param lla Latitude, longitude, HAE (rad, rad, m).
 *
 * @return Derivative of NED to ECEF DCM wrt latitude, evaluated at LLH.
 */
Matrix3 d_cen_wrt_lat(const Vector3& lla);

/**
 * Partial derivative of NED to ECEF DCM wrt longitude.
 *
 * @param lla Latitude, longitude, HAE (rad, rad, m).
 *
 * @return Jacobian of NED to ECEF DCM wrt longitude, evaluated at LLH.
 */
Matrix3 d_cen_wrt_lon(const Vector3& lla);

/**
 * Derivative of sensor to reference DCM wrt roll.
 *
 *
 * @param rpy Roll, pitch, yaw in radians.
 *
 * @return Derivative of sensor to reference frame DCM wrt roll, evaluated at `rpy`.
 */
Matrix3 d_cns_wrt_r(const Vector3& rpy);

/**
 * Derivative of sensor to reference DCM wrt pitch.
 *
 *
 * @param rpy Roll, pitch, yaw in radians.
 *
 * @return Derivative of sensor to reference frame DCM wrt pitch, evaluated at `rpy`.
 */
Matrix3 d_cns_wrt_p(const Vector3& rpy);

/**
 * Derivative of sensor to reference DCM wrt yaw.
 *
 *
 * @param rpy Roll, pitch, yaw in radians.
 *
 * @return Derivative of sensor to reference frame DCM wrt yaw, evaluated at `rpy`.
 */
Matrix3 d_cns_wrt_y(const Vector3& rpy);

/**
 * Derivative of the llh_to_ecef() function wrt `llh`.
 *
 * @param llh Latitude, longitude and HAE (rad, rad, meters).
 *
 * @return Derivative of llh_to_ecef() wrt and evaluated at `llh`.
 */
Matrix3 d_llh_to_ecef_wrt_llh(const Vector3& llh);

/**
 * Derivative of the ecef_to_llh() navutils function wrt `ecef`. Since ecef_to_llh() is an
 * iterative function and not directly invertible, this isn't a true deterministic derivative (it is
 * derived from the ecef_to_llh() function) and tends to be inaccurate along the bottom row (alt
 * term), but the effective error is low (about < 1e-6 m/m).
 *
 * @param ecef XYZ ECEF coordinates, in meters.
 *
 * @return Derivative of ecef_to_llh() wrt and evaluated at `ecef`.
 */
Matrix3 d_ecef_to_llh_wrt_ecef(const Vector3& ecef);

/**
 * Derivative of ECEF to NED DCM wrt one element of ECEF vector.
 *
 * @param dk Derivative of ecef_to_llh() with respect to `ecef` x, y or z; obtainable by selecting
 * the correct column of d_ecef_to_llh_wrt_ecef() return value (0 for x, 1, for y, 2 for z).
 * @param llh Latitude, longitude and HAE (rad, rad, meters).
 *
 * @return Derivative of ECEF to NED DCM wrt ECEF x, y or z coordinate, depending on the value of
 * `dk`.
 */
Matrix3 d_cne_wrt_k(const Vector3& dk, const Vector3& llh);

/**
 * Derivative of the dcm_to_rpy() function wrt 3 variables (usually RPY), where the DCM is composed
 * of the product of 2 other DCMs. The derivative does not exist at pitch = PI/2 (derived from
 * the composite DCM) and this function is unreliable in that region; user beware.
 *
 * Following the naming of the parameters, this is the derivative of `dcm_to_rpy(dot(C_2_to_3,
 * C_1_to_2))` where parameter `a = C_2_to_3` and `b = C_1_to_2`.
 *
 * @param a DCM `C_2_to_3`
 * @param dadx derivative of `C_2_to_3` wrt variable 1 (x Euler angle).
 * @param dady derivative of `C_2_to_3` wrt variable 2 (y Euler angle).
 * @param dadz derivative of `C_2_to_3` wrt variable 3 (z Euler angle).
 * @param b DCM `C_1_to_2`
 * @param dbdx derivative of `C_1_to_2` wrt variable 1 (x Euler angle).
 * @param dbdy derivative of `C_1_to_2` wrt variable 2 (y Euler angle).
 * @param dbdz derivative of `C_1_to_2` wrt variable 3 (z Euler angle).
 *
 * @return Derivative of RPY representation of C_1_to_3 wrt some 3-variable input.
 */
Matrix3 d_dcm_to_rpy(const Matrix3& a,
                     const Matrix3& dadx,
                     const Matrix3& dady,
                     const Matrix3& dadz,
                     const Matrix3& b,
                     const Matrix3& dbdx,
                     const Matrix3& dbdy,
                     const Matrix3& dbdz);

/**
 * Derivative of the RPY representation of the 'standard' tilt corrected DCM wrt the tilt vector.
 * In other words
 * \f$ \frac{\partial f}{\delta \psi} dcm_to_rpy(C^b_n[I - \Psi]) \f$ where \f$ \Psi \f$ is the
 * skew-symmetric format of tilt vector \f$ \psi \f$.
 *
 * @param tilts NED frame x, y, z, tilt errors.
 * @param C_nav_to_platform Nav to platform/sensor DCM.
 *
 * @return Derivative wrt and evaluated at `tilts`.
 */
Matrix3 d_rpy_tilt_corr_wrt_tilt(const Vector3& tilts, const Matrix3& C_nav_to_platform);

/**
 * Derivative of the correct_dcm_with_tilt() function wrt to the tilts. Derivative does not exist
 * when the magnitude of the tilts is 0. Result will include 'inf' in this case.
 *
 * @param tilts NED frame x, y, z, tilt errors, rad.
 * @param C_nav_to_platform Nav to platform/sensor DCM.
 *
 * @return Derivative wrt and evaluated at `tilts`.
 */
Matrix3 d_rpy_correct_dcm_with_tilt_wrt_tilt(const Vector3& tilts,
                                             const Matrix3& C_nav_to_platform);

/**
 * Derivative of the quaternion 'small rotation' propagation function, wrt tilts.
 * See Titterton + Weston 2nd ed, eqs 11.39 and 11.40. Derivative does not exist
 * when the magnitude of the rotation is 0. Result will include 'inf' in this case.
 *
 * @param q Quaternion that rotates from the platform frame at time t0 to the reference frame.
 * @param r Small rotation vector, rad.
 *
 * @return Derivative (4x3) wrt and evaluated at `r`.
 */
Matrix d_quat_prop_wrt_r(const Vector4& q, const Vector3& r);

/**
 * Derivative of the quaternion tilt correction, wrt tilts.
 *
 * @param q Quaternion that rotates from the platform frame at time to the estimated reference
 * frame.
 *
 * @return Derivative (4x3) wrt and evaluated at tilts.
 */
Matrix d_quat_tilt_corr_wrt_tilt(const Vector4& q);

/**
 * Derivative of the llh_to_quat_en() function, wrt `llh`.
 *
 * @param llh Latitude, longitude, HAE (rad, rad, meters).
 *
 * @return Derivative wrt and evaluated at `llh`.
 */
Matrix d_llh_to_quat_en_wrt_llh(const Vector3& llh);

/**
 * Derivative of the quat_to_rpy() function, wrt the quaternion.
 *
 * @param q Quaternion that rotates from platform to reference frame.
 *
 * @return Derivative (3x4) wrt and evaluated at `q`.
 */
Matrix d_quat_to_rpy_wrt_q(const Vector4& q);

/**
 * Derivative of the quat_norm() function, wrt the quaternion.
 *
 * @param q Quaternion.
 *
 * @return Derivative (4x4) of the normalization function wrt the quaternion.
 */
Matrix d_quat_norm_wrt_q(const Vector4& q);

/**
 * Derivative of the ortho_dcm() function wrt a single tilt element. Note that the
 * orthonormalization function is iterative, and this derivative is for only a single iteration.
 *
 * @param C_nav_to_platform Reference to platform DCM.
 * @param tilts Tilt correction vector, estimated reference to corrected reference (radians).
 * @param dtilt Derivative of the skew-symmetric matrix of tilts wrt the element the overall
 * derivative is to be with respect to. For instance, if C_nav_to_platform is the NED to platform
 * DCM, and the derivative is to be wrt the down (z) tilt element, then this parameter would be
 * \f$\begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix} \f$.
 *
 * @return Derivative of the corrected and orthonormalized DCM wrt one of the tilt elements.
 */
Matrix3 d_ortho_dcm_wrt_tilt(const Matrix3& C_nav_to_platform,
                             const Vector3& tilts,
                             const Matrix3& dtilt);
/**
 * Derivative of the rpy_to_dcm() function (i.e. the reference to platform DCM)
 * wrt to the roll angle.
 *
 * @param rpy Roll, pitch, yaw angles (radians).
 *
 * @return Derivative of DCM wrt roll, evaluated at `rpy`.
 */
Matrix3 d_rpy_to_dcm_wrt_r(const Vector3& rpy);

/**
 * Derivative of the rpy_to_dcm() function (i.e. the reference to platform DCM)
 * wrt to the pitch angle.
 *
 * @param rpy Roll, pitch, yaw angles (radians).
 *
 * @return Derivative of DCM wrt pitch, evaluated at `rpy`.
 */
Matrix3 d_rpy_to_dcm_wrt_p(const Vector3& rpy);

/**
 * Derivative of the rpy_to_dcm() function (i.e. the reference to platform DCM)
 * wrt to the yaw angle.
 *
 * @param rpy Roll, pitch, yaw angles (radians).
 *
 * @return Derivative of DCM wrt yaw, evaluated at `rpy`.
 */
Matrix3 d_rpy_to_dcm_wrt_y(const Vector3& rpy);

/**
 * Derivative of a 'sensor to platform' position lever arm correction, wrt the reference to sensor
 * quaternion.
 *
 * @param q_s_to_n Sensor to reference quaternion.
 * @param l_ps_p Platform to sensor lever arm, in the platform frame (meters).
 * @param C_p_to_s Platform to sensor orientation (sensor mount DCM).
 *
 * @return Derivative (3x4) of the position translation, wrt `q_s_to_n`.
 */
Matrix d_sensor_to_platform_pos_wrt_q(const Vector4& q_s_to_n,
                                      const Vector3& l_ps_p,
                                      const Matrix3& C_p_to_s);

/**
 * Derivative of a 'platform to sensor' position lever arm correction, wrt the reference to platform
 * quaternion.
 *
 * @param q_p_to_n Reference to platform quaternion.
 * @param l_ps_p Platform to sensor lever arm, in the platform frame (meters).
 *
 * @return Derivative (3x4) of the position translation, wrt `q_p_to_n`.
 */
Matrix d_platform_to_sensor_pos_wrt_q(const Vector4& q_p_to_n, const Vector3& l_ps_p);
}  // namespace navutils
}  // namespace navtk
