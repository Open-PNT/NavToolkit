#pragma once

#include <navtk/tensors.hpp>

namespace navtk {
namespace navutils {

/**
 * Complex conjugate of the quaternion. This is analogous to a DCM transpose.
 *
 * @see [Coordinate Frames](../tutorial/coordinate_frames.html) for more details
 * about quaternions.
 *
 * @param q Quaternion with real part as first element.
 * @return Complex conjugate; real element stays the same and non-real elements are negated.
 */
Vector4 quat_conj(const Vector4& q);

/**
 * Normalize the quaternion to magnitude 1.
 *
 * @see [Coordinate Frames](../tutorial/coordinate_frames.html) for more details
 * about quaternions.
 *
 * @param q Quaternion to normalize.
 *
 * @return Normalized quaternion.
 */
Vector4 quat_norm(const Vector4& q);

/**
 * Multiplies 2 quaternions together by quaternion multiplication. See Titterton and Weston, 2nd ed.
 * eq 3.56.
 *
 * For this function, if `p` is the quaternion that describes the rotation from frame C
 * to frame B
 *
 * `p` = \f$\textbf{q}_\text{C}^\text{B}\f$
 *
 * and if `q` is the quaternion that describes the rotation from frame B
 * to frame A
 *
 * `q` = \f$\textbf{q}_\text{B}^\text{A}\f$
 *
 * then this function will return the quaternion that describes the rotation
 * from frame C to frame A
 *
 * \f$\textbf{q}_\text{C}^\text{A} = \textbf{q}_\text{B}^\text{A}\textbf{q}_\text{C}^\text{B}\f$
 *
 * @see [Coordinate Frames](../tutorial/coordinate_frames.html) for more details
 * about quaternions and how they are multiplied.
 *
 * @param p Quaternion that rotates a vector from frame C to frame B
 * @param q Quaternion that rotates a vector from frame B to frame A
 *
 * @return Composite quaternion that rotates a vector from frame C to frame A
 */
Vector4 quat_mult(const Vector4& q, const Vector4& p);

/**
 * Rotate a 3-dimensional vector using a quaternion.
 *
 * This function will output the vector `r` after applying a vector rotation
 * described by the quaternion `q`.
 *
 * @see [Coordinate Frames](../tutorial/coordinate_frames.html) for more details
 * about quaternions.
 *
 * @param q Quaternion that describes a rotation to be applied
 * @param r Input 3-length vector
 *
 * @return Vector obtained by rotating `r` by `q`.
 */
Vector3 quat_rot(const Vector4& q, const Vector3& r);

/**
 * Propagate a quaternion. See Titterton and Weston 2nd ed. eqs 11.39-11.42.
 *
 * Consider a quaternion `q` that describes the *frame rotation* to
 * rotate a coordinate frame from frame \f$\text{A}\f$ to frame \f$\text{B}\f$, expressed as
 * \f$\textbf{q}_\text{B}^\text{A}\f$.  (Note that this is equivalent to *vector
 * rotation* required to rotate a vector from frame \f$\text{B}\f$
 * to frame \f$\text{A}\f$.  See Appendix
 * in [Coordinate Frames](../tutorial/coordinate_frames.html) for more details
 * on the difference between a frame rotation and a vector rotation.)
 *
 * Now, consider we have an incremental rotation `r` of the \f$\text{B}\f$ frame that causes
 * the \f$\text{B}\f$ frame to have a slight rotation to become
 * the \f$\text{B}^{\prime}\f$ frame.  For this function, this incremental rotation is expressed as
 * a rotation vector which describes the frame rotation to apply to convert frame \f$\text{B}\f$
 * into frame \f$\text{B}^{\prime}\f$.
 *
 * In the above case, the quaternion that will be output by the function is
 * \f$\textbf{q}_{\text{B}^{\prime}}^\text{A}\f$.
 *
 * @see [Coordinate Frames](../tutorial/coordinate_frames.html) for more details
 * about quaternions.
 *
 * @param q Quaternion that describes a frame rotation from frame
 * \f$\text{A}\f$ to frame \f$\text{B}\f$ (\f$\textbf{q}_\text{B}^\text{A}\f$)
 * @param r Rotation vector describing an incremental rotation of
 * the \f$\text{B}\f$ frame to become the \f$\text{B}^{\prime}\f$
 *
 * @return Quaternion describing the frame rotation from frame \f$\text{A}\f$
 * to frame \f$\text{B}^{\prime}\f$ (\f$\textbf{q}_{\text{B}^{\prime}}^\text{A}\f$)
 */
Vector4 quat_prop(const Vector4& q, const Vector3& r);

/**
 * Calculate the quaternion that rotates a vector from the local NED frame to the
 * ECEF frame (\f$\textbf{q}_\text{NED}^\text{ECEF}\f$).
 *
 * @see [Coordinate Frames](../tutorial/coordinate_frames.html) for more details
 * about quaternions.
 *
 * @param llh Position expressed as length 3 vector of latitude, longitude and ellipsoidal height
 * (radians, radians, meters)
 *
 * @return NED to ECEF quaternion \f$\textbf{q}_\text{NED}^\text{ECEF}\f$
 */
Vector4 llh_to_quat_en(const Vector3& llh);

/**
 *
 * Corrects a quaternion with a vector of tilts.
 *
 * If the quaternion `q` describes the vector rotation from frame \f$\text{B}\f$
 * to frame \f$\text{A}\f$ (\f$\textbf{q}_\text{B}^\text{A}\f$), and the vector `t` represents the
 * small "tilt" corrections that describe the frame rotation from the \f$\text{A}_{corrected}\f$
 * frame to the \f$\text{A}\f$ frame, then this function will return the
 * quaternion representing a vector rotation from the \f$\text{B}\f$ frame to the
 * \f$\text{A}_{corrected}\f$ frame (\f$\textbf{q}_\text{B}^{\text{A}_{corrected}}\f$).
 *
 * By way of example, suppose that a quaternion is being calculated by an inertial
 * mechanization algorithm which describes the sensor frame \f$\text{S}\f$ relative to what is
 * assumed to be the \f$\text{NED}\f$ frame (\f$\textbf{q}_\text{S}^{\text{NED}_{assumed}}\f$).
 * There are tilt errors (perhaps estimated by a Kalman filter) of \f$\epsilon_x\f$,
 * \f$\epsilon_y\f$, and \f$\epsilon_z\f$ which describe how much the true NED frame would need to
 * be rotated about the x, y, and z axes to become the \f$\text{NED}_{assumed}\f$ frame.  When these
 * rotations (which are essentially a rotation vector) are converted into a quaternion, this frame
 * rotation is expressed, following the conventions described in [Coordinate
 * Frames](../tutorial/coordinate_frames.html), as
 * \f$\textbf{q}_{\text{NED}_{assumed}}^{\text{NED}_{true}}\f$.
 *
 * The output of this function in this case is
 *
 * \f$\textbf{q}_\text{S}^{\text{NED}_{true}} =
 * \textbf{q}_{\text{NED}_{assumed}}^{\text{NED}_{true}}
 * \textbf{q}_{S}^{\text{NED}_{assumed}}\f$
 *
 * Note that this definition is consistent with the definition of tilts used in Titterton
 * and Weston 2nd ed, eq 12.6.
 *
 * @param q Quaternion that describes a vector rotation from arbitrary frame \f$\text{B}\f$
 * to arbitrary frame \f$\text{A}\f$: \f$\textbf{q}_{\text{B}}^{\text{A}}\f$
 * @param t Vector of tilt rotations about the x, y, and z axes that describe
 * a frame rotation from frame \f$\text{A}_{corrected}\f$ to frame \f$\text{A}\f$
 *
 * @return Quaternion that describes a vector rotation from frame \f$\text{B}\f$
 * to frame \f$\text{A}_{corrected}\f$ (\f$\textbf{q}_\text{B}^{\text{A}_{corrected}}\f$)
 */
Vector4 correct_quat_with_tilt(const Vector4& q, const Vector3& t);

}  // namespace navutils
}  // namespace navtk
