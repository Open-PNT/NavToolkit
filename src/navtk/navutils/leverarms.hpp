#pragma once

#include <navtk/factory.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace navutils {

/**
 * Calculate the pose (position and orientation) of the platform frame given
 * the pose of a sensor frame and the relationship (pose/lever arm and
 * relative orientation) between the platform frame and the sensor frame.  The pose of the sensor
 * frame can be expressed relative to two different coordinate frames,
 *
 * \f$ \mathbf{C}^\text{P}_\text{K} = \mathbf{C}^\text{P}_\text{S} \mathbf{C}^\text{S}_\text{K} \f$
 *
 * \f$ \mathbf{p}^\text{J}_{\text{J}\to \text{P}} = \mathbf{p}^\text{J}_{\text{J}\to \text{S}} -
 * \mathbf{C}^\text{J}_\text{K} \mathbf{C}^\text{K}_\text{P} \mathbf{p}^\text{P}_{\text{P}\to
 * \text{S}} \f$
 *
 * @param sensor_pose A `std::pair` consisting of the position of the sensor
 * frame in some 3D Cartesian frame \f$J\f$(such as the ECEF frame)
 * \f$ \mathbf{p}^\text{J}_{\text{J}\to \text{S}}\f$, and the DCM that rotates from some
 * (potentially) other Cartesian frame \f$K\f$(such as the NED frame) to the sensor frame
 * \f$\mathbf{C}^\text{S}_\text{K} \f$. Frames \f$J\f$and \f$K\f$can be the same frame.
 *
 * @param platform_to_sensor_in_platform A 'lever arm' pointing from the origin of
 * the platform frame to the origin of the sensor frame, coordinatized in
 * the platform frame, \f$\mathbf{p}^\text{P}_{\text{P}\to \text{S}}\f$. Units should be the same as
 * those used with \p sensor_pose .
 *
 * @param C_platform_to_sensor The DCM that rotates from the platform frame to the sensor
 * frame, \f$\mathbf{C}^\text{S}_\text{P}\f$.
 *
 * @param C_k_to_j The DCM that rotates from the \f$K\f$frame to the \f$J\f$frame,
 * \f$\mathbf{C}^\text{J}_\text{K}\f$. Defaults to identity matrix.
 *
 * @return A `std::pair` consisting of \f$\mathbf{p}^\text{J}_{\text{J}\to \text{P}}\f$ and
 * \f$\mathbf{C}^\text{P}_\text{K}\f$.
 */
std::pair<Vector3, Matrix3> sensor_to_platform(const std::pair<Vector3, Matrix3>& sensor_pose,
                                               const Vector3& platform_to_sensor_in_platform,
                                               const Matrix3& C_platform_to_sensor,
                                               const Matrix3& C_k_to_j = eye(3));

/**
 * Calculate the pose (position and orientation) of a sensor frame given
 * the pose of the platform frame and the relationship (pose/lever arm and
 * relative orientation) between the platform frame and the sensor frame.
 *
 * \f$ \mathbf{C}^\text{S}_\text{K} = \mathbf{C}^\text{S}_\text{P} \mathbf{C}^\text{P}_\text{K} \f$
 *
 * \f$ \mathbf{p}^\text{J}_{\text{J}\to \text{S}} = \mathbf{p}^\text{J}_{\text{J}\to \text{P}} +
 * \mathbf{C}^\text{J}_\text{K} \mathbf{C}^\text{K}_\text{P} \mathbf{p}^\text{P}_{\text{P}\to
 * \text{S}} \f$
 *
 * @param platform_pose A `std::pair` consisting of the position of the platform
 * frame in some 3D Cartesian frame \f$J\f$ (such as the ECEF frame)
 * \f$ \mathbf{p}^\text{J}_{\text{J}\to \text{P}}\f$, and the DCM that rotates from some
 * (potentially) other Cartesian frame \f$K\f$ (such as the NED frame) to the platform frame
 * \f$\mathbf{C}^\text{P}_\text{K} \f$. Frames \f$J\f$ and \f$K\f$ can be the same frame.
 *
 * @param platform_to_sensor_in_platform A 'lever arm' pointing from the origin of
 * the platform frame to the origin of the sensor frame, coordinatized in
 * the platform frame, \f$\mathbf{p}^\text{P}_{\text{P}\to \text{S}}\f$. Units should be the same as
 * those used with \p platform_pose .
 *
 * @param C_platform_to_sensor The DCM that rotates from the platform frame to the sensor
 * frame, \f$\mathbf{C}^\text{S}_\text{P}\f$.
 *
 * @param C_k_to_j The DCM that rotates from the \f$K\f$ frame to the \f$J\f$ frame,
 * \f$\mathbf{C}^\text{J}_\text{K}\f$.  Defaults to identity matrix.
 *
 * @return A `std::pair` consisting of \f$\mathbf{p}^\text{J}_{\text{J}\to \text{S}}\f$ and
 * \f$\mathbf{C}^\text{S}_\text{K}\f$.
 */
std::pair<Vector3, Matrix3> platform_to_sensor(const std::pair<Vector3, Matrix3>& platform_pose,
                                               const Vector3& platform_to_sensor_in_platform,
                                               const Matrix3& C_platform_to_sensor,
                                               const Matrix3& C_k_to_j = eye(3));

/**
 * Calculate the relative position and orientation of an observation
 * in the sensor frame from the same observation in the platform frame.
 *
 * \f$ \mathbf{C}^\text{O}_\text{S} = \mathbf{C}^\text{O}_\text{P} \mathbf{C}^\text{P}_\text{S} \f$
 *
 * \f$ \mathbf{p}^\text{S}_{\text{S}\to \text{O}} = \mathbf{C}^\text{S}_\text{P}(
 * \mathbf{p}^\text{P}_{\text{P}\to \text{O}} - \mathbf{p}^\text{P}_{\text{P}\to \text{S}}) \f$
 *
 * @param obs_in_platform A `std::pair` consisting of the coordinates of the
 * observation frame in the platform frame \f$ \mathbf{p}^\text{P}_{\text{P}\to \text{O}} \f$, and
 * the DCM that rotates from the platform frame to the observation frame \f$
 * \mathbf{C}^\text{O}_\text{P} \f$.
 *
 * @param platform_to_sensor_in_platform A 'lever arm' pointing from the origin of
 * the platform frame to the origin of the sensor frame, coordinatized in
 * the platform frame, \f$\mathbf{p}^\text{P}_{\text{P}\to \text{S}}\f$. Units should be the same as
 * those used with \p obs_in_platform .
 *
 * @param C_platform_to_sensor The DCM that rotates from the platform frame to the sensor
 * frame, \f$\mathbf{C}^\text{S}_\text{P}\f$.
 *
 * @return A `std::pair` consisting of the relative position of the
 * observed frame in the sensor frame \f$ \mathbf{p}^\text{S}_{\text{S}\to \text{O}} \f$, and the
 * DCM that rotates from the sensor frame to the observed frame \f$ \mathbf{C}^\text{O}_\text{S}\f$.
 */
std::pair<Vector3, Matrix3> obs_in_platform_to_sensor(
    const std::pair<Vector3, Matrix3>& obs_in_platform,
    const Vector3& platform_to_sensor_in_platform,
    const Matrix3& C_platform_to_sensor);

/**
 * Calculate the relative position and orientation of an observation
 * in the platform frame from the same observation in some sensor frame.
 *
 * \f$ \mathbf{C}^\text{O}_\text{P} = \mathbf{C}^\text{O}_\text{S} \mathbf{C}^\text{S}_\text{P} \f$
 *
 * \f$ \mathbf{p}^\text{P}_{\text{P}\to \text{O}} = \mathbf{p}^\text{P}_{\text{P}\to \text{S}} +
 * \mathbf{C}^\text{P}_\text{S} \mathbf{p}^\text{S}_{\text{S}\to \text{O}}\f$
 *
 * @param obs_in_sensor A `std::pair` consisting of the coordinates of the
 * observation frame in the sensor frame \f$ \mathbf{p}^\text{S}_{\text{S}\to \text{O}} \f$, and the
 * DCM that rotates from the sensor frame to the observation frame \f$ \mathbf{C}^\text{O}_\text{S}
 * \f$.
 *
 * @param platform_to_sensor_in_platform A 'lever arm' pointing from the origin of
 * the platform frame to the origin of the sensor frame, coordinatized in
 * the platform frame, \f$\mathbf{p}^\text{P}_{\text{P}\to \text{S}}\f$. Units should be the same as
 * those used with \p obs_in_sensor .
 *
 * @param C_platform_to_sensor The DCM that rotates from the platform frame to the sensor
 * frame, \f$\mathbf{C}^\text{S}_\text{P}\f$.
 *
 * @return A `std::pair` consisting of the relative position of the
 * observed frame in the platform frame \f$ \mathbf{p}^\text{P}_{\text{P}\to \text{O}} \f$, and the
 * DCM that rotates from the platform frame to the observed frame \f$
 * \mathbf{C}^\text{O}_\text{P}\f$.
 */
std::pair<Vector3, Matrix3> obs_in_sensor_to_platform(
    const std::pair<Vector3, Matrix3>& obs_in_sensor,
    const Vector3& platform_to_sensor_in_platform,
    const Matrix3& C_platform_to_sensor);

}  // namespace navutils
}  // namespace navtk
