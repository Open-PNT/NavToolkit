#pragma once

#include <navtk/aspn.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Abstract class that provides accessors for some commonly desired position,
 * velocity and attitude representations.
 */
class InertialPosVelAtt : public aspn_xtensor::AspnBase {
public:
	/**
	 * Destructor.
	 */
	virtual ~InertialPosVelAtt() = default;

	/**
	 * Determines if this instance is 'wander-capable', meaning that
	 * it has a separate internal representation of a wander angle.
	 * Generally, replacing a wander-capable value with one that is not
	 * will result in a loss of information.
	 *
	 * @return If this instance is able to store a valid wander angle
	 * value.
	 */
	virtual bool is_wander_capable() const = 0;

	/**
	 * Position as latitude, longitude and altitude. The return value of
	 * this function may not be valid near the poles.
	 *
	 * @return Vector3 composed of latitude in radians, longitude in
	 * radians, and altitude in meters HAE.
	 */
	virtual Vector3 get_llh() const = 0;

	/**
	 * Velocity in the NED frame. The return value of
	 * this function may not be valid near the poles.
	 *
	 * @return Vector3 composed of velocity in the North, East and Down
	 * frame, \f$\frac{m}{s}\f$.
	 */
	virtual Vector3 get_vned() const = 0;

	/**
	 * Attitude as a sensor-to-NED frame or platform-to-NED frame DCM.
	 * (Depends on whether the mechanization is in original inertial
	 * sensor frame or the inertial measurements are first rotated
	 * to a platform frame prior to mechanization). The return value of
	 * this function may not be valid near the poles.
	 *
	 * @return DCM that rotates from the frame whose PVA is
	 * described by this instance (either sensor or platform frame) to the
	 * North, East and Down frame.
	 */
	virtual Matrix3 get_C_s_to_ned() const = 0;

	/**
	 * Position in wander frame format (DCM and altitude).
	 *
	 * @return Pair containing the n to e frame DCM and height above
	 * ellipsoid in meters.
	 */
	virtual std::pair<Matrix3, double> get_C_n_to_e_h() const = 0;

	/**
	 * Velocity in wander n frame.
	 *
	 * @return Velocity in n frame, \f$\frac{m}{s}\f$.
	 */
	virtual Vector3 get_vn() const = 0;

	/**
	 * Attitude as the sensor to l frame DCM.
	 *
	 * @return DCM that rotates from the sensor frame to the l frame,.
	 */
	virtual Matrix3 get_C_s_to_l() const = 0;

	/**
	 * Return a deep copy of the instance.
	 *
	 * @return Cloned InertialPosVelAtt.
	 */
	virtual std::shared_ptr<InertialPosVelAtt> clone() const = 0;

	/**
	 * The time at which the PVA is considered to be valid.
	 */
	aspn_xtensor::TypeTimestamp time_validity;

protected:
	/**
	 * Constructor.
	 *
	 * @param t Time of validity.
	 * @param message_type the ASPN message type assigned to the derived class that is implementing
	 * the InertialPosVelAtt interface. This will likely be specific to the running program.
	 * Defaults to ASPN_EXTENDED_BEGIN since InertialPosVelAtt extends the set of defined ASPN
	 * messages. Note, if this default value is not overridden by a program using NavToolkit, then
	 * the type assigned to the derived class of InertialPosVelAtt may conflict with another type
	 * used by that program. Users should be careful to ensure that all ASPN message types used by
	 * their program have unique types if they are using AspnBase::get_message_type to identify a
	 * message type.
	 */
	InertialPosVelAtt(const aspn_xtensor::TypeTimestamp& t = aspn_xtensor::to_type_timestamp(),
	                  AspnMessageType message_type         = ASPN_EXTENDED_BEGIN)
	    : aspn_xtensor::TypeHeader(message_type, 0, 0, 0, 0), time_validity(t){};
};

}  // namespace inertial
}  // namespace navtk
