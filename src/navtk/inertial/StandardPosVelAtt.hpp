#pragma once

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/linear_algebra.hpp>

namespace navtk {
namespace inertial {

/**
 * Implementation of InertialPosVelAtt designed for use in navigation
 * that does not approach the poles, where mathematical issues with
 * NED frame representations arise.
 */
class StandardPosVelAtt : public InertialPosVelAtt {
public:
	/**
	 * Constructor.
	 *
	 * @param time Time of validity.
	 * @param llh Latitude, longitude and altitude (rad, rad, m HAE).
	 * @param vned North, East and Down velocity, \f$\frac{m}{s}\f$.
	 * @param C_s_to_ned Sensor to NED frame DCM.
	 * @param message_type the ASPN message type assigned to StandardPosVelAtt. This will likely be
	 * specific to the running program. Defaults to ASPN_EXTENDED_BEGIN since StandardPosVelAtt
	 * extends the set of defined ASPN messages. Note, if this default value is not overridden by a
	 * program using NavToolkit, then the type assigned to StandardPosVelAtt may conflict with
	 * another type used by that program. Users should be careful to ensure that all ASPN message
	 * types used by their program have unique types if they are using AspnBase::get_message_type to
	 * identify a message type.
	 */
	StandardPosVelAtt(
	    const aspn_xtensor::TypeTimestamp& time = aspn_xtensor::TypeTimestamp((int64_t)0),
	    Vector3 llh                             = zeros(3),
	    Vector3 vned                            = zeros(3),
	    Matrix3 C_s_to_ned                      = eye(3),
	    AspnMessageType message_type            = ASPN_EXTENDED_BEGIN);

	/**
	 * Constructor.
	 *
	 * @param time Time of validity.
	 * @param tup Tuple of llh, vned and C_s_to_ned (see other constructor).
	 * @param message_type the ASPN message type assigned to StandardPosVelAtt. This will likely be
	 * specific to the running program. Defaults to ASPN_EXTENDED_BEGIN since StandardPosVelAtt
	 * extends the set of defined ASPN messages. Note, if this default value is not overridden by a
	 * program using NavToolkit, then the type assigned to StandardPosVelAtt may conflict with
	 * another type used by that program. Users should be careful to ensure that all ASPN message
	 * types used by their program have unique types if they are using AspnBase::get_message_type to
	 * identify a message type.
	 */
	StandardPosVelAtt(const aspn_xtensor::TypeTimestamp& time,
	                  const std::tuple<Vector3, Vector3, Matrix3>& tup,
	                  AspnMessageType message_type = ASPN_EXTENDED_BEGIN);

	/**
	 * Return a deep copy of the instance.
	 *
	 * @return Clone.
	 */
	std::shared_ptr<InertialPosVelAtt> clone() const override;

	Vector3 get_llh() const override;
	Vector3 get_vned() const override;
	Matrix3 get_C_s_to_ned() const override;

	/**
	 * Position in wander frame format (DCM and altitude).
	 *
	 * @return Pair containing the n to e frame DCM and height above
	 * ellipsoid in meters. The DCM is constructed with a wander angle
	 * of 0, as this class does not have knowledge of the wander angle.
	 * If wander angle is known, use the overloaded version.
	 */
	std::pair<Matrix3, double> get_C_n_to_e_h() const override;

	/**
	 * Velocity in n frame. As this class has no internal knowledge of
	 * the wander angle, this is effectively an ENU frame representation.
	 *
	 * @return Velocity in the n frame, \f$\frac{m}{s}\f$.
	 */
	Vector3 get_vn() const override;

	/**
	 * Attitude as a sensor to l frame DCM. As this class has no internal
	 * knowledge of the wander angle, this is effectively the sensor to
	 * NED frame DCM.
	 *
	 * @return Sensor to l frame DCM.
	 */
	Matrix3 get_C_s_to_l() const override;

	/**
	 * Position in wander frame format (DCM and altitude), calculated
	 * using externally supplied wander angle.
	 *
	 * @param wander Wander angle, rad.
	 *
	 * @return Pair containing the n to e frame DCM, and the altitude
	 * in meters HAE.
	 */
	std::pair<Matrix3, double> get_C_n_to_e_h(double wander) const;

	/**
	 * Velocity in n frame, calculated using externally supplied wander
	 * angle.
	 *
	 * @param wander Wander angle, rad.
	 *
	 * @return Velocity in the n frame, \f$\frac{m}{s}\f$.
	 */
	Vector3 get_vn(double wander) const;

	/**
	 * Attitude as a sensor to l frame DCM, calculated using externally
	 * supplied wander angle.
	 *
	 * @param wander Wander angle, rad.
	 *
	 * @return Sensor to l frame DCM.
	 */
	Matrix3 get_C_s_to_l(double wander) const;

	bool is_wander_capable() const override { return false; }

private:
	Vector3 llh;
	Vector3 vned;
	Matrix3 C_s_to_ned;
};

}  // namespace inertial
}  // namespace navtk
