#pragma once

#include <navtk/aspn.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Implementation of InertialPosVelAtt designed for use with wander
 * frame navigation, valid worldwide. Methods of this class that
 * convert to/from non-wander compatible representations may not be
 * valid under all scenarios; see method documentation.
 */
class WanderPosVelAtt : public InertialPosVelAtt {
public:
	/**
	 * Constructor.
	 *
	 * @param time Time of validity for this instance
	 * @param C_n_to_e The n frame to e frame DCM, which is derived from
	 * latitude, longitude and wander angle as described in Savage 4.4.2.1-2.
	 * @param alt Altitude, meters HAE.
	 * @param vn Velocity in the n frame, \f$\frac{m}{s}\f$.
	 * @param C_s_to_l The sensor to l frame DCM.
	 * @param message_type the ASPN message type assigned to WanderPosVelAtt. This will likely be
	 * specific to the running program. Defaults to ASPN_EXTENDED_BEGIN since WanderPosVelAtt
	 * extends the set of defined ASPN messages. Note, if this default value is not overridden by a
	 * program using NavToolkit, then the type assigned to WanderPosVelAtt may conflict with
	 * another type used by that program. Users should be careful to ensure that all ASPN message
	 * types used by their program have unique types if they are using AspnBase::get_message_type to
	 * identify a message type.
	 */
	WanderPosVelAtt(
	    const aspn_xtensor::TypeTimestamp& time = aspn_xtensor::TypeTimestamp((int64_t)0),
	    Matrix3 C_n_to_e                        = eye(3),
	    double alt                              = 0.0,
	    Vector3 vn                              = zeros(3),
	    Matrix3 C_s_to_l                        = eye(3),
	    AspnMessageType message_type            = ASPN_EXTENDED_BEGIN);

	/**
	 * Constructor.
	 *
	 * @param time Time of validity for this instance
	 * @param tup Tuple containing `C_n_to_e`, `altitude`, `v_n` and `C_s_to_l`;
	 * see other constructor.
	 * @param message_type the ASPN message type assigned to WanderPosVelAtt. This will likely be
	 * specific to the running program. Defaults to ASPN_EXTENDED_BEGIN since WanderPosVelAtt
	 * extends the set of defined ASPN messages. Note, if this default value is not overridden by a
	 * program using NavToolkit, then the type assigned to WanderPosVelAtt may conflict with
	 * another type used by that program. Users should be careful to ensure that all ASPN message
	 * types used by their program have unique types if they are using AspnBase::get_message_type to
	 * identify a message type.
	 */
	WanderPosVelAtt(const aspn_xtensor::TypeTimestamp& time,
	                const std::tuple<Matrix3, double, Vector3, Matrix3>& tup,
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

	std::pair<Matrix3, double> get_C_n_to_e_h() const override;

	Vector3 get_vn() const override;

	Matrix3 get_C_s_to_l() const override;

	bool is_wander_capable() const override { return true; }

private:
	Matrix3 C_n_to_e;
	double h;
	Vector3 vn;
	Matrix3 C_s_to_l;
};

}  // namespace inertial
}  // namespace navtk
