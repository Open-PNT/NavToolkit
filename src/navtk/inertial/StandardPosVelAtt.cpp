#include <navtk/inertial/StandardPosVelAtt.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

StandardPosVelAtt::StandardPosVelAtt(const aspn_xtensor::TypeTimestamp& time,
                                     Vector3 llh,
                                     Vector3 vned,
                                     Matrix3 C_s_to_ned,
                                     AspnMessageType message_type)
    : InertialPosVelAtt(time, message_type),
      llh(std::move(llh)),
      vned(std::move(vned)),
      C_s_to_ned(std::move(C_s_to_ned)) {}

StandardPosVelAtt::StandardPosVelAtt(const aspn_xtensor::TypeTimestamp& time,
                                     const std::tuple<Vector3, Vector3, Matrix3>& tup,
                                     AspnMessageType message_type)
    : StandardPosVelAtt(time, std::get<0>(tup), std::get<1>(tup), std::get<2>(tup), message_type) {}

std::shared_ptr<InertialPosVelAtt> StandardPosVelAtt::clone() const {
	return std::make_shared<StandardPosVelAtt>(StandardPosVelAtt(*this));
}

Vector3 StandardPosVelAtt::get_llh() const { return llh; }
Vector3 StandardPosVelAtt::get_vned() const { return vned; }
Matrix3 StandardPosVelAtt::get_C_s_to_ned() const { return C_s_to_ned; }

std::pair<Matrix3, double> StandardPosVelAtt::get_C_n_to_e_h() const {
	auto C_n_to_e = navutils::lat_lon_wander_to_C_n_to_e(llh[0], llh[1], 0.0);
	return {C_n_to_e, llh[2]};
}

Vector3 StandardPosVelAtt::get_vn() const {
	auto C_ned_to_n = navutils::wander_to_C_ned_to_n(0.0);
	return dot(C_ned_to_n, vned);
}

Matrix3 StandardPosVelAtt::get_C_s_to_l() const {
	auto C_ned_to_l = navutils::wander_to_C_ned_to_l(0.0);
	return dot(C_ned_to_l, C_s_to_ned);
}

// Overloads that allow for wander angle to be passed in
std::pair<Matrix3, double> StandardPosVelAtt::get_C_n_to_e_h(double wander) const {
	auto C_n_to_e = navutils::lat_lon_wander_to_C_n_to_e(llh[0], llh[1], wander);
	return {C_n_to_e, llh[2]};
}

Vector3 StandardPosVelAtt::get_vn(double wander) const {
	auto C_ned_to_n = navutils::wander_to_C_ned_to_n(wander);
	return dot(C_ned_to_n, vned);
}

Matrix3 StandardPosVelAtt::get_C_s_to_l(double wander) const {
	auto C_ned_to_l = navutils::wander_to_C_ned_to_l(wander);
	return dot(C_ned_to_l, C_s_to_ned);
}

}  // namespace inertial
}  // namespace navtk
