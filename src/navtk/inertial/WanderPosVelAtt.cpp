#include <navtk/inertial/WanderPosVelAtt.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

WanderPosVelAtt::WanderPosVelAtt(const aspn_xtensor::TypeTimestamp& time,
                                 Matrix3 C_n_to_e,
                                 double alt,
                                 Vector3 v_n,
                                 Matrix3 C_s_to_l,
                                 AspnMessageType message_type)
    : InertialPosVelAtt(time, message_type),
      C_n_to_e(std::move(C_n_to_e)),
      h(alt),
      vn(std::move(v_n)),
      C_s_to_l(std::move(C_s_to_l)) {}

WanderPosVelAtt::WanderPosVelAtt(const aspn_xtensor::TypeTimestamp& time,
                                 const std::tuple<Matrix3, double, Vector3, Matrix3>& tup,
                                 AspnMessageType message_type)
    : WanderPosVelAtt(time,
                      std::get<0>(tup),
                      std::get<1>(tup),
                      std::get<2>(tup),
                      std::get<3>(tup),
                      message_type) {}

std::shared_ptr<InertialPosVelAtt> WanderPosVelAtt::clone() const {
	return std::make_shared<WanderPosVelAtt>(WanderPosVelAtt(*this));
}

Vector3 WanderPosVelAtt::get_llh() const { return navutils::C_n_to_e_h_to_llh(C_n_to_e, h); }

Vector3 WanderPosVelAtt::get_vned() const {
	auto wander     = navutils::C_n_to_e_to_wander(C_n_to_e);
	auto C_ned_to_n = navutils::wander_to_C_ned_to_n(wander);
	return dot(xt::transpose(C_ned_to_n), vn);
}

Matrix3 WanderPosVelAtt::get_C_s_to_ned() const {
	auto wander     = navutils::C_n_to_e_to_wander(C_n_to_e);
	auto C_ned_to_l = navutils::wander_to_C_ned_to_l(wander);
	return dot(xt::transpose(C_ned_to_l), C_s_to_l);
}

std::pair<Matrix3, double> WanderPosVelAtt::get_C_n_to_e_h() const { return {C_n_to_e, h}; }

Vector3 WanderPosVelAtt::get_vn() const { return vn; }

Matrix3 WanderPosVelAtt::get_C_s_to_l() const { return C_s_to_l; }
}  // namespace inertial
}  // namespace navtk
