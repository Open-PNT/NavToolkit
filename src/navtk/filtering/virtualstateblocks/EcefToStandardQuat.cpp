#include <navtk/filtering/virtualstateblocks/EcefToStandardQuat.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/derivatives.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/quaternions.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

using navtk::navutils::d_cne_wrt_k;
using navtk::navutils::quat_conj;
using navtk::navutils::quat_mult;

constexpr Size EcefToStandardQuat::POS_START;
constexpr Size EcefToStandardQuat::POS_END;
constexpr Size EcefToStandardQuat::VEL_START;
constexpr Size EcefToStandardQuat::VEL_END;
constexpr Size EcefToStandardQuat::QUAT_START;
constexpr Size EcefToStandardQuat::QUAT_END;

EcefToStandardQuat::EcefToStandardQuat(std::string current, std::string target)
    : VirtualStateBlock(std::move(current), std::move(target)) {}

not_null<std::shared_ptr<VirtualStateBlock>> EcefToStandardQuat::clone() {
	return std::make_shared<EcefToStandardQuat>(*this);
}

Vector EcefToStandardQuat::convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto llh = navutils::ecef_to_llh(xt::view(x, xt::range(POS_START, POS_END)));

	auto q_ecef_to_ned = navutils::dcm_to_quat(transpose(navutils::llh_to_cen(llh)));

	auto vel_ned = navutils::quat_rot(q_ecef_to_ned, xt::view(x, xt::range(VEL_START, VEL_END)));

	auto q_s_to_ned = quat_mult(q_ecef_to_ned, xt::view(x, xt::range(QUAT_START, QUAT_END)));

	auto x_out = xt::concatenate(
	    xt::xtuple(llh, vel_ned, q_s_to_ned, xt::view(x, xt::range(QUAT_END, num_rows(x)))));
	return x_out;
}

Matrix EcefToStandardQuat::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto pos_range  = xt::range(POS_START, POS_END);
	auto vel_range  = xt::range(VEL_START, VEL_END);
	auto quat_range = xt::range(QUAT_START, QUAT_END);

	auto llh           = navutils::ecef_to_llh(xt::view(x, pos_range));
	auto C_nav_to_ecef = navutils::llh_to_cen(llh);

	auto jac = eye(num_rows(x));

	auto llh_der_ecef                   = navutils::d_ecef_to_llh_wrt_ecef(xt::view(x, pos_range));
	xt::view(jac, pos_range, pos_range) = llh_der_ecef;

	auto C_ecef_to_ned_der_x = d_cne_wrt_k(xt::view(llh_der_ecef, xt::all(), 0), llh);
	auto C_ecef_to_ned_der_y = d_cne_wrt_k(xt::view(llh_der_ecef, xt::all(), 1), llh);
	auto C_ecef_to_ned_der_z = d_cne_wrt_k(xt::view(llh_der_ecef, xt::all(), 2), llh);

	xt::view(jac, vel_range, vel_range)     = transpose(C_nav_to_ecef);
	xt::view(jac, vel_range, POS_START)     = dot(C_ecef_to_ned_der_x, xt::view(x, vel_range));
	xt::view(jac, vel_range, POS_START + 1) = dot(C_ecef_to_ned_der_y, xt::view(x, vel_range));
	xt::view(jac, vel_range, POS_START + 2) = dot(C_ecef_to_ned_der_z, xt::view(x, vel_range));

	// q_ned_to_ecef
	auto q = quat_conj(navutils::llh_to_quat_en(llh));

	auto q_ned_to_ecef_der_llh = navutils::d_llh_to_quat_en_wrt_llh(llh);

	xt::view(jac, quat_range, POS_START) = quat_mult(
	    quat_conj(xt::view(q_ned_to_ecef_der_llh, xt::all(), 0)), xt::view(x, quat_range));
	xt::view(jac, quat_range, POS_START + 1) = quat_mult(
	    quat_conj(xt::view(q_ned_to_ecef_der_llh, xt::all(), 1)), xt::view(x, quat_range));

	xt::view(jac, quat_range, pos_range) = dot(xt::view(jac, quat_range, pos_range), llh_der_ecef);

	xt::view(jac, quat_range, quat_range) = Matrix{{q(0), -q(1), -q(2), -q(3)},
	                                               {q(1), q(0), -q(3), q(2)},
	                                               {q(2), q(3), q(0), -q(1)},
	                                               {q(3), -q(2), q(1), q(0)}};

	return jac;
}
}  // namespace filtering
}  // namespace navtk
