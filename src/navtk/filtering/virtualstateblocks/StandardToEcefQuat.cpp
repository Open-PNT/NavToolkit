#include <navtk/filtering/virtualstateblocks/StandardToEcefQuat.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/derivatives.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/quaternions.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

using navtk::navutils::quat_mult;

constexpr Size StandardToEcefQuat::POS_START;
constexpr Size StandardToEcefQuat::POS_END;
constexpr Size StandardToEcefQuat::VEL_START;
constexpr Size StandardToEcefQuat::VEL_END;
constexpr Size StandardToEcefQuat::QUAT_START;
constexpr Size StandardToEcefQuat::QUAT_END;

StandardToEcefQuat::StandardToEcefQuat(std::string current, std::string target)
    : VirtualStateBlock(std::move(current), std::move(target)) {}

not_null<std::shared_ptr<VirtualStateBlock>> StandardToEcefQuat::clone() {
	return std::make_shared<StandardToEcefQuat>(*this);
}

Vector StandardToEcefQuat::convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) {

	auto q_ned_to_ecef =
	    navutils::dcm_to_quat(navutils::llh_to_cen(xt::view(x, xt::range(POS_START, POS_END))));

	auto pos_ecef = navutils::llh_to_ecef(xt::view(x, xt::range(POS_START, POS_END)));
	auto vel_ecef = navutils::quat_rot(q_ned_to_ecef, xt::view(x, xt::range(VEL_START, VEL_END)));
	auto q_p_to_ecef = quat_mult(q_ned_to_ecef, xt::view(x, xt::range(QUAT_START, QUAT_END)));

	auto out = xt::concatenate(
	    xt::xtuple(pos_ecef, vel_ecef, q_p_to_ecef, xt::view(x, xt::range(QUAT_END, num_rows(x)))));

	return out;
}

Matrix StandardToEcefQuat::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto pos_range  = xt::range(POS_START, POS_END);
	auto vel_range  = xt::range(VEL_START, VEL_END);
	auto quat_range = xt::range(QUAT_START, QUAT_END);

	auto C_ned_to_ecef         = navutils::llh_to_cen(xt::view(x, pos_range));
	auto llh_to_ecef_der_llh   = navutils::d_llh_to_ecef_wrt_llh(xt::view(x, pos_range));
	auto C_ned_to_ecef_der_lat = navutils::d_cen_wrt_lat(xt::view(x, pos_range));
	auto C_ned_to_ecef_der_lon = navutils::d_cen_wrt_lon(xt::view(x, pos_range));

	auto jac                                = eye(num_rows(x));
	xt::view(jac, pos_range, pos_range)     = llh_to_ecef_der_llh;
	xt::view(jac, vel_range, vel_range)     = C_ned_to_ecef;
	xt::view(jac, vel_range, POS_START)     = dot(C_ned_to_ecef_der_lat, xt::view(x, vel_range));
	xt::view(jac, vel_range, POS_START + 1) = dot(C_ned_to_ecef_der_lon, xt::view(x, vel_range));

	auto q_ned_to_ecef_dllh = navutils::d_llh_to_quat_en_wrt_llh(xt::view(x, pos_range));
	xt::view(jac, quat_range, POS_START) =
	    quat_mult(xt::view(q_ned_to_ecef_dllh, xt::all(), 0), xt::view(x, quat_range));
	xt::view(jac, quat_range, POS_START + 1) =
	    quat_mult(xt::view(q_ned_to_ecef_dllh, xt::all(), 1), xt::view(x, quat_range));

	// q_nav_to_ecef
	auto q = navutils::llh_to_quat_en(xt::view(x, pos_range));

	// Just the first term of quat_mult
	xt::view(jac, quat_range, quat_range) = Matrix{{q(0), -q(1), -q(2), -q(3)},
	                                               {q(1), q(0), -q(3), q(2)},
	                                               {q(2), q(3), q(0), -q(1)},
	                                               {q(3), -q(2), q(1), q(0)}};

	return jac;
}
}  // namespace filtering
}  // namespace navtk
