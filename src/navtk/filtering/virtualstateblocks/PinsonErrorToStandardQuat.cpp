#include <navtk/filtering/virtualstateblocks/PinsonErrorToStandardQuat.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/derivatives.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/quaternions.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

constexpr Size PinsonErrorToStandardQuat::POS_START;
constexpr Size PinsonErrorToStandardQuat::POS_END;
constexpr Size PinsonErrorToStandardQuat::VEL_START;
constexpr Size PinsonErrorToStandardQuat::VEL_END;
constexpr Size PinsonErrorToStandardQuat::TILT_START;
constexpr Size PinsonErrorToStandardQuat::TILT_END;
constexpr Size PinsonErrorToStandardQuat::QUAT_START;
constexpr Size PinsonErrorToStandardQuat::QUAT_END;

using navtk::navutils::dcm_to_quat;

PinsonErrorToStandardQuat::PinsonErrorToStandardQuat(
    std::string current,
    std::string target,
    std::function<NavSolution(const aspn_xtensor::TypeTimestamp& time)> ref_fun)
    : VirtualStateBlock(std::move(current), std::move(target)), ref_fun(std::move(ref_fun)) {}

not_null<std::shared_ptr<VirtualStateBlock>> PinsonErrorToStandardQuat::clone() {
	return std::make_shared<PinsonErrorToStandardQuat>(*this);
}

Vector PinsonErrorToStandardQuat::convert_estimate(const Vector& x,
                                                   const aspn_xtensor::TypeTimestamp& time) {
	auto sol          = ref_fun(time);
	auto delta_lat    = navutils::north_to_delta_lat(x(POS_START), sol.pos(0), sol.pos(2));
	auto delta_lon    = navutils::east_to_delta_lon(x(POS_START + 1), sol.pos(0), sol.pos(2));
	auto delta_alt    = -x(POS_START + 2);
	auto corr_llh     = Vector3{delta_lat, delta_lon, delta_alt} + sol.pos;
	auto corr_vel_ned = sol.vel + xt::view(x, xt::range(VEL_START, VEL_END));

	auto q_s_to_ned      = dcm_to_quat(transpose(sol.rot_mat));
	auto corr_q_s_to_ned = navutils::quat_norm(navutils::correct_quat_with_tilt(
	    q_s_to_ned, -xt::view(x, xt::range(TILT_START, TILT_END))));
	return xt::concatenate(xt::xtuple(
	    corr_llh, corr_vel_ned, corr_q_s_to_ned, xt::view(x, xt::range(TILT_END, num_rows(x)))));
}

Matrix PinsonErrorToStandardQuat::jacobian(const Vector& x,
                                           const aspn_xtensor::TypeTimestamp& time) {
	auto sol = ref_fun(time);
	Matrix m2r{{1.0 / navutils::delta_lat_to_north(1, sol.pos(0), sol.pos(2)), 0, 0},
	           {0, 1.0 / navutils::delta_lon_to_east(1, sol.pos(0), sol.pos(2)), 0},
	           {0, 0, -1}};
	auto jac = zeros(num_rows(x) + 1, num_rows(x));
	xt::view(jac, xt::range(POS_START, POS_END), xt::range(POS_START, POS_END)) = m2r;
	xt::view(jac, xt::range(VEL_START, VEL_END), xt::range(VEL_START, VEL_END)) = eye(3);

	auto q_s_to_ned_der_tilt =
	    -navutils::d_quat_tilt_corr_wrt_tilt(dcm_to_quat(transpose(sol.rot_mat)));

	auto q_s_to_ned = navutils::quat_norm(navutils::correct_quat_with_tilt(
	    dcm_to_quat(transpose(sol.rot_mat)), -xt::view(x, xt::range(TILT_START, TILT_END))));

	auto norm_der_q = navutils::d_quat_norm_wrt_q(q_s_to_ned);

	xt::view(jac, xt::range(QUAT_START, QUAT_END), xt::range(TILT_START, TILT_END)) =
	    dot(norm_der_q, q_s_to_ned_der_tilt);

	xt::view(jac, xt::range(QUAT_END, num_rows(x) + 1), xt::range(TILT_END, num_rows(x))) =
	    eye(num_rows(x) - 9);
	return jac;
}
}  // namespace filtering
}  // namespace navtk
