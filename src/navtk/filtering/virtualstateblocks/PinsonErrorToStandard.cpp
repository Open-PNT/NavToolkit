#include <navtk/filtering/virtualstateblocks/PinsonErrorToStandard.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/derivatives.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

using navtk::navutils::d_ortho_dcm_wrt_tilt;

constexpr Size PinsonErrorToStandard::POS_START;
constexpr Size PinsonErrorToStandard::POS_END;
constexpr Size PinsonErrorToStandard::VEL_START;
constexpr Size PinsonErrorToStandard::VEL_END;
constexpr Size PinsonErrorToStandard::ATT_START;
constexpr Size PinsonErrorToStandard::ATT_END;

PinsonErrorToStandard::PinsonErrorToStandard(
    std::string current,
    std::string target,
    std::function<NavSolution(const aspn_xtensor::TypeTimestamp& time)> ref_fun)
    : VirtualStateBlock(std::move(current), std::move(target)), ref_fun(std::move(ref_fun)) {}

not_null<std::shared_ptr<VirtualStateBlock>> PinsonErrorToStandard::clone() {
	return std::make_shared<PinsonErrorToStandard>(*this);
}

Vector PinsonErrorToStandard::convert_estimate(const Vector& x,
                                               const aspn_xtensor::TypeTimestamp& time) {
	auto sol          = ref_fun(time);
	auto delta_lat    = navutils::north_to_delta_lat(x(POS_START), sol.pos(0), sol.pos(2));
	auto delta_lon    = navutils::east_to_delta_lon(x(POS_START + 1), sol.pos(0), sol.pos(2));
	auto delta_alt    = -x(POS_START + 2);
	auto corr_llh     = Vector3{delta_lat, delta_lon, delta_alt} + sol.pos;
	auto corr_vel_ned = sol.vel + xt::view(x, xt::range(VEL_START, VEL_END));

	auto corr_rpy = navutils::dcm_to_rpy(xt::transpose(navutils::ortho_dcm(
	    dot(sol.rot_mat, eye(3) + navutils::skew(xt::view(x, xt::range(ATT_START, ATT_END)))))));

	return xt::concatenate(
	    xt::xtuple(corr_llh, corr_vel_ned, corr_rpy, xt::view(x, xt::range(ATT_END, num_rows(x)))));
}

Matrix PinsonErrorToStandard::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp& time) {
	auto pos_range = xt::range(POS_START, POS_END);
	auto att_range = xt::range(ATT_START, ATT_END);

	auto sol = ref_fun(time);
	Matrix m2r{{1.0 / navutils::delta_lat_to_north(1, sol.pos(0), sol.pos(2)), 0, 0},
	           {0, 1.0 / navutils::delta_lon_to_east(1, sol.pos(0), sol.pos(2)), 0},
	           {0, 0, -1}};
	auto jac                            = eye(num_rows(x));
	xt::view(jac, pos_range, pos_range) = m2r;

	Matrix dx{{0, 0, 0}, {0, 0, -1}, {0, 1, 0}};
	Matrix dy{{0, 0, 1}, {0, 0, 0}, {-1, 0, 0}};
	Matrix dz{{0, -1, 0}, {1, 0, 0}, {0, 0, 0}};

	auto ddx = d_ortho_dcm_wrt_tilt(sol.rot_mat, xt::view(x, att_range), dx);
	auto ddy = d_ortho_dcm_wrt_tilt(sol.rot_mat, xt::view(x, att_range), dy);
	auto ddz = d_ortho_dcm_wrt_tilt(sol.rot_mat, xt::view(x, att_range), dz);
	auto corr_C_ned_to_s =
	    navutils::ortho_dcm(dot(sol.rot_mat, eye(3) + navutils::skew(xt::view(x, att_range))));

	xt::view(jac, att_range, att_range) = navutils::d_dcm_to_rpy(corr_C_ned_to_s, ddx, ddy, ddz);

	return jac;
}
}  // namespace filtering
}  // namespace navtk
