#include <navtk/filtering/virtualstateblocks/StandardToEcef.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/derivatives.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

constexpr Size StandardToEcef::POS_START;
constexpr Size StandardToEcef::POS_END;
constexpr Size StandardToEcef::VEL_START;
constexpr Size StandardToEcef::VEL_END;
constexpr Size StandardToEcef::RPY_START;
constexpr Size StandardToEcef::RPY_END;

StandardToEcef::StandardToEcef(std::string current, std::string target)
    : VirtualStateBlock(std::move(current), std::move(target)) {}

not_null<std::shared_ptr<VirtualStateBlock>> StandardToEcef::clone() {
	return std::make_shared<StandardToEcef>(*this);
}

Vector StandardToEcef::convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) {

	auto C_ned_to_ecef = navutils::llh_to_cen(xt::view(x, xt::range(POS_START, POS_END)));
	auto pos_ecef      = navutils::llh_to_ecef(xt::view(x, xt::range(POS_START, POS_END)));
	auto vel_ecef      = dot(C_ned_to_ecef, xt::view(x, xt::range(VEL_START, VEL_END)));
	auto rpy_ecef      = navutils::dcm_to_rpy(
        dot(C_ned_to_ecef, navutils::rpy_to_dcm(xt::view(x, xt::range(RPY_START, RPY_END)))));
	return xt::concatenate(
	    xt::xtuple(pos_ecef, vel_ecef, rpy_ecef, xt::view(x, xt::range(RPY_END, num_rows(x)))));
}

Matrix StandardToEcef::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto pos_range = xt::range(POS_START, POS_END);
	auto vel_range = xt::range(VEL_START, VEL_END);
	auto rpy_range = xt::range(RPY_START, RPY_END);

	auto C_ned_to_ecef = navutils::llh_to_cen(xt::view(x, pos_range));
	auto llh_to_ecef   = navutils::d_llh_to_ecef_wrt_llh(xt::view(x, pos_range));

	auto C_ned_to_s            = transpose(navutils::rpy_to_dcm(xt::view(x, rpy_range)));
	auto C_ned_to_ecef_der_lat = navutils::d_cen_wrt_lat(xt::view(x, pos_range));
	auto C_ned_to_ecef_der_lon = navutils::d_cen_wrt_lon(xt::view(x, pos_range));
	auto C_s_to_ned_der_r      = navutils::d_cns_wrt_r(xt::view(x, rpy_range));
	auto C_s_to_ned_der_p      = navutils::d_cns_wrt_p(xt::view(x, rpy_range));
	auto C_s_to_ned_der_y      = navutils::d_cns_wrt_y(xt::view(x, rpy_range));

	auto jac                                = eye(num_rows(x));
	xt::view(jac, pos_range, pos_range)     = llh_to_ecef;
	xt::view(jac, vel_range, vel_range)     = C_ned_to_ecef;
	xt::view(jac, vel_range, POS_START)     = dot(C_ned_to_ecef_der_lat, xt::view(x, vel_range));
	xt::view(jac, vel_range, POS_START + 1) = dot(C_ned_to_ecef_der_lon, xt::view(x, vel_range));

	const auto Z3      = zeros(3, 3);
	auto C_ecef_to_ned = transpose(C_ned_to_ecef);
	auto ab            = dot(C_ned_to_s, C_ecef_to_ned);

	auto dx = dot(C_ned_to_s, transpose(C_ned_to_ecef_der_lat));
	auto dy = dot(C_ned_to_s, transpose(C_ned_to_ecef_der_lon));
	auto dz = Z3;

	xt::view(jac, rpy_range, pos_range) = navutils::d_dcm_to_rpy(ab, dx, dy, dz);

	dx                                  = dot(transpose(C_s_to_ned_der_r), C_ecef_to_ned);
	dy                                  = dot(transpose(C_s_to_ned_der_p), C_ecef_to_ned);
	dz                                  = dot(transpose(C_s_to_ned_der_y), C_ecef_to_ned);
	xt::view(jac, rpy_range, rpy_range) = navutils::d_dcm_to_rpy(ab, dx, dy, dz);

	return jac;
}
}  // namespace filtering
}  // namespace navtk
