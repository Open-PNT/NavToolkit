#include <navtk/filtering/virtualstateblocks/EcefToStandard.hpp>

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

constexpr Size EcefToStandard::POS_START;
constexpr Size EcefToStandard::POS_END;
constexpr Size EcefToStandard::VEL_START;
constexpr Size EcefToStandard::VEL_END;
constexpr Size EcefToStandard::RPY_START;
constexpr Size EcefToStandard::RPY_END;

EcefToStandard::EcefToStandard(std::string current, std::string target)
    : VirtualStateBlock(std::move(current), std::move(target)) {}

not_null<std::shared_ptr<VirtualStateBlock>> EcefToStandard::clone() {
	return std::make_shared<EcefToStandard>(*this);
}

Vector EcefToStandard::convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto llh           = navutils::ecef_to_llh(xt::view(x, xt::range(POS_START, POS_END)));
	auto C_ned_to_ecef = navutils::llh_to_cen(llh);
	auto vel_ned       = dot(transpose(C_ned_to_ecef), xt::view(x, xt::range(VEL_START, VEL_END)));
	auto rpy =
	    navutils::dcm_to_rpy(dot(xt::transpose(C_ned_to_ecef),
	                             navutils::rpy_to_dcm(xt::view(x, xt::range(RPY_START, RPY_END)))));
	auto x_out = xt::concatenate(
	    xt::xtuple(llh, vel_ned, rpy, xt::view(x, xt::range(RPY_END, num_rows(x)))));
	return x_out;
}

Matrix EcefToStandard::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto pos_range = xt::range(POS_START, POS_END);
	auto vel_range = xt::range(VEL_START, VEL_END);
	auto rpy_range = xt::range(RPY_START, RPY_END);

	auto llh           = navutils::ecef_to_llh(xt::view(x, pos_range));
	auto C_ned_to_ecef = navutils::llh_to_cen(llh);
	auto C_ecef_to_s   = xt::transpose(navutils::rpy_to_dcm(xt::view(x, rpy_range)));

	auto out = eye(num_rows(x));

	auto llh_der_ecef                   = navutils::d_ecef_to_llh_wrt_ecef(xt::view(x, pos_range));
	xt::view(out, pos_range, pos_range) = llh_der_ecef;

	auto C_ecef_to_ned_der_x = d_cne_wrt_k(xt::view(llh_der_ecef, xt::all(), 0), llh);
	auto C_ecef_to_ned_der_y = d_cne_wrt_k(xt::view(llh_der_ecef, xt::all(), 1), llh);
	auto C_ecef_to_ned_der_z = d_cne_wrt_k(xt::view(llh_der_ecef, xt::all(), 2), llh);

	xt::view(out, vel_range, vel_range)     = transpose(C_ned_to_ecef);
	xt::view(out, vel_range, POS_START)     = dot(C_ecef_to_ned_der_x, xt::view(x, vel_range));
	xt::view(out, vel_range, POS_START + 1) = dot(C_ecef_to_ned_der_y, xt::view(x, vel_range));
	xt::view(out, vel_range, POS_START + 2) = dot(C_ecef_to_ned_der_z, xt::view(x, vel_range));

	auto C_ecef_to_s_der_r = transpose(navutils::d_cns_wrt_r(xt::view(x, rpy_range)));
	auto C_ecef_to_s_der_p = transpose(navutils::d_cns_wrt_p(xt::view(x, rpy_range)));
	auto C_ecef_to_s_der_y = transpose(navutils::d_cns_wrt_y(xt::view(x, rpy_range)));

	const auto Z3 = zeros(3, 3);
	auto ab       = dot(C_ecef_to_s, C_ned_to_ecef);

	auto rpy_ned_to_s_der_ecef = navutils::d_dcm_to_rpy(C_ecef_to_s,
	                                                    Z3,
	                                                    Z3,
	                                                    Z3,
	                                                    C_ned_to_ecef,
	                                                    transpose(C_ecef_to_ned_der_x),
	                                                    transpose(C_ecef_to_ned_der_y),
	                                                    transpose(C_ecef_to_ned_der_z),
	                                                    ab);

	auto rpy_ned_to_s_der_rpy = navutils::d_dcm_to_rpy(C_ecef_to_s,
	                                                   C_ecef_to_s_der_r,
	                                                   C_ecef_to_s_der_p,
	                                                   C_ecef_to_s_der_y,
	                                                   C_ned_to_ecef,
	                                                   Z3,
	                                                   Z3,
	                                                   Z3,
	                                                   ab);

	xt::view(out, rpy_range, pos_range) = rpy_ned_to_s_der_ecef;
	xt::view(out, rpy_range, rpy_range) = rpy_ned_to_s_der_rpy;

	return out;
}

}  // namespace filtering
}  // namespace navtk
