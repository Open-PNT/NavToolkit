#include <navtk/filtering/virtualstateblocks/QuatToRpyPva.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/inspect.hpp>
#include <navtk/navutils/derivatives.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

constexpr Size QuatToRpyPva::RPY_START;
constexpr Size QuatToRpyPva::RPY_END;
constexpr Size QuatToRpyPva::QUAT_START;
constexpr Size QuatToRpyPva::QUAT_END;

QuatToRpyPva::QuatToRpyPva(std::string current, std::string target)
    : VirtualStateBlock(std::move(current), std::move(target)) {}

not_null<std::shared_ptr<VirtualStateBlock>> QuatToRpyPva::clone() {
	return std::make_shared<QuatToRpyPva>(*this);
}

Vector QuatToRpyPva::convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) {

	auto rpy = navutils::quat_to_rpy(xt::view(x, xt::range(QUAT_START, QUAT_END)));

	return xt::concatenate(xt::xtuple(
	    xt::view(x, xt::range(0, QUAT_START)), rpy, xt::view(x, xt::range(QUAT_END, num_rows(x)))));
}

Matrix QuatToRpyPva::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	auto jac = zeros(num_rows(x) - 1, num_rows(x));
	xt::view(jac, xt::range(0, QUAT_START), xt::range(0, QUAT_START)) = eye(6);
	xt::view(jac, xt::range(RPY_START, RPY_END), xt::range(QUAT_START, QUAT_END)) =
	    navutils::d_quat_to_rpy_wrt_q(xt::view(x, xt::range(QUAT_START, QUAT_END)));
	xt::view(jac, xt::range(RPY_END, num_rows(x) - 1), xt::range(QUAT_END, num_rows(x))) =
	    eye(num_rows(x) - QUAT_END);
	return jac;
}

}  // namespace filtering
}  // namespace navtk
