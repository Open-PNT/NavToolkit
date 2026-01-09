#include <navtk/filtering/virtualstateblocks/NumericalVirtualStateBlock.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/linear_algebra.hpp>

namespace navtk {
namespace filtering {

NumericalVirtualStateBlock::NumericalVirtualStateBlock(std::string current, std::string target)
    : VirtualStateBlock(std::move(current), std::move(target)) {}

Matrix NumericalVirtualStateBlock::jacobian(const Vector& x,
                                            const aspn_xtensor::TypeTimestamp& time) {
	std::function<Vector(const Vector&)> lam = [&, time = time](const Vector& x) {
		return fx(x, time);
	};
	return calc_numerical_jacobian(lam, x);
}

Vector NumericalVirtualStateBlock::convert_estimate(const Vector& x,
                                                    const aspn_xtensor::TypeTimestamp& time) {
	return fx(x, time);
}

EstimateWithCovariance NumericalVirtualStateBlock::convert(
    const EstimateWithCovariance& ec, const aspn_xtensor::TypeTimestamp& time) {
	std::function<Vector(const Vector&)> lam = [&, time = time](const Vector& x) {
		return fx(x, time);
	};
	return first_order_approx(ec, lam);
}

}  // namespace filtering
}  // namespace navtk
