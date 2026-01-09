#include <navtk/filtering/virtualstateblocks/FirstOrderVirtualStateBlock.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/linear_algebra.hpp>

namespace navtk {
namespace filtering {

FirstOrderVirtualStateBlock::FirstOrderVirtualStateBlock(std::string current,
                                                         std::string target,
                                                         std::function<Vector(const Vector&)> fx,
                                                         std::function<Matrix(const Vector&)> jx)
    : VirtualStateBlock(std::move(current), std::move(target)),
      fx(std::move(fx)),
      jx(std::move(jx)) {}

not_null<std::shared_ptr<VirtualStateBlock>> FirstOrderVirtualStateBlock::clone() {
	return std::make_shared<FirstOrderVirtualStateBlock>(*this);
}

Matrix FirstOrderVirtualStateBlock::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	return jx ? jx(x) : calc_numerical_jacobian(fx, x);
}

Vector FirstOrderVirtualStateBlock::convert_estimate(const Vector& x,
                                                     const aspn_xtensor::TypeTimestamp&) {
	return fx(x);
}

}  // namespace filtering
}  // namespace navtk
