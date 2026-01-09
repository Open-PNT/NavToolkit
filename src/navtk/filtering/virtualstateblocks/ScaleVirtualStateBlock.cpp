#include <navtk/filtering/virtualstateblocks/ScaleVirtualStateBlock.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/linear_algebra.hpp>

namespace navtk {
namespace filtering {

ScaleVirtualStateBlock::ScaleVirtualStateBlock(std::string current,
                                               std::string target,
                                               const Vector& scale)
    : VirtualStateBlock(std::move(current), std::move(target)), jac(diag(scale)) {}

not_null<std::shared_ptr<VirtualStateBlock>> ScaleVirtualStateBlock::clone() {
	return std::make_shared<ScaleVirtualStateBlock>(*this);
}

Vector ScaleVirtualStateBlock::convert_estimate(const Vector& x,
                                                const aspn_xtensor::TypeTimestamp&) {
	return dot(jac, x);
}

Matrix ScaleVirtualStateBlock::jacobian(const Vector&, const aspn_xtensor::TypeTimestamp&) {
	return jac;
}

}  // namespace filtering
}  // namespace navtk
