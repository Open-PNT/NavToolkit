#include <navtk/filtering/virtualstateblocks/ShiftVirtualStateBlock.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/linear_algebra.hpp>

namespace navtk {
namespace filtering {

ShiftVirtualStateBlock::ShiftVirtualStateBlock(
    std::string current,
    std::string target,
    Vector3 l_ps_p,
    Matrix3 C_platform_to_sensor,
    std::function<Vector(const Vector&, const Vector3&, const Matrix3&)> fx,
    std::function<Matrix(const Vector&, const Vector3&, const Matrix3&)> jx)
    : VirtualStateBlock(std::move(current), std::move(target)),
      l_ps_p(std::move(l_ps_p)),
      C_platform_to_sensor(std::move(C_platform_to_sensor)),
      fx(std::move(fx)),
      jx(std::move(jx)) {}

not_null<std::shared_ptr<VirtualStateBlock>> ShiftVirtualStateBlock::clone() {
	return std::make_shared<ShiftVirtualStateBlock>(*this);
}

Matrix ShiftVirtualStateBlock::jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	return jx(x, l_ps_p, C_platform_to_sensor);
}

Vector ShiftVirtualStateBlock::convert_estimate(const Vector& x,
                                                const aspn_xtensor::TypeTimestamp&) {
	return fx(x, l_ps_p, C_platform_to_sensor);
}

}  // namespace filtering
}  // namespace navtk
