#pragma once

#include <memory>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {

/**
 * VirtualStateBlock that implements a shift and/rotation of an
 * EstimateWithCovariance from one 3D reference frame to another.
 */
class ShiftVirtualStateBlock : public VirtualStateBlock {
public:
	/**
	 * Constructor.
	 * @param current Represents the initial format of the state prior
	 * to shifting.
	 * @param target Represents the final format of the state after it
	 * has been shifted.
	 * @param l_ps_p The value to store in #l_ps_p.
	 * @param C_platform_to_sensor The value to store in #C_platform_to_sensor.
	 * @param fx The value to store in #fx.
	 * @param jx The value to store in #jx.
	 */
	ShiftVirtualStateBlock(std::string current,
	                       std::string target,
	                       Vector3 l_ps_p,
	                       Matrix3 C_platform_to_sensor,
	                       std::function<Vector(const Vector&, const Vector3&, const Matrix3&)> fx,
	                       std::function<Matrix(const Vector&, const Vector3&, const Matrix3&)> jx);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

protected:
	/**
	 * Lever arm platform to sensor in platform frame. A Vector3 that points from the origin of
	 * the arbitrarily defined platform frame to the sensor frame.
	 */
	Vector3 l_ps_p;
	/**
	 * DCM that rotates from the platform frame to the sensor frame.
	 */
	Matrix3 C_platform_to_sensor;
	/**
	 * Function that shifts an input EstimateWithCovariance from `current` to `target` when passed
	 * an EstimateWithCovariance in `current` representation, `l_ps_p` and `C_platform_to_sensor`.
	 */
	std::function<Vector(const Vector&, const Vector3&, const Matrix3&)> fx = 0;
	/**
	 * Function that returns the Jacobian of the transform effected by `fx` when passed a state
	 * vector in `current` representation, `l_ps_p`, and `C_platform_to_sensor`.
	 */
	std::function<Matrix(const Vector&, const Vector3&, const Matrix3&)> jx = 0;
};

}  // namespace filtering
}  // namespace navtk
