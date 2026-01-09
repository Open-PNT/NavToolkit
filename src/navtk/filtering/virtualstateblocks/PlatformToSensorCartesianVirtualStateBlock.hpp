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
 * VirtualStateBlock that applies the navtk::navutils::platform_to_sensor
 * function to a joint Gaussian state/covariance that consists of the
 * 3 position states of the platform frame origin w.r.t. some frame j
 * (such as ECEF) and the 3 Euler angles that represent the orientation
 * of the sensor frame w.r.t another frame k (such as a fixed navigation
 * frame, or the ECEF frame).
 *
 * Note: In general Euler angle states are not ideal, and under some
 * circumstances the Jacobian of this transform can introduce significant
 * error, so use with care.
 */
class PlatformToSensorCartesianVirtualStateBlock : public VirtualStateBlock {
public:
	/**
	 * Constructor.
	 * @param current Label that corresponds to the platform frame position/orientation (input to
	 * the transform).
	 * @param target Label that corresponds to the sensor frame position/orientation (output of the
	 * transform).
	 * @param l_ps_p The value to store in #l_ps_p.
	 * @param C_platform_to_sensor The value to store in #C_platform_to_sensor.
	 * @param C_k_to_j The value to store in #C_k_to_j.
	 */
	PlatformToSensorCartesianVirtualStateBlock(std::string current,
	                                           std::string target,
	                                           Vector3 l_ps_p,
	                                           Matrix3 C_platform_to_sensor,
	                                           Matrix3 C_k_to_j);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	/**
	 * Produces the Jacobian of the convert function with respect to `x`.
	 *
	 * @param x A 6-state vector containing 3 Cartesian position states
	 * and 3 Euler angle states corresponding to the 'platform' frame
	 * position and orientation.
	 *
	 * @return A 6x6 Matrix that maps the platform frame states to the
	 * sensor frame, correct to first-order.
	 */
	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

	/**
	 * Applies a modified version of the navtk::navutils::platform_to_sensor
	 * function to a state estimate that contains platform frame
	 * position/orientation states.
	 *
	 * @param x The platform frame state estimate (Cartesian position states in meters, Euler
	 * angles in rad).
	 *
	 * @return The sensor frame estimate.
	 */
	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

protected:
	/**
	 * Lever arm pointing from the platform frame origin to the sensor frame origin in platform
	 * frame coordinates.
	 */
	Vector3 l_ps_p;
	/**
	 * Platform frame to sensor frame DCM.
	 */
	Matrix3 C_platform_to_sensor;
	/**
	 * DCM that relates the respective reference frames of the position and orientation states. For
	 * instance, if sensor position was in ECEF coordinates and the Euler angles were RPY (w.r.t to
	 * some fixed navigation frame), then this parameter would be `C_nav_to_ecef`, or the navigation
	 * to ECEF frame DCM. If the relative rotation between these 2 frames is not constant, then the
	 * more generic ShiftVirtualStateBlock should be used.
	 */
	Matrix3 C_k_to_j;
};

}  // namespace filtering
}  // namespace navtk
