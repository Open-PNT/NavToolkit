#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Converts a PVA with quaternion attitude to one with RPY attitude. Note that as the derivative
 * of this conversion does not exist everywhere, (i.e. when `pitch == pi/2`), the conversion of the
 * covariance may fail in this region. It is recommended that this VSB ony be used to generate
 * 'human readable' attitude representations for display or similar purposes and not to be used
 * directly in MeasurementProcessors.
 *
 * The first 6 states of the input state vector are assumed to be position/velocity and are passed
 * through without modification. The input attitude quaternion should be in states 6-9 as follows:
 *
 * 6 - 'Own' to reference quaternion scalar element.
 *
 * 7 - 'Own' to reference quaternion 'i' vector element.
 *
 * 8 - 'Own' to reference quaternion 'j' vector element.
 *
 * 9 - 'Own' to reference quaternion 'k' vector element.
 *
 * Any additional states on the input after the quaternion will be appended to the out state vector.
 *
 * As stated, the first six states of the output state vector are the same as the input states
 * (assumed to be position/velocity). The next three states in the output are the transformed
 * attitude states:
 *
 * 6 - Roll (rad).
 *
 * 7 - Pitch (rad).
 *
 * 8 - Yaw (rad).
 *
 * These angles are 3-2-1 rotations w.r.t. whatever reference frame the input quaternion uses.
 * The first 6 states of the output are the same as the input. Any trailing states coming after the
 * input quaternion states are retained without modification, but as there are 1 fewer attitude
 * states their position in the state vector is shifted down by 1.
 *
 */
class QuatToRpyPva : public VirtualStateBlock {
public:
	/**
	 * Constructor.
	 * @param current Label associated with the block to be converted.
	 * @param target Label to give to the virtual block.
	 */
	QuatToRpyPva(std::string current, std::string target);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

private:
	static constexpr Size RPY_START  = 6;
	static constexpr Size RPY_END    = 9;
	static constexpr Size QUAT_START = 6;
	static constexpr Size QUAT_END   = 10;
};

}  // namespace filtering
}  // namespace navtk
