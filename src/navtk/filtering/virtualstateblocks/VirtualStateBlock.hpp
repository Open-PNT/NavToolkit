#pragma once

#include <memory>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {

/**
 * Class used to transform an EstimateWithCovariance from its current
 * representation to another.
 */
class VirtualStateBlock {
public:
	/**
	 * Default destructor.
	 */
	virtual ~VirtualStateBlock() = default;

	/**
	 * Disable default constructor.
	 */
	VirtualStateBlock() = delete;

	/**
	 * Default copy constructor.
	 */
	VirtualStateBlock(const VirtualStateBlock&) = default;

	/**
	 * Default copy assignment operator.
	 *
	 * @return A copy of the original.
	 */
	VirtualStateBlock& operator=(const VirtualStateBlock&) = default;

	/**
	 * Default move constructor.
	 */
	VirtualStateBlock(VirtualStateBlock&&) = default;

	/**
	 * Default move assignment operator.
	 *
	 * @return A new VirtualStateBlock containing the data from the original.
	 */
	VirtualStateBlock& operator=(VirtualStateBlock&&) = default;

	/**
	 * Constructor.
	 *
	 * @param current Label given to a particular collection of Gaussian
	 * quantities this instance is capable of converting into target
	 * representation.
	 *
	 * @param target The label associated with the converted, virtual
	 * states.
	 */
	VirtualStateBlock(std::string current, std::string target);

	/**
	 * Create a copy of the VirtualStateBlock with the same properties.
	 *
	 * @return A shared pointer to a copy of the VirtualStateBlock. A unique pointer is not used
	 * here because of an issue with the Python bindings:
	 * https://github.com/pybind/pybind11/issues/673
	 */
	// TODO (PNTOS-266) Use unique pointers here.
	virtual not_null<std::shared_ptr<VirtualStateBlock>> clone() = 0;

	/**
	 * Converts an EstimateWithCovariance from the N-state `current`
	 * representation to the M-state `target` representation.
	 *
	 * @param ec An estimate and covariance corresponding to a quantity
	 * represented by the `current` field (such as the estimate
	 * and covariance associated with a StateBlock whose
	 * `label == current`).
	 * @param time Time of validity for ec.
	 *
	 * @return The estimate and covariance associated with `target`,
	 * obtained by mapping and using some (possibly) non-linear transform.
	 */
	virtual EstimateWithCovariance convert(const EstimateWithCovariance& ec,
	                                       const aspn_xtensor::TypeTimestamp& time);

	/**
	 * Converts a state vector from the N-state `current`
	 * representation to the M-state `target` representation.
	 *
	 * @param x An estimate corresponding to a quantity represented by the
	 * `current` field (such as the estimate associated with a StateBlock whose
	 * `label == current`).
	 * @param time Time of validity for `x`.
	 *
	 * @return The estimate associated with `target`, obtained by mapping and
	 * using some (possibly) non-linear transform.
	 */
	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp& time) = 0;

	/**
	 * Calculate the Jacobian of the underlying transform function `f(x)`,
	 * evaluated about the input `x`.
	 *
	 * @param x State vector about which to evaluate the Jacobian.
	 * @param time Time of validity for `x`.
	 *
	 * @return An MxN Matrix representing the first-order derivative of
	 * the mapping function `f(x)` with respect to the N states of `x`.
	 */
	virtual Matrix jacobian(const Vector& x, const aspn_xtensor::TypeTimestamp& time) = 0;

	/**
	 * Receive and use arbitrary aux data sent from the sensor. This method will be called by the
	 * fusion engine when the fusion engine receives aux data from a
	 * give_virtual_state_block_aux_data call. The default implementation logs a warning that the
	 * VSB does not use the given type of aux data.
	 *
	 * To receive aux data, child classes should override this function and use
	 * `std::dynamic_pointer_cast` or similar to check the message type of each message in the
	 * incoming AspnBaseVector against the specific aspn_xtensor::AspnBase subclasses desired.
	 */
	virtual void receive_aux_data(const AspnBaseVector&);

	/** @return #current label */
	std::string get_current() const;

	/** @return #target label */
	std::string get_target() const;

protected:
	/**
	 * Label representing the format of EstimateWithCovariance quantities
	 * this VirtualStateBlock is capable of converting into the `target`
	 * representation. In other words, `current` represents a number
	 * states whose order, units and associated reference frames are
	 * fixed.
	 */
	std::string current = "";

	/**
	 * Label representing the format of EstimateWithCovariance quantities
	 * this VirtualStateBlock is capable of representing. In other words,
	 * `target` represents a number states whose order, units and
	 * associated reference frames are fixed, and are calculated by
	 * a mapping of states in the `current` format.
	 */
	std::string target = "";
};

}  // namespace filtering
}  // namespace navtk
