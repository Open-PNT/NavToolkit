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
 * A VirtualStateBlock that extracts some subset of states from another StateBlock.
 */
class StateExtractor : public VirtualStateBlock {

public:
	/**
	 * Constructor.
	 *
	 * @param current Label attached to the StateBlock (real or virtual) that this instance will
	 * convert.
	 * @param target Label to attach to the converted (virtual) StateBlock.
	 * @param incoming_state_size The number of states in the StateBlock that #current refers to.
	 * @param indices Collection of indices of states from #current that comprise #target.
	 *
	 * @throw std::invalid_argument if \p indices is empty, not unique, contains an index outside
	 * of [0, incoming_state_size), or if incoming_state_size == 0; but only if the error mode is
	 * ErrorMode::DIE for any case.
	 */
	StateExtractor(std::string current,
	               std::string target,
	               Size incoming_state_size,
	               const std::vector<Size>& indices);

	not_null<std::shared_ptr<VirtualStateBlock>> clone() override;

	/**
	 * Creates a state vector from a subset of the input.
	 *
	 * @param x A state vector representing the StateBlock referenced by #current.
	 *
	 * @return A vector made from a subset of the elements of the input as
	 * determined by the indices parameter passed into the constructor.
	 *
	 * @throw std::invalid_argument if the number of states do not match the incoming_state_size
	 * parameter that was provided to the constructor and the error mode is ErrorMode::DIE.
	 */
	virtual Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override;

	/**
	 * Returns the Jacobian of the mapping. Pre-computed on class initialization
	 * and invariant to state values.
	 *
	 * @return MxN Matrix that extracts M states from the the N-length state vector x, where M is
	 * the number of indices passed in to the constructor. For the kth index supplied, a 1 will
	 * be placed in the kth row, index column of the returned matrix.
	 */
	virtual Matrix jacobian(const Vector&, const aspn_xtensor::TypeTimestamp&) override;

private:
	/* Jacobian calculated during initialization.*/
	Matrix jac;
};

}  // namespace filtering
}  // namespace navtk
