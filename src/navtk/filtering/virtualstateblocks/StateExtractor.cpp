#include <navtk/filtering/virtualstateblocks/StateExtractor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/linear_algebra.hpp>

namespace navtk {
namespace filtering {

StateExtractor::StateExtractor(std::string current,
                               std::string target,
                               Size incoming_state_size,
                               const std::vector<Size>& indices)
    : VirtualStateBlock(std::move(current), std::move(target)) {

	if (incoming_state_size == 0)
		log_or_throw<std::invalid_argument>("Argument incoming_state_size must be 1 or greater.");

	if (indices.size() == 0)
		log_or_throw<std::invalid_argument>(
		    "Must provide at least 1 index for an element to keep.");
	if (std::any_of(
	        indices.begin(), indices.end(), [incoming_state_size = incoming_state_size](Size ind) {
		        return ind >= incoming_state_size;
	        }))
		log_or_throw<std::invalid_argument>(
		    "One or more indices provided exceeds the length of the expected state vector.");
	// Has to be assigned, or num_rows call is ambiguous
	auto sorted = indices;
	std::sort(sorted.begin(), sorted.end());
	auto un = std::unique(sorted.begin(), sorted.end());
	sorted.erase(un, sorted.end());
	if (indices.size() != sorted.size())
		log_or_throw<std::invalid_argument>("Repeat indices are not allowed.");

	jac = zeros(indices.size(), incoming_state_size);
	for (Size i = 0; i < indices.size(); i++) {
		jac(i, indices[i]) = 1.0;
	}
}

not_null<std::shared_ptr<VirtualStateBlock>> StateExtractor::clone() {
	return std::make_shared<StateExtractor>(*this);
}

Vector StateExtractor::convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	if (num_cols(jac) != num_rows(x))
		log_or_throw<std::invalid_argument>(
		    "State block to alias does not contain the expected number of states.");
	return dot(jac, x);
}

Matrix StateExtractor::jacobian(const Vector&, const aspn_xtensor::TypeTimestamp&) { return jac; }

}  // namespace filtering
}  // namespace navtk
