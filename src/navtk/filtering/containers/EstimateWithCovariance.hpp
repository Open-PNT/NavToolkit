#pragma once

#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A simple container for holding an estimate and covariance.
 */
struct EstimateWithCovariance {
	/**
	 * Maps the parameters directly to the properties using `std::move`.
	 *
	 * @param estimate Initial value for #estimate.
	 * @param covariance Initial value for #covariance.
	 *
	 * @throw std::range_error If estimate is not Nx1 or covariance is not NxN and the error
	 * mode is ErrorMode::DIE.
	 * @throw std::domain_error If covariance is not symmetric and the error mode is
	 * ErrorMode::DIE.
	 */
	EstimateWithCovariance(Vector estimate, Matrix covariance);

	/**
	 * An Nx1 estimate.
	 */
	Vector estimate;

	/**
	 * An NxN covariance matrix for the estimate.
	 */
	Matrix covariance;
};
}  // namespace filtering
}  // namespace navtk
