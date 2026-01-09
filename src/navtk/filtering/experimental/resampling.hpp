#pragma once

#include <functional>

#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {
namespace experimental {

/**
 * Structure to hold a vector of indexes and their count from the resampling function
 *
 */
struct ResamplingResult {
	/**
	 * `index` is the index of the parent particles.
	 */
	std::vector<size_t> index;
	/**
	 * `index_count` is a count of the number of children for each mapped particle.
	 */
	std::vector<size_t> index_count;
};

/**
 * Stratified resampling method to select samples as children (duplicates) to higher weighted
 * samples
 *
 * @param weights The posterior weights.
 * @param m_arg A pointer to the number of samples to be returned. Default = `nullptr`.
 *
 * @return A structure containing a vector of indexes to resample the distribution and the number of
 * particle count mapped to the index.
 *
 */
ResamplingResult systematic_resampling(const Vector& weights, const size_t* m_arg);

/**
 * Residual resampling method to select samples as children (duplicates) to higher weighted samples
 *
 * @param weights The posterior weights.
 * @param m_arg A pointer to the number of samples to be returned. Default = `nullptr`.
 *
 * @return A structure containing a vector of indexes to resample the distribution and the number of
 * particle count mapped to the index.
 *
 */
ResamplingResult residual_resample_with_replacement(const Vector& weights, const size_t* m_arg);

}  // namespace experimental
}  // namespace filtering
}  // namespace navtk
