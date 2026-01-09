#pragma once

#include <utility>

#include <navtk/tensors.hpp>

namespace navtk {
namespace utils {

/**
 * Normalize a vector of data to some baseline value.
 *
 * @tparam T Data type stored in vector; must have - operator defined.
 * @param orig Original data set. Original data with `min_val` subtracted from each element is
 * returned through this parameter.
 * @param min_val Baseline value.
 */
template <typename T>
void normalize(std::vector<T> &orig, T min_val) {
	for (Size k = 0; k < orig.size(); k++) {
		orig[k] -= min_val;
	}
}

/**
 * Generate differences between consecutive elements in a vector.
 *
 * @tparam T Data type stored in vector; must have - operator defined.
 * @param data Data to difference, size N.
 *
 * @return A vector of size `N - 1`, where the Nth element is equal to `data[N + 1] - data[N]`.
 * Returns an empty vector if N < 2.
 */
template <typename T>
std::vector<T> diff(std::vector<T> data) {
	if (data.size() > 1) {
		std::vector<T> out;
		out.reserve(data.size() - 1);
		for (auto x = data.cbegin(); x < data.cend() - 1; x++) {
			out.push_back(*(x + 1) - *x);
		}
		return out;
	}
	return std::vector<T>();
}

/**
 * Generate a vector of indices marking consecutive duplicates.
 *
 * @tparam T Data type stored in vector; must have == operator defined.
 * @param data The data to check.
 *
 * @return A vector of indices where the index `k` will be included if `data[k-1] == data[k]`.
 * Return vector will be empty of no duplicates are found.
 */
template <typename T>
std::vector<Size> find_duplicates(const std::vector<T> &data) {
	std::vector<Size> duplicate_indices;
	for (auto k = data.cbegin(); k < data.cend() - 1; k++) {
		if (*k == *(k + 1)) {
			duplicate_indices.push_back(k + 1 - data.cbegin());
		}
	}
	return duplicate_indices;
}

/**
 * Generate a vector of indices marking consecutive duplicates. Template specialization for pairs,
 * where equivalence is based only on the equality of the 'first' members of consecutive elements.
 *
 * @param data The data to check.
 *
 * @return A vector of indices where the index `k` will be included if `data[k-1] == data[k]`.
 * Return vector will be empty if no duplicates are found.
 */
template <>
std::vector<Size> find_duplicates(const std::vector<std::pair<double, double>> &data);

/**
 * Generate a vector of indices marking elements considered outside of the range of another data
 * set.
 *
 * @tparam T Data type stored in vector.
 * @param query_time Values checked for 'belonging'.
 * @param data Data set of pairs of tags (first) and data elements (second); must be sorted
 * according to 'first' members.
 *
 * @return A vector of indices where the index `k` will be included if `query_time[k]` is not
 * between or equal to the first and last elements of data as determined by comparison with the
 * 'first' members of each pair. Return vector will be empty if all elements `query_time` are within
 * `data`.
 */
template <typename T>
std::vector<Size> find_outside(const std::vector<double> &query_time,
                               const std::vector<std::pair<double, T>> &data) {
	std::vector<Size> external_indices;
	for (Size k = 0; k < query_time.size(); k++) {
		if (query_time[k] < data[0].first || query_time[k] > data[data.size() - 1].first) {
			external_indices.push_back(k);
		}
	}
	return external_indices;
}

/**
 * Remove elements from vector of data based on a set of indices.
 *
 * @tparam T Data type stored in vector.
 * @param data The vector to remove elements from.
 * @param to_remove Indices of elements to remove. Should be sorted.
 */
template <typename T>
void remove_at_indices(std::vector<T> &data, const std::vector<Size> &to_remove) {
	if (!to_remove.empty()) {
		for (auto rm = to_remove.cend() - 1; rm >= to_remove.cbegin(); rm--) {
			data.erase(data.cbegin() + *rm);
		}
	}
}

/**
 * Unzip a vector of pairs to a pair of vectors.
 *
 * @tparam U Data type in 'first' members.
 * @tparam T Data type in 'second' members.
 * @param orig Data to unzip.
 *
 * @return A vector of all first members, and a vector of all second members, copied from the input.
 */
template <typename U, typename T>
std::pair<std::vector<U>, std::vector<T>> split_vector_pairs(
    const std::vector<std::pair<U, T>> &orig) {
	std::vector<U> first;
	std::vector<T> second;
	first.reserve(orig.size());
	second.reserve(orig.size());
	for (auto k = orig.cbegin(); k < orig.cend(); k++) {
		first.push_back((*k).first);
		second.push_back((*k).second);
	}
	return {first, second};
}

/**
 * Zip a pair of vectors into a vector of paired elements, and sort according to the first elements.
 *
 * @tparam T Data type in 'second' members.
 * @param tags Data to sort by, size N.
 * @param data Data to be zipped with tags, must be at least size N. If more than N elements are
 * present they are ignored.
 *
 * @return A sorted vector of `{tags[k], data[k]}.
 */
template <typename T>
std::vector<std::pair<double, T>> pair_and_time_sort_data(const std::vector<double> &tags,
                                                          const std::vector<T> &data) {
	std::vector<std::pair<double, T>> paired;
	for (Size k = 0; k < tags.size(); k++) {
		paired.push_back(std::make_pair(tags[k], data[k]));
	}
	sort(paired.begin(),
	     paired.begin() + paired.size(),
	     [](std::pair<double, T> &msg1, std::pair<double, T> &msg2) {
		     return msg1.first < msg2.first;
	     });
	return paired;
}
}  // namespace utils
}  // namespace navtk
