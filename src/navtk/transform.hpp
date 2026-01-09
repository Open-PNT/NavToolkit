#pragma once

#include <navtk/navutils/math.hpp>

namespace navtk {

/**
 * Return an array with \p replacement_value substituted in for NaN entries.
 * @param arr The input array that may have NaN entries.
 * @param replacement_value The value to replace the NaN entry.
 *
 * @return \p arr with NaN values replaced with \p replacement_value.
 */
Matrix replace_nan(const Matrix& arr, const double replacement_value);

/**
 * Return an array with \p replacement_value substituted in for NaN entries.
 * @param arr The input array that may have NaN entries.
 * @param replacement_value The value to replace the NaN entry.
 *
 * @return \p arr with NaN values replaced with \p replacement_value.
 */
Vector replace_nan(const Vector& arr, const double replacement_value);

/**
 * Create a non-contiguous slice that excludes the given range of indices, to
 * be used as a parameter to `xt::view`.
 *
 * Ranges are in the form `start_val <= idx < stop_val`.
 *
 * @param start_val the first index to be dropped
 * @param stop_val the index after the last to be dropped
 * @param step increment between indices, defaults to 1 which includes every
 * index in the given range.
 * @return An `xt::drop` pre-loaded with indices in the given range.
 */
decltype(xt::drop(std::vector<Size>())) drop_range(Size start_val, Size stop_val, Size step = 1);

}  // namespace navtk
