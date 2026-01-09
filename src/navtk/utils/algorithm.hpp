#pragma once

#include <algorithm>
#include <functional>
#include <type_traits>
#include <utility>

namespace navtk {
namespace utils {

/**
 * Calculate the area of a right trapezoid
 * @param x0 The value that gives the lower x bound of the trapezoid.
 * @param y0 The value that gives the lower y bound of the trapezoid.
 * @param x1 The value that gives the upper x bound of the trapezoid.
 * @param y1 The value that gives the upper y bound of the trapezoid.
 * @return Area of the trapezoid formed by the points (`x0`, `y0`), (`x0`, `0`), (`x1`, `y1`),
 * (`x1`, `0`).
 */
template <typename X, typename Y>
Y trapezoidal_area(X x0, const Y& y0, X x1, const Y& y1) {
	static_assert(std::is_convertible<X, double>::value, "X must be convertible to a double");
	return (x1 - x0) * (y0 + y1) / 2.0;
}

/**
 * Gets the elements in a specified range from a container.
 * The container must have random access iterators.
 */
template <typename Iterator, typename Compare = std::less<typename Iterator::value_type>>
struct InRange {
	/**
	 * Type used to order the container
	 */
	using T = typename Iterator::value_type;

	/**
	 * @param begin The beginning of the container search.
	 * @param end The end (one past the last element) of the container search.
	 * @param t0 The beginning of value of the range.
	 * @param t1 The end value of the range.
	 *
	 * @return Pair of iterators representing elements in the range [t0, t1).
	 */
	std::pair<Iterator, Iterator> get(Iterator begin,
	                                  Iterator end,
	                                  const T& t0,
	                                  const T& t1) const {
		Compare compare;
		auto lower = std::lower_bound(begin, end, t0, compare);
		auto upper = std::upper_bound(lower, end, t1, compare);
		if (lower == upper) lower = upper = end;
		return {lower, upper};
	}
};

/**
 * Gets the elements nearest to the specified value in the container.
 * The container must have random access iterators and be ordered.
 */
template <typename Iterator, typename Compare = std::less<typename Iterator::value_type>>
struct NearestNeighbors {
	/**
	 * Type used to order the container
	 */
	using T = typename Iterator::value_type;

	/**
	 * @param begin The beginning of the container search.
	 * @param end The end (one past the last element) of the container search.
	 * @param t The element to find neighbors of.
	 *
	 * @return Pair of iterators representing the nearest elements to `t`. If `t` is before
	 * all elements then return will be `{end, begin}`; if `t` is after all elements return will
	 * be `{end - 1, end}`. If t is exactly matched, both iterators will be to the last such element
	 * that matched `t`. Finally, if `t` is between two elements then the return will be the
	 * elements before and after `t`.
	 */
	std::pair<Iterator, Iterator> get(Iterator begin, Iterator end, const T& t) const {
		if (begin == end) return {end, end};

		Compare compare;
		auto upper = std::upper_bound(begin, end, t, compare);
		if (upper == begin) {
			return {end, begin};
		}

		auto lower = upper - 1;
		if (!compare(*lower, t) && !compare(t, *lower)) {
			upper = lower;  // exact match
		}
		return {lower, upper};
	}
};

}  // namespace utils
}  // namespace navtk
