#pragma once

#include <memory>
#include <type_traits>

#include <navtk/aspn.hpp>
#include <navtk/inertial/ImuErrors.hpp>
#include <navtk/not_null.hpp>
#include <navtk/utils/IteratorAdapter.hpp>
#include <navtk/utils/Ordered.hpp>

namespace navtk {
namespace filtering {

/**
 * Compare object for ordering `std::shared_ptr<TimestampedData>` in a container
 */
template <typename T, typename Compare = std::less<int64_t>>
struct TimestampedDataPointerCompare {
	/**
	 * Comparison operator
	 * @param x first value to compare
	 * @param y second value to compare
	 * @return The result of comparison of time of validity in nanoseconds. For example, when
	 * Compare is `std::less` (the default), this is equivalent to `x->get_time_of_validity() <
	 * y.get_time_of_validity()`.  Using `get_aspn_c()` to avoid copying
	 * `aspn_xtensor::TypeTimestamp` values.
	 */
	bool operator()(not_null<std::shared_ptr<T>> x, not_null<std::shared_ptr<T>> y) const {
		return Compare()(x->get_aspn_c()->time_of_validity.elapsed_nsec,
		                 y->get_aspn_c()->time_of_validity.elapsed_nsec);
	}
};

/**
 * Compare object for ordering `std::shared_ptr<TimestampedData>` in a container
 */
template <>
struct TimestampedDataPointerCompare<inertial::ImuErrors, std::less<int64_t>> {
	/**
	 * Comparison operator
	 * @param x first value to compare
	 * @param y second value to compare
	 * @return `Compare()(x->get_time_of_validity(), y->get_time_of_validity())`. When Compare is
	 * `std::less` (the default), this is equivalent to
	 * `x->get_time_of_validity() < y.get_time_of_validity()`.
	 */
	bool operator()(not_null<std::shared_ptr<inertial::ImuErrors>> x,
	                not_null<std::shared_ptr<inertial::ImuErrors>> y) const {
		return std::less<int64_t>()(x->time_validity.get_elapsed_nsec(),
		                            y->time_validity.get_elapsed_nsec());
	}
};

/**
 * Function for getting the time from `std::shared_ptr<TimestampedData>`
 * @param it Iterator of time values.
 * @return The time from the iterator.
 */
template <typename Time, typename Iterator>
Time get_time_value(Iterator it) {
	return (*it)->get_time_of_validity();
}

template <>
/**
 * Function for getting the time from `std::shared_ptr<TimestampedData>`
 * @param it Iterator of time values.
 * @return The time from the iterator.
 */
aspn_xtensor::TypeTimestamp get_time_value(
    utils::RingBuffer<std::shared_ptr<inertial::ImuErrors>>::const_iterator it);

/**
 * Iterator adapter for getting the time from `std::shared_ptr<TimestampedData>`
 */
template <typename Time, typename Iterator>
using TimestampedDataTimeIterator =
    utils::IteratorAdapter<Iterator, Time, &get_time_value<Time, Iterator>>;


namespace detail {
template <typename T, typename D>
using match_const = typename std::conditional<std::is_const<T>::value,
                                              typename std::add_const<D>::type,
                                              typename std::remove_const<D>::type>::type;
}


/**
 * Templated `TimestampedDataSeries` (a convenience type that allows
 * `std::shared_ptr<TimestampedData>` or `std::shared_ptr<const TimestampedData>`)
 */
template <typename D = aspn_xtensor::AspnBase,
          typename T = detail::match_const<D, aspn_xtensor::TypeTimestamp>>
using TimestampedDataSeries = utils::OrderedRing<
    std::shared_ptr<D>,
    TimestampedDataPointerCompare<D>,
    TimestampedDataTimeIterator<T, typename utils::RingBuffer<std::shared_ptr<D>>::const_iterator>,
    std::less<T>>;

}  // namespace filtering
}  // namespace navtk
