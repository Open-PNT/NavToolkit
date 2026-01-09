#pragma once

#include <navtk/tensors.hpp>
#include <navtk/utils/RingBuffer.hpp>

namespace navtk {
namespace utils {

/**
 * Utility class for checking for outliers in sequences of `double` data.
 *
 * Callers use an OutlierDetection object by passing sequential measurements
 * into the `is_outlier` method. This class will save the `buffer_size`-most-recent
 * values and pass a navtk::Vector of those values to subclasses.
 *
 * Implementations of this class must override `is_last_item_an_outlier` which will be
 * given a navtk::Vector of the `buffer_size` most recent values passed into `is_outlier`
 * and must determine whether the most recent value (the last value in the vector) is
 * an outlier given the data so far.
 *
 * Because this class maintains a history of values, different streams of data (including each
 * individual state in a filtering solution) must each have their own instance. Because
 * `is_outlier` only examines the most recent `value` relative to the history so far, it is not
 * possible to use this class to detect outliers retroactively.
 */
class OutlierDetection {
public:
	virtual ~OutlierDetection() = default;
	/**
	 * Constructor
	 * @param buffer_size Determines the size of the ring buffer where the data
	 * is stored for outlier analysis
	 *
	 */
	OutlierDetection(size_t buffer_size);

	/**
	 * Adds the given value to the value history buffer and returns true iff it
	 * is an outlier relative to values stored in the buffer so far.
	 * @param value Value to be processed by the outlier detection algorithm.
	 *
	 * @return boolean for whether the data value is an outlier or not
	 */
	bool is_outlier(double value);

private:
	/**
	 * determines whether the value before is an outlier. Overridden in
	 * the child classes to do this in different ways (sigma multiplier
	 * vs. threshold method)
	 *
	 * @param data Vector of values where data is stored for outlier analysis
	 * @return boolean for whether the data is an outlier or not
	 */
	virtual bool is_last_item_an_outlier(navtk::Vector const& data) const = 0;

	/**
	 * Buffer of values used in the outlier detection algorithm.
	 */
	RingBuffer<double> value_history;

};  // class OutlierDetection

}  // namespace utils
}  // namespace navtk
