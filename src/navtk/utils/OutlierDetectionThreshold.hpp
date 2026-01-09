#include <navtk/utils/OutlierDetection.hpp>

namespace navtk {

namespace utils {
/**
 * OutlierDetection algorithm which checks whether the current value is within a certain
 * `threshold` relative to other data points.
 *
 * Callers use an OutlierDetectionThreshold object by passing a navtk::Vector into
 * the `is_last_item_an_outlier` method. This method returns true or false depending on
 * whether the data point in question is within or outside of the threshold
 * relative to the other data points.
 *
 * Implementations of this class override `is_last_item_an_outlier`, which will be
 * given a navtk::Vector of `buffer_size` length.
 *
 */
class OutlierDetectionThreshold : public OutlierDetection {

public:
	/**
	 * Constructor
	 * @param buffer_size Determines the size of the ring buffer where data is stored
	 * for analysis
	 * @param threshold The threshold value
	 *
	 */
	OutlierDetectionThreshold(size_t buffer_size, double threshold);

	/**
	 * determines whether the value before is an outlier using the threshold value
	 *
	 * @param data Vector of values where data is stored for outlier analysis
	 * @return boolean for whether the data is an outlier or not
	 */
	bool is_last_item_an_outlier(navtk::Vector const& data) const override;

private:
	/**
	 * Variable for threshold value
	 */
	double threshold;
};


}  // namespace utils
}  // namespace navtk
