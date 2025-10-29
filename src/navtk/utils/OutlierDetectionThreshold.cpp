#include <navtk/utils/OutlierDetection.hpp>
#include <navtk/utils/OutlierDetectionThreshold.hpp>

#include <xtensor/core/xmath.hpp>
#include <xtensor/misc/xsort.hpp>
#include <xtensor/views/xindex_view.hpp>

#include <navtk/factory.hpp>
#include <navtk/tensors.hpp>

namespace navtk {

namespace utils {

OutlierDetectionThreshold::OutlierDetectionThreshold(size_t buffer_size, double threshold)
    : OutlierDetection(buffer_size), threshold(threshold) {
	if (std::isnan(threshold))
		log_or_throw<std::invalid_argument>("Threshold value must be a number");
	if (threshold < 0)
		log_or_throw<std::invalid_argument>("Threshold value must be positive or zero.");
}

bool OutlierDetectionThreshold::is_last_item_an_outlier(
    navtk::Vector const& value_history_vector) const {

	if (num_rows(value_history_vector) > 3) {
		auto median     = xt::median(value_history_vector);
		auto last_value = value_history_vector[num_rows(value_history_vector) - 1];

		// Check if one just loaded is outside of the allowable window for being "good"
		if (std::abs(last_value - median) > threshold) {
			return true;
		}
	}

	return false;
}

}  // namespace utils
}  // namespace navtk
