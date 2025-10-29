#include <navtk/utils/OutlierDetection.hpp>
#include <navtk/utils/OutlierDetectionSigma.hpp>

#include <xtensor/core/xmath.hpp>
#include <xtensor/misc/xsort.hpp>
#include <xtensor/views/xindex_view.hpp>

#include <navtk/factory.hpp>
#include <navtk/tensors.hpp>

namespace navtk {

namespace utils {

OutlierDetectionSigma::OutlierDetectionSigma(size_t buffer_size, double sigma)
    : OutlierDetection(buffer_size), sigma(sigma) {
	if (std::isnan(sigma)) log_or_throw<std::invalid_argument>("Sigma multiplier must be a number");
	if (sigma < 0)
		log_or_throw<std::invalid_argument>("Sigma multiplier must be positive or zero.");
}

bool OutlierDetectionSigma::is_last_item_an_outlier(
    navtk::Vector const& value_history_vector) const {

	if (num_rows(value_history_vector) > 3) {

		auto median = xt::median(value_history_vector);

		auto diff_from_median = xt::abs(value_history_vector - median);
		auto median_of_diff   = xt::median(diff_from_median);
		auto hist_below_median =
		    xt::filter(value_history_vector, diff_from_median <= median_of_diff);
		auto std    = xt::stddev(hist_below_median)();
		double mean = xt::mean(hist_below_median)();

		// Check if one just loaded is outside of the allowable window for being "good"
		auto last_value = value_history_vector[num_rows(value_history_vector) - 1];
		if (std::abs(last_value - mean) > (sigma * std)) {
			return true;
		}
	}
	return false;
}


}  // namespace utils
}  // namespace navtk
