#include <navtk/utils/OutlierDetection.hpp>

#include <xtensor/misc/xsort.hpp>

#include <navtk/factory.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace utils {

OutlierDetection::OutlierDetection(size_t buffer_size) : value_history(buffer_size) {}

bool OutlierDetection::is_outlier(double value) {

	value_history.push_back(value);

	navtk::Vector value_history_vector = navtk::zeros(value_history.size());
	navtk::Size ii                     = 0;

	for (auto &val : value_history) value_history_vector[ii++] = val;

	bool result = is_last_item_an_outlier(value_history_vector);
	return result;
}

}  // namespace utils
}  // namespace navtk
