#include <navtk/utils/sortable_vectors.hpp>

#include <utility>

#include <navtk/tensors.hpp>

namespace navtk {
namespace utils {

template <>
std::vector<Size> find_duplicates(const std::vector<std::pair<double, double>>& data) {
	std::vector<Size> duplicate_indices;
	for (auto k = data.cbegin(); k < data.cend() - 1; k++) {
		if ((*k).first == (*(k + 1)).first) {
			duplicate_indices.push_back(k + 1 - data.cbegin());
		}
	}
	return duplicate_indices;
}
}  // namespace utils
}  // namespace navtk
