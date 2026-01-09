#include <navtk/transform.hpp>

#include <navtk/errors.hpp>
#include <navtk/inspect.hpp>

namespace navtk {

Matrix replace_nan(const Matrix& arr, const double replacement_value) {
	Matrix out = arr;
	// test for nan entries
	for (size_t i = 0; i < num_rows(arr); i++) {
		for (size_t j = 0; j < num_cols(arr); j++) {
			if (arr(i, j) != arr(i, j)) out(i, j) = replacement_value;
		}
	}
	return out;
}

Vector replace_nan(const Vector& arr, const double replacement_value) {
	Vector out = arr;
	// test for nan entries
	for (size_t i = 0; i < num_rows(arr); i++) {
		if (arr(i) != arr(i)) out(i) = replacement_value;
	}
	return out;
}

decltype(xt::drop(std::vector<Size>())) drop_range(Size start_val, Size stop_val, Size step) {
	if (step == 0 || step > stop_val - start_val)
		log_or_throw<std::invalid_argument>("invalid step size");
	if (start_val > stop_val) log_or_throw<std::invalid_argument>("start_val must be <= stop_val");
	std::vector<Size> indices;
	for (auto idx = start_val; idx < stop_val; idx += step) indices.push_back(idx);
	return xt::drop(indices);
}

}  // namespace navtk
