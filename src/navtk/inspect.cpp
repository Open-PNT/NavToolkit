#include <navtk/inspect.hpp>

namespace navtk {


std::vector<std::pair<Size, Size>> non_symmetric_elements_impl(const Matrix& mat,
                                                               double rtol,
                                                               double atol,
                                                               bool quick_fail) {
	std::vector<std::pair<Size, Size>> failures;
	auto rows = num_rows(mat);

	for (Size row = 0; row < rows; ++row) {
		for (Size col = row + 1; col < rows; ++col) {
			auto a         = mat(col, row);
			auto precision = atol + (rtol * std::abs(a));
			if (std::abs(mat(row, col) - a) > precision) {
				failures.push_back({row, col});
				if (quick_fail) {
					return failures;
				}
			}
		}
	}
	return failures;
}

bool is_identity(const Matrix& m) {
	auto rows = num_rows(m);
	if (rows == 0) {
		return false;
	}

	for (Size row = 0; row < rows; ++row) {
		if (m(row, row) != 1) {
			return false;
		}
	}
	return is_diagonal(m);
}

bool is_diagonal(const Matrix& mat) {
	auto rows = num_rows(mat);

	for (Size row = 0; row < rows; ++row) {
		for (Size col = row + 1; col < rows; ++col) {
			// double comp, should we add any wiggle room?
			if (mat(row, col) != 0 || mat(col, row) != 0) {
				return false;
			}
		}
	}
	return true;
}

std::vector<std::pair<Size, Size>> non_symmetric_elements(const Matrix& mat,
                                                          double rtol,
                                                          double atol) {
	return non_symmetric_elements_impl(mat, rtol, atol, false);
}

bool is_symmetric(const Matrix& mat, double rtol, double atol) {
	return non_symmetric_elements_impl(mat, rtol, atol, true).empty();
}

Size num_rows(const Matrix& m) { return has_zero_size(m) ? 0 : m.shape()[0]; }
Size num_rows(const Vector& c) { return has_zero_size(c) ? 0 : c.size(); }
Size num_cols(const Matrix& m) { return has_zero_size(m) ? 0 : m.shape()[1]; }
Size num_cols(const Vector& r) { return has_zero_size(r) ? 0 : r.size(); }

}  // namespace navtk
