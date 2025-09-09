#include <navtk/utils/ValidationContext.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <spdlog/spdlog.h>

#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/utils/DimensionValidator.hpp>

using std::map;
using std::pair;
using std::set;
using std::string;
using std::to_string;


namespace navtk {
namespace utils {

class DimensionValidator;

namespace {
const string DEFAULT_MATRIX_NAME = "Matrix";
const string DEFAULT_VECTOR_NAME = "Vector";
}  // namespace

ValidationContext::ValidationContext() : ValidationContext(get_global_error_mode()) {}

ValidationContext::ValidationContext(ErrorMode mode)
    : dimension_validator{mode == ErrorMode::OFF ? nullptr
                                                 : std::make_unique<DimensionValidator>()},
      matrices_to_validate{},
      current_matrix{},
      current_matrix_name{},
      cached_result{mode == ErrorMode::OFF ? ValidationResult::NOT_CHECKED
                                           : ValidationResult::GOOD},
      mode(mode) {}

ValidationContext::~ValidationContext() {
	if (validation_needed()) {
		spdlog::error("Validation context destroyed without calling validate() function.");
	}
}

ValidationContext& ValidationContext::add_matrix(const Matrix& matrix) {
	return add_matrix(matrix, DEFAULT_MATRIX_NAME);
}

ValidationContext& ValidationContext::add_matrix(const Matrix& matrix, const string& name) {
	if (is_enabled()) {
		matrices_to_validate.push_back({matrix, name});
		current_matrix      = &matrices_to_validate.back().first;
		current_matrix_name = &matrices_to_validate.back().second;
	}
	return *this;
}

ValidationContext& ValidationContext::add_matrix(const Vector& vec) {
	if (is_enabled()) {
		return add_matrix(to_matrix(vec), DEFAULT_VECTOR_NAME);
	}
	return *this;
}

ValidationContext& ValidationContext::add_matrix(const Vector& vec, const string& name) {
	if (is_enabled()) {
		return add_matrix(to_matrix(vec), name);
	}
	return *this;
}

ValidationContext& ValidationContext::symmetric(double rtol, double atol) {
	if (!check_current_matrix()) return *this;

	auto rows = num_rows(*current_matrix);
	auto cols = num_cols(*current_matrix);

	if (rows != cols) {
		cached_result = ValidationResult::BAD;
		log_or_throw<std::out_of_range>(get_mode(),
		                                "{} must be symmetric, but has dimensions {} x {}.",
		                                *current_matrix_name,
		                                rows,
		                                cols);
		return *this;
	}

	auto bad = non_symmetric_elements(*current_matrix, rtol, atol);
	if (!bad.empty()) {
		cached_result = ValidationResult::BAD;
	}
	for (auto bad_pair = bad.cbegin(); bad_pair != bad.cend(); bad_pair++) {
		log_or_throw<std::domain_error>(get_mode(),
		                                "{0} must be symmetric, but {0}[{1},{2}] != {0}[{2},{1}]",
		                                *current_matrix_name,
		                                bad_pair->first,
		                                bad_pair->second);
	}

	return *this;
}

ValidationContext& ValidationContext::max(double limit) {
	if (!check_current_matrix()) return *this;

	for (Size row = 0; row < num_rows(*current_matrix); ++row) {
		for (Size col = 0; col < num_cols(*current_matrix); ++col) {
			auto element = (*current_matrix)(row, col);
			if (element > limit) {
				cached_result = ValidationResult::BAD;
				log_or_throw<std::invalid_argument>(get_mode(),
				                                    "{}[{},{}] (value: {}) > {} (maximum)",
				                                    *current_matrix_name,
				                                    row,
				                                    col,
				                                    element,
				                                    limit);
			}
		}
	}

	return *this;
}

ValidationContext& ValidationContext::min(double limit) {
	if (!check_current_matrix()) return *this;

	for (Size row = 0; row < num_rows(*current_matrix); ++row) {
		for (Size col = 0; col < num_cols(*current_matrix); ++col) {
			auto element = (*current_matrix)(row, col);
			if (element < limit) {
				cached_result = ValidationResult::BAD;
				log_or_throw<std::invalid_argument>(get_mode(),
				                                    "{}[{},{}] (value: {}) < {} (minimum)",
				                                    *current_matrix_name,
				                                    row,
				                                    col,
				                                    element,
				                                    limit);
			}
		}
	}

	return *this;
}

ValidationContext& ValidationContext::dim(Size rows, int cols) {
	return this->dim(rows, static_cast<Size>(cols));
}

ValidationContext& ValidationContext::dim(int rows, Size cols) {
	return this->dim(static_cast<Size>(rows), cols);
}

ValidationContext& ValidationContext::dim(int rows, int cols) {
	return this->dim(static_cast<Size>(rows), static_cast<Size>(cols));
}

ValidationContext& ValidationContext::dim(Size rows, Size cols) {
	if (check_current_matrix()) {
		dimension_validator->dim(*current_matrix_name, *current_matrix, rows, cols);
		mark_validation_needed();
	}

	return *this;
}

ValidationContext& ValidationContext::dim(int rows, char cols) {
	return this->dim(static_cast<Size>(rows), cols);
}

ValidationContext& ValidationContext::dim(Size rows, char cols) {
	if (check_current_matrix()) {
		dimension_validator->dim(*current_matrix_name, *current_matrix, rows, cols);
		mark_validation_needed();
	}

	return *this;
}

ValidationContext& ValidationContext::dim(char rows, int cols) {
	return this->dim(rows, static_cast<Size>(cols));
}

ValidationContext& ValidationContext::dim(char rows, Size cols) {
	if (check_current_matrix()) {
		dimension_validator->dim(*current_matrix_name, *current_matrix, rows, cols);
		mark_validation_needed();
	}

	return *this;
}

ValidationContext& ValidationContext::dim(char rows, char cols) {
	if (check_current_matrix()) {
		dimension_validator->dim(*current_matrix_name, *current_matrix, rows, cols);
		mark_validation_needed();
	}

	return *this;
}

ValidationResult ValidationContext::validate() {
	if (validation_needed()) {
		dimension_validator->perform_validation(get_mode(), /*out*/ cached_result);
	}
	return cached_result;
}

ErrorMode ValidationContext::get_mode() const { return mode; }

bool ValidationContext::is_enabled() const { return get_mode() != ErrorMode::OFF; }

ValidationContext::operator bool() const { return is_enabled(); }


bool ValidationContext::check_current_matrix() const {
	if (current_matrix == nullptr && is_enabled()) {
		log_or_throw(
		    get_mode(),
		    "No matrices have been set to validate.  Add with 'add_matrix' function first.");
	}
	return current_matrix != nullptr;
}

void ValidationContext::mark_validation_needed() {
	if (cached_result != ValidationResult::BAD) cached_result = ValidationResult::NOT_CHECKED;
}

bool ValidationContext::validation_needed() const {
	return is_enabled() && cached_result == ValidationResult::NOT_CHECKED;
}

}  // namespace utils
}  // namespace navtk
