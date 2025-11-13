#include <navtk/utils/DimensionValidator.hpp>

#include <algorithm>
#include <iterator>
#include <list>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <navtk/inspect.hpp>

using std::list;
using std::string;

namespace navtk {
namespace utils {

namespace {

bool is_number(const string& s) {
	return !s.empty() && s.find_first_not_of("0123456789") == string::npos;
}

string write_column(int width, string text, char lpad = 0, char rpad = ' ', char gap = ' ') {

	auto sstream = std::stringstream{};

	auto nrpad = (rpad == 0) ? 0 : (width - text.length()) / ((lpad == 0) ? 1 : 2);
	auto nlpad = (lpad == 0) ? 0 : (width - nrpad - text.length());

	if (lpad != 0) {
		for (auto i = size_t{0}; i < nlpad; ++i) {
			sstream << lpad;
		}
	}

	sstream << text;

	if (rpad != 0) {
		for (auto i = size_t{0}; i < nrpad; ++i) {
			sstream << rpad;
		}
	}

	if (gap != 0) {
		sstream << gap;
	}

	return sstream.str();
}

string concoct_fixed_description(string rows, string cols) {
	auto sstream = std::stringstream{};

	if (rows == "1") {
		sstream << "a " << cols << "-vector";
	} else if (cols == "1") {
		sstream << "a " << rows << "-vector";
	} else {
		sstream << rows << " by " << cols;
	}

	return sstream.str();
}

}  // namespace

/**
 * Container for data needed to execute dimension checks against a matrix.
 */
struct MatrixDimInfo {
	/**
	 * A pointer to the matrix to check.  Assumes that the lifetime of the matrix is controlled
	 * outside of the validation object, and that the lifetime will exceed that of the validation
	 * object.
	 */
	const Matrix* matrix;

	/**
	 * A copy of the name of the matrix.  Because of temporary strings and block scope, the
	 * lifetime of the string cannot be assumed to exceed that of the validation object.
	 */
	const string matrix_name;

	/**
	 * String form of desired row dimension.  Could be a string representation of an integer,
	 * or could be a single character representing a dimension variable.
	 */
	string declared_row_dimension;

	/**
	 * String form of desired column dimension.  Could be a string representation of an integer,
	 * or could be a single character representing a dimension variable.
	 */
	string declared_col_dimension;

	/**
	 * Set of possible row dimensions, assuming declared_row_dimension is a variable.
	 */
	std::set<Size> possible_row_dimensions;

	/**
	 * Set of possible column dimensions, assuming declared_col_dimension is a variable.
	 */
	std::set<Size> possible_col_dimensions;
};

/**
 * Implementation of dimension validation check.
 */
class DimensionValidatorPrivate {
public:
	/**
	 * For named variables, the possible values they might take based on scanned matrix dimensions.
	 */
	std::map<char, std::set<Size>> possible_values;

	/**
	 * All dimension information required per matrix to scan.
	 */
	list<MatrixDimInfo> matrix_dimension_info;

	/**
	 * Matrix names we've already complained about while building our error string.
	 */
	std::set<string> seen_in_error;

	/**
	 * Provides possible dimensions for the given dimension variable letter.
	 * If this letter hasn't been checked before, the function returns the
	 * same dimensions that were passed in.  If it has been checked before, the
	 * return value is a set intersection of the first dimensions that were
	 * saved and the dimensions that are being passed in now.
	 *
	 * @param letter The dimension variable.
	 * @param dimensions New possible dimensions for the letter argument.
	 *
	 * @returns The best guess at possible dimensions for the letter argument.
	 */
	std::set<Size> infer_values(char letter, std::set<Size> dimensions) {
		// insert() will not overwrite a map entry for the key if it already exists
		const auto& result = possible_values.insert({letter, dimensions});

		// map_entry is either the passed-in dimensions argument or the value that was inserted in
		// the map prior to this call
		const auto& map_entry = result.first->second;
		auto possible         = std::set<Size>{};

		// Our list of possibilities should only include those in both sets -
		// the existing map_entry and the passed in dimensions
		set_intersection(map_entry.begin(),
		                 map_entry.end(),
		                 dimensions.begin(),
		                 dimensions.end(),
		                 std::inserter(possible, possible.begin()));

		return possible;
	}

	/**
	 * Store the provided information for a matrix for validation later.
	 *
	 * @param name The name of the matrix to show in errors.
	 * @param matrix The matrix to validate.
	 * @param rows The desired number of rows in the matrix.
	 * @param cols The desired number of columns in the matrix.
	 */
	void dim(const string& name, const Matrix& matrix, Size rows, Size cols) {
		auto info = MatrixDimInfo{
		    &matrix, name, std::to_string(rows), std::to_string(cols), {rows}, {cols}};
		matrix_dimension_info.push_back(info);
	}

	/**
	 * Store the provided information for a matrix for validation later.
	 *
	 * @param name The name of the matrix to show in errors.
	 * @param matrix The matrix to validate.
	 * @param rows The desired number of rows in the matrix.
	 * @param cols Variable name for number of columns in the matrix.
	 */
	void dim(const string& name, const Matrix& matrix, Size rows, char cols) {
		auto inference_source = num_cols(matrix);

		if (num_cols(matrix) == rows) {
			inference_source = num_rows(matrix);
		}

		auto info = MatrixDimInfo{&matrix,
		                          name,
		                          std::to_string(rows),
		                          string(1, cols),
		                          {rows},
		                          infer_values(cols, {inference_source})};

		matrix_dimension_info.push_back(info);
	}

	/**
	 * Store the provided information for a matrix for validation later.
	 *
	 * @param name The name of the matrix to show in errors.
	 * @param matrix The matrix to validate.
	 * @param rows Variable name for number of rows in the matrix.
	 * @param cols The desired number of columns in the matrix.
	 */
	void dim(const string& name, const Matrix& matrix, char rows, Size cols) {
		auto inference_source = num_rows(matrix);

		if (num_rows(matrix) == cols) {
			inference_source = num_cols(matrix);
		}

		auto info = MatrixDimInfo{&matrix,
		                          name,
		                          string(1, rows),
		                          std::to_string(cols),
		                          infer_values(rows, {inference_source}),
		                          {cols}};

		matrix_dimension_info.push_back(info);
	}

	/**
	 * Store the provided information for a matrix for validation later.
	 *
	 * @param name The name of the matrix to show in errors.
	 * @param matrix The matrix to validate.
	 * @param rows Variable name for number of rows in the matrix.
	 * @param cols Variable name for number of columns in the matrix.
	 */
	void dim(const string& name, const Matrix& matrix, char rows, char cols) {
		auto possible_row_dimensions = infer_values(rows, {num_rows(matrix), num_cols(matrix)});
		auto possible_col_dimensions = infer_values(cols, {num_rows(matrix), num_cols(matrix)});

		// If the same variable is used for both rows and columns, AND they don't actually match,
		// then there aren't any possible dimensions for the variable.  It will fail validation.
		if ((rows == cols) && (num_rows(matrix) != num_cols(matrix))) {
			possible_row_dimensions.clear();
			possible_col_dimensions.clear();
		}

		auto info = MatrixDimInfo{&matrix,
		                          name,
		                          string(1, rows),
		                          string(1, cols),
		                          possible_row_dimensions,
		                          possible_col_dimensions};

		matrix_dimension_info.push_back(info);
	}

	/**
	 * Build a list of things compare_to needs to be the same as. Looks up other things that have
	 * the same character-based dimension defined.
	 *
	 * @param name The name of the matrix that needs comparison.
	 * @param compare_to The dimension variable to check to give comparisons.
	 *
	 * @returns A list of strings containing other matrices' usage of compare_to.
	 */
	list<string> concoct_comparison(string name, string compare_to) {
		auto out = list<string>{};

		if (seen_in_error.count(compare_to) == 0) {
			for (const auto& dim_info : matrix_dimension_info) {
				const auto& name2 = dim_info.matrix_name;
				auto rows         = dim_info.declared_row_dimension;
				auto cols         = dim_info.declared_col_dimension;

				if (name2 != name) {
					if (rows == compare_to) out.push_back("as " + name2 + " has rows");
					if (cols == compare_to) out.push_back("as " + name2 + " has columns");
				}
			}
			seen_in_error.insert(compare_to);
		}

		return out;
	}

	/**
	 * Build our pretty table of dimensions real vs actual.
	 *
	 * @param failed Matrix information for all matrices that failed dimension checks.
	 *
	 * @returns A string to use for displaying in an exception.
	 */
	string concoct_exception_message(list<MatrixDimInfo> failed) {
		auto strings = std::vector<string>{};

		for (const auto& dim_info : matrix_dimension_info) {
			const auto& name   = dim_info.matrix_name;
			const auto& matrix = *(dim_info.matrix);
			strings.push_back(name);
			strings.push_back(dim_info.declared_row_dimension + "x" +
			                  dim_info.declared_col_dimension);
			auto actualRows = num_rows(matrix);
			auto actualCols = num_cols(matrix);
			strings.push_back(std::to_string(actualRows) + "x" + std::to_string(actualCols));
		}


		auto headers = std::vector<string>{"Matrix", "Required", "Actual"};
		auto widths =
		    std::vector<std::size_t>{headers[0].length(), headers[1].length(), headers[2].length()};

		for (auto i = std::size_t{0}; i < strings.size(); ++i) {
			auto str = strings[i];
			if (str.length() > widths[i % widths.size()]) {
				widths[i % widths.size()] = str.length();
			}
		}

		auto build_string = std::stringstream{};

		build_string << "Invalid matrix dimensions." << std::endl << std::endl;

		for (auto i = size_t{0}; i < headers.size(); ++i) {
			build_string << write_column(widths[i], headers[i], ' ', ' ');
		}

		build_string << std::endl;

		for (auto width : widths) {
			build_string << write_column(width, "", '=', 0);
		}

		for (auto i = size_t{0}; i < strings.size(); ++i) {
			auto windex = i % widths.size();
			if (windex == 0) build_string << std::endl;
			build_string << write_column(widths[windex], strings[i], (windex == 0) ? 0 : ' ');
		}

		build_string << std::endl;

		for (const auto& dim_info : failed) {
			auto name   = dim_info.matrix_name;
			auto matrix = *(dim_info.matrix);
			auto rows   = dim_info.declared_row_dimension;
			auto cols   = dim_info.declared_col_dimension;

			if (is_number(rows) && is_number(cols)) {
				auto expected = concoct_fixed_description(rows, cols);
				auto actual   = concoct_fixed_description(std::to_string(num_rows(matrix)),
                                                        std::to_string(num_cols(matrix)));
				build_string << std::endl << name << " must be " << expected << ", got " << actual;
			}

			if (is_number(rows)) {
				build_string << std::endl
				             << name << " must have exactly " << rows << " rows (has "
				             << num_rows(matrix) << ")";
			} else {
				auto comparisons = concoct_comparison(name, rows);
				for (auto comparison : comparisons) {
					build_string << std::endl
					             << name << " must have the same number of rows " << comparison;
				}
			}

			if (is_number(cols)) {
				build_string << std::endl
				             << name << " must have exactly " << cols << " columns (has "
				             << num_cols(matrix) << ")";
			} else {
				auto comparisons = concoct_comparison(name, cols);
				for (auto comparison : comparisons) {
					build_string << std::endl
					             << name << " must have the same number of columns " << comparison;
				}
			}
		}

		return build_string.str();
	}
};

DimensionValidator::DimensionValidator()
    : implementation{std::make_unique<DimensionValidatorPrivate>()} {}

DimensionValidator::~DimensionValidator() {}

void DimensionValidator::perform_validation(ErrorMode mode, ValidationResult& result_out) {
	auto failed = list<MatrixDimInfo>{};

	for (const auto& dim_info : implementation->matrix_dimension_info) {
		auto expectedRows = dim_info.possible_row_dimensions;
		auto expectedCols = dim_info.possible_col_dimensions;
		auto actualRows   = num_rows(*(dim_info.matrix));
		auto actualCols   = num_cols(*(dim_info.matrix));

		if (expectedRows.count(actualRows) > 0 && expectedCols.count(actualCols) > 0) {
			// Remove all items in the set except the actual dimensions
			expectedRows.clear();
			expectedRows.insert(actualRows);
			expectedCols.clear();
			expectedCols.insert(actualCols);
		} else {
			failed.push_back(dim_info);
		}
	}

	if (failed.size() > 0) {
		result_out = ValidationResult::BAD;
		log_or_throw<std::range_error>(
		    mode, "{}", implementation->concoct_exception_message(failed));
	} else
		result_out = ValidationResult::GOOD;
}

void DimensionValidator::dim(const string& name, const Matrix& matrix, Size rows, Size cols) {
	implementation->dim(name, matrix, rows, cols);
}


void DimensionValidator::dim(const string& name, const Matrix& matrix, Size rows, char cols) {
	implementation->dim(name, matrix, rows, cols);
}


void DimensionValidator::dim(const string& name, const Matrix& matrix, char rows, Size cols) {
	implementation->dim(name, matrix, rows, cols);
}


void DimensionValidator::dim(const string& name, const Matrix& matrix, char rows, char cols) {
	implementation->dim(name, matrix, rows, cols);
}

}  // namespace utils
}  // namespace navtk
