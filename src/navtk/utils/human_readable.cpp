#include <navtk/utils/human_readable.hpp>

#include <string>

#include <xtensor/core/xmath.hpp>
#include <xtensor/generators/xrandom.hpp>
#include <xtensor/io/xio.hpp>
#include <xtensor/views/xview.hpp>

namespace navtk {
namespace utils {

std::string repr(const Matrix& matrix, const std::string& decl) {
	std::ostringstream os;
	os << decl << " ";
	os << matrix;
	std::string out = os.str();
	// Align columns of output
	std::string indent(decl.size() + 1, ' ');
	size_t pos = 0;
	while ((pos = out.find("\n", pos)) != std::string::npos) {
		out.replace(++pos, 0, indent);
		pos += indent.size();
	}
	return out;
}


std::string repr(const Matrix& matrix) {
	auto shape = matrix.shape();
	if (shape[0] != 1 && shape[1] == 1) return repr(matrix, "Vector");
	if (shape[0] == 1 && shape[1] != 1) return repr(matrix, "Vector");
	return repr(matrix, "Matrix");
}

namespace {

/**
 * The maximum number of mismatched coefficients for diff() to attempt to
 * assign. If there are more than this many mismatches, diff will give up
 * and just report how dissimilar matrices are. If there are this many or
 * less, diff will write out assignment statements for each mismatched
 * coefficient.
 */
const Size MAX_DIFF_MISMATCH = 5;

/**
 * Writes the given array to the given ostream as comma-separated data.
 */
template <class T, std::size_t N>
void write_shape(std::ostream& out, const std::array<T, N>& shape) {
	bool need_comma = false;
	for (const auto& dim : shape) {
		if (need_comma) out << ", ";
		out << dim;
		need_comma = true;
	}
}

/**
 * Writes a human-readable description of the shape of the given matrix,
 * including the total number of coefficients.
 *
 * @param out Destination stream
 * @param matrix_name Name to use to refer to the matrix in the output
 * @param matrix Matrix with the shape to write.
 */
void write_named_shape(std::ostream& out, const std::string& matrix_name, const Matrix& matrix) {
	bool parenthesize_name = false;
	for (const auto& c : matrix_name)
		if (!std::isalnum(c)) {
			parenthesize_name = true;
			break;
		}
	out << "\n//\t";
	if (parenthesize_name) out << "(";
	out << matrix_name;
	if (parenthesize_name) out << ")";
	out << ".shape() == {";
	write_shape(out, matrix.shape());
	out << "} /* size: " << matrix.size() << " */";
}

/**
 * Writes a human-readable explanation of transposed matrices.
 */
void write_transpose(std::ostream& out,
                     const std::string& before_name,
                     const std::string& after_name,
                     const Matrix& before,
                     const Matrix& after) {
	out << "//Transposed:";
	write_named_shape(out, before_name, before);
	write_named_shape(out, after_name, after);
	out << "\n" << after_name << " = xt::transpose(" << before_name << ")";
}

void diff(std::ostream& out,
          const std::string& before_name,
          const std::string& after_name,
          const Matrix& before,
          const Matrix& after,
          double rtol,
          double atol,
          bool inputs_transformed = false) {
	auto b_shape = before.shape();
	auto a_shape = after.shape();

	// Detect the case where before and after are both 2d matrices, but
	// num_rows(before) == num_cols(after) and vice versa. Assume the matrices
	// were transposed, then try to compute the difference between a transposed
	// version of `before` and `after`.
	//
	// This could cause infinite recursion without the a_shape[1] != a_shape[0]
	// check, which blocks square matrices. They're handled separately below.
	if (b_shape.size() == 2 && a_shape.size() == 2 && a_shape[1] != a_shape[0] &&
	    a_shape[0] == b_shape[1] && a_shape[1] == b_shape[0]) {
		// Differentiate between "possibly transposed" and just "transposed."
		if (xt::allclose(before, xt::transpose(after), rtol, atol)) {
			write_transpose(out, before_name, after_name, before, after);
			return;
		}
		out << "//Possibly transposed:";
		write_named_shape(out, before_name, before);
		write_named_shape(out, after_name, after);
		out << "\n";
		diff(out,
		     "xt::transpose(" + before_name + ")",
		     after_name,
		     xt::transpose(before),
		     after,
		     rtol,
		     atol,
		     true);
		return;
	}

	// If the shapes don't match but aren't a transpose of each other, give up.
	if (b_shape != a_shape) {
		out << "//Different shapes:";
		write_named_shape(out, before_name, before);
		write_named_shape(out, after_name, after);
		return;
	}

	// Create a matrix of bool, where each coefficient is true only if the
	// corresponding coefficient in before and after are "close" with respect
	// to the rtol and atol.
	auto match = xt::isclose(before, after, rtol, atol);

	// Count "mismatches" -- coefficients in before/after that failed the
	// isclose check.
	auto size       = std::min({before.size(), after.size(), match.size()});
	auto mismatches = size;
	for (const auto isMatch : match)
		if (isMatch) --mismatches;

	// Check for the case where two square matrices are the transpose of each
	// other.
	if (mismatches && a_shape.size() == 2 && a_shape[0] == a_shape[1] &&
	    xt::allclose(before, xt::transpose(after), rtol, atol)) {
		write_transpose(out, before_name, after_name, before, after);
		return;
	}

	// If there weren't any matches between before and after, or the number
	// of mismatches is larger than MAX_DIFF_MISMATCH, give up.
	if ((mismatches && mismatches == size) || mismatches > MAX_DIFF_MISMATCH) {
		out << "//" << after_name << " and " << before_name << " have ";
		if (mismatches == size)
			out << "no coefficients";
		else
			out << (size - mismatches) << " coefficients (of " << size << ")";
		out << " in common.";
		return;
	}

	// If this isn't a recursive call, and everything has matched, then
	// before ~= after, and we don't want to write anything to the stream.
	if (!mismatches && !inputs_transformed) return;

	// Don't waste cycles looking for mismatches to print if there weren't any
	if (!mismatches) return;

	// Iterate through the three matrices before, after and match. For each
	// coefficient that didn't match, write a line showing the difference.
	auto m_shape = match.shape();
	for (decltype(size) idx = 0; idx < size; ++idx)
		if (!match[xt::unravel_index(idx, m_shape)]) {
			auto apos = xt::unravel_index(idx, a_shape);
			auto bpos = xt::unravel_index(idx, b_shape);
			out << "\n" << after_name << "(";
			write_shape(out, apos);
			out << ") = " << after[apos] << ";  // " << before_name << "(";
			write_shape(out, bpos);
			out << ") == " << before[bpos];
		}
}

}  // namespace

std::string diff(const std::string& before_name,
                 const std::string& after_name,
                 const Matrix& before,
                 const Matrix& after,
                 double rtol,
                 double atol) {
	std::ostringstream out;
	diff(out, before_name, after_name, before, after, rtol, atol);
	return out.str();
}

std::string diff(const Matrix& before, const Matrix& after, double rtol, double atol) {
	return diff("before", "after", before, after, rtol, atol);
}

}  // namespace utils
}  // namespace navtk
