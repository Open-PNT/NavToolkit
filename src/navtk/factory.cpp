#include <navtk/factory.hpp>

#include <navtk/errors.hpp>

namespace navtk {

Matrix eye(Size rows, Size cols, int diagonal_index) {
	if (rows == cols && diagonal_index == 0) {
		return eye(rows);
	}
	if (cols != 0 && diagonal_index >= int(cols)) {
		log_or_throw<std::invalid_argument>(
		    "Exception Occurred: Diagonal index specified for eye() is greater than the column "
		    "indexes available.\n");
	} else if (rows != 0 && diagonal_index <= -int(rows)) {
		log_or_throw<std::invalid_argument>(
		    "Exception Occurred: Diagonal index specified for eye() is less than the row indexes "
		    "available.\n");
	}
	return xt::eye<Scalar>({rows, cols}, diagonal_index);
}
Matrix eye(Size size) {
	// xt::eye is pretty slow (see https://github.com/xtensor-stack/xtensor/issues/1022)
	// In testing multiple alternatives this naive method was the fastest
	Matrix out = xt::zeros<Scalar>({size, size});
	for (Size i = 0; i < size; ++i) {
		out(i, i) = 1.0;
	}
	return out;
}


Matrix block_diag(std::initializer_list<Matrix> matrices) {
	Size rows(0), cols(0), row(0), col(0);
	for (const auto& m : matrices) {
		rows += num_rows(m);
		cols += num_cols(m);
	}
	Matrix out = zeros(rows, cols);
	for (const auto& m : matrices) {
		// Read the size of the current matrix
		rows = num_rows(m);
		cols = num_cols(m);
		if (!rows || !cols) continue;

		// Copy the current matrix into the output matrix at the cursor position
		xt::view(out, xt::range(row, row + rows), xt::range(col, col + cols)) = m;

		// Move the cursor position to the bottom-right of the current matrix
		row += rows;
		col += cols;
	}
	return out;
}

}  // namespace navtk
