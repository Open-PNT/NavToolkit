#include <navtk/utils/GriddedInterpolant.hpp>

#include <xtensor/core/xmath.hpp>

#include <navtk/errors.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/utils/ValidationContext.hpp>

namespace navtk {
namespace utils {

GriddedInterpolant::GriddedInterpolant(Vector x_vector, Vector y_vector, Matrix q_mat)
    : x_vec(std::move(x_vector)),
      y_vec(std::move(y_vector)),
      q(std::move(q_mat)),
      num_x_elem(x_vec.size()),
      num_y_elem(y_vec.size()),
      x_width(x_vec[num_x_elem - 1] - x_vec[0]),
      y_width(y_vec[num_y_elem - 1] - y_vec[0]),
      x_max(xt::amax(x_vec)[0]),
      x_min(xt::amin(x_vec)[0]),
      y_max(xt::amax(y_vec)[0]),
      y_min(xt::amin(y_vec)[0]) {
	if (ValidationContext validation{}) {
		if (num_x_elem <= 2)
			log_or_throw<std::invalid_argument>("x_vector must have at least three elements");
		if (num_y_elem <= 2)
			log_or_throw<std::invalid_argument>("y_vector must have at least three elements");
		validation.add_matrix(q, "q").dim(num_x_elem, num_y_elem).validate();
	}

	// check spacing of X and Y
	auto x_vec_diff = xt::view(x_vec, xt::range(1, num_x_elem - 1)) -
	                  xt::view(x_vec, xt::range(0, num_x_elem - 2));
	auto y_vec_diff = xt::view(y_vec, xt::range(1, num_y_elem - 1)) -
	                  xt::view(y_vec, xt::range(0, num_y_elem - 2));
	x_spacing = xt::mean(x_vec_diff)[0];
	y_spacing = xt::mean(y_vec_diff)[0];

	if (ValidationContext{}) {
		// check if monotonic and for equal spacing
		if (xt::amin(x_vec_diff)[0] <= 0 || xt::amin(y_vec_diff)[0] <= 0) {
			log_or_throw<std::invalid_argument>("Vectors are not monotonically increasing");
		}
		if (xt::amax(x_vec_diff)[0] - xt::amin(x_vec_diff)[0] > 1e-7) {
			log_or_throw<std::invalid_argument>("x_vector spacing not uniform");
		}
		if (xt::amax(y_vec_diff)[0] - xt::amin(y_vec_diff)[0] > 1e-7) {
			log_or_throw<std::invalid_argument>("y_vector spacing not uniform");
		}
	}
}

double GriddedInterpolant::interpolate(double x, double y) {
	if (x > x_max || x < x_min || y > y_max || y < y_min) {
		log_or_throw<std::invalid_argument>(
		    "Query point must be inside grid, extrapolation not supported");
	}

	// determine where x,y is on map
	auto x_idx = static_cast<int>(std::floor((num_x_elem - 1) * (x - x_vec[0]) / x_width));
	auto y_idx = static_cast<int>(std::floor((num_y_elem - 1) * (y - y_vec[0]) / y_width));

	// determine denominator values for 3 point derivative
	auto x_denominator = get_3_point_denominators(x_idx, x_vec);
	auto y_denominator = get_3_point_denominators(y_idx, y_vec);

	// calculate bigF matrix composed of map point and derivative values
	Matrix F = big_f(x_denominator, y_denominator);

	// calculate interpolated value
	Matrix coefs = dot(dot(A, F), xt::transpose(A));
	auto x_off   = (x - x_vec[x_denominator.idx]) / x_spacing;
	auto y_off   = (y - y_vec[y_denominator.idx]) / y_spacing;
	Matrix mat   = dot(dot(Matrix{{1, x_off, std::pow(x_off, 2), std::pow(x_off, 3)}}, coefs),
                     xt::transpose(Matrix{{1, y_off, std::pow(y_off, 2), std::pow(y_off, 3)}}));

	return mat(0, 0);
}

GriddedInterpolant::Denominator GriddedInterpolant::get_3_point_denominators(
    int idx, const Vector &vec) const {
	// Determine denominator values for 3 point derivative
	int index  = idx;
	int plus2  = 2;
	int minus1 = 1;

	if (index == 0) {
		// bottom side of map (on edge or inside 1st unit square)
		minus1 = 0;
	} else if (index == static_cast<int>((num_cols(vec) - 2))) {
		// inside bottom unit square (only use first order dy)
		plus2 = 1;
	} else if (index == static_cast<int>((num_cols(vec) - 1))) {
		// top edge of map; move index to bottom side of unit grid
		index += -1;
		plus2 = 1;
	}

	return {index, plus2, minus1};
}

Matrix GriddedInterpolant::big_f(const Denominator &x, const Denominator &y) {

	auto f_00 = q(x.idx, y.idx);
	auto f_10 = q(x.idx + 1, y.idx);
	auto f_01 = q(x.idx, y.idx + 1);
	auto f_11 = q(x.idx + 1, y.idx + 1);

	auto f_00_x = (q(x.idx + 1, y.idx) - q(x.idx - x.m1, y.idx)) / (x.m1 + 1);
	auto f_10_x = (q(x.idx + x.p2, y.idx) - q(x.idx, y.idx)) / x.p2;
	auto f_01_x = (q(x.idx + 1, y.idx + 1) - q(x.idx - x.m1, y.idx + 1)) / (x.m1 + 1);
	auto f_11_x = (q(x.idx + x.p2, y.idx + 1) - q(x.idx, y.idx + 1)) / x.p2;

	auto f_00_y = (q(x.idx, y.idx + 1) - q(x.idx, y.idx - y.m1)) / (y.m1 + 1);
	auto f_10_y = (q(x.idx, y.idx + y.p2) - q(x.idx, y.idx)) / y.p2;
	auto f_01_y = (q(x.idx + 1, y.idx + 1) - q(x.idx + 1, y.idx - y.m1)) / (y.m1 + 1);
	auto f_11_y = (q(x.idx + 1, y.idx + y.p2) - q(x.idx + 1, y.idx)) / y.p2;

	auto f_00_xy = (((q(x.idx + 1, y.idx + 1) - q(x.idx - x.m1, y.idx + 1)) / (x.m1 + 1)) -
	                ((q(x.idx + 1, y.idx - y.m1) - q(x.idx - x.m1, y.idx - y.m1)) / (x.m1 + 1))) /
	               (y.m1 + 1);
	auto f_10_xy = (((q(x.idx + x.p2, y.idx + 1) - q(x.idx, y.idx + 1)) / x.p2) -
	                ((q(x.idx + x.p2, y.idx - y.m1) - q(x.idx, y.idx - y.m1)) / x.p2)) /
	               (y.m1 + 1);
	auto f_01_xy = (((q(x.idx + 1, y.idx + y.p2) - q(x.idx - x.m1, y.idx + y.p2)) / (x.m1 + 1)) -
	                ((q(x.idx + 1, y.idx) - q(x.idx - x.m1, y.idx)) / (x.m1 + 1))) /
	               y.p2;
	auto f_11_xy = (((q(x.idx + x.p2, y.idx + y.p2) - q(x.idx, y.idx + y.p2)) / x.p2) -
	                ((q(x.idx + x.p2, y.idx) - q(x.idx, y.idx)) / x.p2)) /
	               y.p2;

	return Matrix{{f_00, f_01, f_00_y, f_01_y},
	              {f_10, f_11, f_10_y, f_11_y},
	              {f_00_x, f_01_x, f_00_xy, f_01_xy},
	              {f_10_x, f_11_x, f_10_xy, f_11_xy}};
}

}  // namespace utils
}  // namespace navtk
