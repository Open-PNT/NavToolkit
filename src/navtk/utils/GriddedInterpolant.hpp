#pragma once

#include <navtk/tensors.hpp>

namespace navtk {
namespace utils {

/**
 * Class which performs bivariate cubic interpolation on a 2D grid of equally spaced data
 */
class GriddedInterpolant {
private:
	/** Input points for a three-point derivative */
	struct Denominator {
		int idx;  //!< index
		int p2;   //!< plus2
		int m1;   //!< minus1
	};

public:
	/**
	 * Constructor
	 *
	 * @param x_vector 1 x N Vector representing x-axis values, where N > 2.
	 * @param y_vector 1 x M Vector representing y-axis values, where M > 2.
	 * @param q_mat N x M grid of map values at grid points (x,y)
	 *
	 * @throw std::invalid_argument if \p x_vector or \p y_vector have less than three elements,
	 * or if \p q is not NxM and the error mode is ErrorMode::DIE for either case.
	 */
	GriddedInterpolant(Vector x_vector, Vector y_vector, Matrix q_mat);

	/**
	 * Function which provides the interpolated value on the map q at the point (
	 * \p x , \p y ) using a bivariate cubic interpolation method.
	 *
	 * @param x X query point for interpolated value on map q
	 * @param y Y query point for interpolated value on map q
	 *
	 * @return Interpolated value at ( \p x , \p y ).
	 * @throw std::invalid_argument if \p x or \p y are outside map q and the error mode is
	 * ErrorMode::DIE.
	 */
	double interpolate(double x, double y);

private:
	GriddedInterpolant::Denominator get_3_point_denominators(int idx, const Vector& vec) const;

	Matrix big_f(const Denominator& x, const Denominator& y);

private:
	Vector x_vec;            //!< x-axis values
	Vector y_vec;            //!< y-axis values
	Matrix q;                //!< Grid of map values at grid points (x,y)
	std::size_t num_x_elem;  //!< Number of distinct values for x-axis
	std::size_t num_y_elem;  //!< Number of distinct values for y-axis
	double x_spacing;        //!< Step value between distinct values of x-axis
	double y_spacing;        //!< Step value between distinct values of y-axis
	double x_width;          //!< Range of x-axis values
	double y_width;          //!< Range of y-axis values
	double x_max;            //!< Maximum value of x-axis
	double x_min;            //!< Minimum value of x-axis
	double y_max;            //!< Maximum value of y-axis
	double y_min;            //!< Minimum value of y-axis

	const Matrix A = Matrix{{1, 0, 0, 0}, {0, 0, 1, 0}, {-3, 3, -2, -1}, {2, -2, 1, 1}};
};
}  // namespace utils
}  // namespace navtk
