#pragma once

#include <memory>
#include <typeinfo>

#include <navtk/tensors.hpp>
#include <navtk/utils/InterpolationModel.hpp>
#include <navtk/utils/Ordered.hpp>

namespace navtk {
namespace utils {

/**
 * Quadratic spline interpolation model.
 */
class QuadraticSplineModel : public InterpolationModel {

public:
	/**
	 * Constructor.
	 *
	 * @param x Independent sample points, size N.
	 * @param y Dependent sampled values at each `x` element, size N.
	 */
	QuadraticSplineModel(const std::vector<double> &x, const std::vector<double> &y);

	double y_at(double x_interp) override;

private:
	/*
	 * Set of 3 polynomial coefficients [a, b, c] (A * spline_model = y_meas) such that for a given
	 * query point x the interpolated value at x, y(x) = ax^2 + bx + c.
	 */
	Vector spline_model;
};

}  // namespace utils
}  // namespace navtk
