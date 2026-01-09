#pragma once

#include <memory>
#include <typeinfo>

#include <navtk/tensors.hpp>
#include <navtk/utils/InterpolationModel.hpp>
#include <navtk/utils/Ordered.hpp>

namespace navtk {
namespace utils {

/**
 * Cubic spline model, heavily inspired by scipy's implementation of the 'natural' cubic spline
 * with no extrapolation.
 *
 * https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html#scipy.interpolate.CubicSpline
 */
class CubicSplineModel : public InterpolationModel {

public:
	/**
	 * Constructor.
	 *
	 * @param x Independent sample points, size N.
	 * @param y Dependent sampled values at each `x` element, size N. Also 0th order terms of
	 * splines polynomial coefficients.
	 */
	CubicSplineModel(const std::vector<double> &x, const std::vector<double> &y);

	double y_at(double x_interp) override;

private:
	/*
	 * Solution of the system of equations that describes the cubic spline model; also happens to be
	 * the set of first-order terms of the polynomial coefficients, size N.
	 */
	Vector der;

	/*
	 * Second order terms of the splines polynomial coefficients, size N.
	 */
	Vector c;

	/*
	 * 3rd order terms of the splines polynomial coefficients, size N.
	 */
	Vector d;
};

}  // namespace utils
}  // namespace navtk
