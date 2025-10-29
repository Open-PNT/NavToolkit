#include <navtk/magnetic/MagnetometerCalibrationCaruso2d.hpp>

#include <algorithm>

#include <xtensor/core/xmath.hpp>
#include <xtensor/generators/xbuilder.hpp>
#include <xtensor/views/xview.hpp>

#include <navtk/factory.hpp>
#include <navtk/inspect.hpp>

namespace navtk {
namespace magnetic {

using navtk::num_rows;
using navtk::zeros;
using xt::amax;
using xt::amin;
using xt::eye;
using xt::row;

void MagnetometerCalibrationCaruso2d::generate_calibration(const Matrix& mag) {
	double x_max = amax(row(mag, 0))(0);
	double x_min = amin(row(mag, 0))(0);
	double y_max = amax(row(mag, 1))(0);
	double y_min = amin(row(mag, 1))(0);

	double x_diff = x_max - x_min;
	double y_diff = y_max - y_min;

	// Eqn 2.9
	double sfx = std::max(1.0, y_diff / x_diff);
	double sfy = std::max(1.0, x_diff / y_diff);
	// Eqn 2.10
	double bx = x_diff / 2.0 - x_max;
	double by = y_diff / 2.0 - y_max;

	size_t dims        = num_rows(mag);
	scale_factor       = eye(dims);
	scale_factor(0, 0) = sfx;
	scale_factor(1, 1) = sfy;

	bias    = zeros(dims);
	bias(0) = bx;
	bias(1) = by;

	calibrated = true;
}

}  // namespace magnetic
}  // namespace navtk
