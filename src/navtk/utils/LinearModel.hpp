#pragma once

#include <memory>
#include <typeinfo>

#include <navtk/tensors.hpp>
#include <navtk/utils/InterpolationModel.hpp>
#include <navtk/utils/Ordered.hpp>

namespace navtk {
namespace utils {

/**
 * Linear interpolation model.
 */
class LinearModel : public InterpolationModel {

public:
	/**
	 * Constructor.
	 *
	 * @param x Independent sample points, size N.
	 * @param y Dependent sampled values at each `x` element, size N.
	 */
	LinearModel(const std::vector<double> &x, const std::vector<double> &y);

	double y_at(double x_interp) override;
};

}  // namespace utils
}  // namespace navtk
