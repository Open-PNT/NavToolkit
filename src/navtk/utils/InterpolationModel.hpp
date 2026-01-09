#pragma once

#include <memory>
#include <typeinfo>

#include <navtk/tensors.hpp>
#include <navtk/utils/Ordered.hpp>

namespace navtk {
namespace utils {

/**
 * Base class that implements the details of some interpolation scheme.
 */
class InterpolationModel {
public:
	/**
	 * Destructor.
	 */
	virtual ~InterpolationModel() = default;

	/**
	 * Deleted.
	 */
	InterpolationModel() = delete;

	/**
	 * Deleted.
	 */
	InterpolationModel(const InterpolationModel &) = delete;

	/**
	 * Deleted.
	 */
	InterpolationModel &operator=(const InterpolationModel &) = delete;

	/**
	 * Deleted.
	 */
	InterpolationModel(InterpolationModel &&) = delete;

	/**
	 * Deleted.
	 */
	InterpolationModel &operator=(InterpolationModel &&) = delete;

	/**
	 * Get the interpolated data point at query value.
	 *
	 * @param x_interp Query point to generate interpolated data at.
	 *
	 * @return Interpolated data at `x_interp`.
	 */
	virtual double y_at(double x_interp) = 0;

protected:
	/**
	 * Protected constructor.
	 *
	 * @param x Independent sample points, size N.
	 * @param y Dependent sampled values at each `x` element, size N.
	 */
	InterpolationModel(const std::vector<double> &x, const std::vector<double> &y) : x(x), y(y){};

	/**
	 * Base data times/x values, size N.
	 */
	std::vector<double> x;

	/**
	 * Base data, size N.
	 */
	std::vector<double> y;

	/**
	 * Used to search over `x` to find points to use for interpolation.
	 */
	NearestNeighbors<std::vector<double>::const_iterator, std::less<double>> nn;
};

}  // namespace utils
}  // namespace navtk
