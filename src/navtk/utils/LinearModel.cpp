#include <navtk/utils/LinearModel.hpp>

#include <navtk/tensors.hpp>
#include <navtk/utils/interpolation.hpp>

namespace navtk {
namespace utils {

using std::vector;

LinearModel::LinearModel(const vector<double> &x, const vector<double> &y)
    : InterpolationModel(x, y) {}

double LinearModel::y_at(double x_interp) {
	auto ind = std::min(
	    static_cast<Size>(nn.get(x.cbegin(), x.cend(), x_interp).first - x.cbegin()), y.size() - 2);
	return linear_interpolate(x[ind], y[ind], x[ind + 1], y[ind + 1], x_interp);
}

}  // namespace utils
}  // namespace navtk
