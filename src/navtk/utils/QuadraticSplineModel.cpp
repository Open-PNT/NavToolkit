#include <navtk/utils/QuadraticSplineModel.hpp>

#include <xtensor/containers/xadapt.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/interpolation.hpp>
#include <navtk/utils/sortable_vectors.hpp>

namespace navtk {
namespace utils {

using std::vector;
using xt::range;
using xt::view;

QuadraticSplineModel::QuadraticSplineModel(const vector<double> &x, const vector<double> &y)
    : InterpolationModel(x, y) {
	auto dy                      = xt::adapt(diff(y), {y.size() - 1});
	auto yr                      = zeros(y.size());
	view(yr, range(1, y.size())) = 2 * dy;

	auto l_diag  = ones(y.size() - 1);
	auto m_diag  = ones(y.size());
	auto u_diag  = zeros(y.size() - 1);
	u_diag[0]    = -1;
	spline_model = solve_tridiagonal_overwrite(l_diag, m_diag, u_diag, yr);
}

double QuadraticSplineModel::y_at(double x_interp) {
	auto ind = std::min(
	    static_cast<Size>(nn.get(x.cbegin(), x.cend(), x_interp).first - x.cbegin()), y.size() - 2);
	auto dt = (x_interp - x[ind]) / (x[ind + 1] - x[ind]);
	return y[ind] + spline_model[ind] * dt +
	       (spline_model[ind + 1] - spline_model[ind]) / 2.0 * dt * dt;
}


}  // namespace utils
}  // namespace navtk
