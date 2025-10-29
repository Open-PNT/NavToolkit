#include <navtk/utils/CubicSplineModel.hpp>

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

CubicSplineModel::CubicSplineModel(const vector<double> &x, const vector<double> &y)
    : InterpolationModel(x, y) {
	auto n = y.size();

	auto dx         = xt::adapt(diff(x), {n - 1});
	auto dy         = xt::adapt(diff(y), {n - 1});
	auto m_diag     = zeros(n);
	auto lower_diag = zeros(n - 1);
	auto upper_diag = zeros(n - 1);
	auto yr         = zeros(n);
	auto slope      = dy / dx;

	auto not_front              = range(1, n - 1);
	auto not_back               = range(0, n - 2);
	view(m_diag, not_front)     = 2 * (view(dx, not_back) + view(dx, not_front));
	view(upper_diag, not_front) = view(dx, not_back);
	view(lower_diag, not_back)  = view(dx, not_front);
	view(yr, not_front)         = 3.0 * (view(dx, not_front) * view(slope, not_back) +
                                 view(dx, not_back) * view(slope, not_front));

	m_diag[0]     = 2 * dx[0];
	upper_diag[0] = dx[0];
	yr[0]         = 3 * dy[0];

	m_diag[n - 1]     = 2 * dx[n - 2];
	lower_diag[n - 2] = dx[n - 2];
	yr[n - 1]         = 3 * dy[n - 2];

	der = solve_tridiagonal_overwrite(lower_diag, m_diag, upper_diag, yr);

	auto t = (view(der, range(1, n)) + view(der, range(0, n - 1)) - 2 * slope) / dx;
	c      = (slope - view(der, range(0, n - 1))) / dx - t;
	d      = t / dx;
}

double CubicSplineModel::y_at(double x_interp) {
	auto ind = std::min(
	    static_cast<Size>(nn.get(x.cbegin(), x.cend(), x_interp).first - x.cbegin()), y.size() - 2);
	auto dt = x_interp - x[ind];
	return y[ind] + der[ind] * dt + c[ind] * dt * dt + d[ind] * dt * dt * dt;
}


}  // namespace utils
}  // namespace navtk
