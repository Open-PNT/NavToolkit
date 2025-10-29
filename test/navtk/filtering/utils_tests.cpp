#include <gtest/gtest.h>
#include <tensor_assert.hpp>
#include <xtensor/generators/xrandom.hpp>
#include <xtensor/io/xio.hpp>

#include <navtk/experimental/random.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/tensors.hpp>

using namespace navtk::filtering;
using namespace navtk;
using navtk::experimental::rand_n;
using std::cos;
using std::pow;
using std::sin;
using std::sqrt;

static const Vector X{12.3, -32.1, 0.083};

TEST(calc_numerical_jacobian, simple) {
	Matrix expected{{2.0, 3.0, -4.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}};

	Matrix actual = calc_numerical_jacobian(
	    [](const Vector& v) { return Vector{2 * v[0] + 3 * v[1] - 4 * v[2], v[0], v[2]}; },
	    X,
	    Vector{1e-7, 1e-7, 1e-7});

	ASSERT_ALLCLOSE(actual, expected);
}

TEST(calc_numerical_jacobian, complicated) {
	Matrix expected{
	    {0.0, 2 * sin(X[1]) * cos(X[1]) * sqrt(X[2]), pow(sin(X[1]), 2.0) / (2.0 * sqrt(X[2]))},
	    {-2.0 / pow(X[0], 3.0), 0.0, 0.0},
	    {0.0, sin(pow(X[1], 2)) * 2 * X[1], 0.0}};
	auto arb_fun = [](const Vector& v) {
		return Vector{pow(sin(v[1]), 2.0) * sqrt(v[2]), 1.0 / pow(v[0], 2.0), -cos(pow(v[1], 2.0))};
	};
	Matrix actual = calc_numerical_jacobian(arb_fun, X, 1e-7);
	ASSERT_ALLCLOSE(actual, expected);
}

TEST(calc_numerical_jacobian, zeroEps) {
	auto arb_fun = [](const Vector& v) {
		return Vector{pow(sin(v[1]), 2.0) * sqrt(v[2]), 1.0 / pow(v[0], 2.0), -cos(pow(v[1], 2.0))};
	};
	ASSERT_ALLCLOSE(calc_numerical_jacobian(arb_fun, X, 0),
	                calc_numerical_jacobian(arb_fun, X, 0.001));
}

TEST(calc_numerical_jacobian, mByN) {
	auto arb_fun    = [](const Vector& v) { return Vector{2.0 * v[0], 3.0 * v[1]}; };
	Matrix expected = Matrix{{2.0, 0.0, 0.0}, {0.0, 3.0, 0.0}};
	ASSERT_ALLCLOSE(calc_numerical_jacobian(arb_fun, X), expected);
}

TEST(calc_numerical_jacobian, singleX) {
	auto arb_fun = [](const Vector& v) { return Vector{2.0 * v[0]}; };
	ASSERT_ALLCLOSE(calc_numerical_jacobian(arb_fun, Vector{1.0}), Matrix{{2.0}});
}

TEST(calc_numerical_hessians, simple) {
	double x0 = 0.5;
	double x1 = 0.4;
	double x2 = 1.3;
	Vector v{x0, x1, x2};
	Vector small{1e-4, 1e-4, 1e-4};

	auto arb_fun = [](const Vector& v) {
		return Vector{pow(v[0], 3.0) + sin(v[1]), pow(cos(v[0]) * cos(v[2]), 2.0)};
	};

	Matrix exp_jac{{3 * pow(x0, 2.0), cos(x1), 0.0},
	               {-2.0 * cos(x0) * pow(cos(x2), 2.0) * sin(x0),
	                0.0,
	                -2.0 * pow(cos(x0), 2.0) * cos(x2) * sin(x2)}};

	Matrix exp_h0{{6.0 * x0, 0.0, 0.0}, {0.0, -sin(x1), 0.0}, {0.0, 0.0, 0.0}};
	Matrix exp_h1{{-2.0 * pow(cos(x2), 2.0) * (-pow(sin(x0), 2.0) + pow(cos(x0), 2.0)),
	               0.0,
	               4.0 * cos(x0) * sin(x0) * cos(x2) * sin(x2)},
	              {0.0, 0.0, 0.0},
	              {
	                  4.0 * cos(x0) * sin(x0) * cos(x2) * sin(x2),
	                  0.0,
	                  -2.0 * pow(cos(x0), 2.0) * (-pow(sin(x2), 2.0) + pow(cos(x2), 2.0)),
	              }};

	ASSERT_ALLCLOSE(exp_jac, calc_numerical_jacobian(arb_fun, v, small));
	auto hess = calc_numerical_hessians(arb_fun, v, small);
	ASSERT_TRUE(hess.size() == 2);
	ASSERT_ALLCLOSE(exp_h0, hess[0]);
	ASSERT_ALLCLOSE(exp_h1, hess[1]);
}

TEST(stats, calc_mean_cov) {
	Matrix const TEST{{4.0, 3.0, 4.0, 6.0}, {2.0, 4.0, -6.0, 20.0}};
	Vector const exp_mean{17.0 / 4.0, 5.0};
	Matrix const exp_cov{{76.0 / 48.0, 31.0 / 3.0}, {31.0 / 3.0, 356.0 / 3.0}};
	auto calc = calc_mean_cov(TEST);
	ASSERT_ALLCLOSE(exp_mean, calc.estimate);
	ASSERT_ALLCLOSE(exp_cov, calc.covariance);
}
