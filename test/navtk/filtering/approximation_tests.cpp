#include <gtest/gtest.h>
#include <tensor_assert.hpp>
#include <xtensor/io/xio.hpp>

#include <navtk/filtering/utils.hpp>

using namespace navtk::filtering;
using namespace navtk;
using std::cos;
using std::pow;
using std::sin;

struct ApproximationTests : public ::testing::Test {
public:
	Vector x;
	Matrix cov;
	EstimateWithCovariance ec;
	ApproximationTests()
	    : ::testing::Test(), x({3.0, -2.0}), cov({{0.01, 0.2}, {0.2, 9.0}}), ec({x, cov}) {}

	void accuracy_test(std::function<Vector(const Vector&)> fx,
	                   EstimateWithCovariance ec,
	                   EstimateWithCovariance expected,
	                   double thresh,
	                   std::function<Matrix(const Vector&)> jx,
	                   std::function<std::vector<Matrix>(const Vector&)> hx) {
		auto comp_res_n = compare_approx(fx, ec, expected, thresh);
		auto comp_res_a = compare_approx(fx, ec, expected, thresh, jx, hx);

		ASSERT_TRUE(comp_res_n.res2_xok && comp_res_n.res2_pok);
		ASSERT_TRUE(comp_res_a.res2_xok && comp_res_a.res2_pok);
	}

	/**
	 * Container that stores and compares the validity of various numerical approximations
	 * of mapping functions.
	 */
	struct ApproxResults {

		/**
		 * Max allowable relative error when compared to expected value to
		 * allow a pass, as a decimal (0.01 == 1%).
		 *
		 * Note that the same rtol is applied to states and covariance
		 * matrices.
		 *
		 * TODO: More accurate/fleshed out testing paramaters.
		 */
		double rtol = 0;

		/**
		 * Expected mean and covariance of mapped values.
		 */
		EstimateWithCovariance res_ex;

		/**
		 * Mean and covariance of values resulting from 1st order approximation.
		 */
		EstimateWithCovariance res1;

		/**
		 * Mean and covariance of values resulting from 2nd order approximation.
		 */
		EstimateWithCovariance res2;

		/**
		 * Indicates if first order mapping of x is within acceptable threshold.
		 */
		bool res1_xok = false;

		/**
		 * Indicates if first order mapping of p is within acceptable threshold.
		 */
		bool res1_pok = false;

		/**
		 * Indicates if second order mapping of x is within acceptable threshold.
		 */
		bool res2_xok = false;

		/**
		 * Indicates if second order mapping of p is within acceptable threshold.
		 */
		bool res2_pok = false;

		/**
		 * @param res_ex Expected mean and covariance of mapped values.
		 * @param res1 Mean and covariance of values resulting from 1st order approximation.
		 * @param res2 Mean and covariance of values resulting from 2nd order approximation.
		 * @param per_acc Acceptable relative error percentage when compared to Monte Carlo for a
		 * valid approximation.
		 */
		ApproxResults(EstimateWithCovariance res_ex,
		              EstimateWithCovariance res1,
		              EstimateWithCovariance res2,
		              double per_acc = 1.0)
		    : rtol(per_acc / 100.0), res_ex(res_ex), res1(res1), res2(res2) {
			res1_xok = xt::allclose(res_ex.estimate, res1.estimate, rtol, 0.0);
			res1_pok = xt::allclose(res_ex.covariance, res1.covariance, rtol, 0.0);
			res2_xok = xt::allclose(res_ex.estimate, res2.estimate, rtol, 0.0);
			res2_pok = xt::allclose(res_ex.covariance, res2.covariance, rtol, 0.0);
		}
	};

	/**
	 * Evaluates numerical approximations of f(x) and enables comparison with the results of a Monte
	 * Carlo generated mapping.
	 *
	 * @param fx A function that takes an N length observation and maps it to
	 * a M length result.
	 * @param ec First and second moments of N state starting distribution.
	 * @param per_acc_thresh Acceptable relative error level when compared to Monte Carlo solution
	 * for a valid approximation.
	 * @param num_mc Number of Monte Carlo iterations to use when generating solution.
	 * @param jx Function that accepts x and returns an MxN Jacobian valid at x.
	 * When null, defaults to the calc_numerical_jacobian function using its default values.
	 * @param hx Function that accepts x and returns an N length vector of
	 * MxN Hessian matrices valid at x. When null, defaults to the getNumericalHessian
	 * function using its default values.
	 */
	ApproxResults compare_approx(std::function<Vector(const Vector&)> fx,
	                             const EstimateWithCovariance& ec,
	                             double per_acc_thresh                                = 1.0,
	                             Size num_mc                                          = 100000,
	                             std::function<Matrix(const Vector&)> jx              = 0,
	                             std::function<std::vector<Matrix>(const Vector&)> hx = 0) {
		return ApproxResults(monte_carlo_approx(ec, fx, num_mc),
		                     first_order_approx(ec, fx, jx),
		                     second_order_approx(ec, fx, jx, hx),
		                     per_acc_thresh);
	}

	/**
	 * Evaluates numerical approximations of f(x) and enables comparison with a user provided
	 * expected result.
	 *
	 * @param fx A function that takes an N length observation and maps it to
	 * a M length result.
	 * @param ec First and second moments of N state starting distribution.
	 * @param expected Mean and covariance of expected mapped solution.
	 * @param per_acc_thresh Acceptable relative error level when compared to Monte Carlo solution
	 * for a valid approximation.
	 * @param jx Function that accepts x and returns an MxN Jacobian valid at x.
	 * When null, defaults to the calc_numerical_jacobian function using its default values.
	 * @param hx Function that accepts x and returns an N length vector of
	 * MxN Hessian matrices valid at x. When null, defaults to the getNumericalHessian
	 * function using its default values.
	 */
	ApproxResults compare_approx(std::function<Vector(const Vector&)> fx,
	                             const EstimateWithCovariance& ec,
	                             EstimateWithCovariance expected,
	                             double per_acc_thresh                                = 1.0,
	                             std::function<Matrix(const Vector&)> jx              = 0,
	                             std::function<std::vector<Matrix>(const Vector&)> hx = 0) {
		return ApproxResults(expected,
		                     first_order_approx(ec, fx, jx),
		                     second_order_approx(ec, fx, jx, hx),
		                     per_acc_thresh);
	}
};

// Tests disabled by default; since they rely on randomness and the test
// thresholds are somewhat arbitrary (they should be based on the number of MC
// runs executed) there is a small chance of failure regardless.
// @todo Re-enable once PNTOS-62 gives us seeded randomness.
TEST_F(ApproximationTests, DISABLED_simpleMap_SLOW) {
	Matrix cov{{4.0, 0.0}, {0.0, 9.0}};
	Vector nom{2.0, 2.0};
	auto expX = x + nom;
	EstimateWithCovariance ec_local{x, cov};
	auto adds = [nom = nom](Vector x) { return nom + x; };

	auto mc = monte_carlo_approx(ec_local, adds, 10000);
	// Since some values are expected to be zero, they can never meet a rtol
	// threshold of anything less than 1 (since abs(a-b) == b when a == 0)
	// so we must use an atol in this case, which is less desirable
	// as the units on each state may differ, so not one atol fits all.
	// Basically, MC tests are of limited utility unless we get way more
	// complicated.
	ASSERT_ALLCLOSE_EX(expX, mc.estimate, 0.1, 0.1);
	ASSERT_ALLCLOSE_EX(cov, mc.covariance, 0.1, 0.2);
}

TEST_F(ApproximationTests, DISABLED_simpleCross_SLOW) {
	Matrix cov{{4.0, 0.2}, {0.2, 9.0}};
	Vector nom{2.0, 2.0};
	EstimateWithCovariance ec_local{x, cov};
	auto expX = x + nom;

	auto adds = [nom = nom](Vector x) { return nom + x; };

	auto mc = monte_carlo_approx(ec_local, adds, 10000);
	ASSERT_ALLCLOSE_EX(expX, mc.estimate, 0.1, 0.1);
	ASSERT_ALLCLOSE_EX(cov, mc.covariance, 0.1, 0.2);
}

TEST_F(ApproximationTests, zeroCov) {
	Matrix cov{{0.0, 0.0}, {0.0, 0.0}};
	Vector nom{2.0, 2.0};
	EstimateWithCovariance ec_local{x, cov};
	auto expX = x + nom;

	auto adds = [nom = nom](const Vector& x) { return Vector{nom + x}; };

	// 0 cov means no noise, so each sample will be identical and
	// exact
	auto mc = monte_carlo_approx(ec_local, adds, 2);
	ASSERT_ALLCLOSE(expX, mc.estimate);
	ASSERT_ALLCLOSE(cov, mc.covariance);
}

TEST_F(ApproximationTests, fullNonLinearCompare) {

	auto fx = [](Vector x) {
		double x0 = x[0];
		double x1 = x[1];
		return Vector{pow(x0, 3.0), sin(x0) * x1};
	};

	auto jx = [](Vector x) {
		double x0 = x[0];
		double x1 = x[1];
		return Matrix{{3.0 * pow(x0, 2.0), 0.0}, {cos(x0) * x1, sin(x0)}};
	};

	auto hx = [](Vector x) {
		double x0 = x[0];
		double x1 = x[1];
		Matrix h0{{6.0 * x0, 0.0}, {0.0, 0.0}};
		Matrix h1{{-sin(x0) * x1, cos(x0)}, {cos(x0), 0.0}};
		return std::vector<Matrix>{h0, h1};
	};

	// 100000 run mc results; max linearization error is around 42%
	EstimateWithCovariance expected{Vector{27.105393, -0.477044},
	                                Matrix{{7.314657, 1.250589}, {1.250589, 0.450207}}};
	accuracy_test(fx, ec, expected, 42, jx, hx);
}

TEST_F(ApproximationTests, mildNonLinearCompare) {

	auto fx = [](Vector x) {
		double x0 = x[0];
		double x1 = x[1];
		return Vector{pow(x0, 3.0), x0 * x1};
	};

	auto jx = [](Vector x) {
		double x0 = x[0];
		double x1 = x[1];
		return Matrix{{3.0 * pow(x0, 2.0), 0.0}, {x1, x0}};
	};

	auto hx = [](Vector x) {
		Matrix h0{{6.0 * x[0], 0.0}, {0.0, 0.0}};
		Matrix h1{{0.0, 1.0}, {1.0, 0.0}};
		return std::vector<Matrix>{h0, h1};
	};

	// 100000 run mc results; max linearization error is around 3%
	EstimateWithCovariance expected{Vector{27.101005, -5.762152},
	                                Matrix{{7.324305, 15.717858}, {15.717858, 78.803933}}};

	accuracy_test(fx, ec, expected, 3, jx, hx);
}


TEST_F(ApproximationTests, linearCompare) {

	auto fx = [](Vector x) {
		double x0 = x[0];
		double x1 = x[1];
		return Vector{3.0 * x0, 2.0 * x1};
	};

	auto jx = [](Vector) { return Matrix{{3.0, 0.0}, {0.0, 2.0}}; };

	auto hx = [](Vector) {
		Matrix h{{0.0, 0.0}, {0.0, 0.0}};
		return std::vector<Matrix>{h, h};
	};

	// hand calc
	EstimateWithCovariance expected{Vector{9.0, -4.0}, Matrix{{0.09, 1.2}, {1.2, 36.0}}};

	accuracy_test(fx, ec, expected, 0.001, jx, hx);
}


TEST_F(ApproximationTests, noopCompare) {
	auto fx = [](Vector x) {
		double x0 = x[0];
		double x1 = x[1];
		return Vector{x0, x1};
	};

	auto jx = [](Vector) { return Matrix{{1.0, 0.0}, {0.0, 1.0}}; };

	auto hx = [](Vector) {
		Matrix h{{0.0, 0.0}, {0.0, 0.0}};
		return std::vector<Matrix>{h, h};
	};

	EstimateWithCovariance expected{x, cov};
	accuracy_test(fx, ec, expected, 0.001, jx, hx);
}
