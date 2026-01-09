#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>
#include <xtensor-blas/xlinalg.hpp>

#include <navtk/experimental/random.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

TEST(TensorsTests, dot) {
	navtk::Matrix mat   = navtk::Matrix{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
	navtk::Matrix vec   = navtk::Matrix{{4}, {5}, {7}};
	navtk::Matrix inner = navtk::dot(xt::transpose(vec), vec);

	EXPECT_ALLCLOSE((navtk::Matrix{{35}, {83}, {131}}), navtk::dot(mat, vec));
	EXPECT_ALLCLOSE((navtk::Matrix{{90}}), inner);
	EXPECT_ALLCLOSE((navtk::Matrix{{90}}), navtk::dot(xt::transpose(vec), vec));
	EXPECT_ALLCLOSE((navtk::Matrix{{16, 20, 28}, {20, 25, 35}, {28, 35, 49}}),
	                navtk::dot(vec, xt::transpose(vec)));
}

TEST(TensorsTests, A_dot_transposeB) {
	navtk::Matrix mat    = navtk::Matrix{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
	navtk::Matrix vec1x3 = navtk::Matrix{{4, 5, 7}};
	navtk::Matrix vec3x1 = navtk::Matrix{{4}, {5}, {7}};

	EXPECT_ALLCLOSE((navtk::dot(vec1x3, xt::transpose(vec1x3))),
	                navtk::transpose_a_dot_b(vec1x3, vec1x3));
	EXPECT_ALLCLOSE((navtk::dot(vec3x1, xt::transpose(vec3x1))),
	                navtk::transpose_a_dot_b(vec3x1, vec3x1));
	EXPECT_ALLCLOSE((navtk::dot(mat, xt::transpose(vec1x3))),
	                navtk::transpose_a_dot_b(mat, vec1x3));
	EXPECT_ALLCLOSE((navtk::dot(vec1x3, xt::transpose(mat))),
	                navtk::transpose_a_dot_b(vec1x3, mat));
	EXPECT_ALLCLOSE((navtk::dot(mat, xt::transpose(mat))), navtk::transpose_a_dot_b(mat, mat));
}

TEST(TensorsTests, TransposeA_dot_B) {
	navtk::Matrix mat    = navtk::Matrix{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
	navtk::Matrix vec1x3 = navtk::Matrix{{4, 5, 7}};
	navtk::Matrix vec3x1 = navtk::Matrix{{4}, {5}, {7}};

	EXPECT_ALLCLOSE((navtk::dot(xt::transpose(vec1x3), vec1x3)),
	                navtk::a_dot_transpose_b(vec1x3, vec1x3));
	EXPECT_ALLCLOSE((navtk::dot(xt::transpose(vec3x1), vec3x1)),
	                navtk::a_dot_transpose_b(vec3x1, vec3x1));
	EXPECT_ALLCLOSE((navtk::dot(xt::transpose(mat), vec3x1)),
	                navtk::a_dot_transpose_b(mat, vec3x1));
	EXPECT_ALLCLOSE((navtk::dot(xt::transpose(vec3x1), mat)),
	                navtk::a_dot_transpose_b(vec3x1, mat));
	EXPECT_ALLCLOSE((navtk::dot(xt::transpose(mat), mat)), navtk::a_dot_transpose_b(mat, mat));
}

TEST(TensorsTests, DotVector) {
	navtk::Vector a{2, 3, 4};
	navtk::Vector b{4, 5, 3};
	EXPECT_ALLCLOSE(navtk::dot(a, b), navtk::Vector{35});
}

TEST(TensorsTests, Inverse) {
	navtk::Matrix mat = navtk::eye(3, 3) * 5;

	EXPECT_ALLCLOSE(mat / 5 / 5, navtk::inverse(mat));
	EXPECT_ALLCLOSE(mat / 5 / 5, navtk::inverse(mat + 1 - 1));
}

TEST(TensorsTests, MatrixNFixedSizes) {
// This xtensor exception is due to an assertion that is enabled by XTENSOR_ENABLE_ASSERT.
#ifdef XTENSOR_ENABLE_ASSERT
	EXPECT_THROW({ navtk::Matrix3 mat3 = navtk::zeros(4, 3); }, std::runtime_error);
#endif

	navtk::Matrix3 mat3 = navtk::eye(3, 3);
	navtk::Matrix exp   = navtk::Matrix{{3, 1, 1}, {1, 3, 1}, {1, 1, 3}};
	ASSERT_ALLCLOSE(2 * mat3 + 1, exp);

	navtk::MatrixN<3, 3> matn = navtk::eye(3, 3);
	navtk::Matrix a           = 2 * matn + 1;
	navtk::Matrix b           = 2 * mat3 + 1;
	ASSERT_ALLCLOSE(a, b);
	ASSERT_ALLCLOSE((2 * matn + 1) * matn, a * matn);

	navtk::MatrixN<3, 1> col = navtk::Matrix{{1}, {2}, {3}};
	navtk::MatrixN<1, 3> row = navtk::Matrix{{1, 2, 3}};
	ASSERT_DOUBLE_EQ(navtk::dot(row, col)(0, 0), 14);

	navtk::Matrix outer = navtk::Matrix{{1, 2, 3}, {2, 4, 6}, {3, 6, 9}};
	ASSERT_ALLCLOSE(navtk::dot(col, row), outer);

	navtk::MatrixN<4, 1> col4 = navtk::zeros(4, 1);
	EXPECT_THROW({ navtk::Matrix out = col4 * navtk::Matrix(col); }, std::runtime_error);
}

struct VectorManip : public ::testing::Test {
	navtk::Matrix m4;
	navtk::Matrix m3;
	navtk::Matrix row4;
	navtk::Matrix row3;
	navtk::Vector4 expected4;
	navtk::Vector3 expected3;
	VectorManip()
	    : m4(navtk::eye(4)),
	      m3(navtk::eye(3)),
	      row4({{1.0, 2.0, 3.0, 4.0}}),
	      row3({{1.0, 2.0, 3.0}}),
	      expected4({1.0, 2.0, 3.0, 4.0}),
	      expected3({1.0, 2.0, 3.0}) {}
};

TEST(MatrixExp, WolframExample) {
	// Test the general 2x2 solution from the WolframAlpha MatrixExponential page
	double a = -1.2;
	double b = 3.4;
	double c = 1.1;
	double d = 0.3;
	navtk::Matrix m{{a, b}, {c, d}};
	double delta = std::sqrt(std::abs(std::pow(a - d, 2.0) + 4 * b * c));

	double ex  = std::exp((a + d) / 2.0);
	double sh  = std::sinh(0.5 * delta);
	double ch  = std::cosh(0.5 * delta);
	double m11 = ex * (delta * ch + (a - d) * sh);
	double m12 = 2.0 * b * ex * sh;
	double m21 = 2.0 * c * ex * sh;
	double m22 = ex * (delta * ch + (d - a) * sh);

	navtk::Matrix expected{{m11 / delta, m12 / delta}, {m21 / delta, m22 / delta}};
	ASSERT_ALLCLOSE(expected, navtk::expm(m));
}

TEST(MatrixExp, ScipyResults) {
	// Test against several known results, calculated using SciPy 1.3.1
	navtk::Matrix base_matrix{{-1.2, 3.4}, {1.1, 0.3}};
	const auto expected_results = std::vector<navtk::Matrix>{
	    {{3614872.312133219093084, 9281174.313892992213368},
	     {3002732.866259497124702, 7709508.038850714452565}},
	    {{1.674413083588018, 4.093346442570992}, {1.324317966714145, 3.480301220016396}},
	    {{0.911782141349008, 1.614261946479542}, {0.522261217978676, 1.623956529501747}},
	    {{0.904422155073079, 0.327374964626781}, {0.105915429732194, 1.048852286526071}},
	    {{0.977023421330418, 0.067410077332204}, {0.021809142666301, 1.006763161329919}},
	    {{0.997610347243890, 0.006793902240311}, {0.002198027195395, 1.000607657055792}}};
	const auto relative_tolerance = 1e-12;
	const auto absolute_tolerance = 0.0;
	const auto scales             = navtk::Vector{10, 1, 5e-1, 1e-1, 2e-2, 2e-3};
	for (navtk::Size ii = 0; ii < navtk::num_rows(scales); ii++) {
		navtk::Matrix test_matrix = base_matrix * scales[ii];
		ASSERT_ALLCLOSE_EX(
		    expected_results[ii], navtk::expm(test_matrix), relative_tolerance, absolute_tolerance);
	}
}

TEST(MatrixExp, BadMatrixSize) {
	navtk::Matrix test_matrix = navtk::ones(2, 3);
	EXPECT_UB_OR_DIE(navtk::expm(test_matrix), "dimension", std::range_error);
}

struct CholeskyTest : public ::testing::Test {
	navtk::Matrix data, xt_chol_returns_zeros_on_diagonal;
	navtk::Matrix invalid_data;
	navtk::Matrix null_data, non_zeros_on_diagonal;
	navtk::Matrix diagonal_data;
	navtk::Matrix expected_data;
	navtk::Matrix expected_diagonal_data;

	CholeskyTest()
	    : data({{4, 12, -16}, {12, 37, -43}, {-16, -43, 98}}),
	      xt_chol_returns_zeros_on_diagonal(
	          {{0.0499879405, 0.0106069669, 0.0999033202, 0.0112222837},
	           {0.0106069669, 0.049336541, 0.0212092073, 0.0964835372},
	           {0.0999033202, 0.0212092073, 0.0050021377, 0.0005001602},
	           {0.0112222837, 0.0964835372, 0.0005001602, 0.0050021377}}),
	      invalid_data({{-4, 12, -16}, {12, 37, -43}, {-16, -43, 98}}),
	      null_data({{}}),
	      non_zeros_on_diagonal({{0.22357983026203415, 0.0, 0.0, 0.0},
	                             {0.0, 0.2221183040633977, 0.0, 0.0},
	                             {0.0, 0.0, 0.07072579232500686, 0.0},
	                             {0.0, 0.0, 0.0, 0.07072579232500686}}),
	      diagonal_data({{4, 0, 0}, {0, 16, 0}, {0, 0, 9}}),
	      expected_data({{2, 0, 0}, {6, 1, 0}, {-8, 5, 3}}),
	      expected_diagonal_data({{2, 0, 0}, {0, 4, 0}, {0, 0, 3}}) {}
};

TEST_F(CholeskyTest, CholeskyExampleNonPositiveDefinite) {
	ASSERT_ALLCLOSE(non_zeros_on_diagonal, navtk::chol(xt_chol_returns_zeros_on_diagonal));
}

TEST_F(CholeskyTest, CholeskyExample) { ASSERT_ALLCLOSE(expected_data, navtk::chol(data)); }

TEST_F(CholeskyTest, CholeskyExampleDiagonal) {

	ASSERT_ALLCLOSE(expected_diagonal_data, navtk::chol(diagonal_data));
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, CholeskyTest, CholeskyExampleNULL) {
	EXPECT_HONORS_MODE(navtk::chol(test.null_data), "N > 0");
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, CholeskyTest, CholeskyExampleInvalidData) {
	EXPECT_HONORS_MODE(navtk::chol(test.invalid_data), "diagonal");
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, CholeskyTest, CholeskyExampleNonSquare) {
	EXPECT_HONORS_MODE(navtk::chol(navtk::ones(2, 4)), "dimension");
}


TEST(LinAlgTests, CovarianceCompareNumpy) {
	navtk::Matrix m{{5, 0, 3, 7}, {1, -5, 7, 3}, {4, 9, 8, 10}};
	auto C = navtk::calc_cov(m);
	navtk::Matrix exp{{8.9166666666666667, 8.166666666666667, -0.75},
	                  {8.166666666666667, 25., -0.5},
	                  {-0.75, -0.5, 6.916666666666667}};
	ASSERT_ALLCLOSE(exp, C);
}

TEST(LinAlgTests, CholeskyCompareNumpy) {
	navtk::Matrix m{{3.0, 0.15}, {0.15, 4.0}};
	navtk::Matrix exp{{1.73205081, 0.0}, {0.08660254, 1.99812412}};
	auto ch = navtk::chol(m);
	ASSERT_ALLCLOSE(exp, ch);
}

TEST(LinAlgTests, TraceTestEye) {
	navtk::Matrix m = navtk::eye(3);
	ASSERT_DOUBLE_EQ(xt::linalg::trace(m)[0], 3.0);
}

TEST(LinAlgTests, Single) {
	navtk::Matrix m{{4.0}};
	ASSERT_DOUBLE_EQ(xt::linalg::trace(m)[0], 4.0);
}

TEST(LinAlgTests, EyeMult) {
	navtk::Matrix m = navtk::eye(3) * 2.0;
	ASSERT_DOUBLE_EQ(xt::linalg::trace(m)[0], 6.0);
}

TEST(LinAlgTests, TraceTestVaried) {
	navtk::Matrix m{{1.11, 1.9}, {-313.0, 2.9}};
	ASSERT_DOUBLE_EQ(xt::linalg::trace(m)[0], 4.01);
}

TEST(LinAlgTests, TraceTestVariedNeg) {
	navtk::Matrix m{{-1.5, 1.9}, {-313.0, 2.9}};
	ASSERT_DOUBLE_EQ(xt::linalg::trace(m)[0], 1.4);
}

// TODO: xtensor will actually calculate the trace of a non-square matrix without
// throwing (just starting at top left and summing till it hits an edge);
// Need to determine what sort of behavior we prefer.
// See https://git.aspn.us/pntos/navtk/-/issues/105
TEST(LinAlgTests, DISABLED_NonSquare) {
	navtk::Matrix m{{-1.5, 1.9, 33.0}, {-313.0, 2.9, 3838.0}};
	xt::linalg::trace(m);
	ASSERT_THROW(xt::linalg::trace(m), std::invalid_argument);
}

TEST(LinAlgTests, TridiagonalSolve) {
	// Set up a tridiagonal system, and compare classic solver vs alternatives
	std::vector<double> y{2.0, 4.0, 7.0, 6.0, 5.0, 1.0, 2.0, 3.0};
	std::vector<double> x{3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};

	auto n = x.size();

	// Natural cubic spline setup for equidistant data
	navtk::Vector rhs = navtk::zeros(n);
	rhs[0]            = 3 * (y[1] - y[0]);
	for (navtk::Size k = 1; k < n - 1; k++) {
		rhs[k] = 3 * (y[k + 1] - y[k - 1]);
	}
	rhs[n - 1] = 3 * (y[n - 1] - y[n - 2]);

	navtk::Matrix coef           = navtk::eye(n) * 4 + xt::eye(n, 1) + xt::eye(n, -1);
	xt::view(coef, 0, 0)         = 2.0;
	xt::view(coef, n - 1, n - 1) = 2.0;

	auto classic = xt::linalg::solve(coef, rhs);

	// Extract the 3 diagonals
	auto lower       = navtk::zeros(n - 1);
	auto upper       = navtk::zeros(n - 1);
	navtk::Vector di = xt::diagonal(coef);
	for (navtk::Size k = 0; k < n - 1; k++) {
		lower[k] = coef(k + 1, k);
		upper[k] = coef(k, k + 1);
	}

	auto tri_copy = navtk::solve_tridiagonal(lower, di, upper, rhs);
	auto tri_over = navtk::solve_tridiagonal_overwrite(lower, di, upper, rhs);
	ASSERT_ALLCLOSE(classic, tri_copy);
	ASSERT_ALLCLOSE(classic, tri_over);
}

TEST(LinAlgTests, wahba) {
	navtk::Matrix3 B{{-0.0160328419948849, 0.00786993156961537, 0.651952773089760},
	                 {-0.748191406234014, 0.318333566880978, 27.3748010568415},
	                 {255.273572491651, -112.738115121616, -9656.00506857569}};

	navtk::Matrix3 expected{{0.101872199536885, -0.994693639916338, 0.0143742015953294},
	                        {-0.994517205294065, -0.102175844679193, -0.0222626399642131},
	                        {0.0236132025697418, -0.0120274466982685, -0.999648816930386}};

	auto res = navtk::solve_wahba_svd(B);
	ASSERT_ALLCLOSE_EX(expected, res, 1e-10, 1e-10);
}

TEST(LinAlgTests, testWahbaRotApplied) {
	navtk::Vector3 m1{9.7, 3.2, -4.4};
	navtk::Vector3 m2{0.6, -3.3, 1.1};
	navtk::Vector3 rpy{2.1, -0.3, -1.1};
	auto C_n_to_s = xt::transpose(navtk::navutils::rpy_to_dcm(rpy));
	auto rot1     = navtk::dot(C_n_to_s, m1);
	auto rot2     = navtk::dot(C_n_to_s, m2);
	auto b        = xt::linalg::outer(rot1, m1) + xt::linalg::outer(rot2, m2);
	auto res      = navtk::solve_wahba_svd(b);
	ASSERT_ALLCLOSE_EX(C_n_to_s, res, 1e-10, 1e-10);
}

TEST(LinAlgTests, testWahbaScaleNot_SLOW) {
	navtk::Vector3 rpy{2.1, -0.3, -1.1};
	auto C_n_to_s         = xt::transpose(navtk::navutils::rpy_to_dcm(rpy));
	navtk::Matrix3 b      = navtk::zeros(3, 3);
	navtk::Matrix3 b2     = navtk::zeros(3, 3);
	navtk::Size num_loops = 100;

	for (navtk::Size k = 0; k < num_loops; k++) {
		auto m1   = navtk::experimental::rand_n(3);
		auto m2   = navtk::experimental::rand_n(3);
		auto rot1 = navtk::dot(C_n_to_s, m1);
		auto rot2 = navtk::dot(C_n_to_s, m2);
		b += xt::linalg::outer(rot1, m1) + xt::linalg::outer(rot2, m2);

		b2 += 1.0 / num_loops *
		      (xt::linalg::outer(rot1 / navtk::norm(rot1), m1 / navtk::norm(m1)) +
		       xt::linalg::outer(rot2 / navtk::norm(rot2), m2 / navtk::norm(m2)));
	}
	auto res  = navtk::solve_wahba_svd(b);
	auto res2 = navtk::solve_wahba_svd(b2);
	ASSERT_ALLCLOSE(res, res2);
	ASSERT_ALLCLOSE(res, C_n_to_s);
}

struct DavenportTestResults {
	double noise_level;
	std::vector<navtk::Matrix3> davenport_return;
	navtk::Size num_meas_provided;
	// Only first entry, which should be the best available solution
	navtk::Vector3 tilt_err;
};

std::vector<DavenportTestResults> davenport_test_base(
    const navtk::Vector3& rpy,
    const std::function<std::pair<navtk::Vector3, navtk::Vector3>(const double,
                                                                  const navtk::Matrix3&)>& data_gen,
    navtk::Size num_increases = 0) {

	std::vector<DavenportTestResults> out;

	auto C_n_to_s         = xt::transpose(navtk::navutils::rpy_to_dcm(rpy));
	navtk::Matrix3 b      = navtk::zeros(3, 3);
	navtk::Vector3 cr     = navtk::zeros(3);
	navtk::Size num_loops = 100;
	std::vector<navtk::Matrix3> res;

	double noise_level    = 1e-10;
	navtk::Size num_outer = 0;
	while (num_outer <= num_increases) {
		navtk::Size k = 0;
		while (k < num_loops) {
			auto meas_pair = data_gen(noise_level, C_n_to_s);
			auto r_vec     = meas_pair.first;
			auto b_vec     = meas_pair.second;

			b += xt::linalg::outer(b_vec, r_vec);
			cr += navtk::cross(b_vec, r_vec);
			res = navtk::solve_wahba_davenport(b, cr);
			if (res.size() == 1) {
				break;
			}
			k++;
		}
		auto tilts = navtk::navutils::dcm_to_rpy(navtk::dot(xt::transpose(res[0]), C_n_to_s));
		out.push_back(DavenportTestResults{noise_level, res, k, tilts});

		b  = navtk::zeros(3, 3);
		cr = navtk::zeros(3);
		noise_level *= 10;
		num_outer += 1;
	}
	return out;
}

TEST(LinAlgTests, testDavenport) {
	navtk::Vector3 rpy{2.1, -0.3, -1.1};

	auto f = [](const double,
	            const navtk::Matrix3& csn) -> std::pair<navtk::Vector3, navtk::Vector3> {
		auto r_vec = navtk::experimental::rand_n(3);
		auto b_vec = navtk::dot(csn, r_vec);
		return {r_vec, b_vec};
	};

	auto solutions = davenport_test_base(rpy, f);
	for (auto sol = solutions.cbegin(); sol < solutions.cend(); sol++) {
		ASSERT_TRUE(sol->davenport_return.size() == 1);
	}
	ASSERT_TRUE(xt::all(xt::abs(solutions[0].tilt_err) <
	                    navtk::ones(3, 3) * 0.1 * navtk::navutils::PI / 180.0));
}

TEST(LinAlgTests, testDavenportNoisy_SLOW) {
	navtk::Vector3 rpy{2.1, -0.3, -1.1};

	auto f = [](const double noise_level,
	            const navtk::Matrix3& csn) -> std::pair<navtk::Vector3, navtk::Vector3> {
		// Both meas and noise are regenerated each time
		auto r_vec = navtk::experimental::rand_n(3);
		auto b_vec = navtk::dot(csn, r_vec);

		// Add noise
		r_vec += navtk::experimental::rand_n(3) * noise_level;
		b_vec += navtk::experimental::rand_n(3) * noise_level;
		return {r_vec, b_vec};
	};

	auto solutions = davenport_test_base(rpy, f, 9);
	for (auto sol = solutions.cbegin(); sol < solutions.cend(); sol++) {
		ASSERT_TRUE(sol->davenport_return.size() == 1);
	}
	ASSERT_TRUE(xt::all(xt::abs(solutions[0].tilt_err) <
	                    navtk::ones(3, 3) * 0.1 * navtk::navutils::PI / 180.0));
}

TEST(LinAlgTests, testDavenportBiased_SLOW) {
	navtk::Vector3 rpy{2.1, -0.3, -1.1};

	double last_noise_level = 0;
	navtk::Vector3 b_bias   = navtk::zeros(3);
	navtk::Vector3 r_bias   = navtk::zeros(3);

	auto f = [&last_noise_level, &b_bias, &r_bias](
	             const double noise_level,
	             const navtk::Matrix3& csn) -> std::pair<navtk::Vector3, navtk::Vector3> {
		// Bias; one draw for each iteration
		if (last_noise_level != noise_level) {
			b_bias           = navtk::experimental::rand_n(3) * noise_level;
			r_bias           = navtk::experimental::rand_n(3) * noise_level;
			last_noise_level = noise_level;
		}

		auto r_vec = navtk::experimental::rand_n(3);
		auto b_vec = navtk::dot(csn, r_vec);
		return {r_vec + r_bias, b_vec + b_bias};
	};

	auto solutions = davenport_test_base(rpy, f, 9);
	for (auto sol = solutions.cbegin(); sol < solutions.cend(); sol++) {
		ASSERT_TRUE(sol->davenport_return.size() == 1);
	}
	ASSERT_TRUE(xt::all(xt::abs(solutions[0].tilt_err) <
	                    navtk::ones(3, 3) * 0.1 * navtk::navutils::PI / 180.0));
}

TEST(LinAlgTests, testDavenportScaled_SLOW) {
	navtk::Vector3 rpy{2.1, -0.3, -1.1};

	auto f = [](const double noise_level,
	            const navtk::Matrix3& csn) -> std::pair<navtk::Vector3, navtk::Vector3> {
		auto r_vec = navtk::experimental::rand_n(3);
		auto b_vec = navtk::dot(csn, r_vec);
		return {r_vec * (1 + navtk::experimental::rand_n(3) * noise_level),
		        b_vec * (1 + navtk::experimental::rand_n(3) * noise_level)};
	};

	auto solutions = davenport_test_base(rpy, f, 9);
	for (auto sol = solutions.cbegin(); sol < solutions.cend(); sol++) {
		ASSERT_TRUE(sol->davenport_return.size() == 1);
	}
	ASSERT_TRUE(xt::all(xt::abs(solutions[0].tilt_err) <
	                    navtk::ones(3, 3) * 0.1 * navtk::navutils::PI / 180.0));
}

TEST(LinAlgTests, compareWahbaVector) {
	navtk::Vector3 rpy{2.1, -0.3, -1.1};
	auto C_n_to_s0       = xt::transpose(navtk::navutils::rpy_to_dcm(rpy));
	navtk::Size num_meas = 20;
	std::vector<navtk::Vector3> ref;
	std::vector<navtk::Vector3> platform;
	for (navtk::Size k = 0; k < num_meas; k++) {
		ref.push_back(navtk::experimental::rand_n(3) * navtk::experimental::rand_n(1));
		platform.push_back(navtk::dot(C_n_to_s0, ref.back()));
	}
	auto dav_res = navtk::solve_wahba_davenport(platform, ref);
	auto wah_res = navtk::solve_wahba_svd(platform, ref);
	ASSERT_ALLCLOSE(dav_res[0], C_n_to_s0);
	ASSERT_ALLCLOSE(wah_res, C_n_to_s0);
}

ERROR_MODE_SENSITIVE_TEST(TEST, LinAlgTests, WahbaEmptyVectors) {
	auto res_wahba = navtk::zeros(3, 3);
	std::vector<navtk::Matrix3> res_dav(1, navtk::zeros(3, 3));
	EXPECT_HONORS_MODE_EX(res_wahba = navtk::solve_wahba_svd(std::vector<navtk::Vector3>(),
	                                                         std::vector<navtk::Vector3>()),
	                      "at least 2",
	                      std::runtime_error);
	EXPECT_HONORS_MODE_EX(res_dav = navtk::solve_wahba_davenport(std::vector<navtk::Vector3>(),
	                                                             std::vector<navtk::Vector3>()),
	                      "at least 2",
	                      std::runtime_error);
	ASSERT_ALLCLOSE(res_wahba, navtk::zeros(3, 3));
	ASSERT_TRUE(res_dav.size() == 1);
	ASSERT_ALLCLOSE(res_dav[0], navtk::zeros(3, 3));
}

ERROR_MODE_SENSITIVE_TEST(TEST, LinAlgTests, WahbaSingleVectors) {
	auto res_wahba = navtk::zeros(3, 3);
	std::vector<navtk::Matrix3> res_dav(1, navtk::zeros(3, 3));

	EXPECT_HONORS_MODE_EX(
	    res_wahba = navtk::solve_wahba_svd(std::vector<navtk::Vector3>(1, navtk::zeros(3)),
	                                       std::vector<navtk::Vector3>(1, navtk::zeros(3))),
	    "at least 2",
	    std::runtime_error);
	EXPECT_HONORS_MODE_EX(
	    res_dav = navtk::solve_wahba_davenport(std::vector<navtk::Vector3>(1, navtk::zeros(3)),
	                                           std::vector<navtk::Vector3>(1, navtk::zeros(3))),
	    "at least 2",
	    std::runtime_error);
	ASSERT_ALLCLOSE(res_wahba, navtk::zeros(3, 3));
	ASSERT_TRUE(res_dav.size() == 1);
	ASSERT_ALLCLOSE(res_dav[0], navtk::zeros(3, 3));
}

ERROR_MODE_SENSITIVE_TEST(TEST, LinAlgTests, WahbaDiffSizeVectors) {
	auto res_wahba = navtk::zeros(3, 3);
	std::vector<navtk::Matrix3> res_dav(1, navtk::zeros(3, 3));

	EXPECT_HONORS_MODE_EX(
	    res_wahba = navtk::solve_wahba_svd(std::vector<navtk::Vector3>(2, navtk::zeros(3)),
	                                       std::vector<navtk::Vector3>(3, navtk::zeros(3))),
	    "same size",
	    std::runtime_error);
	EXPECT_HONORS_MODE_EX(
	    res_dav = navtk::solve_wahba_davenport(std::vector<navtk::Vector3>(2, navtk::zeros(3)),
	                                           std::vector<navtk::Vector3>(3, navtk::zeros(3))),
	    "same size",
	    std::runtime_error);
	ASSERT_ALLCLOSE(res_wahba, navtk::zeros(3, 3));
	ASSERT_TRUE(res_dav.size() == 1);
	ASSERT_ALLCLOSE(res_dav[0], navtk::zeros(3, 3));
}

TEST(LinAlgTests, matrix_power_basic) {
	navtk::Matrix m{{1.0, 2.0}, {3.0, 4.0}};
	auto res = navtk::eye(2);
	for (int k = 1; k < 12; ++k) {
		res              = navtk::dot(m, res);
		navtk::Matrix pw = navtk::matrix_power(m, k);
		ASSERT_ALLCLOSE(res, pw);
	}
}

TEST(LinAlgTests, xt_matrix_power_working_odd) {
	navtk::Matrix m{{1.0, 2.0}, {3.0, 4.0}};
	auto res = navtk::eye(2);
	for (int k = 0; k < 3; ++k) {
		res = navtk::dot(m, res);
	}
	for (int k = 5; k < 12; k += 2) {
		res              = navtk::dot(m, navtk::dot(m, res));
		navtk::Matrix pw = xt::linalg::matrix_power(m, k);
		ASSERT_ALLCLOSE(res, pw);
	}
}

TEST(LinAlgTests, xt_matrix_power_working_small) {
	navtk::Matrix m{{1.0, 2.0}, {3.0, 4.0}};
	auto res = navtk::eye(2);
	for (int k = 1; k < 4; ++k) {
		res              = navtk::dot(m, res);
		navtk::Matrix pw = xt::linalg::matrix_power(m, k);
		ASSERT_ALLCLOSE(res, pw);
	}
}

// TODO: When this breaks it means xtensor-blas is fixed and we can
// replace our matrix_power with theirs
TEST(LinAlgTests, xt_matrix_power_broken_even) {
	navtk::Matrix m{{1.0, 2.0}, {3.0, 4.0}};
	for (int k = 4; k < 12; k += 2) {
		navtk::Matrix pw = xt::linalg::matrix_power(m, k);

		// xt version just returns m^1 when k is divisible by 4, otherwise m^(k/2)
		if (std::div(k, 4).rem == 0) {
			ASSERT_ALLCLOSE(m, pw);
		} else {
			auto res = navtk::eye(2);
			for (int i = 0; i < k / 2; ++i) {
				res = navtk::dot(m, res);
			}
			ASSERT_ALLCLOSE(res, pw);
		}
	}
}
