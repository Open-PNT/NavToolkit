#include <memory>
#include <numeric>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <filtering/fusion/strategies/ConfiguredStrategy.hpp>
#include <tensor_assert.hpp>
#include <xtensor/generators/xrandom.hpp>

#include <navtk/filtering/containers/LinearizedStrategyBase.hpp>
#include <navtk/filtering/experimental/containers/RbpfModel.hpp>
#include <navtk/filtering/fusion/strategies/EkfStrategy.hpp>
#include <navtk/filtering/fusion/strategies/FusionStrategy.hpp>
#include <navtk/filtering/fusion/strategies/UkfStrategy.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using namespace std;
using namespace navtk::filtering;
using namespace navtk::filtering::experimental;
using namespace navtk;
using ::testing::AssertionException;
using ::testing::TestPartResult;

// This class defines tolerances (RTOL, ATOL) that will be used in the
// ASSERT_ALLCLOSE calls throughout this file. Explicit specializations of
// this template (such as the Tolerances<RbpfModelCondition<T>> definition
// below) can be added to loosen or tighten the tolerances for a particular
// class. Values of T will be the TypeParam of the
// TYPED_TEST_SUITE_P(FusionStrategyTests);
template <class T>
struct Tolerances {
	static constexpr double RTOL = 1e-5;
	static constexpr double ATOL = 1e-8;
};

// A version of ASSERT_ALLCLOSE that uses Tolerances<TypeParam> for its values
// of RTOL and ATOL.
#define ASSERT_ALLCLOSE_T(...) \
	ASSERT_ALLCLOSE_EX(__VA_ARGS__, Tolerances<TypeParam>::RTOL, Tolerances<TypeParam>::ATOL)

// Fixture for the TYPED_TEST_SUITE_P. <class T> are FusionStrategy subclasses
// added to the FusionStrategyTestsTypes typedef below.
template <class T>
struct FusionStrategyTests : ::testing::Test {
	// Make sure only FusionStrategy subclasses are tested through this fixture.
	static enable_if_t<is_base_of<FusionStrategy, T>::value>* guard;

	// Sample values, where xN is a sample state estimate and pN is a sample covariance, and N is
	// the number of states.
	const Vector x0, x1, x2, x3;
	const Matrix p0, p1, p2, p3;

	// All the sample values above in a form that can be iterated
	vector<pair<const Vector&, const Matrix&>> all_the_things;

	// clang-format off
	FusionStrategyTests()
	    : ::testing::Test(),
	      x0({}),
	      x1({100}),
	      x2({200, 300}),
	      x3({500, 600, 700}),
	      p0({}),
	      p1({{2}}),
	      p2({{2  ,  .4},
	          { .4, 3  }}),
	      p3({{2   ,  .06,  .04},
	          { .06, 3   ,  .02},
	          { .04,  .02, 4   }}),
	      all_the_things({{x0, p0}, {x1, p1}, {x2, p2}, {x3, p3}}) {}
	// clang-format on

	static unique_ptr<T> make_target(Vector x, Matrix p) {
		auto target = make_unique<T>(x, p);
		skip_unless_match(*target, x, p);
		return target;
	}

	static bool is_match(const T& target, const Vector& x, const Matrix& p) {
		auto estimate = to_matrix(target.get_estimate());
		auto P        = target.get_covariance();
		Tolerances<T> tol;
		filtering::testing::AllCloseHelper ach{tol.RTOL, tol.ATOL, false};
		return ach.allclose(estimate, to_matrix(x)) && ach.allclose(p, P);
	}

	static void skip_unless_match(const T& target, const Vector& x, const Matrix& p) {
		if (!is_match(target, x, p)) {
			GTEST_SKIP() << "Fix InitialValuePreserved to enable this test.";
		}
	}

	static unique_ptr<T> make_target(const pair<const Vector&, const Matrix&>& pair) {
		return make_target(pair.first, pair.second);
	}

	unique_ptr<T> make_target(size_t index) { return make_target(all_the_things.at(index)); }

	void SetUp() override { xt::random::seed(2535736701); }
};

TYPED_TEST_SUITE_P(FusionStrategyTests);

TYPED_TEST_P(FusionStrategyTests, InitialValuesPreserved) {
	for (const auto& thing : this->all_the_things) {
		TypeParam target(thing.first, thing.second);
		ASSERT_ALLCLOSE_T(thing.first, target.get_estimate());
		ASSERT_ALLCLOSE_EX(thing.second, target.get_covariance(), 0.1, 0.1);
	}
}

TYPED_TEST_P(FusionStrategyTests, ValuesPreservedAcrossAssign) {
	for (const auto& thing : this->all_the_things) {
		TypeParam target = *this->make_target(thing);
		ASSERT_ALLCLOSE_T(thing.first, target.get_estimate());
		ASSERT_ALLCLOSE_T(thing.second, target.get_covariance());
	}
}

TYPED_TEST_P(FusionStrategyTests, AssignOverwriteAll) {
	for (const auto& thing : this->all_the_things) {
		auto size   = num_rows(thing.first);
		auto target = this->make_target(zeros(size), zeros(size, size));
		target->set_estimate_slice(thing.first);
		target->set_covariance_slice(thing.second);
		ASSERT_ALLCLOSE_T(thing.first, target->get_estimate());
		ASSERT_ALLCLOSE_T(thing.second, target->get_covariance());
	}
}

TYPED_TEST_P(FusionStrategyTests, EmptyOverwrite) {
	for (const auto& thing : this->all_the_things) {
		auto target = this->make_target(thing);
		target->set_estimate_slice(this->x0);
		target->set_covariance_slice(this->p0);
		ASSERT_ALLCLOSE_T(thing.first, target->get_estimate());
		ASSERT_ALLCLOSE_T(thing.second, target->get_covariance());
	}
}

TYPED_TEST_P(FusionStrategyTests, TopLeftCorner) {
	auto target = this->make_target(3);
	target->set_estimate_slice(this->x2);
	target->set_covariance_slice(this->p2);
	Vector expected_estimate{200, 300, 700};
	// clang-format off
	Matrix expected_covariance{{2   ,  .4 ,  .04},
	                  { .4 , 3   ,  .02},
	                  { .04,  .02, 4   }};
	// clang-format on
	ASSERT_ALLCLOSE_T(expected_estimate, target->get_estimate());
	ASSERT_ALLCLOSE_T(expected_covariance, target->get_covariance());
}

TYPED_TEST_P(FusionStrategyTests, TopRightCorner) {
	auto target    = this->make_target(3);
	Matrix p_slice = this->p2;
	p_slice(1, 0)  = 5;
	target->set_covariance_slice(p_slice, 0, 1);
	// clang-format off
	Matrix expected_covariance{{2  , 2  ,  .4},
	                  {2  ,  5, 3  },
	                  { .4, 3  , 4  }};
	// clang-format on
	ASSERT_ALLCLOSE_T(expected_covariance, target->get_covariance());
}

TYPED_TEST_P(FusionStrategyTests, BottomLeftCorner) {
	auto target = this->make_target(3);
	target->set_estimate_slice(this->x2, 1);
	Matrix p_slice = this->p2;
	p_slice(0, 1)  = 5;
	target->set_covariance_slice(p_slice, 1, 0);
	Vector expected_estimate{500, 200, 300};
	// clang-format off
	Matrix expected_covariance{{2  , 2  ,  .4},
	                  {2  ,  5, 3  },
	                  { .4, 3  , 4  }};
	// clang-format on
	ASSERT_ALLCLOSE_T(expected_estimate, target->get_estimate());
	ASSERT_ALLCLOSE_T(expected_covariance, target->get_covariance());
}

TYPED_TEST_P(FusionStrategyTests, BottomRightCorner) {
	auto target = this->make_target(3);
	target->set_covariance_slice(this->p2, 1);
	// clang-format off
	Matrix expected_covariance{{2   ,  .06,  .04},
	                  { .06, 2   ,  .4 },
	                  { .04,  .4 , 3   }};
	// clang-format on
	ASSERT_ALLCLOSE_T(expected_covariance, target->get_covariance());
}

TYPED_TEST_P(FusionStrategyTests, Middle) {
	auto target = this->make_target(zeros(5), eye(5));
	target->set_estimate_slice(this->x3, 1);
	target->set_covariance_slice(this->p3, 1, 1);
	Vector expected_estimate{0, 500, 600, 700, 0};
	// clang-format off
	Matrix expected_covariance{{1   , 0   , 0   , 0   , 0   },
	                  {0   , 2   ,  .06,  .04, 0   },
	                  {0   ,  .06, 3   ,  .02, 0   },
	                  {0   ,  .04,  .02, 4   , 0   },
	                  {0   , 0   , 0   , 0   , 1   }};
	// clang-format on
	ASSERT_ALLCLOSE_T(expected_estimate, target->get_estimate());
	ASSERT_ALLCLOSE_T(expected_covariance, target->get_covariance());
}

TYPED_TEST_P(FusionStrategyTests, AddStates) {
	auto target = this->make_target(3);
	ASSERT_EQ(3, target->on_fusion_engine_state_block_added(2));
	ASSERT_EQ(5, target->on_fusion_engine_state_block_added(1));
	Vector expected_estimate{500, 600, 700, 0, 0, 0};
	// clang-format off
	Matrix expected_covariance{{2   ,  .06,  .04, 0   , 0   , 0   },
	                  { .06, 3   ,  .02, 0   , 0   , 0   },
	                  { .04,  .02, 4   , 0   , 0   , 0   },
	                  {0   , 0   , 0   , 1   , 0   , 0   },
	                  {0   , 0   , 0   , 0   , 1   , 0   },
	                  {0   , 0   , 0   , 0   , 0   , 1   }};
	// clang-format on
	ASSERT_ALLCLOSE_T(expected_estimate, target->get_estimate());
	ASSERT_ALLCLOSE_T(expected_covariance, target->get_covariance());
}

TYPED_TEST_P(FusionStrategyTests, AddStatesStartingFromEmpty) {
	auto target = this->make_target({}, {});
	ASSERT_EQ(0, target->on_fusion_engine_state_block_added(4));
	ASSERT_ALLCLOSE_T(zeros(4), target->get_estimate());
	ASSERT_ALLCLOSE_T(eye(4), target->get_covariance());
}

TYPED_TEST_P(FusionStrategyTests, AddStatesWithCrossCovariance) {
	auto target = this->make_target(zeros(3), zeros(3, 3));
	auto initial_estimate{zeros(2)};
	auto initial_covariance{ones(2, 2)};
	// clang-format off
	Matrix cross_covariance{ { 1.1 , 2.1 },
	                		 { 1.2 , 2.2 },
	                		 { 1.3 , 2.3 } };
	Matrix expected_covariance{ { 0.0 , 0.0 , 0.0 , 1.1 , 2.1 },
	                			{ 0.0 , 0.0 , 0.0 , 1.2 , 2.2 },
	                			{ 0.0 , 0.0 , 0.0 , 1.3 , 2.3 },
	                			{ 1.1 , 1.2 , 1.3 , 1.0 , 1.0 },
	                			{ 2.1 , 2.2 , 2.3 , 1.0 , 1.0 } };
	// clang-format on
	ASSERT_EQ(3,
	          target->on_fusion_engine_state_block_added(
	              initial_estimate, initial_covariance, cross_covariance));
	ASSERT_ALLCLOSE_T(expected_covariance, target->get_covariance());
}

TYPED_TEST_P(FusionStrategyTests, RemoveOneState) {
	auto target = this->make_target(3);
	target->on_fusion_engine_state_block_removed(1, 1);
	Vector expected_estimate{500, 700};
	// clang-format off
	Matrix expected_covariance{{2   ,  .04},
	                  { .04, 4   }};
	// clang-format on
	ASSERT_ALLCLOSE_T(expected_covariance, target->get_covariance());
	ASSERT_ALLCLOSE_T(expected_estimate, target->get_estimate());
}

TYPED_TEST_P(FusionStrategyTests, RemoveAllStates) {
	auto target = this->make_target(3);
	target->on_fusion_engine_state_block_removed(0, 3);
	ASSERT_ALLCLOSE_T(Vector(), target->get_estimate());
	ASSERT_ALLCLOSE_T(Matrix(), target->get_covariance());
}

TYPED_TEST_P(FusionStrategyTests, CloneIsDetached) {
	auto target = this->make_target(3);
	LinearizedStrategyBase victim;
	not_null<shared_ptr<FusionStrategy>> clone = target->clone();
	ASSERT_NE(nullptr, dynamic_cast<TypeParam*>(clone.get().get()))
	    << "target->clone() returned wrong type.";
	target->set_estimate_slice(fill(999, 3));
	target->set_covariance_slice(eye(3) * 999);
	ASSERT_ALLCLOSE_T(this->x3, clone->get_estimate());
	ASSERT_ALLCLOSE_T(this->p3, clone->get_covariance());
}

TYPED_TEST_P(FusionStrategyTests, SymmetryValidation) {
	auto target = this->make_target(3);
	target->set_covariance_slice(eye(3) * 3.0);
	auto base_cov = target->get_covariance();
	auto rows     = num_rows(base_cov);
	for (int k = 0; k < 20; k++) {
		auto new_cov = base_cov;
		new_cov(0, rows - 1) += pow(10.0, -k);
		target->set_covariance_slice(new_cov);
		target->symmetricize_covariance();
		auto cov = target->get_covariance();
		auto res = navtk::utils::ValidationContext(navtk::ErrorMode::LOG)
		               .add_matrix(cov)
		               .symmetric()
		               .validate();
		ASSERT_TRUE(navtk::is_symmetric(cov));
		ASSERT_TRUE(res == navtk::utils::ValidationResult::GOOD);
	}
}

TYPED_TEST_P(FusionStrategyTests, AddStates3ValidatesCrossCov) {
	auto target = this->make_target(3);
	EXPECT_UB_OR_DIE(target->on_fusion_engine_state_block_added(zeros(2), ones(2, 2), ones(1, 4)),
	                 "cross_covariance");
}

TYPED_TEST_P(FusionStrategyTests, AddStates3ValidatesInitialCov) {
	auto target = this->make_target(3);
	EXPECT_UB_OR_DIE(target->on_fusion_engine_state_block_added(zeros(2), ones(2, 3), ones(2, 3)),
	                 "initial_covariance");
}

TYPED_TEST_P(FusionStrategyTests, AddStates3ValidatesInitialEst) {
	auto target = this->make_target(3);
	EXPECT_UB_OR_DIE(target->on_fusion_engine_state_block_added(zeros(9), ones(2, 2), ones(2, 3)),
	                 "initial_estimate");
}

TYPED_TEST_P(FusionStrategyTests, AddStates2ValidatesInitialCov) {
	auto target = this->make_target(3);
	EXPECT_UB_OR_DIE(target->on_fusion_engine_state_block_added(zeros(2), ones(2, 3)),
	                 "initial_covariance");
}

TYPED_TEST_P(FusionStrategyTests, AddStates2ValidatesInitialEst) {
	auto target = this->make_target(3);
	EXPECT_UB_OR_DIE(target->on_fusion_engine_state_block_added(zeros(9), ones(2, 2)),
	                 "initial_estimate");
}

TYPED_TEST_P(FusionStrategyTests, RemoveStatesValidation) {
	auto target = this->make_target(3);
	EXPECT_UB_OR_DIE(
	    target->on_fusion_engine_state_block_removed(4, 1), "invalid", std::invalid_argument);
	EXPECT_UB_OR_DIE(
	    target->on_fusion_engine_state_block_removed(1, 4), "more states", std::invalid_argument);
	EXPECT_UB_OR_DIE(
	    target->on_fusion_engine_state_block_removed(2, 2), "invalid", std::invalid_argument);
}

TYPED_TEST_P(FusionStrategyTests, SetEstimateSliceValidation) {
	auto target = this->make_target(3);
	EXPECT_UB_OR_DIE(target->set_estimate_slice(zeros(4), 0), "beyond the end");
	EXPECT_UB_OR_DIE(target->set_estimate_slice(zeros(1), 4), "beyond the end");
	EXPECT_UB_OR_DIE(target->set_estimate_slice(zeros(2), 2), "beyond the end");
}

TYPED_TEST_P(FusionStrategyTests, SetCovarianceSliceValidation) {
	auto target = this->make_target(3);
	EXPECT_UB_OR_DIE(target->set_covariance_slice(zeros(4, 1), 0, 0), "beyond the end");
	EXPECT_UB_OR_DIE(target->set_covariance_slice(zeros(1, 4), 0, 0), "beyond the end");
	EXPECT_UB_OR_DIE(target->set_covariance_slice(zeros(4, 4), 0, 0), "beyond the end");
	EXPECT_UB_OR_DIE(target->set_covariance_slice(zeros(1, 1), 4, 0), "beyond the end");
	EXPECT_UB_OR_DIE(target->set_covariance_slice(zeros(1, 1), 0, 4), "beyond the end");
	EXPECT_UB_OR_DIE(target->set_covariance_slice(zeros(1, 1), 4, 4), "beyond the end");
	EXPECT_UB_OR_DIE(target->set_covariance_slice(zeros(2, 2), 0, 2), "beyond the end");
	EXPECT_UB_OR_DIE(target->set_covariance_slice(zeros(2, 2), 2, 0), "beyond the end");
	EXPECT_UB_OR_DIE(target->set_covariance_slice(zeros(2, 2), 2, 2), "beyond the end");

	// Off-diagonal, validation should block a slice that would overwrite itself.
	EXPECT_UB_OR_DIE(
	    target->set_covariance_slice(zeros(2, 1), 0, 1), "overlap", std::invalid_argument);
}

REGISTER_TYPED_TEST_SUITE_P(FusionStrategyTests,
                            InitialValuesPreserved,
                            ValuesPreservedAcrossAssign,
                            AssignOverwriteAll,
                            EmptyOverwrite,
                            TopLeftCorner,
                            TopRightCorner,
                            BottomLeftCorner,
                            BottomRightCorner,
                            Middle,
                            AddStates,
                            AddStatesStartingFromEmpty,
                            AddStatesWithCrossCovariance,
                            RemoveOneState,
                            RemoveAllStates,
                            CloneIsDetached,
                            AddStates3ValidatesCrossCov,
                            AddStates3ValidatesInitialCov,
                            AddStates3ValidatesInitialEst,
                            AddStates2ValidatesInitialCov,
                            AddStates2ValidatesInitialEst,
                            RemoveStatesValidation,
                            SetEstimateSliceValidation,
                            SetCovarianceSliceValidation,
                            SymmetryValidation);

// A wrapper for proxy classes that allows their behavior to be validated.
// It expects to have to override a FusionStrategy& get_model() function, and
// will supply an instance of the Backing class.
template <typename T, typename Backing = LinearizedStrategyBase>
class TestableProxy : public T {
public:
	TestableProxy() : model(make_unique<Backing>()) {}
	TestableProxy(unique_ptr<FusionStrategy> src) : model(std::move(src)) {}
	TestableProxy(const TestableProxy<T, Backing>&) = default;
	not_null<shared_ptr<FusionStrategy>> clone() const override {
		return make_shared<TestableProxy<T, Backing>>(model->clone().get());
	}

protected:
	shared_ptr<FusionStrategy> model;
	FusionStrategy& get_model() override { return *model; }
};

// A subclass of RbpfModel that marks some of its own states as particles.
// This lets us use the TYPED_TEST_SUITE_P(FusionStrategyTests) to test the
// RBPF under conditions besides its default of "nothing is marked," to make
// sure expected FusionStrategy behaviors hold regardless of the marks.
template <vector<size_t> (*make_marks)(Size rows)>
class RbpfModelCondition : public RbpfModel {
public:
	RbpfModelCondition(Vector x0, Matrix p0) : RbpfModel() {
		on_fusion_engine_state_block_added(navtk::num_rows(x0));
		set_estimate_slice(x0);
		set_covariance_slice(p0);
		set_marked_states(make_marks(num_rows(x0)));
		set_particle_count_target(10);
	}

	void on_state_count_changed() override {
		RbpfModel::on_state_count_changed();
		set_marked_states(make_marks(get_num_states()));
	}

	not_null<shared_ptr<FusionStrategy>> clone() const override {
		return make_shared<RbpfModelCondition<make_marks>>(*this);
	}
};

// A possible template parameter for RbpfModelCondition which marks all states
// in the RBPF as particles.
vector<size_t> all_particles(Size rows) {
	vector<size_t> out(rows, 0);
	for (auto ii = rows; ii--;) out[ii] = ii;
	return out;
}

// A possible template parameter for RbpfModelCondition which marks every other
// state in the RBPF as particles, to test the condition of "some marked, some
// not"
vector<size_t> half_particles(Size rows) {
	vector<size_t> out(rows, 0);
	for (Size ii = 0; ii < rows; ii += 2) out[ii] = ii;
	return out;
}

// A possible template parameter for RbpfModelCondition which no states in the RBPF as particles.
vector<size_t> none_particles(Size rows) { return vector<size_t>(rows, 0); }

// Explicit specialization of Tolerances<T> to make RBPF tests that involve
// actual particles much more forgiving of randomness.
template <vector<size_t> (*make_marks)(Size rows)>
struct Tolerances<RbpfModelCondition<make_marks>> {
	static constexpr double RTOL = 1e-8;
	static constexpr double ATOL = 1e-8;
};

// Add subclasses of FusionStrategy that you wish to test to this template
// parameter list.
typedef ::testing::Types<ConfiguredStrategy<LinearizedStrategyBase>,
                         RbpfModelCondition<all_particles>,
                         RbpfModelCondition<half_particles>,
                         RbpfModelCondition<none_particles>,
                         ConfiguredStrategy<EkfStrategy>,
                         ConfiguredStrategy<UkfStrategy>>
    FusionStrategyTestsTypes;

// TODO (PNTOS-293) Remove the "_SLOW" marking here once a faster RNG is in place.
INSTANTIATE_TYPED_TEST_SUITE_P(EwC_SLOW, FusionStrategyTests, FusionStrategyTestsTypes, );
