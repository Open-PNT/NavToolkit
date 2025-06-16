#include <memory>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/stateblocks/EarthModel.hpp>
#include <navtk/filtering/stateblocks/GravityModel.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/discretization_strategy.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::dot;
using navtk::eye;
using navtk::Matrix;
using navtk::Scalar;
using navtk::Vector;
using navtk::Vector3;
using navtk::zeros;
using navtk::filtering::hg1700_model;
using navtk::filtering::hg9900_model;
using navtk::filtering::ImuModel;
using navtk::filtering::NavSolution;
using navtk::filtering::NULL_GEN_XHAT_AND_P_FUNCTION;
using navtk::filtering::Pinson15NedBlock;
using navtk::filtering::Pose;
using navtk::navutils::delta_lon_to_east;
using navtk::navutils::east_to_delta_lon;
using std::make_shared;
using xt::transpose;

bool operator==(const Pva& pva1, const Pva& pva2) {
	return pva1.get_time_of_validity().get_elapsed_nsec() ==
	           pva2.get_time_of_validity().get_elapsed_nsec() &&
	       pva1.get_p1() == pva2.get_p1() && pva1.get_p2() == pva2.get_p2() &&
	       pva1.get_p3() == pva2.get_p3() && pva1.get_v1() == pva2.get_v1() &&
	       pva1.get_v2() == pva2.get_v2() && pva1.get_v3() == pva2.get_v3() &&
	       pva1.get_quaternion() == pva2.get_quaternion();
}

bool operator==(const Imu imu1, const Imu& imu2) {
	return imu1.get_time_of_validity().get_elapsed_nsec() ==
	           imu2.get_time_of_validity().get_elapsed_nsec() &&
	       imu1.get_meas_accel() == imu2.get_meas_accel() &&
	       imu1.get_meas_gyro() == imu2.get_meas_gyro();
}

bool operator!=(const Pva& pva1, const Pva& pva2) { return !(pva1 == pva2); }

bool operator!=(const Imu& imu1, const Imu& imu2) { return !(imu1 == imu2); }

class FakeAux : public aspn_xtensor::AspnBase {
public:
	double d;
	FakeAux(double din = 3.14) : aspn_xtensor::AspnBase(ASPN_EXTENDED_BEGIN, 0, 0, 0, 0) {
		d = din;
	}
};

struct PinsonTests : public ::testing::Test {
	std::shared_ptr<Pva> pva_aux;
	std::shared_ptr<Imu> f_and_r_aux;
	AspnBaseVector aux;
	Pinson15NedBlock block;
	Pinson15NedBlock block1700;

	PinsonTests()
	    : pva_aux(std::make_shared<Pva>(navtk::utils::to_positionvelocityattitude(
	          NavSolution{zeros(3), zeros(3), eye(3), aspn_xtensor::TypeTimestamp((int64_t)0)}))),
	      f_and_r_aux(std::make_shared<Imu>(navtk::utils::to_imu(
	          pva_aux->get_time_of_validity(), Vector3{0.0, 0.0, -9.8}, zeros(3)))),
	      aux{pva_aux, f_and_r_aux},
	      block("TheBigP", hg9900_model()),
	      block1700("TheOtherP", hg1700_model()) {}
};

// Make sure aux data updates properly
TEST_F(PinsonTests, auxUp) {
	block.receive_aux_data(aux);
	for (int k = 0; k <= 5; k++) {
		auto aux2 =
		    AspnBaseVector{std::make_shared<Pva>(*pva_aux), std::make_shared<Imu>(*f_and_r_aux)};
		block.receive_aux_data(aux);
		auto block_pva_aux            = block.get_pva_aux();
		auto block_force_and_rate_aux = block.get_force_and_rate_aux();
		ASSERT_TRUE(*block_pva_aux == *pva_aux);
		ASSERT_TRUE(*block_force_and_rate_aux == *f_and_r_aux);
	}
}

// Test that passing aux_data via pointer to capturing lambda function works. It does if the
// original captured reference (a reference to a shared_ptr) is modifed directly by reassignment of
// it's member variables, but not if the `shared_ptr` itself is reassigned.
TEST_F(PinsonTests, auxUpFun_SLOW) {
	// It appears that capture only happens on the original object; if it is re-assigned there is no
	// update
	AspnBaseVector aux_reassign{pva_aux, f_and_r_aux};
	auto f_and_r_aux_mod = std::make_shared<Imu>(*f_and_r_aux);
	AspnBaseVector aux_mod{pva_aux, f_and_r_aux_mod};
	Pinson15NedBlock reassign_block(
	    "b1", hg9900_model(), [&](aspn_xtensor::TypeTimestamp, aspn_xtensor::TypeTimestamp) {
		    return aux_reassign;
	    });
	Pinson15NedBlock mod_block(
	    "b2", hg9900_model(), [&](aspn_xtensor::TypeTimestamp, aspn_xtensor::TypeTimestamp) {
		    return aux_mod;
	    });
	block.receive_aux_data(aux_reassign);
	for (double k = 0; k <= 5; k++) {
		f_and_r_aux_mod->set_meas_accel(Vector{0, 0, k});
		auto f_and_r_aux_reassign = std::make_shared<Imu>(*f_and_r_aux_mod);
		AspnBaseVector aux_reassign{pva_aux, f_and_r_aux_reassign};
		// Must call generate_dynamics for aux to populate via lin_function
		reassign_block.generate_dynamics(
		    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(0.1));
		mod_block.generate_dynamics(
		    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(0.1));

		auto reassign_pva_aux            = reassign_block.get_pva_aux();
		auto reassign_force_and_rate_aux = reassign_block.get_force_and_rate_aux();
		ASSERT_TRUE(*reassign_pva_aux == *pva_aux);
		ASSERT_FALSE(*reassign_force_and_rate_aux == *f_and_r_aux_reassign);

		auto mod_pva_aux            = mod_block.get_pva_aux();
		auto mod_force_and_rate_aux = mod_block.get_force_and_rate_aux();
		ASSERT_TRUE(*mod_pva_aux == *pva_aux);
		ASSERT_TRUE(*mod_force_and_rate_aux == *f_and_r_aux_mod);
	}
}

// Test for warning when unsupported type of aux data is passed
TEST_F(PinsonTests, testWrongAspnBaseVectorType) {
	AspnBaseVector aux_data{std::make_shared<FakeAux>()};
	EXPECT_WARN(block.receive_aux_data(aux_data), block.get_label());
}

class TestablePinson : public Pinson15NedBlock {
public:
	TestablePinson(const std::string& label,
	               ImuModel imu_model,
	               LinearizationPointFunction lin_function = nullptr,
	               navtk::filtering::DiscretizationStrategy discretization_strategy =
	                   &navtk::filtering::second_order_discretization_strategy,
	               navtk::not_null<std::shared_ptr<navtk::filtering::GravityModel>> gravity_model =
	                   make_shared<navtk::filtering::GravityModelSchwartz>())
	    : Pinson15NedBlock(label, imu_model, lin_function, discretization_strategy, gravity_model) {
	}

	TestablePinson(const TestablePinson& block) : Pinson15NedBlock(block) {}

	navtk::not_null<std::shared_ptr<StateBlock<>>> clone() {
		return make_shared<TestablePinson>(*this);
	}
};

// Validate clone() as implemented returns a deep copy by modifying all of the clone's properties
// and asserting they do not equal the original's. Note that lin_function is not tested.
TEST_F(PinsonTests, clone) {
	auto block = TestablePinson("block", hg1700_model());
	block.receive_aux_data(aux);
	block.receive_aux_data(aux);
	auto block_copy_cast = std::dynamic_pointer_cast<TestablePinson>(block.clone());
	auto& block_copy     = *block_copy_cast;

	ASSERT_EQ(block.get_label(), block_copy.get_label());

	ASSERT_EQ(block.get_num_states(), block_copy.get_num_states());

	ASSERT_EQ(block.get_imu_model().accel_random_walk_sigma[0],
	          block_copy.get_imu_model().accel_random_walk_sigma[0]);
	auto imu_edit = block_copy.get_imu_model();
	imu_edit.accel_random_walk_sigma[0] += 1;
	block_copy.receive_aux_data({std::make_shared<ImuModel>(imu_edit)});
	ASSERT_NE(block.get_imu_model().accel_random_walk_sigma[0],
	          block_copy.get_imu_model().accel_random_walk_sigma[0]);

	ASSERT_EQ(block.get_force_and_rate_aux()->get_meas_accel()[0],
	          block_copy.get_force_and_rate_aux()->get_meas_accel()[0]);
	ASSERT_EQ(block.get_pva_aux()->get_p1(), block_copy.get_pva_aux()->get_p1());
	auto aux_edit_pva     = *block_copy.get_pva_aux();
	auto aux_edit_f_and_r = *block_copy.get_force_and_rate_aux();
	aux_edit_f_and_r.set_meas_accel(aux_edit_f_and_r.get_meas_accel() + Vector{1, 0, 0});
	aux_edit_pva.set_p1(aux_edit_pva.get_p1() + 1);
	block_copy.receive_aux_data(
	    {std::make_shared<Pva>(aux_edit_pva), std::make_shared<Imu>(aux_edit_f_and_r)});
	ASSERT_NE(block.get_force_and_rate_aux()->get_meas_accel()[0],
	          block_copy.get_force_and_rate_aux()->get_meas_accel()[0]);
	ASSERT_NE(block.get_pva_aux()->get_p1(), block_copy.get_pva_aux()->get_p1());

	// Compare the function pointer address
	typedef std::pair<Matrix, Matrix>(FnType)(
	    const Matrix& F, const Matrix& G, const Matrix& Q, double dt);

	ASSERT_EQ((size_t)*block.get_discretization_strategy().template target<FnType*>(),
	          (size_t)*block_copy.get_discretization_strategy().template target<FnType*>());

	ASSERT_EQ(block.get_gravity_model(), block_copy.get_gravity_model());
}



// Validate position error states in m are accurately propagated so the units are valid in
// currently-estimated frame. Also compare some propagation qualities arising from different models.
TEST_F(PinsonTests, ScaleM_SLOW) {
	double start_lat         = 0.0;
	double stop_lat          = 1.0;
	double step              = 0.1;
	double initial_east_err  = 1.0;
	double initial_lon_err   = east_to_delta_lon(initial_east_err, start_lat, 0.0);
	double expected_east_err = delta_lon_to_east(initial_lon_err, stop_lat - step, 0.0);
	Vector err               = zeros(15);
	Vector err1700           = zeros(15);
	Matrix P                 = eye(15);
	Matrix P1700             = eye(15);
	err[1]                   = initial_east_err;
	err1700[1]               = initial_east_err;
	block.receive_aux_data(aux);

	for (double k = start_lat; k + .001 < stop_lat; k += step) {
		auto aux2 =
		    AspnBaseVector{std::make_shared<Pva>(*pva_aux), std::make_shared<Imu>(*f_and_r_aux)};
		auto pva = std::dynamic_pointer_cast<Pva>(aux2[0]);
		pva->set_p1(k);
		pva->set_p2(0);
		pva->set_p3(0);
		block.receive_aux_data(aux2);
		block1700.receive_aux_data(aux2);
		auto dyn = block.generate_dynamics(
		    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(10, 0));
		auto dyn1700 = block1700.generate_dynamics(
		    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(10, 0));
		err     = dot(dyn.Phi, err);
		err1700 = dot(dyn1700.Phi, err1700);
		P       = dot(dot(dyn.Phi, P), transpose(dyn.Phi)) + dyn.Qd;
		P1700   = dot(dot(dyn1700.Phi, P1700), transpose(dyn1700.Phi)) + dyn1700.Qd;
	}

	// Error propagation should be the same...
	ASSERT_NEAR(expected_east_err, err[1], 1e-12);
	ASSERT_NEAR(expected_east_err, err1700[1], 1e-12);

	// While the 9900 model should have lower variance all around
	ASSERT_TRUE(xt::all(xt::diagonal(P) <= xt::diagonal(P1700)));
}



TEST_F(PinsonTests, CompareToPreviousResult) {
	// clang-format off

	Matrix expected_phi{{1.00,  0.00,  0.00,  1.00,  0.00,  0.00,  0.00,  4.90,  0.00,  0.50,  0.00,  0.00,  0.00,  0.00,  0.00},
    {0.00,  1.00,  0.00,  0.00,  1.00,  0.00007292115147,  -4.90,  0.00,  0.00,  0.00,  0.50,  0.00,  0.00,  0.00,  0.00},
    {0.00000000814951,  0.00,  1.00000154384554,  0.00,  -0.00007292115147,  1.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.50,  0.00,  0.00,  0.00},
    {0.00,  0.00,  0.00,  0.99999922657297,  0.00,  0.00,  0.00,  9.80,  0.00035731364219,  0.99986111111111,  0.00,  0.00,  0.00,  -4.90,  0.00},
    {1.1885438536036494E-12,  0.00,  0.00000000022516,  0.00,  0.9999992211156,  0.00014584230293,  -9.80,  0.00,  0.00,  0.00,  0.99986111111111,  0.00007292115147,  4.90,  0.00,  0.00},
    {0.00000001629903,  0.00,  0.00000308769109,  0.00000000814951,  -0.00014584230293,  1.00000153321056,  0.00071462728438,  0.00,  0.00,  0.00,  -0.00007292115147,  0.99986111111111,  0.00,  0.00,  0.00},
    {0.00,  0.00,  0.00,  0.00,  0.00000015678559,  1.1432986068972804E-11,  0.99999923175059,  0.00,  0.00,  0.00,  0.0000000783928,  0.00,  -0.99986111111111,  0.00,  0.00},
    {-4.196626355780571E-16,  0.00,  0.00,  -0.00000015784225,  0.00,  0.00,  0.00,  0.99999922391423,  0.00007292115147,  -0.00000007892113,  0.00,  0.00,  0.00,  -0.99986111111111,  -0.00003646057573},
    {-1.151003864133914E-11,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  -0.00007292115147,  0.99999999734125,  0.00,  0.00,  0.00,  0.00,  0.00003646057573,  -0.99986111111111},
    {0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.99972226080247,  0.00,  0.00,  0.00,  0.00,  0.00},
    {0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.99972226080247,  0.00,  0.00,  0.00,  0.00},
    {0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.99972226080247,  0.00,  0.00,  0.00},
    {0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.99972226080247,  0.00,  0.00},
    {0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.99972226080247,  0.00},
    {0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.99972226080247}};

Matrix expected_qd{{8.24017197865543E-12,  0.00,  0.00,  1.6479183703404108E-11,  0.00,  4.074756876753095E-33,  0.00,  8.292350065673561E-13,  -6.046886652110622E-17,  8.351507939480248E-12,  0.00,  0.00,  0.00,  0.00,  0.00},
     {0.00,  8.240171978655426E-12,  0.00,  0.00,  1.6479183703404105E-11,  -1.2017656579392867E-15,  -8.292350174791127E-13,  0.00,  0.00,  0.00,  8.351507939480247E-12,  0.00, 0.00,  0.00,  0.00},
     {0.00,  0.00,  4.176914062500499E-12,  0.00,  6.091707660324459E-16,  8.352667871094247E-12,  0.00,  0.00,  0.00,  0.00,  0.00,  8.351507939480248E-12,  0.00,   0.00,   0.00},
     {1.6479183703404108E-11,  0.00,  0.00,  3.2956048653748266E-11,  0.00,  4.074753725226001E-33,  0.00,  1.6584703056165926E-12, -6.046887717994475E-17,  1.6700696015643975E-11,  0.00,  0.00,  0.00,  -3.91795E-19,  0.00 },
     {0.00,  1.6479183703404108E-11,  6.091707660324459E-16,  0.00,  3.295604872098478E-11,  -1.1851897838136817E-15,  -1.6584703230294208E-12,   0.00,  0.00,  0.00,  1.6700696015643975E-11,  1.2180031508653846E-15,  3.91795E-19,  0.00,  0.00},
     {4.074756876753095E-33,  -1.2017656579392869E-15,  8.352667871094247E-12,  4.0747537252260006E-33,  -1.185189783813682E-15,  1.6703015731937017E-11,  1.2093754462266956E-16,  -6.431687948141506E-40,  0.00,  0.00,  -1.2180031508653846E-15, 1.6700696015643975E-11, 0.00,  0.00,  0.00},
     {0.00,  -8.292350174791127E-13,   0.00,  0.00,  -1.6584703230294206E-12,  1.2093754462266956E-16,  3.3846359848335194E-13,  0.00,  0.00,  0.00,  1.3093961354985395E-18,  0.00,  -7.9947E-20,  0.00,  0.00},
     {8.29235006567356E-13,  0.00,  0.00,  1.6584703056165924E-12,  0.00,  -6.431687948141506E-40,  0.00,  3.3846359673092043E-13,  9.544541964032542E-24,  -1.3182208064880816E-18,  0.00,  0.00,  0.00, -7.9947E-20,  -2.1418676284950054E-24},
     {-6.046886652110622E-17,  0.00,  0.00,  -6.046887717994475E-17,  0.00,  0.00,  0.00,  9.544541964032542E-24,  3.384638585077646E-13,  0.00,  0.00,  0.00,  0.00,  2.915320E-24, -7.9947E-20},
     {8.351507939480248E-12,  0.00,  0.00,  1.6700696015643975E-11,  0.00,  0.00,  0.00,  -1.3182208064880814E-18,  0.00,  3.340603304673392E-11,  0.00,  0.00,  0.00,  0.00,  0.00},
     {0.00,  8.351507939480247E-12,  0.00,  0.00,  1.6700696015643975E-11,  -1.2180031508653845E-15,  1.3093961354985395E-18,  0.00,  0.00,  0.00,  3.340603304673392E-11,  0.00,  0.00,  0.00,  0.00},
     {0.00,  0.00,  8.351507939480248E-12,  0.00,  1.2180031508653845E-15,  1.6700696015643975E-11,  0.00,  0.00,  0.00,  0.00,  0.00,  3.340603304673392E-11,  0.00,   0.00,   0.00},
     {0.00,  0.00,  0.00,  0.00,   3.917949E-19,0.00,  -7.994703E-20,  0.00,  0.00,  0.00,  0.00,  0.00,  1.599163E-19,  0.00,  0.00},
     {0.00,  0.00,  0.00,  -3.917949E-19,  0.00,  0.00,  0.00,  -7.994703E-20,  2.915320E-24,  0.00,  0.00,  0.00,  0.00,  1.599163E-19,  0.00},
     {0.00,  0.00,  0.00,   0.00,  0.00,   0.00,  0.00,  -2.915320E-24, -7.994703E-20,  0.00,  0.00,  0.00,  0.00,  0.00,  1.599163E-19}};
	// clang-format on
	block.receive_aux_data(aux);
	auto dyn = block.generate_dynamics(
	    NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(1, 0));
	// Check is abs(actual - expected)) <= absolute_tolerance + expected*relative_tolerance
	// So these values result in a check of the percent difference
	auto absolute_tolerance = 1e-20;
	auto relative_tolerance = 1e-5;
	ASSERT_ALLCLOSE_EX(expected_phi, dyn.Phi, relative_tolerance, absolute_tolerance);
	ASSERT_ALLCLOSE_EX(expected_qd, dyn.Qd, relative_tolerance, absolute_tolerance);
}

TEST_F(PinsonTests, BadInput) {
	EXPECT_UB_OR_DIE(
	    block.generate_dynamics(
	        NULL_GEN_XHAT_AND_P_FUNCTION, to_type_timestamp(), to_type_timestamp(1, 0)),
	    "Pinson15 Cannot propagate",
	    std::runtime_error);
}

namespace {
std::pair<Vector, Matrix> lil_test_fun(const Vector& estimate,
                                       AspnBaseVector aux,
                                       navtk::Size num_loops) {
	auto b = std::make_shared<Pinson15NedBlock>(
	    "test", hg9900_model(), nullptr, &navtk::filtering::second_order_discretization_strategy);
	auto filter = navtk::filtering::StandardFusionEngine();
	filter.add_state_block(b);
	filter.give_state_block_aux_data(b->get_label(), aux);
	filter.set_state_block_estimate(b->get_label(), estimate);
	double dt = 100.0 / num_loops;
	for (navtk::Size k = 0; k < num_loops; k++) {
		filter.propagate(filter.get_time() + dt);
	}
	return std::pair<Vector, Matrix>(filter.get_state_block_estimate(b->get_label()),
	                                 filter.get_state_block_covariance(b->get_label()));
}
}  // namespace

TEST_F(PinsonTests, vary_rate_SLOW) {
	Vector estimate = navtk::ones(15) * 1e-12;
	auto res1       = lil_test_fun(estimate, aux, 50);
	auto res2       = lil_test_fun(estimate, aux, 200);
	ASSERT_ALLCLOSE(res1.first, res2.first);
	// Discretization must be full or cov equality fails due to linearization; unfortunately
	// it's much slower than 2nd order (~30 s on arm7 CI). Most terms are within the 2%
	// threshold except east/down tilt cross terms, which have about 20% but are near zero
	// (e-21), so we can safely accept that with abs offset
	ASSERT_ALLCLOSE_EX(res1.second, res2.second, 0.02, 1e-18);
}
