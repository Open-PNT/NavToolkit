#include <memory>

#include <gtest/gtest.h>
#include <scalar_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/filtering/virtualstateblocks/ChainedVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/EcefToStandard.hpp>
#include <navtk/filtering/virtualstateblocks/EcefToStandardQuat.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonErrorToStandard.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonErrorToStandardQuat.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonToSensor.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonToSensorLlh.hpp>
#include <navtk/filtering/virtualstateblocks/PlatformToSensorEcef.hpp>
#include <navtk/filtering/virtualstateblocks/PlatformToSensorEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/QuatToRpyPva.hpp>
#include <navtk/filtering/virtualstateblocks/ScaleVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/SensorToPlatformEcef.hpp>
#include <navtk/filtering/virtualstateblocks/SensorToPlatformEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/StandardToEcef.hpp>
#include <navtk/filtering/virtualstateblocks/StandardToEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/StateExtractor.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/navutils/leverarms.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/quaternions.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::TypeMounting;
using aspn_xtensor::TypeTimestamp;
using navtk::dot;
using navtk::Matrix;
using navtk::not_null;
using navtk::Vector;
using navtk::Vector3;
using navtk::filtering::ChainedVirtualStateBlock;
using navtk::filtering::EcefToStandard;
using navtk::filtering::EcefToStandardQuat;
using navtk::filtering::EstimateWithCovariance;
using navtk::filtering::NavSolution;
using navtk::filtering::PinsonErrorToStandard;
using navtk::filtering::PinsonErrorToStandardQuat;
using navtk::filtering::PinsonToSensor;
using navtk::filtering::PinsonToSensorLlh;
using navtk::filtering::PlatformToSensorEcef;
using navtk::filtering::PlatformToSensorEcefQuat;
using navtk::filtering::ScaleVirtualStateBlock;
using navtk::filtering::SensorToPlatformEcef;
using navtk::filtering::SensorToPlatformEcefQuat;
using navtk::filtering::StandardToEcef;
using navtk::filtering::StandardToEcefQuat;
using navtk::filtering::StateExtractor;
using navtk::filtering::VirtualStateBlock;
using navtk::navutils::dcm_to_quat;
using navtk::navutils::PI;
using navtk::navutils::rpy_to_dcm;
using std::make_shared;
using std::shared_ptr;
using std::vector;

struct PinsonVirtualStateBlockTests : public ::testing::Test {
	std::string pinson_name;
	std::string final_name;
	Vector3 nom_pos;
	Vector3 nom_vel;
	Vector3 nom_rpy;
	navtk::Matrix3 dcm;
	NavSolution sol;
	navtk::Vector4 inertial_quat;
	navtk::Vector4 sensor_quat;
	TypeMounting inertial_mount;
	TypeMounting sensor_mount;
	TypeMounting no_mount;
	Vector x;
	Matrix cov;
	EstimateWithCovariance ec;
	Vector ecef;
	double ecef_scale;

	PinsonVirtualStateBlockTests()
	    : ::testing::Test(),
	      pinson_name("pinson"),
	      final_name("fin"),
	      nom_pos({1.0, 2.0, 3.0}),
	      nom_vel({1.1, -2.2, 0.5}),
	      nom_rpy({0.7, 0.8, -1.9}),
	      dcm(xt::transpose(rpy_to_dcm(nom_rpy))),
	      sol(NavSolution(nom_pos, nom_vel, dcm, aspn_xtensor::TypeTimestamp((int64_t)0))),
	      inertial_quat(dcm_to_quat(xt::transpose(rpy_to_dcm(Vector{0.1, -0.6, 1.2})))),
	      sensor_quat(dcm_to_quat(xt::transpose(rpy_to_dcm(Vector{-1.0, 0.4, -2.1})))),
	      inertial_mount(TypeMounting(
	          Vector3{1.5, 5.2, 0.9}, navtk::zeros(3), inertial_quat, navtk::zeros(3, 3))),
	      sensor_mount(TypeMounting(
	          Vector3{0.7, -2.1, 7.6}, navtk::zeros(3), sensor_quat, navtk::zeros(3, 3))),
	      no_mount(TypeMounting(
	          navtk::zeros(3), navtk::zeros(3), Vector{1, 0, 0, 0}, navtk::zeros(3, 3))),
	      x(Vector{1.5,
	               2.0,
	               -3.0,
	               0.12,
	               -0.4,
	               1.1,
	               1e-4,
	               2e-5,
	               -3e-4,
	               1e-5,
	               1e-5,
	               1e-5,
	               1e-8,
	               1e-8,
	               1e-8}),
	      cov(xt::diag(Vector{1e-3,
	                          22.0,
	                          13.0,
	                          0.22,
	                          0.41,
	                          2.0,
	                          1e-8,
	                          2e-8,
	                          3e-8,
	                          1e-6,
	                          1e-6,
	                          1e-6,
	                          1e-6,
	                          1e-6,
	                          1e-6})),
	      ec(EstimateWithCovariance(x, cov)),
	      ecef(Vector{1909564.403496,
	                  4911689.178006,
	                  3581151.192942,
	                  -11.638386,
	                  3.732691,
	                  3.735638,
	                  -1.77689,
	                  -0.25674,
	                  -3.06957}),
	      ecef_scale(100000.0) {}

	// Returns a shared pointer containing a copy of the parameter via the assignment operator.
	not_null<shared_ptr<ChainedVirtualStateBlock>> init_chained_vsb_ptr_from_copy_assignment(
	    const ChainedVirtualStateBlock& other) {
		shared_ptr<ChainedVirtualStateBlock> chained_copy_assign;
		ChainedVirtualStateBlock* chained_copy_assign_temp =
		    new ChainedVirtualStateBlock({make_shared<EcefToStandard>("", "")});
		*chained_copy_assign_temp = other;
		chained_copy_assign.reset(chained_copy_assign_temp);
		return chained_copy_assign;
	}

	/*
	 * Constructs an 'all in one' VirtualStateBlock and a chained block, both converting from
	 * Pinson to double shifted whole state using the sensor mountings supplied, and checks
	 * for equivalancy between conversions and jacobians.
	 */
	void lever_arm_check(const TypeMounting& inertial, const TypeMounting& sensor) {
		auto ref_gen = [](const aspn_xtensor::TypeTimestamp& time) {
			Vector3 nom_pos{1.0, 2.0, 3.0};
			Vector3 nom_vel{1.1, -2.2, 0.5};
			Vector3 nom_rpy{0.7, 0.8, -1.9};
			auto dcm = xt::transpose(rpy_to_dcm(nom_rpy));
			return NavSolution(nom_pos, nom_vel, dcm, time);
		};

		auto complete = PinsonToSensorLlh(pinson_name, final_name, ref_gen, inertial, sensor);

		auto whole_only = make_shared<PinsonErrorToStandard>(pinson_name, "whole", ref_gen);

		auto to_ecef = make_shared<StandardToEcef>("whole", "i_ecef");

		auto scale_vector                       = navtk::ones(15);
		xt::view(scale_vector, xt::range(0, 3)) = ecef_scale;
		auto scale_it =
		    make_shared<ScaleVirtualStateBlock>("i_ecef", "ecef_scaled", 1.0 / scale_vector);

		auto inertial_to_body_ecef =
		    make_shared<SensorToPlatformEcef>("ecef_scaled", "b_ecef", inertial, ecef_scale);

		auto body_to_sensor_ecef =
		    make_shared<PlatformToSensorEcef>("b_ecef", "s_ecef", sensor, ecef_scale);

		auto unscale_it =
		    make_shared<ScaleVirtualStateBlock>("s_ecef", "ecef_unscaled", scale_vector);

		auto to_pva = make_shared<EcefToStandard>("ecef_unscaled", "sensor_whole");

		auto sensor_to_pos = make_shared<StateExtractor>(
		    "sensor_whole", final_name, 15, vector<navtk::Size>{0, 1, 2});

		auto chained = make_shared<ChainedVirtualStateBlock>(
		    vector<not_null<shared_ptr<VirtualStateBlock>>>{whole_only,
		                                                    to_ecef,
		                                                    scale_it,
		                                                    inertial_to_body_ecef,
		                                                    body_to_sensor_ecef,
		                                                    unscale_it,
		                                                    to_pva,
		                                                    sensor_to_pos});
		auto chained_copy_ctr    = make_shared<ChainedVirtualStateBlock>(*chained);
		auto chained_copy_assign = init_chained_vsb_ptr_from_copy_assignment(*chained);

		auto straight = complete.convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));
		auto jac =
		    complete.jacobian(ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0));  // NOT TUNED

		// Define the test
		auto test = [this, &straight, &jac](shared_ptr<ChainedVirtualStateBlock> chained) {
			auto chn  = chained->convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));
			auto jac2 = chained->jacobian(ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0));
			ASSERT_ALLCLOSE(straight.estimate, chn.estimate);
			ASSERT_ALLCLOSE(straight.covariance, chn.covariance);
			loose_altitude_test(jac, jac2, ec.estimate);
		};

		// Test the original
		test(chained);

		// Destroy the original and test the copies
		chained.reset();
		test(chained_copy_ctr);
		test(chained_copy_assign);
	}

	void bad_num_check(std::function<NavSolution(aspn_xtensor::TypeTimestamp)> ref_fun) {
		auto complete =
		    PinsonToSensor(pinson_name, final_name, ref_fun, inertial_mount, sensor_mount);

		auto whole_only = make_shared<PinsonErrorToStandardQuat>(pinson_name, "whole", ref_fun);

		auto to_ecef = make_shared<StandardToEcefQuat>("whole", "i_ecef");

		auto scale_vector                       = navtk::ones(16);
		xt::view(scale_vector, xt::range(0, 3)) = ecef_scale;
		auto scale_it =
		    make_shared<ScaleVirtualStateBlock>("i_ecef", "ecef_scaled", 1.0 / scale_vector);

		auto inertial_to_body_ecef = make_shared<SensorToPlatformEcefQuat>(
		    "ecef_scaled", "b_ecef", inertial_mount, ecef_scale);

		auto body_to_sensor_ecef =
		    make_shared<PlatformToSensorEcefQuat>("b_ecef", "s_ecef", sensor_mount, ecef_scale);

		auto unscale_it =
		    make_shared<ScaleVirtualStateBlock>("s_ecef", "ecef_unscaled", scale_vector);

		auto to_pva = make_shared<EcefToStandardQuat>("ecef_unscaled", "sensor_whole");
		auto to_rpy = make_shared<navtk::filtering::QuatToRpyPva>("sensor_whole", "sensor_rpy");


		auto chained = make_shared<ChainedVirtualStateBlock>(
		    vector<not_null<shared_ptr<VirtualStateBlock>>>{whole_only,
		                                                    to_ecef,
		                                                    scale_it,
		                                                    inertial_to_body_ecef,
		                                                    body_to_sensor_ecef,
		                                                    unscale_it,
		                                                    to_pva,
		                                                    to_rpy});
		auto chained_copy_ctr    = make_shared<ChainedVirtualStateBlock>(*chained);
		auto chained_copy_assign = init_chained_vsb_ptr_from_copy_assignment(*chained);

		auto cov = xt::diag(Vector{
		    1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 1e-4, 2e-4, 3e-4, 1e-2, 2e-2, 3e-2, 1e-5, 2e-5, 3e-5});
		EstimateWithCovariance ec(navtk::zeros(15), cov);
		auto straight = complete.convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));

		// Define the test
		auto test = [&ec, &straight](shared_ptr<ChainedVirtualStateBlock> chained) {
			auto chn = chained->convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));
			ASSERT_ALLCLOSE(straight.estimate, chn.estimate);
			ASSERT_FALSE(xt::any(xt::isnan(chn.covariance)));
			ASSERT_FALSE(xt::any(xt::isinf(chn.covariance)));
		};


		// Test the original
		test(chained);

		// Destroy the original and test the copies
		chained.reset();
		test(chained_copy_ctr);
		test(chained_copy_assign);
	}

	/*
	 * Verifies that the numerical jacobian matches the output of the VirtualStateBlocks jacobian().
	 * @param alias VSB to test
	 * @param xx n state vector to transform
	 * @param eps n length step sizes to use in numerical jacobain calculation
	 * @param rtol Relative tolerance to allow
	 * @param atol absolute tolerance to allow
	 */
	void wrap_test(const shared_ptr<VirtualStateBlock> alias,
	               const Vector& xx,
	               const Vector& eps,
	               double rtol = 1e-5,
	               double atol = 1e-8) {

		auto fx = [alias = alias](Vector x) {
			auto dummy_cov = navtk::zeros(navtk::num_rows(x), navtk::num_rows(x));
			auto ec        = EstimateWithCovariance(x, dummy_cov);
			auto cvt       = alias->convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));
			return cvt.estimate;
		};

		auto jac = alias->jacobian(xx, aspn_xtensor::TypeTimestamp((int64_t)0));
		auto num = navtk::filtering::calc_numerical_jacobian(fx, xx, eps);
		ASSERT_ALLCLOSE_EX(jac, num, rtol, atol);
	}

	/*
	 * Verifies that 2 VirtualStateBlocks perform inverse operations (both the convert functions and
	 * jacobians).
	 */
	void inverse_test(const not_null<shared_ptr<VirtualStateBlock>> alias1,
	                  const not_null<shared_ptr<VirtualStateBlock>> alias2,
	                  const Vector& xx,
	                  const Matrix& cov,
	                  const Vector& eps,
	                  double rtol = 1e-5,
	                  double atol = 1e-8) {


		EstimateWithCovariance ec(xx, cov);

		auto chained = make_shared<ChainedVirtualStateBlock>(
		    vector<not_null<shared_ptr<VirtualStateBlock>>>{alias1, alias2});
		auto chained_copy_ctr    = make_shared<ChainedVirtualStateBlock>(*chained);
		auto chained_copy_assign = init_chained_vsb_ptr_from_copy_assignment(*chained);

		// Check 'manual' chaining of conversions for inverse behavior
		ASSERT_ALLCLOSE(alias2
		                    ->convert(alias1->convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0)),
		                              aspn_xtensor::TypeTimestamp((int64_t)0))
		                    .estimate,
		                xx);
		// In general, original covariance recovery is not going to work
		ASSERT_ALLCLOSE(alias1->jacobian(xx, aspn_xtensor::TypeTimestamp((int64_t)0)),
		                navtk::inverse(alias2->jacobian(
		                    alias1->convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0)).estimate,
		                    aspn_xtensor::TypeTimestamp((int64_t)0))));

		// Define the test
		auto test =
		    [&ec, this, &xx, &eps, &rtol, &atol](shared_ptr<ChainedVirtualStateBlock> chained) {
			    // As a chained VSB
			    ASSERT_ALLCLOSE(
			        chained->convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0)).estimate, xx);
			    ASSERT_ALLCLOSE(chained->jacobian(xx, aspn_xtensor::TypeTimestamp((int64_t)0)),
			                    navtk::eye(navtk::num_rows(xx)));
			    // Check the chained block for numerical accuracy.
			    wrap_test(chained, xx, eps, rtol, atol);
		    };


		// Test the original
		test(chained);

		// Destroy the original and test the copies
		chained.reset();
		test(chained_copy_ctr);
		test(chained_copy_assign);
	}

	/*
	 * For alias testing that involves both a) non-zero lever arm corrections and b)
	 * post-lever arm corrected output being transformed from ecef to lla.
	 * The transform from ecef to lla introduces a small amount of error into the altitude term
	 * on the order of 1e-9. While deterministic on a per-input basis, we have to treat it as random
	 * (since we can't map the input/output for the whole space).
	 * This change in output is incorrectly 'attributed' to the delta input in the numerical
	 * jacobian calculation. As the net effect is relatively small (the 'full' amount of altitude
	 * difference between numerical and hand jacobians is on the order of 1e-6), there is no
	 * workaround for this issue at the moment, and the problem lies with the 'truth' value being
	 * used to verify the implementations of hand-calculated jacobians and not the ones a user will
	 * typically use, the altitude row is basically not checked.
	 */
	void loose_altitude_test(const Matrix& actual, const Matrix& expected, const Vector& x) {
		ASSERT_ALLCLOSE(xt::view(actual, xt::range(0, 1), xt::all()),
		                xt::view(expected, xt::range(0, 1), xt::all()));
		ASSERT_ALLCLOSE(xt::view(actual, xt::range(3, navtk::num_rows(actual)), xt::all()),
		                xt::view(expected, xt::range(3, navtk::num_rows(expected)), xt::all()));

		auto err = navtk::dot(xt::view(actual, 2, xt::all()) - xt::view(expected, 2, xt::all()), x);
		// WholeStateComp has 3e-6 total err based on jac diff since the baseline cuts out a bunch
		// of calculations- rest are smaller (e-7)
		ASSERT_NEAR_EX(0, err[0], 1e-9, 3e-6);
	}
};

TEST_F(PinsonVirtualStateBlockTests, PinsonErrToWholeWrap_SLOW) {
	Vector x_scales                     = navtk::ones(navtk::num_rows(x)) * 1e-3;
	xt::view(x_scales, xt::range(6, 9)) = 5e-5;

	auto ref_gen = [](const aspn_xtensor::TypeTimestamp& time) {
		Vector3 nom_pos{1.0, 2.0, 3.0};
		Vector3 nom_vel{1.1, -2.2, 0.5};
		Vector3 nom_rpy{0.7, 0.8, -1.9};
		auto dcm = xt::transpose(rpy_to_dcm(nom_rpy));
		return NavSolution(nom_pos, nom_vel, dcm, time);
	};

	wrap_test(make_shared<PinsonErrorToStandard>("a", "b", ref_gen), x, x_scales * x);
}

TEST_F(PinsonVirtualStateBlockTests, PvaToEcefWrap_SLOW) {
	Vector x_pva{0.6, 1.2, 123.0, 3.5, 12.2, -1.5, 0.5, -0.2, 1.1};
	Vector x_scales                     = navtk::ones(navtk::num_rows(x_pva)) * 1e-3;
	xt::view(x_scales, xt::range(6, 9)) = 1e-5;
	wrap_test(make_shared<StandardToEcef>("a", "b"), x_pva, x_scales * x_pva);
}

TEST_F(PinsonVirtualStateBlockTests, EcefToBodyWrap_SLOW) {
	/*
	 * The 'hand-generated' jacobian from the lever-arm correcting VirtualStateBlocks do not pass
	 * the 'assert_allclose' test with default tolerances. This turns out to be a limit on the
	 * accuracy of the numerical jacobian caused by the presence of ECEF scale positions.
	 * ECEF coordinates (in meters) are e7 magnitude. RPY perturbation and the resulting
	 * position perturbation from lever arm correction is about 1e-5 (for 1e-5 rad perturbation and
	 * meter-level lever arm). For standard double, this leaves between 4 and 6 decimal places
	 * to represent the 'delta' in the lever arm corrected positions when calculating the numerical
	 * jacobian, and after dividing the diff by twice the perturbation to get the local derivative
	 * only 3-5 decimal places of accuracy can be assumed. As the hand calculated jacobian accounts
	 * for the fact that the ECEF position goes away in the derivative, it can be approximately 7
	 * decimal places more accurate.
	 *
	 * Therefore, an ASSERT_ALLCLOSE type comparison between the numerical jacobian and the output
	 * of the jacobian() function must loosen the default tolerances slightly. Alternatively, one
	 * could reduce the ECEF position in the state vector by a couple orders of magnitude to account
	 * for the offset- an ecef position of 0 will make the numerical match the hand calculated.
	 */
	Vector x_scales                     = navtk::ones(navtk::num_rows(ecef)) * 1e-3;
	xt::view(x_scales, xt::range(6, 9)) = 1e-5;
	wrap_test(make_shared<SensorToPlatformEcef>("a", "b", inertial_mount),
	          ecef,
	          x_scales * ecef,
	          1e-5,
	          1e-4);
}

// Shows the 'less error when ECEF 0' as mentioned in above comments
TEST_F(PinsonVirtualStateBlockTests, EcefToBodyZeroPosWrap_SLOW) {
	Vector ecef{0, 0, 0, -11.638386, 3.732691, 3.735638, -1.77689, -0.25674, -3.06957};
	Vector x_scales                     = navtk::ones(navtk::num_rows(ecef)) * 1e-3;
	xt::view(x_scales, xt::range(6, 9)) = 1e-5;
	Vector eps                          = x_scales * ecef;
	xt::view(eps, xt::range(0, 3))      = 1e-3;  // No 0s in eps allowed; divide by 0
	wrap_test(make_shared<SensorToPlatformEcef>("a", "b", inertial_mount), ecef, eps, 1e-7, 1e-10);
}

TEST_F(PinsonVirtualStateBlockTests, BodyToSensorWrap_SLOW) {
	Vector x_scales                     = navtk::ones(navtk::num_rows(ecef)) * 1e-3;
	xt::view(x_scales, xt::range(6, 9)) = 1e-5;
	Vector eps                          = x_scales * ecef;
	wrap_test(make_shared<PlatformToSensorEcef>("a", "b", sensor_mount), ecef, eps, 1e-5, 1e-4);
}

TEST_F(PinsonVirtualStateBlockTests, BodyToSensorZeroPosWrap_SLOW) {
	Vector ecef{0, 0, 0, -11.638386, 3.732691, 3.735638, -1.77689, -0.25674, -3.06957};
	Vector x_scales                     = navtk::ones(navtk::num_rows(ecef)) * 1e-3;
	xt::view(x_scales, xt::range(6, 9)) = 1e-5;
	Vector eps                          = x_scales * ecef;
	xt::view(eps, xt::range(0, 3))      = 1e-3;
	wrap_test(make_shared<PlatformToSensorEcef>("a", "b", sensor_mount), ecef, eps, 1e-7, 1e-10);
}

TEST_F(PinsonVirtualStateBlockTests, EcefToStandardWrap_SLOW) {
	Vector x_scales = navtk::ones(navtk::num_rows(ecef)) * 1e-3;
	wrap_test(make_shared<EcefToStandard>("a", "b"), ecef, x_scales * ecef);
}

TEST_F(PinsonVirtualStateBlockTests, LeverLlaInverse_SLOW) {
	Vector pva_v{1.0, 2.0, 300.9, -1.4, 5.9, 20.2, -1.3, -0.4, 2.1};
	Matrix pva_cov        = xt::diag(Vector{1e-8, 3e-7, 10.0, 2.0, 0.5, 0.4, 1e-4, 1e-8, 1e-6});
	auto to_ecef          = make_shared<StandardToEcef>("whole", "i_ecef");
	auto inertial_to_body = make_shared<SensorToPlatformEcef>("i_ecef", "b_ecef", inertial_mount);
	auto body_to_sensor   = make_shared<PlatformToSensorEcef>("b_ecef", "s_ecef", inertial_mount);
	auto sensor_to_pva    = make_shared<EcefToStandard>("s_ecef", "sensor_whole");

	auto chained1 = make_shared<ChainedVirtualStateBlock>(
	    vector<not_null<shared_ptr<VirtualStateBlock>>>{to_ecef, inertial_to_body});
	auto chained1_copy_assign = init_chained_vsb_ptr_from_copy_assignment(*chained1);
	auto chained1_copy_ctr    = make_shared<ChainedVirtualStateBlock>(*chained1);

	auto chained2 = make_shared<ChainedVirtualStateBlock>(
	    vector<not_null<shared_ptr<VirtualStateBlock>>>{body_to_sensor, sensor_to_pva});
	auto chained2_copy_ctr    = make_shared<ChainedVirtualStateBlock>(*chained2);
	auto chained2_copy_assign = init_chained_vsb_ptr_from_copy_assignment(*chained2);

	// Test the originals after cloning
	inverse_test(chained1, chained2, pva_v, pva_cov, pva_v * 1e-3, 1e-5, 1e-6);

	// Destroy the originals and test the copies
	chained1.reset();
	chained2.reset();
	inverse_test(chained1_copy_ctr, chained2_copy_ctr, pva_v, pva_cov, pva_v * 1e-3, 1e-5, 1e-6);
	inverse_test(
	    chained1_copy_assign, chained2_copy_assign, pva_v, pva_cov, pva_v * 1e-3, 1e-5, 1e-6);
}

TEST_F(PinsonVirtualStateBlockTests, LlaEcefInverse_SLOW) {
	Vector pva_v{1.0, 2.0, 300.9, -1.4, 5.9, 20.2, -1.3, -0.4, 2.1};
	Matrix pva_cov = xt::diag(Vector{1e-8, 3e-7, 10.0, 2.0, 0.5, 0.4, 1e-4, 1e-8, 1e-6});
	auto to_ecef   = make_shared<StandardToEcef>("whole", "i_ecef");
	auto to_pva    = make_shared<EcefToStandard>("i_ecef", "sensor_whole");
	inverse_test(to_ecef, to_pva, pva_v, pva_cov, pva_v * 1e-3, 1e-5, 1e-6);
}

TEST_F(PinsonVirtualStateBlockTests, LeverEcefInverse_SLOW) {
	auto inertial_to_body = make_shared<SensorToPlatformEcef>("i_ecef", "b_ecef", inertial_mount);
	auto body_to_sensor   = make_shared<PlatformToSensorEcef>("b_ecef", "s_ecef", inertial_mount);
	Matrix ecef_cov       = xt::diag(Vector{5.0, 10.0, 30.0, 2.0, 0.5, 0.4, 1e-4, 1e-8, 1e-6});
	inverse_test(inertial_to_body, body_to_sensor, ecef, ecef_cov, ecef * 1e-3);
}

TEST_F(PinsonVirtualStateBlockTests, WholeStateComp_SLOW) {

	auto ref_gen = [](const aspn_xtensor::TypeTimestamp& time) {
		Vector3 nom_pos{1.0, 2.0, 3.0};
		Vector3 nom_vel{1.1, -2.2, 0.5};
		Vector3 nom_rpy{0.7, 0.8, -1.9};
		auto dcm = xt::transpose(rpy_to_dcm(nom_rpy));
		return NavSolution(nom_pos, nom_vel, dcm, time);
	};

	auto complete      = PinsonToSensorLlh(pinson_name, final_name, ref_gen, no_mount, no_mount);
	auto whole_only    = PinsonErrorToStandard(pinson_name, "whole", ref_gen);
	auto sensor_to_pos = StateExtractor("whole", final_name, 15, {0, 1, 2});

	auto straight    = complete.convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));
	auto converted_1 = whole_only.convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));
	auto chained     = sensor_to_pos.convert(converted_1, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(straight.estimate, chained.estimate);
	ASSERT_ALLCLOSE(straight.covariance, chained.covariance);

	auto jac1 = complete.jacobian(ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0));
	auto jac2 =
	    dot(sensor_to_pos.jacobian(converted_1.estimate, aspn_xtensor::TypeTimestamp((int64_t)0)),
	        whole_only.jacobian(ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0)));
	loose_altitude_test(jac1, jac2, ec.estimate);
}

TEST_F(PinsonVirtualStateBlockTests, compare_with_applyerror_SLOW) {

	auto ref_gen = [](const aspn_xtensor::TypeTimestamp& time) {
		Vector3 nom_pos{1.0, 2.0, 3.0};
		Vector3 nom_vel{1.1, -2.2, 0.5};
		Vector3 nom_rpy{0.7, 0.8, -1.9};
		auto dcm = xt::transpose(rpy_to_dcm(nom_rpy));
		return NavSolution(nom_pos, nom_vel, dcm, time);
	};

	auto whole_only = PinsonErrorToStandard(pinson_name, "whole", ref_gen);

	auto alt = navtk::filtering::apply_error_states<navtk::filtering::Pinson15NedBlock>(
	    ref_gen(aspn_xtensor::TypeTimestamp((int64_t)0)), ec.estimate);
	auto converted = whole_only.convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));

	ASSERT_ALLCLOSE(alt.pos, xt::view(converted.estimate, xt::range(0, 3)));
	ASSERT_ALLCLOSE(alt.vel, xt::view(converted.estimate, xt::range(3, 6)));
	ASSERT_ALLCLOSE(alt.rot_mat,
	                xt::transpose(rpy_to_dcm(xt::view(converted.estimate, xt::range(6, 9)))));
}

/*
 * For following test. Just outputs position in ecef rather than converting it to llh.
 * When that conversion is excluded, then the numerical jacobian of the fx()
 * member of this class matches the chained individual manual jacobians of each
 * component transform. When output is converted to LLH, altitude jacobian terms
 * differ, e.g.
 * (jac2(2, 0) = -1.145e-16;  // jac1(2, 0) == 3.21639e-07
 * jac2(2, 1) = 6.79711e-19;  // jac1(2, 1) == -6.98115e-08)
 * but these terms have little effect on the output (1e-6 level altitude diff)
 * and can be safely alllowed into test.
 */
class PinsonToSensorStandardEcef : public navtk::filtering::NumericalVirtualStateBlock {
public:
	PinsonToSensorStandardEcef(
	    const std::string& current,
	    const std::string& target,
	    std::function<NavSolution(const aspn_xtensor::TypeTimestamp&)> ref_fun,
	    const TypeMounting& inertial_mount,
	    const TypeMounting& sensor_mount,
	    double scale_factor = 1.0)
	    : navtk::filtering::NumericalVirtualStateBlock(current, target),
	      ref_fun(ref_fun),
	      inertial_mount(inertial_mount),
	      sensor_mount(sensor_mount),
	      scale_factor(scale_factor) {}

	PinsonToSensorStandardEcef(const PinsonToSensorStandardEcef& other)
	    : NumericalVirtualStateBlock(other.current, other.target),
	      ref_fun(other.ref_fun),
	      inertial_mount(other.inertial_mount),
	      sensor_mount(other.sensor_mount),
	      scale_factor(other.scale_factor) {}

	not_null<shared_ptr<VirtualStateBlock>> clone() override {
		return make_shared<PinsonToSensorStandardEcef>(*this);
	}

protected:
	Vector fx(const Vector& x, const aspn_xtensor::TypeTimestamp& time) override {
		auto sol = ref_fun(time);

		auto delta_lat  = navtk::navutils::north_to_delta_lat(x(0), sol.pos(0), sol.pos(2));
		auto delta_lon  = navtk::navutils::east_to_delta_lon(x(1), sol.pos(0), sol.pos(2));
		auto delta_alt  = -x(2);
		Vector corr_pos = Vector3{delta_lat, delta_lon, delta_alt} + sol.pos;

		auto sensor_pos_ecef = navtk::navutils::llh_to_ecef(corr_pos);
		auto C_nav_to_sensor =
		    dot(sol.rot_mat, navtk::eye(3) + navtk::navutils::skew(xt::view(x, xt::range(6, 9))));
		auto C_nav_to_ecef = navtk::navutils::llh_to_cen(corr_pos);

		auto pos_meas_at_platform_ecef = navtk::navutils::sensor_to_platform(
		    std::pair<Vector3, navtk::Matrix3>(navtk::zeros(3), C_nav_to_sensor),
		    inertial_mount.get_lever_arm(),
		    navtk::navutils::quat_to_dcm(inertial_mount.get_orientation_quaternion()),
		    C_nav_to_ecef);
		auto pos_meas_at_sensor_ecef = navtk::navutils::platform_to_sensor(
		    pos_meas_at_platform_ecef,
		    sensor_mount.get_lever_arm(),
		    navtk::navutils::quat_to_dcm(sensor_mount.get_orientation_quaternion()),
		    C_nav_to_ecef);
		return (pos_meas_at_sensor_ecef.first + sensor_pos_ecef) / scale_factor;
	}

private:
	std::function<NavSolution(const aspn_xtensor::TypeTimestamp&)> ref_fun = 0;
	TypeMounting inertial_mount;
	TypeMounting sensor_mount;
	double scale_factor;
};

TEST_F(PinsonVirtualStateBlockTests, OnlyToEcef_SLOW) {

	auto ref_gen = [](const aspn_xtensor::TypeTimestamp& time) {
		Vector3 nom_pos{1.0, 2.0, 3.0};
		Vector3 nom_vel{1.1, -2.2, 0.5};
		Vector3 nom_rpy{0.7, 0.8, -1.9};
		auto dcm = xt::transpose(rpy_to_dcm(nom_rpy));
		return NavSolution(nom_pos, nom_vel, dcm, time);
	};

	auto complete = PinsonToSensorStandardEcef(
	    pinson_name, final_name, ref_gen, inertial_mount, sensor_mount, ecef_scale);
	auto whole_only = make_shared<PinsonErrorToStandard>(pinson_name, "whole", ref_gen);
	auto to_ecef    = make_shared<StandardToEcef>("whole", "i_ecef");

	auto scale_vector                       = navtk::ones(15);
	xt::view(scale_vector, xt::range(0, 3)) = ecef_scale;
	auto scale_it =
	    make_shared<ScaleVirtualStateBlock>("i_ecef", "ecef_scaled", 1.0 / scale_vector);

	auto inertial_to_body =
	    make_shared<SensorToPlatformEcef>("ecef_scaled", "b_ecef", inertial_mount, ecef_scale);
	auto body_to_sensor =
	    make_shared<PlatformToSensorEcef>("b_ecef", "s_ecef", sensor_mount, ecef_scale);
	auto sensor_to_pos =
	    make_shared<StateExtractor>("sensor_whole", final_name, 15, vector<navtk::Size>{0, 1, 2});

	auto fx = [alias = &complete](Vector x) {
		auto dummy_cov = navtk::zeros(navtk::num_rows(x), navtk::num_rows(x));
		auto ec        = EstimateWithCovariance(x, dummy_cov);
		auto cvt       = alias->convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));
		return cvt.estimate;
	};

	// North tilt term apparently very nonlinear in this region; numerical jac needed tuned
	Vector eps1{1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 5e-7, 1e-5, 1e-5, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
	auto jac1 = xt::view(navtk::filtering::calc_numerical_jacobian(fx, ec.estimate, eps1),
	                     xt::range(0, 3),
	                     xt::all());

	auto chained =
	    make_shared<ChainedVirtualStateBlock>(vector<not_null<shared_ptr<VirtualStateBlock>>>{
	        whole_only, to_ecef, scale_it, inertial_to_body, body_to_sensor, sensor_to_pos});
	auto chained_copy_ctr    = make_shared<ChainedVirtualStateBlock>(*chained);
	auto chained_copy_assign = init_chained_vsb_ptr_from_copy_assignment(*chained);

	auto straight = complete.convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));

	// Define the test
	auto test = [this, &straight, &jac1](shared_ptr<ChainedVirtualStateBlock> chained) {
		auto chn = chained->convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));
		auto jac = chained->jacobian(ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(straight.estimate, chn.estimate);
		ASSERT_ALLCLOSE(straight.covariance, chn.covariance);
		ASSERT_ALLCLOSE_EX(jac1, jac, 1e-5, 1e-7);
	};

	// Test the original
	test(chained);

	// Destroy the original and test the copies
	chained.reset();
	test(chained_copy_ctr);
	test(chained_copy_assign);
}

TEST_F(PinsonVirtualStateBlockTests, InertialLeverArmComp_SLOW) {
	lever_arm_check(inertial_mount, no_mount);
}

TEST_F(PinsonVirtualStateBlockTests, SensorLeverArmComp_SLOW) {
	lever_arm_check(no_mount, sensor_mount);
}

TEST_F(PinsonVirtualStateBlockTests, AllComp_SLOW) {
	lever_arm_check(inertial_mount, sensor_mount);
}

TEST_F(PinsonVirtualStateBlockTests, NoOpComp_SLOW) { lever_arm_check(no_mount, no_mount); }

TEST_F(PinsonVirtualStateBlockTests, BadPitchAngles_SLOW) {
	// 0 latitude causes 0 in denominators of jacobians
	Matrix ll_test{{0, PI / 2, 0, 0, 0, 0, 0, 0, 0},
	               {0, PI, 0, 0, 0, 0, 0, 0, 0},
	               {0, -PI / 2, 0, 0, 0, 0, 0, 0, 0},
	               {1e-8, -PI, 0, 0, 0, 0, 0, 0, 0},
	               {1e-9, -PI, 0, 0, 0, 0, 0, 0, 0},
	               {0, 0, 0, 0, 0, 0, 0, PI / 2, 0},
	               {0, 0, 0, 0, 0, 0, 0, -PI / 2, 0},
	               {0, PI / 2, 0, 0, 0, 0, 0, PI / 2, 0}};

	for (navtk::Size r = 0; r < navtk::num_rows(ll_test); r++) {
		auto v   = xt::view(ll_test, r, xt::all());
		auto eps = v * 1e-3 + 1e-9;

		auto reff = [v = v](aspn_xtensor::TypeTimestamp) {
			return NavSolution(xt::view(v, xt::range(0, 3)),
			                   xt::view(v, xt::range(3, 6)),
			                   xt::transpose(rpy_to_dcm(xt::view(v, xt::range(6, 9)))),
			                   aspn_xtensor::TypeTimestamp((int64_t)0));
		};

		bad_num_check(reff);
	}
}

TEST(quat_testing, QuatRotatedVectorJacobian) {
	Vector3 rpy{1.2, -0.75, -0.6};
	Vector rb{1212.0, 19.34, -652.0};
	auto q    = navtk::navutils::rpy_to_quat(rpy);
	auto hand = navtk::navutils::quat_to_dcm(q);

	auto fx = [q = q](const Vector& x) {
		Vector res = navtk::navutils::quat_rot(q, x);
		return res;
	};

	auto num = navtk::filtering::calc_numerical_jacobian(fx, rb);
	ASSERT_ALLCLOSE(hand, num);
}

TEST_F(PinsonVirtualStateBlockTests, PinsonErrToWholeQuatWrap_SLOW) {
	Vector x_scales                     = navtk::ones(navtk::num_rows(x)) * 1e-3;
	xt::view(x_scales, xt::range(6, 9)) = 1e-5;
	auto ref_gen                        = [](const aspn_xtensor::TypeTimestamp& time) {
        Vector3 nom_pos{1.0, 2.0, 3.0};
        Vector3 nom_vel{1.1, -2.2, 0.5};
        Vector3 nom_rpy{0.7, 0.8, -1.9};
        auto dcm = xt::transpose(rpy_to_dcm(nom_rpy));
        return NavSolution(nom_pos, nom_vel, dcm, time);
	};
	wrap_test(make_shared<PinsonErrorToStandardQuat>("a", "b", ref_gen), x, x_scales * x);
}

TEST_F(PinsonVirtualStateBlockTests, QuatRpyWrap_SLOW) {
	auto q = navtk::navutils::rpy_to_quat(Vector3{-1.77689, -0.25674, -3.06957});
	Vector v{0.6, -1.4, 1234.0, -11.638386, 3.732691, 3.735638, q(0), q(1), q(2), q(3)};

	Vector x_scales                      = navtk::ones(navtk::num_rows(v)) * 1e-3;
	xt::view(x_scales, xt::range(6, 10)) = 1e-5;
	wrap_test(make_shared<navtk::filtering::QuatToRpyPva>("a", "b"), v, x_scales * v);
}

TEST_F(PinsonVirtualStateBlockTests, PinsonErrToWholeQuatComp_SLOW) {
	auto ref_gen = [](const aspn_xtensor::TypeTimestamp& time) {
		Vector3 nom_pos{1.0, 2.0, 3.0};
		Vector3 nom_vel{1.1, -2.2, 0.5};
		Vector3 nom_rpy{0.7, 0.8, -1.9};
		auto dcm = xt::transpose(rpy_to_dcm(nom_rpy));
		return NavSolution(nom_pos, nom_vel, dcm, time);
	};
	auto a1      = make_shared<PinsonErrorToStandardQuat>("a", "b", ref_gen);
	auto a2      = make_shared<navtk::filtering::QuatToRpyPva>("b", "c");
	auto b1      = PinsonErrorToStandard("a", "c", ref_gen);
	auto chained = make_shared<ChainedVirtualStateBlock>(
	    vector<not_null<shared_ptr<VirtualStateBlock>>>{a1, a2});
	auto chained_copy_ctr    = make_shared<ChainedVirtualStateBlock>(*chained);
	auto chained_copy_assign = init_chained_vsb_ptr_from_copy_assignment(*chained);

	auto cvt2 = b1.convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));
	auto jac2 = b1.jacobian(ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0));

	// Define the test
	auto test = [this, &cvt2, &jac2](shared_ptr<ChainedVirtualStateBlock> chained) {
		auto cvt = chained->convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));
		auto jac = chained->jacobian(ec.estimate, aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(cvt.estimate, cvt2.estimate);
		ASSERT_ALLCLOSE(cvt.covariance, cvt2.covariance);
		// Absolute error on order of 1e-8 across terms; 2 terms failing are 3-4 orders of magnitude
		// smaller that other jac terms and failing based on relative error level on the order of
		// e-4; Test thresholds adjusted to accept this minor diff
		ASSERT_ALLCLOSE_EX(jac, jac2, 1e-9, 1e-4);
	};

	// Test the original
	test(chained);

	// Destroy the original and test the copies
	chained.reset();
	test(chained_copy_ctr);
	test(chained_copy_assign);
}

TEST_F(PinsonVirtualStateBlockTests, InverseQuat_SLOW) {
	auto q = navtk::navutils::rpy_to_quat(Vector3{-1.77689, -0.25674, -3.06957});
	Vector v{0.6, -1.4, 1234.0, -11.638386, 3.732691, 3.735638, q(0), q(1), q(2), q(3)};

	auto q_to_e = make_shared<StandardToEcefQuat>("i_ecef", "b_ecef");
	auto e_to_q = make_shared<EcefToStandardQuat>("b_ecef", "s_ecef");

	// What are reasonable quat cov terms?
	Matrix cov = xt::diag(Vector{1e-9, 1e-8, 30.0, 2.0, 0.5, 0.4, 1e-4, 1e-8, 1e-6, 1e-6});

	inverse_test(q_to_e, e_to_q, v, cov, v * 1e-3, 1e-5, 1e-6);
}

TEST_F(PinsonVirtualStateBlockTests, QuatToEcefJac_SLOW) {
	auto q = navtk::navutils::rpy_to_quat(Vector3{-1.77689, -0.25674, -3.06957});
	Vector v{0.6, -1.4, 1234.0, -11.638386, 3.732691, 3.735638, q(0), q(1), q(2), q(3)};

	auto q_to_e = make_shared<StandardToEcefQuat>("i_ecef", "b_ecef");
	wrap_test(q_to_e, v, v * 1e-3);
}

TEST_F(PinsonVirtualStateBlockTests, EcefToQuatJac_SLOW) {
	auto q = navtk::navutils::rpy_to_quat(Vector3{-1.77689, -0.25674, -3.06957});
	Vector v{ecef(0), ecef(1), ecef(2), -11.638386, 3.732691, 3.735638, q(0), q(1), q(2), q(3)};

	auto e_to_q = make_shared<EcefToStandardQuat>("i_ecef", "b_ecef");
	wrap_test(e_to_q, v, v * 1e-3);
}

TEST_F(PinsonVirtualStateBlockTests, QuatBodyLeverInverse_SLOW) {
	auto q = navtk::navutils::rpy_to_quat(Vector3{-1.77689, -0.25674, -3.06957});
	Vector v{ecef(0), ecef(1), ecef(2), -11.638386, 3.732691, 3.735638, q(0), q(1), q(2), q(3)};
	Matrix cov = xt::diag(Vector{5.0, 10.0, 30.0, 2.0, 0.5, 0.4, 1e-4, 1e-8, 1e-6, 1e-6});

	auto inertial_to_body =
	    make_shared<SensorToPlatformEcefQuat>("i_ecef", "b_ecef", inertial_mount, 100000);
	auto body_to_sensor =
	    make_shared<PlatformToSensorEcefQuat>("b_ecef", "s_ecef", inertial_mount, 100000);
	inverse_test(inertial_to_body, body_to_sensor, v, cov, v * 1e-3);
}

TEST_F(PinsonVirtualStateBlockTests, QuatBody_SLOW) {
	double scale = 10000;
	auto q       = navtk::navutils::rpy_to_quat(Vector3{-1.77689, -0.25674, -3.06957});
	Vector v{ecef(0) / scale,
	         ecef(1) / scale,
	         ecef(2) / scale,
	         -11.638386,
	         3.732691,
	         3.735638,
	         q(0),
	         q(1),
	         q(2),
	         q(3)};
	Matrix cov = xt::diag(Vector{5.0, 10.0, 30.0, 2.0, 0.5, 0.4, 1e-4, 1e-8, 1e-6, 1e-6});

	auto sensor_to_body =
	    make_shared<SensorToPlatformEcefQuat>("b_ecef", "s_ecef", inertial_mount, scale);
	wrap_test(sensor_to_body, v, v * 1e-3);
}

TEST_F(PinsonVirtualStateBlockTests, QuatBody2_SLOW) {
	double scale = 10000;
	auto q       = navtk::navutils::rpy_to_quat(Vector3{-1.77689, -0.25674, -3.06957});
	Vector v{ecef(0) / scale,
	         ecef(1) / scale,
	         ecef(2) / scale,
	         -11.638386,
	         3.732691,
	         3.735638,
	         q(0),
	         q(1),
	         q(2),
	         q(3)};
	Matrix cov = xt::diag(Vector{5.0, 10.0, 30.0, 2.0, 0.5, 0.4, 1e-4, 1e-8, 1e-6, 1e-6});

	auto body_to_sensor =
	    make_shared<PlatformToSensorEcefQuat>("b_ecef", "s_ecef", sensor_mount, scale);
	wrap_test(body_to_sensor, v, v * 1e-3);
}

TEST_F(PinsonVirtualStateBlockTests, Unified_SLOW) {
	auto ref_fun = [](const aspn_xtensor::TypeTimestamp& time) {
		Vector3 nom_pos{0.0, -navtk::navutils::PI, 0.0};
		Vector3 nom_vel{0.0, 0.0, 0.0};
		Vector3 nom_rpy{0.0, 0.0, 0.0};
		auto dcm = xt::transpose(rpy_to_dcm(nom_rpy));
		return NavSolution(nom_pos, nom_vel, dcm, time);
	};

	auto complete = PinsonToSensor(pinson_name, final_name, ref_fun, no_mount, no_mount);

	auto whole_only = make_shared<PinsonErrorToStandardQuat>(pinson_name, "whole", ref_fun);

	auto to_ecef = make_shared<StandardToEcefQuat>("whole", "i_ecef");

	auto scale_vector                       = navtk::ones(16);
	xt::view(scale_vector, xt::range(0, 3)) = ecef_scale;
	auto scale_it =
	    make_shared<ScaleVirtualStateBlock>("i_ecef", "ecef_scaled", 1.0 / scale_vector);

	auto inertial_to_body_ecef =
	    make_shared<SensorToPlatformEcefQuat>("ecef_scaled", "b_ecef", no_mount, ecef_scale);

	auto body_to_sensor_ecef =
	    make_shared<PlatformToSensorEcefQuat>("b_ecef", "s_ecef", no_mount, ecef_scale);

	auto unscale_it = make_shared<ScaleVirtualStateBlock>("s_ecef", "ecef_unscaled", scale_vector);

	auto to_pva = make_shared<EcefToStandardQuat>("ecef_unscaled", "sensor_whole");
	auto to_rpy = make_shared<navtk::filtering::QuatToRpyPva>("sensor_whole", "sensor_rpy");

	auto chained = make_shared<ChainedVirtualStateBlock>(
	    vector<not_null<shared_ptr<VirtualStateBlock>>>{whole_only,
	                                                    to_ecef,
	                                                    scale_it,
	                                                    inertial_to_body_ecef,
	                                                    body_to_sensor_ecef,
	                                                    unscale_it,
	                                                    to_pva,
	                                                    to_rpy});
	auto chained_copy_ctr    = make_shared<ChainedVirtualStateBlock>(*chained);
	auto chained_copy_assign = init_chained_vsb_ptr_from_copy_assignment(*chained);

	auto straight = complete.convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));

	// Define the test
	auto test = [this, &straight](shared_ptr<ChainedVirtualStateBlock> chained) {
		auto chn = chained->convert(ec, aspn_xtensor::TypeTimestamp((int64_t)0));
		// 0 rpy highlights diffs between tilt corr methods, slightly larger errors
		ASSERT_ALLCLOSE_EX(straight.estimate, chn.estimate, 1e-5, 1e-7);
		ASSERT_FALSE(xt::any(xt::isnan(chn.covariance)));
		ASSERT_FALSE(xt::any(xt::isinf(chn.covariance)));
	};

	// Test the original
	test(chained);

	// Destroy the original and test the copies
	chained.reset();
	test(chained_copy_ctr);
	test(chained_copy_assign);
}

TEST_F(PinsonVirtualStateBlockTests, chained_cache) {

	auto ref_gen = [](const aspn_xtensor::TypeTimestamp& time) {
		return NavSolution(navtk::ones(3) * to_seconds(time),
		                   navtk::ones(3) * to_seconds(time),
		                   navtk::eye(3),
		                   time);
	};

	auto complete =
	    std::make_shared<PinsonToSensorLlh>(pinson_name, final_name, ref_gen, no_mount, no_mount);
	auto complete_wrapped =
	    ChainedVirtualStateBlock(vector<not_null<shared_ptr<VirtualStateBlock>>>{complete});

	auto x = navtk::zeros(15);
	// Triggers cached condition at this x/time
	complete_wrapped.jacobian(x, aspn_xtensor::to_type_timestamp(1.0));

	// Ensure that even though x is same we don't return a cached value
	auto est1 = complete->convert_estimate(x, aspn_xtensor::to_type_timestamp(5.0));
	auto est2 = complete_wrapped.convert_estimate(x, aspn_xtensor::to_type_timestamp(5.0));
	ASSERT_ALLCLOSE(est1, est2);
}
