#include <memory>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/processors/Attitude3dMeasurementProcessor.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonErrorToStandard.hpp>
#include <navtk/filtering/virtualstateblocks/StateExtractor.hpp>
#include <navtk/inertial/BufferedImu.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::MeasurementAttitude3D;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::dot;
using navtk::eye;
using navtk::Matrix;
using navtk::Matrix3;
using navtk::Vector;
using navtk::Vector3;
using navtk::zeros;
using navtk::filtering::Attitude3dMeasurementProcessor;
using navtk::filtering::NavSolution;
using navtk::filtering::PairedPva;
using navtk::filtering::StandardFusionEngine;
using navtk::inertial::BufferedImu;
using navtk::navutils::dcm_to_quat;
using navtk::navutils::dcm_to_rpy;
using navtk::navutils::quat_to_dcm;
using navtk::navutils::rpy_to_dcm;
using navtk::navutils::rpy_to_quat;
using navtk::navutils::skew;

void feedback(navtk::filtering::StandardFusionEngine& engine,
              navtk::inertial::BufferedImu& ins,
              const std::string& label) {
	auto sol       = ins.calc_pva();
	auto curr_errs = ins.get_imu_errors(engine.get_time());
	auto x         = engine.get_state_block_estimate(label);
	auto corr_sol =
	    navtk::filtering::apply_error_states<navtk::filtering::Pinson15NedBlock>(*sol, x);
	curr_errs->accel_biases -= xt::view(x, xt::range(9, 12));
	curr_errs->gyro_biases -= xt::view(x, xt::range(12, 15));
	x = navtk::zeros(15);

	auto accepted = ins.reset(
	    std::make_shared<aspn_xtensor::MeasurementPositionVelocityAttitude>(corr_sol), curr_errs);
	if (accepted) {
		engine.set_state_block_estimate(label, x);
	}
}

StandardFusionEngine base_engine(const std::string& label,
                                 const navtk::filtering::NavSolution& nav0) {
	auto engine = StandardFusionEngine();
	auto model  = navtk::filtering::hg1700_model();
	auto block  = std::make_shared<navtk::filtering::Pinson15NedBlock>(label, model);
	engine.add_state_block(block);
	auto aux = navtk::utils::to_inertial_aux(nav0, zeros(3));
	engine.give_state_block_aux_data(label, aux);
	// Set initial state to something nonzero to make sure to exercise the vsb thoroughly
	Vector x0{0.0, 0.0, 0.0, 0.1, 0.2, 0.3, 1e-4, -1e-4, 1e-5, -4e-4, 3e-4, 5e-4, 1e-6, 3e-6, 2e-6};
	engine.set_state_block_estimate(label, x0);
	return engine;
}

struct Attitude3dProcessorTests : public ::testing::Test {
	Vector3 rpy_ref            = Vector3{1.0001, 2.0002, 3.0003};
	const std::string b_lab    = "pinson";
	const std::string vb_lab   = "rpy_only";
	const std::string proc_lab = "processor";
	const Vector3 rpy0         = Vector3{1.0, 2.0, 3.0};
	const Vector3 tilt         = Vector3{-0.000013, 0.000204, -0.000209};
	std::shared_ptr<MeasurementAttitude3D> dummy_att3d;
	std::shared_ptr<PairedPva> dummy_paired;

	Attitude3dProcessorTests() : ::testing::Test() {
		auto header = TypeHeader(ASPN_MEASUREMENT_ATTITUDE_3D, 0, 0, 0, 0);
		auto time   = TypeTimestamp((int64_t)0);
		dummy_att3d = std::make_shared<MeasurementAttitude3D>(
		    header,
		    time,
		    ASPN_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_NED,
		    rpy_to_quat(rpy0),
		    zeros(3, 3),
		    ASPN_MEASUREMENT_ATTITUDE_3D_ERROR_MODEL_NONE,
		    Vector(),
		    std::vector<aspn_xtensor::TypeIntegrity>{});
		dummy_paired = std::make_shared<PairedPva>(
		    dummy_att3d,
		    NavSolution(zeros(3),
		                zeros(3),
		                dot(xt::transpose(rpy_to_dcm(rpy0)), eye(3) - skew(tilt)),
		                to_type_timestamp(0, 0)));
	}

	void test_model(navtk::Size num, const Vector& x_out) {
		auto labels = std::vector<std::string>{b_lab};
		auto proc   = Attitude3dMeasurementProcessor(proc_lab, labels);
		auto genxp  = [&num](const std::vector<std::string>&) {
            return std::make_shared<navtk::filtering::EstimateWithCovariance>(
                xt::arange(0, (int)num), zeros(num, num));
		};
		auto mod = proc.generate_model(dummy_att3d, genxp);
		auto x   = genxp(labels)->estimate;
		ASSERT_ALLCLOSE_EX(dot(mod->H, x), x_out, 1e-15, 1e-15);
		ASSERT_ALLCLOSE_EX(mod->h(x), x_out, 1e-15, 1e-15);
		ASSERT_ALLCLOSE_EX(
		    navtk::navutils::rpy_to_quat(mod->z), navtk::navutils::rpy_to_quat(rpy0), 1e-15, 1e-15);

		mod = proc.generate_model(dummy_paired, genxp);
		ASSERT_ALLCLOSE_EX(dot(mod->H, x), x_out, 1e-15, 1e-15);
		ASSERT_ALLCLOSE_EX(mod->h(x), x_out, 1e-15, 1e-15);
		ASSERT_ALLCLOSE_EX(mod->z, tilt, 1e-15, 1e-15);
	}

	void full_test_loop(const double atol, const bool do_feedback) {
		navtk::Size num_loops = 5;
		// Set up 2 filters.
		// One filter uses direct processor w/ tilt calcs
		// The other will use direct processor with VSB
		// Compare the results
		auto header = TypeHeader(ASPN_MEASUREMENT_ATTITUDE_3D, 0, 0, 0, 0);
		auto time   = TypeTimestamp((int64_t)0);
		auto pva0   = MeasurementPositionVelocityAttitude(
            header,
            time,
            ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            rpy_to_quat(zeros(3)),
            zeros(9, 9),
            ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
            Vector(),
            std::vector<aspn_xtensor::TypeIntegrity>{});
		auto nav0 = navtk::utils::to_navsolution(pva0);

		Vector3 tilts{4e-2, -3e-2, -1e-2};
		auto corrupt_rpy =
		    dcm_to_rpy(dot(eye(3) + skew(tilts), quat_to_dcm(pva0.get_quaternion())));
		auto corrupt_pva = pva0;
		corrupt_pva.set_quaternion(rpy_to_quat(corrupt_rpy));

		double dt  = 0.02;
		auto btrue = BufferedImu(pva0, nullptr, dt);
		auto b1    = BufferedImu(corrupt_pva, nullptr, dt);
		auto b2    = BufferedImu(corrupt_pva, nullptr, dt);

		auto engine1 = base_engine(b_lab, nav0);
		auto engine2 = base_engine(b_lab, nav0);

		// Add vsbs to second filter
		auto ref_gen = [&b2](const aspn_xtensor::TypeTimestamp& time) {
			return navtk::utils::to_navsolution(*b2.calc_pva(time));
		};

		auto all_whole =
		    std::make_shared<navtk::filtering::PinsonErrorToStandard>(b_lab, "whole", ref_gen);
		auto whole_rpy_vsb = std::make_shared<navtk::filtering::StateExtractor>(
		    "whole", vb_lab, 15, std::vector<navtk::Size>{6, 7, 8});

		engine2.add_virtual_state_block(all_whole);
		engine2.add_virtual_state_block(whole_rpy_vsb);

		auto proc1 = std::make_shared<Attitude3dMeasurementProcessor>(proc_lab, b_lab);
		engine1.add_measurement_processor(proc1);
		auto proc2 = std::make_shared<Attitude3dMeasurementProcessor>(proc_lab, vb_lab);
		engine2.add_measurement_processor(proc2);

		auto dv       = Vector3{0.01, -0.02, -9.8} * dt;
		auto dth      = Vector3{-0.003, -0.004, 0.05} * dt;
		auto tilt_cov = eye(3) * 1e-6;

		for (navtk::Size big_loop = 0; big_loop < num_loops; big_loop++) {
			for (navtk::Size k = 0; k < 5; k++) {
				auto t = to_type_timestamp(dt * (k + 1 + big_loop * 5));
				btrue.mechanize(t, dv, dth);
				b1.mechanize(t, dv, dth);
				b2.mechanize(t, dv, dth);
			}

			auto aux1 = navtk::utils::to_inertial_aux(
			    navtk::utils::to_navsolution(*b1.calc_pva()),
			    b1.calc_force_and_rate(b1.time_span().second)->get_meas_accel());
			engine1.give_state_block_aux_data(b_lab, aux1);

			auto aux2 = navtk::utils::to_inertial_aux(
			    navtk::utils::to_navsolution(*b2.calc_pva()),
			    b2.calc_force_and_rate(b2.time_span().second)->get_meas_accel());
			engine2.give_state_block_aux_data(b_lab, aux2);

			auto sol = btrue.calc_pva();

			auto header   = TypeHeader(ASPN_MEASUREMENT_ATTITUDE_3D, 0, 0, 0, 0);
			auto att_meas = std::make_shared<MeasurementAttitude3D>(
			    header,
			    sol->get_time_of_validity(),
			    ASPN_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_NED,
			    sol->get_quaternion(),
			    tilt_cov,
			    ASPN_MEASUREMENT_ATTITUDE_3D_ERROR_MODEL_NONE,
			    Vector(),
			    std::vector<aspn_xtensor::TypeIntegrity>{});

			engine1.update(proc_lab,
			               std::make_shared<navtk::filtering::PairedPva>(
			                   att_meas, navtk::utils::to_navsolution(*b1.calc_pva())));
			engine2.update(proc_lab, att_meas);

			if (do_feedback) {
				feedback(engine1, b1, b_lab);
				feedback(engine2, b2, b_lab);
			}
		}

		// Use final state estimates to correct the inertials and generate the overall solution
		feedback(engine1, b1, b_lab);
		feedback(engine2, b2, b_lab);

		auto truth = btrue.calc_pva();
		auto s1    = b1.calc_pva();
		auto s2    = b2.calc_pva();

		auto pins_tilt = xt::abs(eye(3) - dot(quat_to_dcm(truth->get_quaternion()),
		                                      xt::transpose(quat_to_dcm(s1->get_quaternion()))));
		auto vsb_tilt  = xt::abs(eye(3) - dot(quat_to_dcm(truth->get_quaternion()),
                                             xt::transpose(quat_to_dcm(s2->get_quaternion()))));

		ASSERT_ALLCLOSE_EX(zeros(3, 3), pins_tilt, 0.0, atol);
		ASSERT_ALLCLOSE_EX(zeros(3, 3), vsb_tilt, 0.0, atol);
	}
};

Vector3 pos_err_meters(const Vector3& a, const Vector3& b) {
	double dn = navtk::navutils::delta_lat_to_north(a(0) - b(0), a(0), a(2));
	double de = navtk::navutils::delta_lon_to_east(a(1) - b(1), a(0), a(2));
	double dd = b(2) - a(2);
	return Vector3{dn, de, dd};
}

TEST_F(Attitude3dProcessorTests, size_based_model_SLOW) {
	// Attitude states are last 3 in sequence
	for (navtk::Size k = 0; k < 6; k++) {
		test_model(k + 3, xt::arange(k, k + 3));
	}
	// Attitude states in fixed location
	Vector expected{6, 7, 8};
	for (navtk::Size k = 6; k < 20; k++) {
		test_model(k + 3, expected);
	}
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, Attitude3dProcessorTests, too_small_block) {
	auto labels = std::vector<std::string>{test.b_lab};
	auto proc   = Attitude3dMeasurementProcessor(test.proc_lab, labels);
	auto genxp  = [](const std::vector<std::string>&) {
        return std::make_shared<navtk::filtering::EstimateWithCovariance>(zeros(2), zeros(2, 2));
	};
	auto res = EXPECT_HONORS_MODE(proc.generate_model(test.dummy_att3d, genxp), "fewer than 3");
	ASSERT_TRUE(res == nullptr);
	res = EXPECT_HONORS_MODE(proc.generate_model(test.dummy_paired, genxp), "fewer than 3");
	ASSERT_TRUE(res == nullptr);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, Attitude3dProcessorTests, unsupported_meas) {
	auto genxp = [](const std::vector<std::string>&) {
		return std::make_shared<navtk::filtering::EstimateWithCovariance>(zeros(3), zeros(3, 3));
	};
	auto proc = Attitude3dMeasurementProcessor(test.proc_lab, std::vector<std::string>{test.b_lab});
	auto res  = EXPECT_HONORS_MODE(
        proc.generate_model(std::make_shared<navtk::filtering::GaussianVectorData>(
                                aspn_xtensor::TypeTimestamp((int64_t)0), zeros(3), eye(3)),
                            genxp),
        "measurement is unsupported");
	ASSERT_TRUE(res == nullptr);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, Attitude3dProcessorTests, bad_state_labels) {
	auto genxp = [](const std::vector<std::string>&) { return nullptr; };
	auto proc  = Attitude3dMeasurementProcessor(test.proc_lab, std::vector<std::string>{});
	auto res   = EXPECT_HONORS_MODE(proc.generate_model(test.dummy_att3d, genxp),
                                  "gen_x_and_p_func returned null");
	ASSERT_TRUE(res == nullptr);
}

TEST_F(Attitude3dProcessorTests, updates_without_feedback_SLOW) {
	// Without feedback, the residual 'bottoms out' at around 14e-5 rad probably due to
	// linearization. Tuning and/or increasing number of loops will not change it significantly
	full_test_loop(14e-5, false);
}

TEST_F(Attitude3dProcessorTests, updates_with_feedback_SLOW) {
	// With feedback a finer estimate is produced, and increasing the number of updates/loops will
	// lower the error. The given value assumes 5 loops/updates.
	full_test_loop(14e-5, true);
}
