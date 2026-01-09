#include <filtering/processors/PseudorangeDopplerProcessorEcefTests.hpp>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/PseudorangeDopplerProcessorEcef.hpp>
#include <navtk/gnssutils/assemble_cps.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

using aspn_xtensor::MetadataGpsLnavEphemeris;
using aspn_xtensor::TypeSatnavTime;
using navtk::Matrix;
using navtk::filtering::PseudorangeDopplerProcessorEcef;

TEST_F(CarrierPhaseObservationTest, testSinglePointClockInit) {
	auto genxp        = [&](const std::vector<std::string>& s) { return gen_xp_ecef(s, ns, 2, 3); };
	Matrix expected_H = {{0, 0, 0, 0, 0, 0, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 1, 0, 0}};
	clock_init_test(genxp, expected_H);
}

TEST_F(CarrierPhaseObservationTest, testSinglePointElevationMask) {
	auto genxp = [&](const std::vector<std::string>& s) { return gen_xp_ecef(s, ns, 2, 3); };
	// clang-format off
	Matrix expected_H = transform_meas_jac(ns, {
		{-0.69510116290395, 0.62080841393133, 0.36253453148858, 0, 0, 0, 299792458, 0, 1.0},
		{0, 0, 0, -0.69510116290395, 0.62080841393133, 0.36253453148858, 0, 299792458, 0}
	});
	// clang-format on
	elevation_mask_test(genxp, expected_H);
}

TEST_F(CarrierPhaseObservationTest, testSinglePoint) {
	auto genxp = [&](const std::vector<std::string>& s) { return gen_xp_ecef(s, ns, 2, 3); };
	// clang-format off
    Matrix expected_H = transform_meas_jac(ns, {
        {-0.69510116290395,  0.62080841393133,  0.36253453148858, 0, 0, 0, 299792458, 0, 1.0, 0},
        {-0.57954885217311, -0.75914314087664, -0.29635252589574, 0, 0, 0, 299792458, 0, 0, 1.0},
        {0, 0, 0, -0.69510116290395,  0.62080841393133,  0.36253453148858, 0, 299792458, 0, 0},
        {0, 0, 0, -0.57954885217311, -0.75914314087664, -0.29635252589574, 0, 299792458, 0, 0}
    });
	// clang-format on
	single_point_test(genxp, expected_H);
}

TEST_F(CarrierPhaseObservationTest, testSinglePointAlternate) {
	auto empty_ns =
	    NavSolution{navtk::filtering::Pose{
	                    navtk::Vector3(), navtk::eye(3), aspn_xtensor::TypeTimestamp((int64_t)0)},
	                navtk::Vector3()};
	auto genxp = [&](const std::vector<std::string>& s) { return gen_xp_ecef(s, empty_ns, 2, 0); };
	// clang-format off
    Matrix expected_H = transform_meas_jac(empty_ns, {
        {-0.91625305231276, -0.03821078935438,  0.39877346915819, 0, 0, 0, 299792458, 0, 1.0, 0},
        {-0.2339043373106,   0.72909355880896, -0.64320396725345, 0, 0, 0, 299792458, 0, 0, 1.0},
        {0, 0, 0, -0.91625305231276, -0.03821078935438,  0.39877346915819, 0, 299792458, 0, 0},
        {0, 0, 0, -0.2339043373106,   0.72909355880896, -0.64320396725345, 0, 299792458, 0, 0}
    });
	// clang-format on
	alternate_single_point_test(genxp, expected_H);
}

TEST_F(CarrierPhaseObservationTest, testPrnSBIndex) {
	auto empty_ns =
	    NavSolution{navtk::filtering::Pose{
	                    navtk::Vector3(), navtk::eye(3), aspn_xtensor::TypeTimestamp((int64_t)0)},
	                navtk::Vector3()};

	// Generate x/p assuming one PVA state block, 1 2 element clock bias block, and the rest of the
	// labels referring to pr bias states
	auto genxp = [&](const std::vector<std::string>& s) {
		return gen_xp_ecef(s, empty_ns, s.size() - 2);
	};
	// clang-format off
	 Matrix expected_h_pr_bias_1 = transform_meas_jac(empty_ns, {
        {-0.91625305231276, -0.03821078935438,  0.39877346915819, 0, 0, 0, 299792458, 0, 1.0, 0},
        {-0.2339043373106,   0.72909355880896, -0.64320396725345, 0, 0, 0, 299792458, 0, 0, 1.0},
        {0, 0, 0, -0.91625305231276, -0.03821078935438,  0.39877346915819, 0, 299792458, 0, 0},
        {0, 0, 0, -0.2339043373106,   0.72909355880896, -0.64320396725345, 0, 299792458, 0, 0}
    });
    Matrix expected_h_pr_bias_2 = transform_meas_jac(empty_ns, {
        {-0.916267736679053, -0.0383199416395394,  0.398729252493203, 0, 0, 0, 299792458, 0, 1.0, 0},
        {0.286065790372675,   0.405922386397669, 0.867982361456544, 0, 0, 0, 299792458, 0, 0, 1.0},
        {0, 0, 0, -0.916267736679053, -0.0383199416395394,  0.398729252493203, 0, 299792458, 0, 0},
        {0, 0, 0, 0.286065790372675,   0.405922386397669, 0.867982361456544, 0, 299792458, 0, 0}
    });
	Matrix expected_h_pr_bias_3 = transform_meas_jac(empty_ns, {
        {0.285923554312540, 0.405919407873993, 0.868030618930295, 0, 0, 0, 299792458, 0, 1.0, 0, 0},
        {-0.234100318421588,  0.729070328973750, -0.643158997701983, 0, 0, 0, 299792458, 0, 0, 1.0, 0},
		{-0.916282402586976, -0.0384290967561643, 0.398685042648887, 0, 0, 0, 299792458, 0, 0, 0, 1.0},
        {0, 0, 0, 0.285923554312540, 0.405919407873993,  0.868030618930295, 0, 299792458, 0, 0, 0},
        {0, 0, 0, -0.234100318421588,   0.729070328973750, -0.643158997701983, 0, 299792458, 0, 0, 0},
		{0, 0, 0, -0.916282402586976,   -0.0384290967561643, 0.398685042648887, 0, 299792458, 0, 0, 0}
    });
	// clang-format on

	prn_sb_index_test(genxp, expected_h_pr_bias_1, expected_h_pr_bias_2, expected_h_pr_bias_3);
}

TEST_F(CarrierPhaseObservationTest, testWrongAspnBaseVectorType) { bad_aux_test(); }

ERROR_MODE_SENSITIVE_TEST(TEST_F, CarrierPhaseObservationTest, testInvalidData) {
	auto genxp = [&](const std::vector<std::string>& s) {
		return test.gen_xp_ecef(s, test.ns, 4, 0);
	};
	test.invalid_data_test(genxp);
}

class TestableRDMP : public PseudorangeDopplerProcessorEcef {
public:
	using PseudorangeDopplerProcessorEcef::assemble_measurements;
	using PseudorangeDopplerProcessorEcef::PseudorangeDopplerProcessorEcef;  // Inherit constructors

	TestableRDMP(const TestableRDMP& processor) : PseudorangeDopplerProcessorEcef(processor) {}

	navtk::not_null<std::shared_ptr<MeasurementProcessor<>>> clone() {
		return std::make_shared<TestableRDMP>(*this);
	}

	bool compare_ephemerides(std::vector<MetadataGpsLnavEphemeris> ephemerides1,
	                         std::vector<MetadataGpsLnavEphemeris> ephemerides2) {
		if (ephemerides1.size() != ephemerides2.size()) return false;
		for (size_t i = 0; i < ephemerides1.size(); ++i) {
			if (ephemerides1[i].get_clock().get_t_oc() != ephemerides2[i].get_clock().get_t_oc())
				return false;
			if (ephemerides1[i].get_prn() != ephemerides2[i].get_prn()) return false;
		}
		return true;
	}

	void clone_test() {
		// Add some ephemerides to the handler then copy the processor
		std::ifstream rinex("hour1190.18n", std::ios_base::in | std::ios_base::binary);
		ASSERT_TRUE(rinex.is_open());
		TypeSatnavTime last_time{0, 0, ASPN_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GPS};
		auto temp_eph = read_ephemeris_from_file_rinex2_1(rinex, last_time);
		AspnBaseVector eph_aux;
		for (auto eph : temp_eph) {
			eph_aux.push_back(std::make_shared<aspn_xtensor::AspnBase>(eph));
		}
		receive_aux_data(eph_aux);
		auto processor_copy = std::dynamic_pointer_cast<TestableRDMP>(clone());

		// Validate clone() as implemented returns a deep copy by modifying all of the clone's
		// public and protected properties and asserting they do not equal the original's.
		ASSERT_EQ(get_label(), processor_copy->get_label());
		for (size_t ii = 0; ii < state_block_labels.size(); ++ii) {
			ASSERT_EQ(state_block_labels[ii], processor_copy->state_block_labels[ii]);
			processor_copy->state_block_labels[ii] += "_copy";
			ASSERT_NE(state_block_labels[ii], processor_copy->state_block_labels[ii]);
		}

		auto ephemerides       = ephemeris_handler.ephemerides_at_time(last_time);
		auto ephemerides_clone = processor_copy->ephemeris_handler.ephemerides_at_time(last_time);
		ASSERT_TRUE(compare_ephemerides(ephemerides, ephemerides_clone));
		ephemeris_handler.add_ephemerides(read_ephemeris_from_file_rinex2_1(rinex, last_time));
		ephemerides       = ephemeris_handler.ephemerides_at_time(last_time);
		ephemerides_clone = processor_copy->ephemeris_handler.ephemerides_at_time(last_time);
		ASSERT_FALSE(compare_ephemerides(ephemerides, ephemerides_clone));

		ASSERT_EQ(cp_to_use.frequency, processor_copy->cp_to_use.frequency);
		cp_to_use = navtk::gnssutils::L1_PLUS_L2;
		ASSERT_NE(cp_to_use.frequency, processor_copy->cp_to_use.frequency);

		ASSERT_EQ(pr_noise_covariance, processor_copy->pr_noise_covariance);
		pr_noise_covariance += 1;
		ASSERT_NE(pr_noise_covariance, processor_copy->pr_noise_covariance);

		ASSERT_EQ(prr_noise_covariance, processor_copy->prr_noise_covariance);
		prr_noise_covariance += 1;
		ASSERT_NE(prr_noise_covariance, processor_copy->prr_noise_covariance);

		ASSERT_EQ(tropo_rel_humidity, processor_copy->tropo_rel_humidity);
		tropo_rel_humidity += 1;
		ASSERT_NE(tropo_rel_humidity, processor_copy->tropo_rel_humidity);

		ASSERT_EQ(elevation_mask, processor_copy->elevation_mask);
		elevation_mask += 1;
		ASSERT_NE(elevation_mask, processor_copy->elevation_mask);

		ASSERT_EQ(apply_tropo_model, processor_copy->apply_tropo_model);
		apply_tropo_model = !apply_tropo_model;
		ASSERT_NE(apply_tropo_model, processor_copy->apply_tropo_model);

		ASSERT_EQ(clock_initialized, processor_copy->clock_initialized);
		clock_initialized = !clock_initialized;
		ASSERT_NE(clock_initialized, processor_copy->clock_initialized);

		ASSERT_EQ(ephemeris_handler_initialized, processor_copy->ephemeris_handler_initialized);
		ephemeris_handler_initialized = !ephemeris_handler_initialized;
		ASSERT_NE(ephemeris_handler_initialized, processor_copy->ephemeris_handler_initialized);
	}
};

#define MAKE_PUBLIC_SPPP_FROM_CLASS_VARS() \
	TestableRDMP("",                       \
	             "",                       \
	             "",                       \
	             cp_to_use,                \
	             pr_noise_cov,             \
	             pr_bias_covariance,       \
	             pr_time_constant,         \
	             prr_noise_cov,            \
	             engine,                   \
	             apply_tropo_model,        \
	             tropo_rel_humidity,       \
	             mask_angle,               \
	             force_clock_initialization);

TEST_F(CarrierPhaseObservationTest, testClone) {
	force_clock_initialization = true;
	TestableRDMP processor     = MAKE_PUBLIC_SPPP_FROM_CLASS_VARS();
	processor.clone_test();
}

TEST_F(CarrierPhaseObservationTest, testCorrectMeasurementsEmptyHandler) {
	// Create a processor but don't give it ephemerides.
	TestableRDMP processor               = MAKE_PUBLIC_SPPP_FROM_CLASS_VARS();
	std::vector<TypeSatnavObs> meas_data = navtk::gnssutils::assemble_cps(*observation, cp_to_use);

	navtk::filtering::PseudorangeDopplerMeasurements result;
	EXPECT_WARN(
	    result = processor.assemble_measurements(meas_data, seconds, ns.pos, week_number),
	    "No ephemeris data has been added to the ephemeris handler. Unable to perform update.");
	EXPECT_TRUE(result.prns.empty());
}

TEST_F(CarrierPhaseObservationTest, testCorrectMeasurementsBadEphemeris) {
	auto satnav_time = observation->get_receiver_clock_time();
	satnav_time.set_seconds_of_week(53898);  // arbitrary value with no corresponding ephemerides
	observation->set_receiver_clock_time(satnav_time);
	std::vector<TypeSatnavObs> meas_data = navtk::gnssutils::assemble_cps(*observation, cp_to_use);

	TestableRDMP processor = MAKE_PUBLIC_SPPP_FROM_CLASS_VARS();
	processor.receive_aux_data(ephemerides);

	navtk::filtering::PseudorangeDopplerMeasurements result;
	EXPECT_WARN(EXPECT_INFO(result = processor.assemble_measurements(
	                            meas_data,
	                            observation->get_receiver_clock_time().get_seconds_of_week(),
	                            ns.pos,
	                            week_number),
	                        "Removed an ephemeris"),
	            "ephemerides.*have been discarded");
	EXPECT_TRUE(result.prns.empty());
}

TEST_F(CarrierPhaseObservationTest, testCorrectMeasurementsMeasEphemPrnMissmatch) {
	// Change PRNs in measurement so there are no common PRNs between the list of observations
	// and the list of ephemerides.
	auto observations = observation->get_obs();
	for (auto& obs : observations) obs.set_prn(11);
	observation->set_obs(observations);
	std::vector<TypeSatnavObs> meas_data = navtk::gnssutils::assemble_cps(*observation, cp_to_use);

	TestableRDMP processor = MAKE_PUBLIC_SPPP_FROM_CLASS_VARS();
	processor.receive_aux_data(ephemerides);

	navtk::filtering::PseudorangeDopplerMeasurements result;
	EXPECT_WARN(result = processor.assemble_measurements(meas_data, seconds, ns.pos, week_number),
	            "Ephemeris handler returned no ephemerides for the given time for the "
	            "given prn\\(s\\). Unable to perform update.");
	EXPECT_TRUE(result.prns.empty());
}

TEST_F(CarrierPhaseObservationTest, testCorrectMeasurementsElevationMaskElimination) {
	mask_angle                           = 1.0;
	std::vector<TypeSatnavObs> meas_data = navtk::gnssutils::assemble_cps(*observation, cp_to_use);

	TestableRDMP processor = MAKE_PUBLIC_SPPP_FROM_CLASS_VARS();
	processor.receive_aux_data(ephemerides);

	navtk::filtering::PseudorangeDopplerMeasurements result;
	EXPECT_WARN(result = processor.assemble_measurements(meas_data, seconds, ns.pos, week_number),
	            "The elevation mask has eliminated every measurement. Unable to perform update.");
	EXPECT_TRUE(result.prns.empty());
}
