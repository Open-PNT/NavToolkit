#include <filtering/processors/SinglePointPseudorangeProcessorTests.hpp>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/SinglePointPseudorangeProcessor.hpp>
#include <navtk/gnssutils/assemble_prs.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::MetadataGpsLnavEphemeris;
using aspn_xtensor::TypeSatnavObs;
using aspn_xtensor::TypeSatnavTime;
using navtk::Matrix;
using navtk::filtering::SinglePointPseudorangeProcessor;

TEST_F(SinglePointTest, testSinglePointClockInit) {
	auto genxp        = [&](const std::vector<std::string>& s) { return gen_xp(s, 2, 3); };
	Matrix expected_H = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}};
	clock_init_test(genxp, expected_H);
}

TEST_F(SinglePointTest, testSinglePointElevationMask) {
	auto genxp = [&](const std::vector<std::string>& s) { return gen_xp(s, 2, 3); };
	// clang-format off
    Matrix expected_H = {{-0.69510116290395, 0.62080841393133, 0.36253453148858, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 1.0}};
	// clang-format on
	elevation_mask_test(genxp, expected_H);
}

TEST_F(SinglePointTest, testSinglePoint) {
	auto genxp = [&](const std::vector<std::string>& s) { return gen_xp(s, 2, 3); };
	// clang-format off
    Matrix expected_H = {
        {-0.69510116290395,  0.62080841393133,  0.36253453148858, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 1.0, 0.0},
        {-0.57954885217311, -0.75914314087664, -0.29635252589574, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 0.0, 1.0}
    };
	// clang-format on
	single_point_test(genxp, expected_H);
}

TEST_F(SinglePointTest, testSinglePointAlternate) {
	auto genxp = [&](const std::vector<std::string>& s) { return gen_xp(s, 2, 0); };
	// clang-format off
    Matrix expected_H = {
        {-0.91625305231276, -0.03821078935438,  0.39877346915819, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 1.0, 0.0},
        {-0.2339043373106,   0.72909355880896, -0.64320396725345, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 0.0, 1.0}
    };
	// clang-format on

	alternate_single_point_test(genxp, expected_H);
}

TEST_F(SinglePointTest, testPrnSBIndex) {
	// Generate x/p assuming one PVA state block, 1 2 element clock bias block, and the rest of the
	// labels referring to pr bias states
	auto genxp = [&](const std::vector<std::string>& s) { return gen_xp(s, 2, 0); };
	// clang-format off
	Matrix expected_h_pr_bias_1 = {
        {-0.91625305231276, -0.03821078935438,  0.39877346915819, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 1.0, 0.0},
        {-0.2339043373106,   0.72909355880896, -0.64320396725345, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 0.0, 1.0}};
    Matrix expected_h_pr_bias_2 = {
		{-0.916267736679053, -0.0383199416395394,  0.398729252493203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 1.0, 0.0},
 		{ 0.286065790372675,  0.405922386397669,  0.867982361456544, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 0.0, 1.0}};
	Matrix expected_h_pr_bias_3 = {
		{ 0.285923554312540,  0.405919407873993,  0.868030618930295, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 1.0, 0, 0.0},
 		{-0.234100318421588,  0.729070328973750, -0.643158997701983, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 0.0, 1.0, 0.0},
 		{-0.916282402586976, -0.0384290967561643, 0.398685042648887, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 0, 0.0, 1.0}};
	Matrix expected_h_pr_bias_4 = {
 		{ 0.286065790372675,  0.405922386397669,  0.867982361456544, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 1.0, 0},
		 {-0.916267736679053, -0.0383199416395394,  0.398729252493203, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 299792458, 0, 0, 1.0}};

	// clang-format on

	prn_sb_index_test(genxp,
	                  expected_h_pr_bias_1,
	                  expected_h_pr_bias_2,
	                  expected_h_pr_bias_3,
	                  expected_h_pr_bias_4);
}

TEST_F(SinglePointTest, testWrongAspnBaseVectorType) { bad_aux_test(); }

ERROR_MODE_SENSITIVE_TEST(TEST_F, SinglePointTest, testInvalidData) {
	auto genxp = [&](const std::vector<std::string>& s) { return test.gen_xp(s, 2, 3); };
	test.invalid_data_test(genxp);
}


class TestableSPPP : public SinglePointPseudorangeProcessor {
public:
	using SinglePointPseudorangeProcessor::correct_measurements;
	using SinglePointPseudorangeProcessor::SinglePointPseudorangeProcessor;  // Inherit constructors

	TestableSPPP(const TestableSPPP& processor) : SinglePointPseudorangeProcessor(processor) {}

	navtk::not_null<std::shared_ptr<MeasurementProcessor<>>> clone() {
		return std::make_shared<TestableSPPP>(*this);
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
			eph_aux.push_back(std::make_shared<aspn_xtensor::MetadataGpsLnavEphemeris>(eph));
		}
		receive_aux_data(eph_aux);
		auto processor_copy = std::dynamic_pointer_cast<TestableSPPP>(clone());

		// Validate clone() as implemented returns a deep copy by modifying all of the clone's
		// public and protected properties and asserting they do not equal the original's.
		ASSERT_EQ(get_label(), processor_copy->get_label());
		for (size_t ii = 0; ii < get_state_block_labels().size(); ++ii) {
			ASSERT_EQ(this->get_state_block_labels()[0],
			          processor_copy->get_state_block_labels()[ii]);
			processor_copy->state_block_labels[ii] += "_copy";
			ASSERT_NE(this->get_state_block_labels()[ii],
			          processor_copy->get_state_block_labels()[ii]);
		}

		auto ephemerides       = ephemeris_handler.ephemerides_at_time(last_time);
		auto ephemerides_clone = processor_copy->ephemeris_handler.ephemerides_at_time(last_time);
		ASSERT_TRUE(compare_ephemerides(ephemerides, ephemerides_clone));
		ephemeris_handler.add_ephemerides(read_ephemeris_from_file_rinex2_1(rinex, last_time));
		ephemerides       = ephemeris_handler.ephemerides_at_time(last_time);
		ephemerides_clone = processor_copy->ephemeris_handler.ephemerides_at_time(last_time);
		ASSERT_FALSE(compare_ephemerides(ephemerides, ephemerides_clone));

		ASSERT_EQ(pr_to_use.code, processor_copy->pr_to_use.code);
		pr_to_use = navtk::gnssutils::PL2;
		ASSERT_NE(pr_to_use.code, processor_copy->pr_to_use.code);

		ASSERT_EQ(pr_noise_covariance, processor_copy->pr_noise_covariance);
		pr_noise_covariance += 1;
		ASSERT_NE(pr_noise_covariance, processor_copy->pr_noise_covariance);

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
	TestableSPPP("",                       \
	             "",                       \
	             "",                       \
	             pr_to_use,                \
	             pr_noise_cov,             \
	             pr_bias_covariance,       \
	             pr_time_constant,         \
	             engine,                   \
	             apply_tropo_model,        \
	             tropo_rel_humidity,       \
	             mask_angle,               \
	             force_clock_initialization);

TEST_F(SinglePointTest, testClone) {
	force_clock_initialization = true;
	TestableSPPP processor     = MAKE_PUBLIC_SPPP_FROM_CLASS_VARS();
	processor.clone_test();
}

TEST_F(SinglePointTest, testCorrectMeasurementsEmptyHandler) {
	// Create a processor but don't give it ephemerides.
	TestableSPPP processor               = MAKE_PUBLIC_SPPP_FROM_CLASS_VARS();
	std::vector<TypeSatnavObs> meas_data = navtk::gnssutils::assemble_prs(*observation, pr_to_use);

	navtk::filtering::CorrectedGnssPseudorangeMeasurement result;
	EXPECT_WARN(
	    result = processor.correct_measurements(meas_data, seconds, ns.pos, week_number),
	    "No ephemeris data has been added to the ephemeris handler. Unable to perform update.");
	EXPECT_TRUE(result.prns.empty());
}

TEST_F(SinglePointTest, testCorrectMeasurementsBadEphemeris) {
	auto satnav_time = observation->get_receiver_clock_time();
	satnav_time.set_seconds_of_week(53898);  // arbitrary value with no corresponding ephemerides
	observation->set_receiver_clock_time(satnav_time);
	std::vector<TypeSatnavObs> meas_data = navtk::gnssutils::assemble_prs(*observation, pr_to_use);

	TestableSPPP processor = MAKE_PUBLIC_SPPP_FROM_CLASS_VARS();
	processor.receive_aux_data(ephemerides);

	navtk::filtering::CorrectedGnssPseudorangeMeasurement result;
	EXPECT_WARN(EXPECT_INFO(result = processor.correct_measurements(
	                            meas_data,
	                            observation->get_receiver_clock_time().get_seconds_of_week(),
	                            ns.pos,
	                            week_number),
	                        "Removed an ephemeris"),
	            "ephemerides.*have been discarded");
	EXPECT_TRUE(result.prns.empty());
}

TEST_F(SinglePointTest, testCorrectMeasurementsMeasEphemPrnMissmatch) {
	// Change PRNs in measurement so there are no common PRNs between the list of observations
	// and the list of ephemerides.
	auto observations = observation->get_obs();
	for (auto& obs : observations) obs.set_prn(11);
	observation->set_obs(observations);
	std::vector<TypeSatnavObs> meas_data = navtk::gnssutils::assemble_prs(*observation, pr_to_use);

	TestableSPPP processor = MAKE_PUBLIC_SPPP_FROM_CLASS_VARS();
	processor.receive_aux_data(ephemerides);

	navtk::filtering::CorrectedGnssPseudorangeMeasurement result;
	EXPECT_WARN(result = processor.correct_measurements(meas_data, seconds, ns.pos, week_number),
	            "Ephemeris handler returned no ephemerides for the given time for the "
	            "given prn\\(s\\). Unable to perform update.");
	EXPECT_TRUE(result.prns.empty());
}

TEST_F(SinglePointTest, testCorrectMeasurementsElevationMaskElimination) {
	mask_angle                           = 1.0;
	std::vector<TypeSatnavObs> meas_data = navtk::gnssutils::assemble_prs(*observation, pr_to_use);

	TestableSPPP processor = MAKE_PUBLIC_SPPP_FROM_CLASS_VARS();
	processor.receive_aux_data(ephemerides);

	navtk::filtering::CorrectedGnssPseudorangeMeasurement result;
	EXPECT_WARN(result = processor.correct_measurements(meas_data, seconds, ns.pos, week_number),
	            "The elevation mask has eliminated every measurement. Unable to perform update.");
	EXPECT_TRUE(result.prns.empty());
}
