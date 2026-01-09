#pragma once

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>
#include <test_data_generation.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/containers/Pose.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/processors/SinglePointPseudorangeProcessor.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/gnssutils/rinex_file_reader.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace {

using aspn_xtensor::MeasurementSatnav;
using aspn_xtensor::TypeSatnavObs;
using aspn_xtensor::TypeSatnavTime;
using navtk::Matrix;
using navtk::Vector;
using navtk::Vector3;
using navtk::filtering::NavSolution;
using navtk::filtering::PairedPva;
using navtk::filtering::SinglePointPseudorangeProcessor;
using navtk::gnssutils::read_ephemeris_from_file_rinex2_1;

using GenXhatPFunction = std::function<std::shared_ptr<navtk::filtering::EstimateWithCovariance>(
    const std::vector<std::string>&)>;

class FakeAux : public aspn_xtensor::AspnBase {
public:
	double d;
	FakeAux(double din = 3.14) : aspn_xtensor::AspnBase(ASPN_UNDEFINED, 0, 0, 0, 0) { d = din; }
};

class SinglePointTest : public ::testing::Test {
public:
	NavSolution ns = NavSolution{
	    navtk::filtering::Pose{Vector3{1, 2, 3},
	                           xt::transpose(navtk::navutils::rpy_to_dcm(Vector3{1, 2, 3})),
	                           aspn_xtensor::TypeTimestamp((int64_t)0)},
	    Vector3()};

	aspn_xtensor::TypeSatnavSatelliteSystem sat_sys{
	    ASPN_TYPE_SATNAV_SATELLITE_SYSTEM_SATELLITE_SYSTEM_SYS_GPS};
	std::vector<aspn_xtensor::TypeSatnavSatelliteSystem> sat_vec{sat_sys};
	size_t a1 = sat_vec.size();
	std::vector<AspnTypeSatnavSignalDescriptorSignalDescriptor> signals{
	    ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1P,
	    ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1C,
	    ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2P,
	    ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2C};
	std::vector<TypeSatnavObs> measurements =
	    navtk::testing::generate_gnss_observations({3, 2}, sat_vec, signals);

	std::vector<TypeSatnavObs> measurements_39 =
	    navtk::testing::generate_gnss_observations({3, 9}, sat_vec, signals);

	std::vector<TypeSatnavObs> measurements_93 =
	    navtk::testing::generate_gnss_observations({9, 3}, sat_vec, signals);

	std::vector<TypeSatnavObs> measurements_923 =
	    navtk::testing::generate_gnss_observations({9, 2, 3}, sat_vec, signals);

	aspn_xtensor::TypeTimestamp timestamp = aspn_xtensor::TypeTimestamp((int64_t)0);
	aspn_xtensor::TypeHeader header{ASPN_MEASUREMENT_SATNAV, 0, 0, 0, 0};

	void SetUp() override {
		std::ifstream rinex("hour1190.18n", std::ios_base::in | std::ios_base::binary);
		if (rinex.is_open()) {
			TypeSatnavTime last_time{0, 0, ASPN_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GPS};
			auto temp_eph = read_ephemeris_from_file_rinex2_1(rinex, last_time);
			rinex.close();
			for (auto ephemeris : temp_eph) {
				ephemerides.push_back(
				    std::make_shared<aspn_xtensor::MetadataGpsLnavEphemeris>(ephemeris));
			}
		}
		processor->receive_aux_data(ephemerides);
	}

	int32_t week_number                      = 1999;
	double seconds                           = 0.0;
	aspn_xtensor::TypeSatnavTime satnav_time = aspn_xtensor::TypeSatnavTime(
	    week_number, seconds, ASPN_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GPS);
	std::shared_ptr<MeasurementSatnav> observation =
	    std::make_shared<MeasurementSatnav>(header,
	                                        timestamp,
	                                        satnav_time,
	                                        4,
	                                        measurements,
	                                        std::vector<aspn_xtensor::TypeIntegrity>{});

	aspn_xtensor::TypeSatnavTime satnav_time2 = aspn_xtensor::TypeSatnavTime(
	    week_number, seconds + 1, ASPN_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GPS);
	std::shared_ptr<MeasurementSatnav> observation_39 =
	    std::make_shared<MeasurementSatnav>(header,
	                                        timestamp,
	                                        satnav_time2,
	                                        4,
	                                        measurements_39,
	                                        std::vector<aspn_xtensor::TypeIntegrity>{});

	aspn_xtensor::TypeSatnavTime satnav_time3 = aspn_xtensor::TypeSatnavTime(
	    week_number, seconds + 2, ASPN_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GPS);
	std::shared_ptr<MeasurementSatnav> observation_923 =
	    std::make_shared<MeasurementSatnav>(header,
	                                        timestamp,
	                                        satnav_time3,
	                                        4,
	                                        measurements_923,
	                                        std::vector<aspn_xtensor::TypeIntegrity>{});

	aspn_xtensor::TypeSatnavTime satnav_time4 = aspn_xtensor::TypeSatnavTime(
	    week_number, seconds + 1, ASPN_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GPS);
	std::shared_ptr<MeasurementSatnav> observation_93 =
	    std::make_shared<MeasurementSatnav>(header,
	                                        timestamp,
	                                        satnav_time4,
	                                        4,
	                                        measurements_93,
	                                        std::vector<aspn_xtensor::TypeIntegrity>{});

	AspnBaseVector ephemerides;
	navtk::not_null<std::shared_ptr<navtk::filtering::StandardFusionEngine>> engine =
	    std::make_shared<navtk::filtering::StandardFusionEngine>();
	navtk::gnssutils::PseudorangeType pr_to_use = navtk::gnssutils::CAL1;
	double pr_noise_cov                         = 9.0;
	double pr_bias_covariance                   = 2.0;
	double pr_time_constant                     = 3600.0;
	bool apply_tropo_model                      = true;
	double tropo_rel_humidity                   = 0.5;
	double mask_angle                           = -1;
	bool force_clock_initialization             = false;
	std::shared_ptr<SinglePointPseudorangeProcessor> processor =
	    initialize_processor_from_class_variables();

	// Setup measurement
	std::shared_ptr<aspn_xtensor::AspnBase> measurement =
	    std::make_shared<PairedPva>(observation, ns);

	virtual std::shared_ptr<SinglePointPseudorangeProcessor>
	initialize_processor_from_class_variables() {
		auto processor =
		    std::make_shared<SinglePointPseudorangeProcessor>("",
		                                                      "",
		                                                      "",
		                                                      pr_to_use,
		                                                      pr_noise_cov,
		                                                      pr_bias_covariance,
		                                                      pr_time_constant,
		                                                      engine,
		                                                      apply_tropo_model,
		                                                      tropo_rel_humidity,
		                                                      mask_angle,
		                                                      force_clock_initialization);
		processor->receive_aux_data(ephemerides);
		return processor;
	}

	std::shared_ptr<navtk::filtering::EstimateWithCovariance> gen_xp(
	    const std::vector<std::string>&, navtk::Size num_bias = 2, double mult = 3) {
		Vector x = navtk::ones(17 + num_bias) * mult;
		// set pseudorange bias estimate to 0
		view(x, xt::range(17, 17 + num_bias)) = 0;
		Matrix x_p                            = navtk::eye(17 + num_bias) * mult;
		return std::make_shared<navtk::filtering::EstimateWithCovariance>(x, x_p);
	}

	void clock_init_test(GenXhatPFunction genxp, const Matrix& expected_H) {
		tropo_rel_humidity         = 0.75;
		force_clock_initialization = true;
		auto processor             = initialize_processor_from_class_variables();
		auto results               = processor->generate_model(measurement, genxp);
		auto h                     = results->h(genxp({""})->estimate);

		EXPECT_ALLCLOSE(Matrix{{0.00010954445447}}, results->R);
		Vector expected_z = {-3.08597945726579};
		EXPECT_ALLCLOSE(expected_z, results->z);
		Vector expected_h = {3};
		EXPECT_ALLCLOSE(expected_h, h);
		EXPECT_ALLCLOSE(expected_H, results->H);
	}

	void elevation_mask_test(GenXhatPFunction genxp, const Matrix& expected_H) {

		pr_to_use          = navtk::gnssutils::PL2;
		tropo_rel_humidity = 0.75;
		mask_angle         = 0.1;
		auto processor     = initialize_processor_from_class_variables();
		auto results       = processor->generate_model(measurement, genxp);
		// should remove PRN 2 because below elevation mask angle, so one fewer bias state than
		// passed
		auto xeval = genxp({""})->estimate;
		xeval      = xt::view(xeval, xt::range(0, navtk::num_rows(xeval) - 1));
		auto h     = results->h(xeval);
		EXPECT_ALLCLOSE(Matrix{{9}}, results->R);
		Vector expected_z = {18020.20227944953};
		EXPECT_ALLCLOSE(expected_z, results->z);
		Vector expected_h = {922952675.1801782};
		EXPECT_ALLCLOSE(expected_h, h);
		EXPECT_ALLCLOSE(expected_H, results->H);
	}

	void single_point_test(GenXhatPFunction genxp, const Matrix& expected_H) {
		pr_to_use          = navtk::gnssutils::PL2;
		tropo_rel_humidity = 0.75;
		auto processor     = initialize_processor_from_class_variables();
		auto results       = processor->generate_model(measurement, genxp);
		auto h             = results->h(genxp({""})->estimate);
		EXPECT_ALLCLOSE(navtk::eye(2) * 9, results->R);
		Vector expected_z = {18020.20227944953, 39196.465577766176};
		EXPECT_ALLCLOSE(expected_z, results->z);
		Vector expected_h = {922952675.1801782, 927411275.1501157};
		EXPECT_ALLCLOSE(expected_h, h);
		EXPECT_ALLCLOSE(expected_H, results->H);
	}

	void alternate_single_point_test(GenXhatPFunction genxp, const Matrix& expected_H) {
		auto ins_solution =
		    NavSolution{navtk::filtering::Pose{
		                    Vector3(), navtk::eye(3), aspn_xtensor::TypeTimestamp((int64_t)0)},
		                Vector3()};
		auto measurement = std::make_shared<PairedPva>(observation, ins_solution);
		auto results     = processor->generate_model(measurement, genxp);
		auto h           = results->h(genxp({""})->estimate);
		EXPECT_ALLCLOSE(navtk::eye(2) * 9, results->R);
		Vector expected_z = {18022.39610814047, 39194.12254739569};
		EXPECT_ALLCLOSE(expected_z, results->z);
		Vector expected_h = {23344810.04399759, 30487634.19455329};
		EXPECT_ALLCLOSE(expected_h, h);
		EXPECT_ALLCLOSE(expected_H, results->H);
	}

	void prn_sb_index_test(GenXhatPFunction genxp,
	                       const Matrix& expected_h_1,
	                       const Matrix& expected_h_2,
	                       const Matrix& expected_h_3,
	                       const Matrix& expected_h_4) {

		auto ins_solution =
		    NavSolution{navtk::filtering::Pose{
		                    Vector3(), navtk::eye(3), aspn_xtensor::TypeTimestamp((int64_t)0)},
		                Vector3()};
		auto measurement = std::make_shared<PairedPva>(observation, ins_solution);

		auto measurement_39 = std::make_shared<PairedPva>(observation_39, ins_solution);

		auto measurement_923 = std::make_shared<PairedPva>(observation_923, ins_solution);

		auto measurement_93 = std::make_shared<PairedPva>(observation_93, ins_solution);

		// Check sb labels
		auto results_1 = processor->generate_model(measurement, genxp);
		ASSERT_EQ(processor->get_state_block_labels(),
		          std::vector<std::string>({"", "", "pr_bias_sv_3", "pr_bias_sv_2"}));
		auto results_2 = processor->generate_model(measurement_39, genxp);
		ASSERT_EQ(processor->get_state_block_labels(),
		          std::vector<std::string>({"", "", "pr_bias_sv_3", "pr_bias_sv_9"}));
		auto results_3 = processor->generate_model(measurement_923, genxp);
		ASSERT_EQ(
		    processor->get_state_block_labels(),
		    std::vector<std::string>({"", "", "pr_bias_sv_9", "pr_bias_sv_2", "pr_bias_sv_3"}));
		auto results_4 = processor->generate_model(measurement_93, genxp);
		ASSERT_EQ(processor->get_state_block_labels(),
		          std::vector<std::string>({"", "", "pr_bias_sv_9", "pr_bias_sv_3"}));

		// Check H matrix generation
		EXPECT_ALLCLOSE(expected_h_1, results_1->H);
		EXPECT_ALLCLOSE(expected_h_2, results_2->H);
		EXPECT_ALLCLOSE(expected_h_3, results_3->H);
		EXPECT_ALLCLOSE(expected_h_4, results_4->H);
	}

	void invalid_data_test(GenXhatPFunction genxp) {
		measurement = std::make_shared<aspn_xtensor::TypeHeader>(ASPN_UNDEFINED, 0, 0, 0, 0);
		decltype(processor->generate_model(measurement, genxp)) model = nullptr;
		EXPECT_HONORS_MODE_EX(model = processor->generate_model(measurement, genxp),
		                      "Measurement is not of correct type",
		                      std::invalid_argument);
		EXPECT_EQ(model, nullptr);
	}

	void bad_aux_test() {
		AspnBaseVector aux_data{std::make_shared<FakeAux>()};
		EXPECT_WARN(processor->receive_aux_data(aux_data), processor->get_label());
	}
};

}  // namespace
