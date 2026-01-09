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
#include <navtk/filtering/processors/PseudorangeDopplerProcessorEcef.hpp>
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
using navtk::filtering::MeasurementProcessor;
using navtk::filtering::NavSolution;
using navtk::filtering::PairedPva;
using navtk::filtering::PseudorangeDopplerProcessorEcef;
using navtk::gnssutils::read_ephemeris_from_file_rinex2_1;

using GenXhatPFunction = std::function<std::shared_ptr<navtk::filtering::EstimateWithCovariance>(
    const std::vector<std::string>&)>;

class FakeAux : public aspn_xtensor::AspnBase {
public:
	double d;
	FakeAux(double din = 3.14) : aspn_xtensor::AspnBase(ASPN_EXTENDED_BEGIN, 0, 0, 0, 0) {
		d = din;
	}
};

class CarrierPhaseObservationTest : public ::testing::Test {
public:
	NavSolution ns = NavSolution(Vector3{1, 2, 3},
	                             Vector3{11, 22, 33},
	                             navtk::navutils::rpy_to_dcm(Vector3{1, 2, 3}),
	                             aspn_xtensor::TypeTimestamp((int64_t)0));

	std::vector<TypeSatnavObs> measurements = navtk::testing::generate_gnss_observations(
	    {3, 2},
	    {aspn_xtensor::TypeSatnavSatelliteSystem(
	        ASPN_TYPE_SATNAV_SATELLITE_SYSTEM_SATELLITE_SYSTEM_SYS_GPS)},
	    {ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1P,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1C,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2P,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2C});

	std::vector<TypeSatnavObs> measurements_39 = navtk::testing::generate_gnss_observations(
	    {3, 9},
	    {aspn_xtensor::TypeSatnavSatelliteSystem(
	        ASPN_TYPE_SATNAV_SATELLITE_SYSTEM_SATELLITE_SYSTEM_SYS_GPS)},
	    {ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1P,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1C,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2P,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2C});

	std::vector<TypeSatnavObs> measurements_93 = navtk::testing::generate_gnss_observations(
	    {9, 3},
	    {aspn_xtensor::TypeSatnavSatelliteSystem(
	        ASPN_TYPE_SATNAV_SATELLITE_SYSTEM_SATELLITE_SYSTEM_SYS_GPS)},
	    {ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1P,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1C,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2P,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2C});

	std::vector<TypeSatnavObs> measurements_923 = navtk::testing::generate_gnss_observations(
	    {9, 2, 3},
	    {aspn_xtensor::TypeSatnavSatelliteSystem(
	        ASPN_TYPE_SATNAV_SATELLITE_SYSTEM_SATELLITE_SYSTEM_SYS_GPS)},
	    {ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1P,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1C,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2P,
	     ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2C});

	aspn_xtensor::TypeTimestamp timestamp = aspn_xtensor::TypeTimestamp((int64_t)0);
	aspn_xtensor::TypeHeader header{ASPN_MEASUREMENT_SATNAV, 0, 0, 0, 0};

	void SetUp() override {
		std::vector<aspn_xtensor::MetadataGpsLnavEphemeris> temp_eph;
		std::ifstream rinex("hour1190.18n", std::ios_base::in | std::ios_base::binary);
		if (rinex.is_open()) {
			TypeSatnavTime last_time{0, 0, ASPN_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GPS};
			temp_eph = read_ephemeris_from_file_rinex2_1(rinex, last_time);
			rinex.close();
		}
		for (auto ephemeris : temp_eph) {
			ephemerides.push_back(
			    std::make_shared<aspn_xtensor::MetadataGpsLnavEphemeris>(ephemeris));
		}
		processor->receive_aux_data(ephemerides);
	}

	// Former is PinsonNed related, w/ -los_ned in first 3 columns
	// we're doing -los_ned * Cne, with los_ned as a row vec
	// which ends up equivalent to `Cen` * -los_ned, w/ los_ned as a col vec
	// which recovers the line-of-sight in ECEF frame, which is what we want
	Matrix transform_meas_jac(const NavSolution& ns, const Matrix& former) {
		auto cen                                       = navtk::navutils::llh_to_cen(ns.pos);
		auto num_states                                = navtk::num_cols(former);
		auto tx                                        = navtk::eye(num_states);
		xt::view(tx, xt::range(0, 3), xt::range(0, 3)) = xt::transpose(cen);
		xt::view(tx, xt::range(3, 6), xt::range(3, 6)) = xt::transpose(cen);
		return navtk::dot(former, tx);
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
	navtk::gnssutils::CarrierPhaseType cp_to_use      = navtk::gnssutils::L1;
	double pr_noise_cov                               = 9.0;
	double pr_bias_covariance                         = 2.0;
	double pr_time_constant                           = 3600.0;
	double prr_noise_cov                              = 9.0;
	bool apply_tropo_model                            = true;
	double tropo_rel_humidity                         = 0.5;
	double mask_angle                                 = -1;
	bool force_clock_initialization                   = false;
	std::shared_ptr<MeasurementProcessor<>> processor = initialize_processor_from_class_variables();

	// Setup measurement
	std::shared_ptr<aspn_xtensor::AspnBase> measurement =
	    std::make_shared<PairedPva>(observation, ns);

	virtual std::shared_ptr<MeasurementProcessor<>> initialize_processor_from_class_variables() {
		auto processor =
		    std::make_shared<PseudorangeDopplerProcessorEcef>("",
		                                                      "",
		                                                      "",
		                                                      cp_to_use,
		                                                      pr_noise_cov,
		                                                      pr_bias_covariance,
		                                                      pr_time_constant,
		                                                      prr_noise_cov,
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

	std::shared_ptr<navtk::filtering::EstimateWithCovariance> gen_xp_ecef(
	    const std::vector<std::string>& s,
	    const NavSolution& ins,
	    navtk::Size num_bias = 2,
	    double mult          = 3) {
		auto xp3 = gen_xp(s, num_bias, mult);
		// State vector in this case should be ECEF position, not NED error
		auto corrected = navtk::filtering::apply_error_states<navtk::filtering::Pinson15NedBlock>(
		    ins, xp3->estimate);
		auto ecef = navtk::navutils::llh_to_ecef(corrected.pos);
		auto vel  = corrected.vel;
		// Covariance incorrect (would need full-up VSB stack to convert), but shouldn't matter
		auto new_x = xt::concatenate(xt::xtuple(
		    ecef, vel, xt::view(xp3->estimate, xt::range(15, navtk::num_rows(xp3->estimate)))));
		return std::make_shared<navtk::filtering::EstimateWithCovariance>(new_x,
		                                                                  navtk::eye(8 + num_bias));
	}

	void clock_init_test(GenXhatPFunction genxp, const Matrix& expected_H) {
		tropo_rel_humidity         = 0.75;
		force_clock_initialization = true;
		auto processor             = initialize_processor_from_class_variables();
		auto results               = processor->generate_model(measurement, genxp);
		auto h                     = results->h(genxp({""})->estimate);

		Matrix expected_R = {{1.095447e-04, 0.000000e+00}, {0.000000e+00, 2.138994e-14}};
		EXPECT_ALLCLOSE(expected_R, results->R);
		Vector expected_z = {-3.085979, -3.000002};
		EXPECT_ALLCLOSE(expected_z, results->z);
		Vector expected_h = {3, 3};
		EXPECT_ALLCLOSE(expected_h, h);
		EXPECT_ALLCLOSE(expected_H, results->H);
	}

	void elevation_mask_test(GenXhatPFunction genxp, const Matrix& expected_H) {

		cp_to_use          = navtk::gnssutils::L2;
		tropo_rel_humidity = 0.75;
		mask_angle         = 0.1;
		auto processor     = initialize_processor_from_class_variables();
		auto results       = processor->generate_model(measurement, genxp);
		// should remove PRN 2 because below elevation mask angle, so one fewer bias state than
		// passed
		auto xeval = genxp({""})->estimate;
		xeval      = xt::view(xeval, xt::range(0, navtk::num_rows(xeval) - 1));
		auto h     = results->h(xeval);
		EXPECT_ALLCLOSE(navtk::eye(2) * 9., results->R);
		Vector expected_z = {18020.20227944953, -0.322357};
		EXPECT_ALLCLOSE(expected_z, results->z);
		Vector expected_h = {9.229527e+08, -8.993779e+08};
		EXPECT_ALLCLOSE(expected_h, h);
		EXPECT_ALLCLOSE(expected_H, results->H);
	}

	void single_point_test(GenXhatPFunction genxp, const Matrix& expected_H) {
		cp_to_use          = navtk::gnssutils::L2;
		tropo_rel_humidity = 0.75;
		auto processor     = initialize_processor_from_class_variables();
		auto results       = processor->generate_model(measurement, genxp);
		auto h             = results->h(genxp({""})->estimate);
		EXPECT_ALLCLOSE(navtk::eye(4) * 9, results->R);
		Vector expected_z = {18020.20227944953, 39196.465577766176, -0.322357, -0.322357};
		EXPECT_ALLCLOSE(expected_z, results->z);
		Vector expected_h = {9.229527e+08, 9.274113e+08, -8.993779e+08, -8.993779e+08};
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
		EXPECT_ALLCLOSE(navtk::eye(4) * 9, results->R);
		Vector expected_z = {18022.39610814047, 39194.12254739569, -0.439578, -0.439578};
		EXPECT_ALLCLOSE(expected_z, results->z);
		Vector expected_h = {23344810.04399759, 30487634.19455329, 258.6117, -108.5702};
		EXPECT_ALLCLOSE(expected_h, h);
		EXPECT_ALLCLOSE(expected_H, results->H);
	}

	void prn_sb_index_test(GenXhatPFunction genxp,
	                       const Matrix& expected_h_1,
	                       const Matrix& expected_h_2,
	                       const Matrix& expected_h_3) {

		auto ins_solution =
		    NavSolution{navtk::filtering::Pose{
		                    Vector3(), navtk::eye(3), aspn_xtensor::TypeTimestamp((int64_t)0)},
		                Vector3()};
		auto measurement = std::make_shared<PairedPva>(observation, ins_solution);

		auto measurement_39 = std::make_shared<PairedPva>(observation_39, ins_solution);

		auto measurement_923 = std::make_shared<PairedPva>(observation_923, ins_solution);

		auto measurement_93 = std::make_shared<PairedPva>(observation_93, ins_solution);

		// measurement_93 is same as 39, just PRN order swapped. Pseudoranges are grouped before
		// Doppler. Time tags on measurements are the same so individual H elements are the same,
		// but this could potentially mess up internal time-based tracking state in future tests so
		// once verified expected_h_4 at time 3.0 or something should be hardcoded as the others are
		auto expected_h_4 =
		    navtk::zeros(navtk::num_rows(expected_h_2), navtk::num_cols(expected_h_2));
		xt::view(expected_h_4, 0, xt::all()) = xt::view(expected_h_2, 1, xt::all());
		xt::view(expected_h_4, 1, xt::all()) = xt::view(expected_h_2, 0, xt::all());
		xt::view(expected_h_4, 2, xt::all()) = xt::view(expected_h_2, 3, xt::all());
		xt::view(expected_h_4, 3, xt::all()) = xt::view(expected_h_2, 2, xt::all());

		// Check the state block labels are adjusted for each measurement
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

		// Since pr biases are organized in order of meas, order of bias appearance is also swapped
		xt::view(expected_h_4,
		         xt::range(0, 2),
		         xt::range(navtk::num_cols(expected_h_2) - 2, navtk::num_cols(expected_h_2))) =
		    navtk::eye(2);

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
