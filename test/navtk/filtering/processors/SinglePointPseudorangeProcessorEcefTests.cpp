#include <filtering/processors/SinglePointPseudorangeProcessorTests.hpp>

#include <fstream>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/processors/SinglePointPseudorangeProcessor.hpp>
#include <navtk/filtering/processors/SinglePointPseudorangeProcessorEcef.hpp>
#include <navtk/gnssutils/assemble_prs.hpp>
#include <navtk/gnssutils/rinex_file_reader.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using navtk::Matrix;
using navtk::filtering::NavSolution;

class SinglePointEcefTest : public SinglePointTest {
public:
	std::shared_ptr<navtk::filtering::SinglePointPseudorangeProcessor>
	initialize_processor_from_class_variables() override {
		auto processor = std::make_shared<navtk::filtering::SinglePointPseudorangeProcessorEcef>(
		    "",
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

	void SetUp() override {
		processor = initialize_processor_from_class_variables();
		::SinglePointTest::SetUp();
	}

	std::shared_ptr<navtk::filtering::EstimateWithCovariance> gen_xp_ecef(
	    const std::vector<std::string>& s,
	    const NavSolution& ins,
	    navtk::Size num_bias = 2,
	    double mult          = 3) {
		auto xp3 = gen_xp(s, num_bias, mult);
		// State vect in this case should be ecef pos, not ned error
		auto corrected = navtk::filtering::apply_error_states<navtk::filtering::Pinson15NedBlock>(
		    ins, xp3->estimate);
		auto ecef = navtk::navutils::llh_to_ecef(corrected.pos);
		// Covariance incorrect (would need full-up VSB stack to convert), but shouldn't matter
		auto new_x = xt::concatenate(xt::xtuple(
		    ecef, xt::view(xp3->estimate, xt::range(15, navtk::num_rows(xp3->estimate)))));
		return std::make_shared<navtk::filtering::EstimateWithCovariance>(new_x,
		                                                                  navtk::eye(5 + num_bias));
	}

	// Former is PinsonNed related, w/ -los_ned in first 3 columns
	// we're doing -los_ned * Cne, with los_ned as a row vec
	// which ends up equivalent to Cen * -los_ned, w/ los_ned as a col vec
	// which recovers the line-of-sight in ecef frame, which is what we want
	Matrix transform_meas_jac(const NavSolution& ns, const Matrix& former) {
		auto cen                                       = navtk::navutils::llh_to_cen(ns.pos);
		auto num_states                                = navtk::num_cols(former);
		auto tx                                        = navtk::eye(num_states);
		xt::view(tx, xt::range(0, 3), xt::range(0, 3)) = xt::transpose(cen);
		return navtk::dot(former, tx);
	}
};

TEST_F(SinglePointEcefTest, testSinglePointClockInit) {
	auto genxp        = [&](const std::vector<std::string>& s) { return gen_xp_ecef(s, ns, 2, 3); };
	Matrix expected_H = {{0, 0, 0, 1, 0, 0, 0}};
	clock_init_test(genxp, expected_H);
}

TEST_F(SinglePointEcefTest, testSinglePointElevationMask) {
	auto genxp = [&](const std::vector<std::string>& s) { return gen_xp_ecef(s, ns, 2, 3); };
	// clang-format off
  Matrix expected_H = transform_meas_jac(ns, {{-0.69510116290395, 0.62080841393133, 0.36253453148858, 299792458, 0, 1.0}});
	// clang-format on
	elevation_mask_test(genxp, expected_H);
}

TEST_F(SinglePointEcefTest, testSinglePoint) {
	auto genxp = [&](const std::vector<std::string>& s) { return gen_xp_ecef(s, ns, 2, 3); };
	// clang-format off
    Matrix expected_H = transform_meas_jac(ns, {
        {-0.69510116290395,  0.62080841393133,  0.36253453148858, 299792458, 0, 1.0, 0.0},
        {-0.57954885217311, -0.75914314087664, -0.29635252589574, 299792458, 0, 0.0, 1.0}
    });
	// clang-format on
	single_point_test(genxp, expected_H);
}

TEST_F(SinglePointEcefTest, testSinglePointAlternate) {
	auto empty_ns =
	    NavSolution{navtk::filtering::Pose{
	                    navtk::Vector3(), navtk::eye(3), aspn_xtensor::TypeTimestamp((int64_t)0)},
	                navtk::Vector3()};
	auto genxp = [&](const std::vector<std::string>& s) { return gen_xp_ecef(s, empty_ns, 2, 0); };
	// clang-format off
    Matrix expected_H = transform_meas_jac(empty_ns, {
        {-0.91625305231276, -0.03821078935438,  0.39877346915819, 299792458, 0, 1.0, 0.0},
        {-0.2339043373106,   0.72909355880896, -0.64320396725345, 299792458, 0, 0.0, 1.0}
    });
	// clang-format on
	alternate_single_point_test(genxp, expected_H);
}

TEST_F(SinglePointEcefTest, testWrongAspnBaseVectorType) { bad_aux_test(); }

ERROR_MODE_SENSITIVE_TEST(TEST_F, SinglePointEcefTest, testInvalidData) {
	auto genxp = [&](const std::vector<std::string>& s) {
		return test.gen_xp_ecef(s, test.ns, 2, 0);
	};
	test.invalid_data_test(genxp);
}
