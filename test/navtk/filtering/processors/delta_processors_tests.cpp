#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/processors/AltitudeMeasurementProcessor.hpp>
#include <navtk/filtering/processors/AltitudeMeasurementProcessorWithBias.hpp>
#include <navtk/filtering/processors/BiasedRangeProcessor.hpp>
#include <navtk/filtering/processors/DeltaPositionMeasurementProcessor.hpp>
#include <navtk/filtering/processors/DirectionToPoints3dMeasurementProcessor.hpp>
#include <navtk/filtering/processors/GeodeticPos2dMeasurementProcessor.hpp>
#include <navtk/filtering/processors/GeodeticPos3dMeasurementProcessor.hpp>
#include <navtk/filtering/processors/MagneticFieldMagnitudeMeasurementProcessor.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/filtering/processors/PositionVelocityAttitudeMeasurementProcessor.hpp>
#include <navtk/filtering/processors/VelocityMeasurementProcessor.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::ErrorMode;
using navtk::ErrorModeLock;
using navtk::Matrix;
using navtk::Vector;
using navtk::Vector3;
using navtk::filtering::EstimateWithCovariance;
using navtk::filtering::MeasurementProcessor;
using navtk::filtering::NavSolution;
using navtk::filtering::PairedPva;
using navtk::filtering::Pose;
using navtk::filtering::StandardMeasurementModel;
using navtk::filtering::StandardMeasurementProcessor;
using GenXhatPFunction =
    std::function<std::shared_ptr<EstimateWithCovariance>(const std::vector<std::string> &)>;

using aspn_xtensor::MeasurementAltitude;
using aspn_xtensor::MeasurementDeltaPosition;
using aspn_xtensor::MeasurementDirection3DToPoints;
using aspn_xtensor::MeasurementMagneticFieldMagnitude;
using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::MeasurementRangeToPoint;
using aspn_xtensor::MeasurementVelocity;
using aspn_xtensor::TypeDirection3DToPoint;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeImageFeature;
using aspn_xtensor::TypeRemotePoint;
using aspn_xtensor::TypeTimestamp;

using navtk::filtering::AltitudeMeasurementProcessor;
using navtk::filtering::AltitudeMeasurementProcessorWithBias;
using navtk::filtering::BiasedRangeProcessor;
using navtk::filtering::DeltaPositionMeasurementProcessor;
using navtk::filtering::DirectionToPoints3dMeasurementProcessor;
using navtk::filtering::GeodeticPos2dMeasurementProcessor;
using navtk::filtering::GeodeticPos3dMeasurementProcessor;
using navtk::filtering::MagneticFieldMagnitudeMeasurementProcessor;
using navtk::filtering::PositionVelocityAttitudeMeasurementProcessor;
using navtk::filtering::VelocityMeasurementProcessor;

struct DeltaTests : public ::testing::Test {
	TypeTimestamp timestamp       = TypeTimestamp((int64_t)0);
	TypeHeader header             = TypeHeader(ASPN_UNDEFINED, 0, 0, 0, 0);
	std::string label             = "processor_label";
	std::string state_label       = "state_label";
	aspn_xtensor::TypeTimestamp t = aspn_xtensor::to_type_timestamp(1, 0);

	int default_num_states = 15;
	double lat             = 1.0;
	double lon             = 1.4;
	double alt             = 100.0;
	double vn              = 10.0;
	double ve              = -1.0;
	double vd              = 2.0;
	double roll            = -1.4;
	double pitch           = 0.3;
	double heading         = -0.001;
	NavSolution nav_sol =
	    NavSolution(Pose(Vector3{lat, lon, alt},
	                     xt::transpose(navtk::navutils::rpy_to_dcm(Vector3{roll, pitch, heading})),
	                     t),
	                Vector3{vn, ve, vd});

	void validate_model(StandardMeasurementModel exp,
	                    std::shared_ptr<StandardMeasurementModel> actual,
	                    int num_states) {
		EXPECT_ALLCLOSE(exp.z, actual->z);
		EXPECT_ALLCLOSE(exp.H, actual->H);
		EXPECT_ALLCLOSE(exp.R, actual->R);
		EXPECT_ALLCLOSE(exp.h(navtk::ones(num_states)), actual->h(navtk::ones(num_states)));
	}

	void test_get_correct_linear_meas_model(
	    navtk::not_null<std::shared_ptr<StandardMeasurementProcessor>> proc,
	    std::shared_ptr<aspn_xtensor::AspnBase> meas,
	    Vector z,
	    Matrix H,
	    Matrix cov,
	    int num_states = 15) {

		GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
			return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
			                                                navtk::zeros(num_states, num_states));
		};

		StandardMeasurementModel exp(z, [=](Vector xhat) { return navtk::dot(H, xhat); }, H, cov);
		auto test = proc->generate_model(meas, dummy_xhat_p);
		ASSERT_NE(test, nullptr);
		validate_model(exp, test, num_states);
	}

	void test_invalid_data(navtk::not_null<std::shared_ptr<StandardMeasurementProcessor>> proc) {
		auto error_modes = {ErrorMode::OFF, ErrorMode::LOG, ErrorMode::DIE};
		for (auto mode : error_modes) {
			auto guard = ErrorModeLock(mode);

			auto measurement =
			    std::make_shared<aspn_xtensor::TypeHeader>(ASPN_UNDEFINED, 0, 0, 0, 0);
			auto genxp = [&](const std::vector<std::string> &) {
				return std::make_shared<navtk::filtering::EstimateWithCovariance>(navtk::zeros(1),
				                                                                  navtk::eye(1));
			};
			decltype(proc->generate_model(measurement, genxp)) model = nullptr;
			EXPECT_HONORS_MODE_EX(model = proc->generate_model(measurement, genxp),
			                      "Measurement is not of correct type",
			                      std::invalid_argument);
			EXPECT_EQ(model, nullptr);
		}
	}

	void test_invalid_paired_pva(
	    navtk::not_null<std::shared_ptr<StandardMeasurementProcessor>> proc) {
		auto error_modes = {ErrorMode::OFF, ErrorMode::LOG, ErrorMode::DIE};
		for (auto mode : error_modes) {
			auto guard = ErrorModeLock(mode);

			auto bad_data = std::make_shared<aspn_xtensor::TypeHeader>(ASPN_UNDEFINED, 0, 0, 0, 0);
			auto bad_paired_pva = std::make_shared<PairedPva>(bad_data, nav_sol);
			auto genxp          = [&](const std::vector<std::string> &) {
                return std::make_shared<navtk::filtering::EstimateWithCovariance>(navtk::zeros(1),
                                                                                  navtk::eye(1));
			};
			decltype(proc->generate_model(bad_paired_pva, genxp)) model = nullptr;
			EXPECT_HONORS_MODE_EX(model = proc->generate_model(bad_paired_pva, genxp),
			                      "PairedPva measurement data is not of correct type",
			                      std::invalid_argument);
			EXPECT_EQ(model, nullptr);
		}
	}
};

TEST_F(DeltaTests, PositionMeasurementProcessor2D) {
	Matrix H                                = navtk::zeros(2, default_num_states);
	xt::view(H, xt::all(), xt::range(0, 2)) = navtk::eye(2);
	auto proc  = std::make_shared<GeodeticPos2dMeasurementProcessor>(label, state_label, H);
	Matrix cov = xt::diag(Vector{1, 2});
	auto data =
	    std::make_shared<MeasurementPosition>(header,
	                                          timestamp,
	                                          ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
	                                          lat,
	                                          lon,
	                                          NAN,
	                                          cov,
	                                          ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
	                                          Vector{},
	                                          std::vector<aspn_xtensor::TypeIntegrity>{});
	auto paired_data = std::make_shared<PairedPva>(data, nav_sol);
	test_get_correct_linear_meas_model(proc, paired_data, navtk::zeros(2), H, cov);
	test_invalid_data(proc);
	test_invalid_paired_pva(proc);
}

TEST_F(DeltaTests, VelocityMeasurementProcessor3D) {
	Matrix H                                = navtk::zeros(3, default_num_states);
	xt::view(H, xt::all(), xt::range(3, 6)) = navtk::eye(3);
	auto proc =
	    std::make_shared<VelocityMeasurementProcessor>(label, state_label, H, true, true, true);
	Matrix cov       = xt::diag(Vector{1, 2, 3});
	auto data        = std::make_shared<MeasurementVelocity>(header,
                                                      timestamp,
                                                      ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED,
                                                      vn,
                                                      ve,
                                                      vd,
                                                      cov,
                                                      ASPN_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE,
                                                      Vector{},
                                                      std::vector<aspn_xtensor::TypeIntegrity>{});
	auto paired_data = std::make_shared<PairedPva>(data, nav_sol);
	test_get_correct_linear_meas_model(proc, paired_data, navtk::zeros(3), H, cov);
	test_invalid_data(proc);
	test_invalid_paired_pva(proc);
}

TEST_F(DeltaTests, VelocityMeasurementProcessor2D) {
	Matrix cov = xt::diag(Vector{1, 2});
	std::vector<std::tuple<bool, bool, bool, double, double, double, Matrix>> permutations = {
	    std::make_tuple(true, true, false, vn, ve, 0, Matrix{{1, 0, 0}, {0, 1, 0}}),
	    std::make_tuple(true, false, true, vn, 0, vd, Matrix{{1, 0, 0}, {0, 0, 1}}),
	    std::make_tuple(true, true, false, vn, ve, 0, Matrix{{0, 1, 0}, {1, 0, 0}}),
	    std::make_tuple(false, true, true, 0, ve, vd, Matrix{{0, 1, 0}, {0, 0, 1}}),
	    std::make_tuple(true, false, true, vn, 0, vd, Matrix{{0, 0, 1}, {1, 0, 0}}),
	    std::make_tuple(false, true, true, 0, ve, vd, Matrix{{0, 0, 1}, {0, 1, 0}})};

	for (const auto &perm : permutations) {
		Matrix H                                = navtk::zeros(2, default_num_states);
		xt::view(H, xt::all(), xt::range(3, 6)) = std::get<6>(perm);
		auto proc                               = std::make_shared<VelocityMeasurementProcessor>(
            label, state_label, H, std::get<0>(perm), std::get<1>(perm), std::get<2>(perm));
		auto data =
		    std::make_shared<MeasurementVelocity>(header,
		                                          timestamp,
		                                          ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED,
		                                          std::get<3>(perm),
		                                          std::get<4>(perm),
		                                          std::get<5>(perm),
		                                          cov,
		                                          ASPN_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE,
		                                          Vector{},
		                                          std::vector<aspn_xtensor::TypeIntegrity>{});
		auto paired_data = std::make_shared<PairedPva>(data, nav_sol);
		test_get_correct_linear_meas_model(proc, paired_data, navtk::zeros(2), H, cov);
		test_invalid_data(proc);
		test_invalid_paired_pva(proc);
	}
}

TEST_F(DeltaTests, VelocityMeasurementProcessor1D) {
	std::vector<std::tuple<bool, bool, bool, double, double, double, Matrix>> permutations = {
	    std::make_tuple(true, false, false, vn, 0, 0, Matrix{{1, 0, 0}}),
	    std::make_tuple(false, true, false, 0, ve, 0, Matrix{{0, 1, 0}}),
	    std::make_tuple(false, false, true, 0, 0, vd, Matrix{{0, 0, 1}})};
	Matrix cov = {{1.0}};

	for (const auto &perm : permutations) {
		Matrix H                                = navtk::zeros(1, default_num_states);
		xt::view(H, xt::all(), xt::range(3, 6)) = std::get<2>(perm);
		auto proc                               = std::make_shared<VelocityMeasurementProcessor>(
            label, state_label, H, std::get<0>(perm), std::get<1>(perm), std::get<2>(perm));
		auto data =
		    std::make_shared<MeasurementVelocity>(header,
		                                          timestamp,
		                                          ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED,
		                                          std::get<3>(perm),
		                                          std::get<4>(perm),
		                                          std::get<5>(perm),
		                                          cov,
		                                          ASPN_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE,
		                                          Vector{},
		                                          std::vector<aspn_xtensor::TypeIntegrity>{});
		auto paired_data = std::make_shared<PairedPva>(data, nav_sol);
		test_get_correct_linear_meas_model(proc, paired_data, navtk::zeros(1), H, Matrix{{cov}});
		test_invalid_data(proc);
		test_invalid_paired_pva(proc);
	}
}

TEST_F(DeltaTests, AltitudeMeasurementProcessor) {
	auto proc        = std::make_shared<AltitudeMeasurementProcessor>(label, state_label);
	auto cov         = 1.0;
	Matrix H         = navtk::zeros(1, default_num_states);
	H(0, 2)          = -1.0;
	auto data        = std::make_shared<MeasurementAltitude>(header,
                                                      timestamp,
                                                      ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL,
                                                      alt,
                                                      cov,
                                                      ASPN_MEASUREMENT_ALTITUDE_ERROR_MODEL_NONE,
                                                      Vector{},
                                                      std::vector<aspn_xtensor::TypeIntegrity>{});
	auto paired_data = std::make_shared<PairedPva>(data, nav_sol);
	test_get_correct_linear_meas_model(proc, paired_data, navtk::zeros(1), H, Matrix{{cov}});
	test_invalid_data(proc);
	test_invalid_paired_pva(proc);
}

TEST_F(DeltaTests, AltitudeMeasurementProcessorWithBias) {
	auto proc =
	    std::make_shared<AltitudeMeasurementProcessorWithBias>(label, state_label, "bias_label");
	auto cov         = 1.0;
	Matrix H         = navtk::zeros(1, default_num_states + 1);
	H(0, 2)          = -1.0;
	H(0, 15)         = 1.0;
	auto data        = std::make_shared<MeasurementAltitude>(header,
                                                      timestamp,
                                                      ASPN_MEASUREMENT_ALTITUDE_REFERENCE_MSL,
                                                      alt,
                                                      cov,
                                                      ASPN_MEASUREMENT_ALTITUDE_ERROR_MODEL_NONE,
                                                      Vector{},
                                                      std::vector<aspn_xtensor::TypeIntegrity>{});
	auto paired_data = std::make_shared<PairedPva>(data, nav_sol);
	test_get_correct_linear_meas_model(
	    proc, paired_data, navtk::zeros(1), H, Matrix{{cov}}, default_num_states + 1);
	test_invalid_data(proc);
	test_invalid_paired_pva(proc);
}

TEST_F(DeltaTests, PositionVelocityAttitudeMeasurementProcessorPV) {
	Matrix H                                = navtk::zeros(6, default_num_states);
	xt::view(H, xt::all(), xt::range(0, 6)) = navtk::eye(6);
	auto proc = std::make_shared<PositionVelocityAttitudeMeasurementProcessor>(
	    label, state_label, H, true, true, true, true, true, true, false);
	Matrix cov = xt::diag(Vector{1, 2, 3, 4, 5, 6});
	auto data  = std::make_shared<MeasurementPositionVelocityAttitude>(
        header,
        timestamp,
        ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
        lat,
        lon,
        alt,
        vn,
        ve,
        vd,
        navtk::Vector4(),
        cov,
        ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
        Vector{},
        std::vector<aspn_xtensor::TypeIntegrity>{});
	auto paired_data = std::make_shared<PairedPva>(data, nav_sol);
	test_get_correct_linear_meas_model(proc, paired_data, navtk::zeros(6), H, cov);
	test_invalid_data(proc);
	test_invalid_paired_pva(proc);
}

TEST_F(DeltaTests, DeltaPositionMeasurementProcessor1D) {
	auto delta_n = 10.0;
	auto delta_e = -20.0;
	auto delta_d = 30.0;
	auto delta_t = 2.0;
	Matrix cov   = {{1.0}};
	Vector3 vel{vn, ve, vd};

	std::vector<std::tuple<bool, bool, bool, double, double, double, Vector3>> permutations = {
	    std::make_tuple(true, false, false, delta_n, 0, 0, Vector3{1, 0, 0}),
	    std::make_tuple(false, true, false, 0, delta_e, 0, Vector3{0, 1, 0}),
	    std::make_tuple(false, false, true, 0, 0, delta_d, Vector3{0, 0, 1})};

	for (const auto &perm : permutations) {
		Matrix H                                = navtk::zeros(1, default_num_states);
		xt::view(H, xt::all(), xt::range(3, 6)) = std::get<6>(perm);
		auto proc = std::make_shared<DeltaPositionMeasurementProcessor>(
		    label, state_label, H, std::get<0>(perm), std::get<1>(perm), std::get<2>(perm));
		Vector z;
		if (std::get<0>(perm)) z = (Vector{delta_n} / delta_t - vel[0]);
		if (std::get<1>(perm)) z = (Vector{delta_e} / delta_t - vel[1]);
		if (std::get<2>(perm)) z = (Vector{delta_d} / delta_t - vel[2]);
		auto data = std::make_shared<MeasurementDeltaPosition>(
		    header,
		    timestamp,
		    ASPN_MEASUREMENT_DELTA_POSITION_REFERENCE_FRAME_NED,
		    delta_t,
		    std::get<3>(perm),
		    std::get<4>(perm),
		    std::get<5>(perm),
		    cov,
		    ASPN_MEASUREMENT_DELTA_POSITION_ERROR_MODEL_NONE,
		    Vector{},
		    std::vector<aspn_xtensor::TypeIntegrity>{});
		auto paired_data = std::make_shared<PairedPva>(data, nav_sol);
		test_get_correct_linear_meas_model(proc, paired_data, z, H, Matrix{{cov}});
		test_invalid_data(proc);
		test_invalid_paired_pva(proc);
	}
}

TEST_F(DeltaTests, DeltaPositionMeasurementProcessor3D) {
	Matrix cov   = xt::diag(Vector{1, 2, 3});
	auto delta_n = 10.0;
	auto delta_e = -20.0;
	auto delta_d = 30.0;
	auto delta_t = 2.0;

	Matrix H                                = navtk::zeros(3, default_num_states);
	xt::view(H, xt::all(), xt::range(3, 6)) = navtk::eye(3);

	auto proc = std::make_shared<DeltaPositionMeasurementProcessor>(
	    label, state_label, H, true, true, true);
	Vector z  = Vector{delta_n, delta_e, delta_d} / delta_t - Vector{vn, ve, vd};
	auto data = std::make_shared<MeasurementDeltaPosition>(
	    header,
	    timestamp,
	    ASPN_MEASUREMENT_DELTA_POSITION_REFERENCE_FRAME_NED,
	    delta_t,
	    delta_n,
	    delta_e,
	    delta_d,
	    cov,
	    ASPN_MEASUREMENT_DELTA_POSITION_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	auto paired_data = std::make_shared<PairedPva>(data, nav_sol);
	test_get_correct_linear_meas_model(proc, paired_data, z, H, cov);
	test_invalid_data(proc);
	test_invalid_paired_pva(proc);
}

TEST_F(DeltaTests, BiasedRangeMeasurementProcessor) {
	double range    = 7000;
	double variance = 10;
	Matrix cov      = xt::diag(Vector{1, 2, 3});

	TypeRemotePoint point(
	    2, 0, ASPN_TYPE_REMOTE_POINT_POSITION_REFERENCE_FRAME_GEODETIC, 0.999, 1.399, 100, cov);
	auto data =
	    std::make_shared<MeasurementRangeToPoint>(header,
	                                              timestamp,
	                                              point,
	                                              range,
	                                              variance,
	                                              ASPN_MEASUREMENT_RANGE_TO_POINT_ERROR_MODEL_NONE,
	                                              Vector(),
	                                              std::vector<aspn_xtensor::TypeIntegrity>{});

	auto proc = std::make_shared<BiasedRangeProcessor>(label, "position_label", "bias_label");
	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		int num_states = 15;
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};

	auto test = proc->generate_model(data, dummy_xhat_p);
	ASSERT_NE(test, nullptr);

	// clang-format off
    auto expected_H = Matrix{{29493.33786475472, -3459703.3987212926, -0.00007171183825, 0.99999993108213, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
	// clang-format on
	auto expected_r = Matrix{{variance}};
	auto expected_z = Vector{range};
	auto expected_h = Vector{1380452.156935312};
	EXPECT_ALLCLOSE(test->z, expected_z);
	EXPECT_ALLCLOSE(test->R, expected_r);
	EXPECT_ALLCLOSE(test->H, expected_H);
	EXPECT_ALLCLOSE(test->h(navtk::ones(15)), expected_h);
	test_invalid_data(proc);
}

TEST_F(DeltaTests, DirectionToPoints3dMeasurementProcessor) {
	Matrix pos_cov = xt::diag(Vector{0.1, 0.1, 25});
	Matrix dir_cov = 0.01 * navtk::eye(2);

	TypeImageFeature observation_characteristics(NAN, NAN, NAN, 0, 0, {});

	TypeRemotePoint point1(
	    2, 0, ASPN_TYPE_REMOTE_POINT_POSITION_REFERENCE_FRAME_GEODETIC, 0.999, 1.399, 100, pos_cov);
	TypeDirection3DToPoint obs1(point1,
	                            ASPN_TYPE_DIRECTION_3D_TO_POINT_REFERENCE_FRAME_AZ_EL,
	                            {-10000, -10000},
	                            dir_cov,
	                            false,
	                            observation_characteristics,
	                            ASPN_TYPE_DIRECTION_3D_TO_POINT_ERROR_MODEL_NONE,
	                            {},
	                            {});

	TypeRemotePoint point2(
	    2, 0, ASPN_TYPE_REMOTE_POINT_POSITION_REFERENCE_FRAME_GEODETIC, 1.001, 1.401, 100, pos_cov);
	TypeDirection3DToPoint obs2(point2,
	                            ASPN_TYPE_DIRECTION_3D_TO_POINT_REFERENCE_FRAME_AZ_EL,
	                            {10000, 10000},
	                            dir_cov,
	                            false,
	                            observation_characteristics,
	                            ASPN_TYPE_DIRECTION_3D_TO_POINT_ERROR_MODEL_NONE,
	                            {},
	                            {});

	std::vector<TypeDirection3DToPoint> obs{obs1, obs2};

	auto data        = std::make_shared<MeasurementDirection3DToPoints>(header, timestamp, obs);
	auto paired_data = std::make_shared<PairedPva>(data, nav_sol);

	auto proc = std::make_shared<DirectionToPoints3dMeasurementProcessor>(label, state_label);
	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		int num_states = 15;
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};

	auto test = proc->generate_model(paired_data, dummy_xhat_p);
	ASSERT_NE(test, nullptr);

	// clang-format off
    auto expected_H = Matrix {
        {0.00023361159898, -0.00043143986297, -0.00015024802203, 0, 0, 0, -0.55716615876854, -3.66599952398765, -0.34123588534417, 0, 0, 0, 0, 0, 0},
        {-0.00007331541292, 0.00013574063418, -0.00023744749272, 0, 0, 0,  1.11644192944662,  0.55716615876854, -1.63278887918422, 0, 0, 0, 0, 0, 0},
        {-0.00023412628548, 0.00043357879319,  0.00015079396475, 0, 0, 0, -0.56452415299252, -3.67939220464842, -0.34487711992366, 0, 0, 0, 0, 0, 0},
        {0.00007344452771, -0.00013703636034,  0.00023786907425, 0, 0, 0,  1.11894022784684,  0.56452415299252, -1.63688490879732, 0, 0, 0, 0, 0, 0}
    };
	auto expected_r = Matrix{
        {0.02000117686635, 0.00000176866256, 0, 0},
        {0.00000176866256, 0.02000282382572, 0, 0},
        {0, 0, 0.02000118550213, 0.00000177813877},
        {0, 0, 0.00000177813877, 0.02000283391944}
    };
	// clang-format on
	auto expected_z =
	    Vector{-9999.999999999998, -9999.999999999998, 9999.999999999998, 9999.999999999998};
	auto expected_h =
	    Vector{0.09786300266447, -0.32747152269061, 0.09810146161053, -0.32929525585519};
	EXPECT_ALLCLOSE(test->z, expected_z);
	EXPECT_ALLCLOSE(test->R, expected_r);
	EXPECT_ALLCLOSE(test->H, expected_H);
	EXPECT_ALLCLOSE(test->h(navtk::ones(15)), expected_h);
	test_invalid_data(proc);
	test_invalid_paired_pva(proc);
}

TEST_F(DeltaTests, MagneticFieldMagnitudeMeasurementProcessor) {
	Vector x{0, 10, 20, 30, 40, 50, 60, 70, 80, 90};
	Vector y{0, 10, 20, 30, 40, 50, 60, 70, 80, 90};
	// clang-format off
	Matrix q = {
        {0,   10,  20,  30,  40,  50,  60,  70,  80,  90},
        {10,  20,  30,  40,  50,  60,  70,  80,  90, 100},
        {20,  30,  40,  50,  60,  70,  80,  90, 100, 110},
        {30,  40,  50,  60,  70,  80,  90, 100, 110, 120},
        {40,  50,  60,  70,  80,  90, 100, 110, 120, 130},
        {50,  60,  70,  80,  90, 100, 110, 120, 130, 140},
        {60,  70,  80,  90, 100, 110, 120, 130, 140, 150},
        {70,  80,  90, 100, 110, 120, 130, 140, 150, 160},
        {80,  90, 100, 110, 120, 130, 140, 150, 160, 170},
        {90, 100, 110, 120, 130, 140, 150, 160, 170, 180},
    };
	// clang-format on
	double magnitude = 0.5;
	double variance  = 0.01;
	auto data        = std::make_shared<MeasurementMagneticFieldMagnitude>(
        header,
        timestamp,
        magnitude,
        variance,
        ASPN_MEASUREMENT_MAGNETIC_FIELD_MAGNITUDE_ERROR_MODEL_NONE,
        Vector{},
        std::vector<aspn_xtensor::TypeIntegrity>{});
	auto paired_data = std::make_shared<PairedPva>(data, nav_sol);

	auto proc =
	    std::make_shared<MagneticFieldMagnitudeMeasurementProcessor>(label, state_label, x, y, q);
	GenXhatPFunction dummy_xhat_p = [=](const std::vector<std::string> &) {
		int num_states = 15;
		return std::make_shared<EstimateWithCovariance>(navtk::ones(num_states),
		                                                navtk::zeros(num_states, num_states));
	};

	auto test = proc->generate_model(paired_data, dummy_xhat_p);
	ASSERT_NE(test, nullptr);

	auto expected_H =
	    Matrix{{0.00000897946883, 0.00001658670873, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
	auto expected_r = Matrix{{variance}};
	auto expected_z = Vector{magnitude};
	auto expected_h = Vector{137.50989639757483};
	EXPECT_ALLCLOSE(test->z, expected_z);
	EXPECT_ALLCLOSE(test->R, expected_r);
	EXPECT_ALLCLOSE(test->H, expected_H);
	EXPECT_ALLCLOSE(test->h(navtk::ones(15)), expected_h);
	test_invalid_data(proc);
	test_invalid_paired_pva(proc);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, DeltaTests, AltitudeMeasurementProcessorWithBiasBadConstruct) {
	EXPECT_HONORS_MODE_EX(
	    (void)AltitudeMeasurementProcessorWithBias("", std::vector<std::string>()),
	    "exactly 2 state block labels.",
	    std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, DeltaTests, BiasedRangeProcessorBadConstruct) {
	EXPECT_HONORS_MODE_EX((void)BiasedRangeProcessor("", std::vector<std::string>()),
	                      "exactly 2 state block labels.",
	                      std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, DeltaTests, VelocityMeasurementProcessor2DBadConstruct) {
	EXPECT_HONORS_MODE_EX((void)VelocityMeasurementProcessor("", "", {}, true, true, true),
	                      "must have exactly 3 rows",
	                      std::range_error);
	EXPECT_HONORS_MODE_EX(
	    (void)VelocityMeasurementProcessor("", std::vector<std::string>(), {}, true, true, true),
	    "must have exactly 3 rows",
	    std::range_error);
}
