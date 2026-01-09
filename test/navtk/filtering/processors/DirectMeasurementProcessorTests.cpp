#include <memory>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/processors/DirectMeasurementProcessor.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::dot;
using navtk::eye;
using navtk::Matrix;
using navtk::num_cols;
using navtk::num_rows;
using navtk::ones;
using navtk::Vector;
using navtk::zeros;
using navtk::filtering::DirectMeasurementProcessor;
using navtk::filtering::StandardMeasurementModel;

class TestableDMP : public DirectMeasurementProcessor {
public:
	TestableDMP(std::string label, std::string state_block_label, Matrix measurement_matrix)
	    : DirectMeasurementProcessor(label, state_block_label, measurement_matrix) {}

	TestableDMP(std::string label,
	            std::vector<std::string> state_block_labels,
	            Matrix measurement_matrix)
	    : DirectMeasurementProcessor(label, state_block_labels, measurement_matrix) {}

	TestableDMP(const TestableDMP& processor) : DirectMeasurementProcessor(processor) {}
	navtk::not_null<std::shared_ptr<MeasurementProcessor<>>> clone() {
		return std::make_shared<TestableDMP>(*this);
	}
};

struct DirectTests : public ::testing::Test {
	std::string label             = "test";
	std::string state_label       = "dState";
	Vector x                      = zeros(3);
	Matrix P                      = eye(3);
	Vector z                      = Vector{1.0};
	Matrix R                      = Matrix{{1.0}};
	Matrix H                      = Matrix{{0.0, 2.0, 0.0}};
	aspn_xtensor::TypeTimestamp t = aspn_xtensor::to_type_timestamp(1, 0);
	TestableDMP dmp;
	std::shared_ptr<navtk::filtering::GaussianVectorData> meas;

	DirectTests()
	    : dmp(label, state_label, H),
	      meas(std::make_shared<navtk::filtering::GaussianVectorData>(t, z, R)) {}
};

std::shared_ptr<navtk::filtering::EstimateWithCovariance> test_gen_x_and_p(
    const std::vector<std::string>&) {
	return std::make_shared<navtk::filtering::EstimateWithCovariance>(zeros(3), eye(3));
}

TEST_F(DirectTests, callAux) { dmp.receive_aux_data(AspnBaseVector()); }

TEST_F(DirectTests, clone) {
	auto dmp_copy_cast = std::dynamic_pointer_cast<TestableDMP>(dmp.clone());
	auto& dmp_copy     = *dmp_copy_cast;

	ASSERT_EQ(dmp.get_state_block_labels()[0], dmp_copy.get_state_block_labels()[0]);

	ASSERT_EQ(dmp.get_label(), dmp_copy.get_label());
	ASSERT_EQ(dmp.get_measurement_matrix(), dmp_copy.get_measurement_matrix());
}

TEST_F(DirectTests, altCtor) {
	DirectMeasurementProcessor(label, std::vector<std::string>(1, state_label), H);
}

TEST_F(DirectTests, provideFunction) {
	std::shared_ptr<StandardMeasurementModel> gen_model =
	    dmp.generate_model(meas, test_gen_x_and_p);
	ASSERT_EQ(gen_model->z, z);
	ASSERT_EQ(gen_model->H, H);
	ASSERT_EQ(gen_model->R, R);
	ASSERT_ALLCLOSE(gen_model->h(x), dot(H, x));
}

TEST_F(DirectTests, badMeasInput) {
	// TODO: This will later need to change for optional empty rather than nullptr
	auto meas_bad = std::make_shared<aspn_xtensor::TypeHeader>(ASPN_UNDEFINED, 0, 0, 0, 0);
	ASSERT_EQ(dmp.generate_model(meas_bad, test_gen_x_and_p), nullptr);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, DirectTests, invalid_data) {
	auto measurement = std::make_shared<aspn_xtensor::TypeHeader>(ASPN_UNDEFINED, 0, 0, 0, 0);
	auto genxp       = [&](const std::vector<std::string>&) {
        return std::make_shared<navtk::filtering::EstimateWithCovariance>(navtk::zeros(1),
                                                                          navtk::eye(1));
	};
	decltype(test.dmp.generate_model(measurement, genxp)) model = nullptr;
	EXPECT_HONORS_MODE_EX(model = test.dmp.generate_model(measurement, genxp),
	                      "Measurement is not of correct type",
	                      std::invalid_argument);
	EXPECT_EQ(model, nullptr);
}
