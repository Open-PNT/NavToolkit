#include <memory>

#include <gtest/gtest.h>
#include <spdlog_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

// Child class which overrides pure virtual methods with dummy implementations so the class can be
// concrete.
class TestableMeasurementProcessor : public navtk::filtering::MeasurementProcessor<> {
	using MeasurementProcessor::MeasurementProcessor;  // Inherit constructors

	std::shared_ptr<navtk::filtering::StandardMeasurementModel> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase>, navtk::filtering::GenXhatPFunction) override {
		return std::make_shared<navtk::filtering::StandardMeasurementModel>(
		    navtk::Vector{}, navtk::Matrix{}, navtk::Matrix{});
	}

	navtk::not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override {
		return std::make_shared<TestableMeasurementProcessor>(*this);
	}
};

TEST(MeasurementProcessorTests, receive_aux_data_warning) {
	auto processor_label = "my_label";
	auto processor       = TestableMeasurementProcessor(processor_label, "state_block_label");
	AspnBaseVector aux_data;
	EXPECT_WARN(processor.receive_aux_data(aux_data), processor_label);
}
