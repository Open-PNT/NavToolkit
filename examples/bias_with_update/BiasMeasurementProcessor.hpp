#pragma once

#include <memory>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/not_null.hpp>

class BiasMeasurementProcessor : public navtk::filtering::MeasurementProcessor<> {

public:
	navtk::Matrix measurement_matrix;

	BiasMeasurementProcessor(std::string label, const std::string &state_block_label)
	    : MeasurementProcessor(std::move(label), state_block_label),
	      measurement_matrix(navtk::Matrix{{1.0}}) {}

	std::shared_ptr<navtk::filtering::StandardMeasurementModel> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase> measurement, navtk::filtering::GenXhatPFunction) {

		std::shared_ptr<navtk::filtering::GaussianVectorData> gvd =
		    std::dynamic_pointer_cast<navtk::filtering::GaussianVectorData>(measurement);

		navtk::Vector z = gvd->estimate;
		navtk::Matrix H = measurement_matrix;
		navtk::Matrix R = gvd->covariance;

		return std::make_shared<navtk::filtering::StandardMeasurementModel>(
		    navtk::filtering::StandardMeasurementModel(z, H, R));
	}

	navtk::not_null<std::shared_ptr<MeasurementProcessor<>>> clone() {
		return std::make_shared<BiasMeasurementProcessor>(get_label(), get_state_block_labels()[0]);
	}
};
