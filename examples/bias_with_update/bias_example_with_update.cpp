#include <memory>

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#include <navtk/factory.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/tensors.hpp>

#include "BiasBlock.hpp"
#include "BiasMeasurementProcessor.hpp"

using aspn_xtensor::to_type_timestamp;

int main() {
	// Create our state block, measurement processor, and fusion engine instance. All together these
	// act as a navigation filter.
	auto block     = std::make_shared<BiasBlock>("my_bias_block");
	auto processor = std::make_shared<BiasMeasurementProcessor>("my_processor", "my_bias_block");
	auto engine    = navtk::filtering::StandardFusionEngine();

	// Add the block and processor to the engine/filter
	engine.add_state_block(block);
	engine.add_measurement_processor(processor);

	// Get and print the initial state estimate and covariance
	auto out     = engine.get_state_block_estimate("my_bias_block");
	auto out_cov = engine.get_state_block_covariance("my_bias_block");
	spdlog::info("Initial:");
	spdlog::info("The state estimate is {}", out);
	spdlog::info("The state covariance is {}\n", out_cov);

	// Propagate to 0.1 seconds
	engine.propagate(to_type_timestamp(0.1));

	// Get and print the state estimate and covariance after propagation
	out     = engine.get_state_block_estimate("my_bias_block");
	out_cov = engine.get_state_block_covariance("my_bias_block");
	spdlog::info("After propagation:");
	spdlog::info("The state estimate is {}", out);
	spdlog::info("The state covariance is {}\n", out_cov);

	// Make a measurement at 0.1 seconds of sensed value 1.0 with cov 1.001
	aspn_xtensor::TypeTimestamp time_validity = to_type_timestamp(0.1);
	navtk::Vector measurement_data            = {1.0};
	navtk::Matrix measurement_covariance      = {{1.001}};
	auto raw_meas = std::make_shared<navtk::filtering::GaussianVectorData>(
	    time_validity, measurement_data, measurement_covariance);

	// Update the filter estimate with the measurement
	engine.update("my_processor", raw_meas);

	// Get and print the state estimate and covariance after the update
	out     = engine.get_state_block_estimate("my_bias_block");
	out_cov = engine.get_state_block_covariance("my_bias_block");
	spdlog::info("After update:");
	spdlog::info("The state estimate is {}", out);
	spdlog::info("The state covariance is {}", out_cov);

	return EXIT_SUCCESS;
}
