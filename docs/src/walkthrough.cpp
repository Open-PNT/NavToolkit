// INCLUDES
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/processors/DirectMeasurementProcessor.hpp>
#include <navtk/filtering/stateblocks/FogmBlock.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/utils/conversions.hpp>
// END

// USINGS
using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::eye;
using navtk::Vector;
using navtk::Vector3;
using navtk::zeros;
using navtk::filtering::DirectMeasurementProcessor;
using navtk::filtering::GaussianVectorData;
using navtk::filtering::hg1700_model;
using navtk::filtering::NavSolution;
using navtk::filtering::Pinson15NedBlock;
using navtk::filtering::Pose;
using navtk::filtering::StandardFusionEngine;
using navtk::filtering::StateBlock;
using std::string;
using std::vector;
using xt::diag;
// END

const auto BARO_SIGMA = 1.0;

int main() {

	// ENGINE CREATION
	auto engine = StandardFusionEngine();
	// END

	// BLOCK CREATION
	auto model = hg1700_model();
	auto block = std::make_shared<Pinson15NedBlock>("pinson15", model);
	// END

	// ADD BLOCK TO ENGINE
	engine.add_state_block(block);
	// END

	// SET COVARIANCE
	auto s0 = Vector{3,
	                 3,
	                 3,
	                 0.03,
	                 0.03,
	                 0.03,
	                 0.0002,
	                 0.0002,
	                 0.0002,
	                 model.accel_bias_sigma(0),
	                 model.accel_bias_sigma(1),
	                 model.accel_bias_sigma(2),
	                 model.gyro_bias_sigma(0),
	                 model.gyro_bias_sigma(1),
	                 model.gyro_bias_sigma(2)};
	auto P0 = diag(s0 * s0);
	engine.set_state_block_covariance("pinson15", P0);
	// END

	// AUX DATA
	auto nav_sol =
	    NavSolution{Vector3{0.0, 0.0, 0.0},  // Position in lat (rad), lon (rad), alt (m).
	                Vector3{1, 0, 0},        // NED Velocity, m/s
	                eye(3),                  // Attitude DCM
	                aspn_xtensor::to_type_timestamp()};  // Time, seconds
	auto aux_data = navtk::utils::to_inertial_aux(nav_sol, Vector3{0.0, 0.0, -9.81}, zeros(3));
	engine.give_state_block_aux_data("pinson15", aux_data);
	// END

	// PROPAGATE
	engine.propagate(to_type_timestamp(1, 0));
	// END

	// GET STATE INFO
	auto states     = engine.get_state_block_estimate("pinson15");
	auto covariance = engine.get_state_block_covariance("pinson15");
	// END

	// MP CREATION
	auto h_alt     = zeros(1, 15);
	h_alt(0, 2)    = 1.0;
	auto processor = std::make_shared<DirectMeasurementProcessor>("altimeter", "pinson15", h_alt);
	// END

	// ADD MP TO ENGINE
	engine.add_measurement_processor(processor);
	// END

	// UPDATE
	auto reference_altitude   = nav_sol.pos[2];
	auto measurement_altitude = 0.5;
	engine.update(
	    "altimeter",
	    std::make_shared<GaussianVectorData>(to_type_timestamp(10, 0),
	                                         Vector{reference_altitude - measurement_altitude},
	                                         navtk::Matrix{{BARO_SIGMA * BARO_SIGMA}}));
	// END

	// CREATE BIAS BLOCK
	auto baro_block = std::make_shared<navtk::filtering::FogmBlock>("baro_bias", 50.0, 10.0, 1);
	// END

	// ADD BIAS BLOCK TO ENGINE
	engine.add_state_block(baro_block);
	// END

	// ADD BIAS MP
	auto h_alt_bias       = zeros(1, 16);
	h_alt_bias(0, 2)      = 1.0;
	h_alt_bias(0, 15)     = 1.0;
	auto biased_processor = std::make_shared<DirectMeasurementProcessor>(
	    "biased_altimeter", vector<string>{"pinson15", "baro_bias"}, h_alt_bias);
	engine.add_measurement_processor(biased_processor);
	// END

	// UPDATE BIAS
	reference_altitude   = nav_sol.pos[2];
	measurement_altitude = 0.5;
	engine.update(
	    "biased_altimeter",
	    std::make_shared<GaussianVectorData>(to_type_timestamp(11, 0),
	                                         Vector{reference_altitude - measurement_altitude},
	                                         navtk::Matrix{{BARO_SIGMA * BARO_SIGMA}}));
	// END

	// UPDATE #2
	reference_altitude   = nav_sol.pos[2];
	measurement_altitude = 0.5;
	engine.update(
	    "altimeter",
	    std::make_shared<GaussianVectorData>(to_type_timestamp(15, 0),
	                                         Vector{reference_altitude - measurement_altitude},
	                                         navtk::Matrix{{BARO_SIGMA * BARO_SIGMA}}));
	// END

}  // main()
