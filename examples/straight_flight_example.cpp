#include <cmath>
#include <memory>
#include <vector>

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <utils/exampleutils.hpp>
#include <xtensor-python/pytensor.hpp>
#include <xtensor/generators/xrandom.hpp>

#include <navtk/factory.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/processors/DirectMeasurementProcessor.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <utils/exampleutils.hpp>

using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::dot;
using navtk::eye;
using navtk::ones;
using navtk::Vector;
using navtk::Vector3;
using navtk::zeros;
using navtk::exampleutils::constant_vel_pva;
using navtk::filtering::apply_error_states;
using navtk::filtering::DirectMeasurementProcessor;
using navtk::filtering::Pinson15NedBlock;
using std::fmod;
using std::make_shared;
using std::vector;
using xt::all;
using xt::range;
using xt::view;
using xt::random::randn;

namespace {
// A small number for floating-point comparison
constexpr auto EPSILON = 0.0001;
// Choose which aiding sources to enable
constexpr auto VELO_ENABLED = true;
constexpr auto POS_ENABLED  = true;
constexpr auto BARO_ENABLED = true;

// Turn feedback on/off
constexpr auto FEEDBACK_ENABLED = true;

// Some convenient constants (in seconds)
constexpr auto MINUTE = 60.0;

// If the source is enabled, choose the measurement sigma in meters
constexpr auto VELO_SIGMA = 10.0;
constexpr auto POS_SIGMA  = 5.0;
constexpr auto BARO_SIGMA = 30.0;

// If the source is enabled, this is the interval (sec) at which we receive measurements
// (NOTE: needs to be a multiple of DT)
constexpr auto VELO_INTERVAL = 20.0;
constexpr auto POS_INTERVAL  = 5 * MINUTE;
constexpr auto BARO_INTERVAL = 5.0;

// Choose interval to apply feedback
constexpr auto FEEDBACK_INTERVAL = 10 * MINUTE;

// How often we propagate our solution
constexpr auto DT = 1.0;

// Number of states in Pinson block
constexpr auto NUM_STATES = 15;

}  // namespace
/**
 *  An example filter with an aircraft flying const velocity 1m/s north straight and level.
 *  This example has toggleable variables to enable/disable velocity aiding, baro aiding,
 *  and gps (3D position) aiding, as well as toggleable INS grade and sensor noise levels,
 *  with all measurements and errors simulated.
 *
 *  This example is intended to illustrate usage of the pinson15 block and how it interacts with
 *  various aiding sensor types, as well as the ease of rapidly plugging in new aiding sensors
 *  into a working filter. Inertial mechanization is simulated by adding modeled inertial errors to
 *  a perfect trajectory. Feedback to this 'inertial' is optionally employed to keep the reference
 *  trajectory from becoming so incorrect during longer examples that the model breaks down.
 */
int main(int argc, char* argv[]) {
	auto plot_results = true;
	if (argc > 1) {
		plot_results = strcmp(argv[1], "1") == 0;
	}

	// indicates the number of minutes (in log file time) to process
	auto process_minutes = 20;
	if (argc > 2) {
		process_minutes = atoi(argv[2]);
	}


	const auto runtime    = process_minutes * MINUTE;
	const auto num_epochs = runtime / DT;

	// Create the fusion engine and add appropriate modules to create a navigation filter.
	auto engine = navtk::filtering::StandardFusionEngine();

	// Choose INS model for the Pinson state block
	// Tactical Grade
	auto model = navtk::filtering::hg1700_model();

	// Navigation Grade
	// auto model = navtk::filtering::hg9900_model();

	// Custom Model
	// auto model = navtk::filtering::ImuModel{zeros(3) + 3e-3,   // accel_random_walk_sigma
	//                                                zeros(3) + 3e-5,   // gyro_random_walk_sigma
	//                                                zeros(3) + 1e-2,   // accel_bias_sigma
	//                                                zeros(3) + 3600,   // accel_bias_tau
	//                                                zeros(3) + 5e-6,   // gyro_bias_sigma
	//                                                zeros(3) + 3600};  // gyro_bias_tau

	// Create our desired state block and add to the engine/filter
	auto block = make_shared<Pinson15NedBlock>("pinson15", model);
	engine.add_state_block(block);

	// An identical state block to model the true errors for comparison against the filter
	auto true_error_block = Pinson15NedBlock("trueError", model);

	// Initialize filter uncertainty. Units are m, m/s, rad, m/s^2, and rad/s
	auto s0 = Vector{3,
	                 3,
	                 3,
	                 0.03,
	                 0.03,
	                 0.03,
	                 0.0002,
	                 0.0002,
	                 0.0002,
	                 model.accel_bias_initial_sigma(0),
	                 model.accel_bias_initial_sigma(1),
	                 model.accel_bias_initial_sigma(2),
	                 model.gyro_bias_initial_sigma(0),
	                 model.gyro_bias_initial_sigma(1),
	                 model.gyro_bias_initial_sigma(2)};

	auto P0 = xt::diag(s0 * s0);
	engine.set_state_block_covariance("pinson15", P0);

	// Create MeasurementProcessors and add to the engine/filter
	auto hAlt  = zeros(1, NUM_STATES);
	hAlt(0, 2) = 1.0;
	engine.add_measurement_processor(
	    make_shared<DirectMeasurementProcessor>("altimeter", "pinson15", hAlt));

	auto hPos                      = zeros(3, NUM_STATES);
	view(hPos, all(), range(0, 3)) = eye(3);
	engine.add_measurement_processor(
	    make_shared<DirectMeasurementProcessor>("gps++", "pinson15", hPos));

	auto hVel                      = zeros(3, NUM_STATES);
	view(hVel, all(), range(3, 6)) = eye(3);
	engine.add_measurement_processor(
	    make_shared<DirectMeasurementProcessor>("odometer", "pinson15", hVel));

	// Setup our simulation runtime and output variables for plotting
	Vector times              = xt::arange<double>(0, runtime, DT);
	auto out_states           = zeros(NUM_STATES, num_epochs);
	auto out_cov              = zeros(NUM_STATES, num_epochs);
	navtk::Matrix true_errors = zeros(NUM_STATES, num_epochs);

	// Create an initial trajectory point to kick off measurement simulation
	auto nav_sol = navtk::filtering::NavSolution{
	    Vector3{0.0, 0.0, 0.0},            // Position in lat (rad), lon (rad), alt (m).
	    Vector3{1, 0, 0},                  // NED Velocity, m/s
	    eye(3),                            // Attitude DCM
	    TypeTimestamp((int64_t)0)};        // Time, seconds
	auto fNed = Vector3{0.0, 0.0, -9.81};  // Measured specific force (m/s^2)

	// Generate a simple 'truth' trajectory and initialize the reference/inertial trajectory
	auto truth   = constant_vel_pva(nav_sol, DT, runtime);
	auto ref_pva = constant_vel_pva(nav_sol, DT, runtime);

	// Generate measurements by adding noise to truth
	auto pos_meas = navtk::exampleutils::noisy_pos_meas(truth, ones(3) * POS_SIGMA);
	auto vel_meas = navtk::exampleutils::noisy_vel_meas(truth, ones(3) * VELO_SIGMA);
	auto alt_meas = navtk::exampleutils::noisy_alt_meas(truth, BARO_SIGMA);

	// Filter loop
	for (int i = 0; i < num_epochs; i++) {
		if (i > 0) {
			auto time = times[i];

			// Propagate the 'true' errors forward
			auto aux_data = navtk::utils::to_inertial_aux(ref_pva[i - 1], fNed, zeros(3));
			true_error_block.receive_aux_data(aux_data);

			// Passing a callback that returns nullptr here because the Pinson15NedBlock does not
			// need it. In general this is not recommended.
			auto dyn =
			    true_error_block.generate_dynamics(navtk::filtering::NULL_GEN_XHAT_AND_P_FUNCTION,
			                                       to_type_timestamp(),
			                                       to_type_timestamp(DT));
			auto unscaledErrors                        = zeros(NUM_STATES);
			view(unscaledErrors, range(9, NUM_STATES)) = randn({6}, 0.0, 1.0);

			view(true_errors, all(), i) = dot(dyn.Phi, view(true_errors, all(), i - 1)) +
			                              dot(navtk::chol(dyn.Qd), unscaledErrors);

			// The available 'inertial' solution is the true location + simulated inertial errors
			Vector errors = -(navtk::to_vec(view(true_errors, range(0, 9), xt::keep(i))));
			ref_pva[i]    = apply_error_states<Pinson15NedBlock>(truth[i], errors);

			auto pBlock =
			    std::dynamic_pointer_cast<Pinson15NedBlock>(engine.get_state_block("pinson15"));
			aux_data = navtk::utils::to_inertial_aux(ref_pva[i], fNed, zeros(3));
			pBlock->receive_aux_data(aux_data);

			engine.propagate(to_type_timestamp(time));

			if (BARO_ENABLED and abs(fmod(time, BARO_INTERVAL)) < EPSILON) {
				auto time_validity          = to_type_timestamp(time);
				auto measurement_data       = Vector{ref_pva[i].pos[2] - alt_meas[i]};
				auto measurement_covariance = navtk::Matrix{{BARO_SIGMA * BARO_SIGMA}};
				engine.update("altimeter",
				              std::make_shared<navtk::filtering::GaussianVectorData>(
				                  time_validity, measurement_data, measurement_covariance));
			}

			if (POS_ENABLED and abs(fmod(time, POS_INTERVAL)) < EPSILON) {
				auto time_validity = to_type_timestamp(time);
				auto delta_lla     = view(pos_meas, range(0, 3), i) - ref_pva[i].pos;
				// Convert delta latitude-longitude-altitude (radians-radians-meters) to delta
				// north-east-down (meters)
				auto lat              = pos_meas(0, i);
				auto alt              = pos_meas(2, i);
				auto measurement_data = {
				    navtk::navutils::delta_lat_to_north(delta_lla(0), lat, alt),
				    navtk::navutils::delta_lon_to_east(delta_lla(1), lat, alt),
				    -delta_lla(2)};

				auto measurement_covariance = eye(3) * POS_SIGMA * POS_SIGMA;

				engine.update("gps++",
				              std::make_shared<navtk::filtering::GaussianVectorData>(
				                  time_validity, measurement_data, measurement_covariance));
			}

			if (VELO_ENABLED and abs(fmod(time, VELO_INTERVAL)) < EPSILON) {
				auto time_validity          = to_type_timestamp(time);
				auto measurement_data       = view(vel_meas, range(0, 3), i) - ref_pva[i].vel;
				auto measurement_covariance = eye(3) * VELO_SIGMA * VELO_SIGMA;

				engine.update("odometer",
				              std::make_shared<navtk::filtering::GaussianVectorData>(
				                  time_validity, measurement_data, measurement_covariance));
			}

			if (FEEDBACK_ENABLED and abs(fmod(time, FEEDBACK_INTERVAL)) < EPSILON) {
				view(true_errors, all(), i) -= engine.get_state_block_estimate("pinson15");

				engine.set_state_block_estimate("pinson15", zeros(NUM_STATES));
			};
		}
		// Save output after updating
		view(out_cov, all(), i)    = xt::diagonal(engine.get_state_block_covariance("pinson15"));
		view(out_states, all(), i) = engine.get_state_block_estimate("pinson15");
	}

	// Plot results
	if (plot_results) {
		auto state_names = vector<std::string>{
		    "North Pos Error",
		    "East Pos Error",
		    "Down Pos Error",
		    "North Vel Error",
		    "East Vel Error",
		    "Down Vel Error",
		};
		auto title_names = vector<std::string>{
		    "Position Error",
		    "Velocity Error",
		};
		auto state_y_labels = vector<std::string>{
		    "meters",
		    "meters",
		    "meters",
		    "m/s",
		    "m/s",
		    "m/s",
		};

		// Import plotting tools from Python
		pybind11::scoped_interpreter guard{true, argc, argv};
		auto pyplot   = pybind11::module::import("matplotlib.pyplot");
		auto figure   = pyplot.attr("figure");
		auto plot     = pyplot.attr("plot");
		auto subplot  = pyplot.attr("subplot");
		auto xlabel   = pyplot.attr("xlabel");
		auto ylabel   = pyplot.attr("ylabel");
		auto suptitle = pyplot.attr("suptitle");
		auto legend   = pyplot.attr("legend");
		auto show     = pyplot.attr("show");
		for (int p_idx = 0; p_idx < 6; p_idx++) {
			if (p_idx % 3 == 0) {
				figure();
				suptitle(title_names[int(p_idx / 3)]);
			}
			subplot(3, 1, p_idx % 3 + 1);
			if (p_idx % 3 == 2) {
				xlabel("Time (s)");
			}
			auto state_line = plot(times,
			                       xt::eval(view(out_states, p_idx, range(0, num_epochs)) -
			                                view(true_errors, p_idx, range(0, num_epochs))),
			                       "k");
			plot(times, xt::eval(sqrt(view(out_cov, p_idx, range(0, num_epochs)))), "b");
			plot(times, xt::eval(-sqrt(view(out_cov, p_idx, range(0, num_epochs)))), "b");
			ylabel(state_y_labels[p_idx]);
			auto legend_labels = pybind11::list(2);
			legend_labels[0]   = state_names[p_idx];
			legend_labels[1]   = "Filter-Computed 1-Sigma";
			legend(legend_labels);
		}

		show();
	}
	return EXIT_SUCCESS;
}
