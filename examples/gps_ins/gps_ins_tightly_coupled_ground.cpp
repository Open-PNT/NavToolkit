#include <memory>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <spdlog/spdlog.h>
#include <gps_ins/utils/aspn2_lcm_to_aspn.hpp>
#include <gps_ins/utils/plot.hpp>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/filtering/processors/GeodeticPos3dMeasurementProcessor.hpp>
#include <navtk/filtering/processors/SinglePointPseudorangeProcessorEcef.hpp>
#include <navtk/filtering/stateblocks/ClockBiasesStateBlock.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/filtering/virtualstateblocks/EcefToStandardQuat.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonErrorToStandardQuat.hpp>
#include <navtk/filtering/virtualstateblocks/PlatformToSensorEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/SensorToPlatformEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/StandardToEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/StateExtractor.hpp>
#include <navtk/gnssutils/PseudorangeType.hpp>
#include <navtk/inertial/BufferedImu.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::TypeMounting;
using aspn_xtensor::TypeTimestamp;
using navtk::Matrix;
using navtk::ones;
using navtk::Vector;
using navtk::zeros;
using navtk::exampleutils::std_to_navtk;
using navtk::filtering::StateBlock;
using navtk::navutils::dcm_to_quat;
using navtk::utils::to_navsolution;
using std::endl;
using std::pair;
using std::shared_ptr;
using std::string;
using std::vector;
using xt::range;
using xt::view;

namespace {
// Log file channel names.
const string IMU_MSG      = "/groundSim/imu";
const string TRUTH_MSG    = "/sensor/span/truth/pva";
const string EPHEM_MSG    = "/groundSim/ephemeris";
const string POSITION_MSG = "/groundSim/pos";
const string GPS_MSG      = "/groundSim/gps";

// Define the StateBlock and MeasurementProcessor labels.
const string PINSON15_LABEL   = "pinson15";
const string GPS_LABEL        = "gps_pseudorange";
const string POS_LABEL        = "gps_llh";
const string CLOCK_BIAS_LABEL = "clock_bias";
const string ECEF_POS_LABEL   = "antenna_center_ecef_pos";
const string LLH_POS_LABEL    = "antenna_center_llh_pos";

// INS Settings.
constexpr bool APPLY_INS_FEEDBACK   = true;
constexpr double FEEDBACK_THRESHOLD = 10.0;  // meters

// Initial state number
constexpr size_t NUM_PINSON_STATES = 15;

// Tightly-coupled GPS processor settings.
constexpr bool APPLY_TROPO_MODEL                      = true;
constexpr double TROPO_REL_HUMIDITY                   = 0.5;
constexpr double ELEV_MASK_ANGLE                      = 0.0;
constexpr bool FORCE_CLOCK_INITIALIZATION             = true;
constexpr navtk::gnssutils::PseudorangeType PR_TO_USE = navtk::gnssutils::CAL1;
constexpr double PR_NOISE_COVARIANCE                  = 9.0;          // meters^2
constexpr double PR_BIAS_TIME_CONSTANT                = 60.0 * 60.0;  // seconds
constexpr double PR_BIAS_COVARIANCE                   = 4.0;          // meters^2

// The rotation between the sensor (inertial) frame and the platform frame.
const navtk::Matrix3 C_INERTIAL_TO_PLATFORM{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
const navtk::Matrix3 C_SENSOR_TO_PLATFORM{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

// Lever arms from the platform to various sensors, in the platform frame
const navtk::Vector3 PLATFORM_TO_SENSOR_ARM{0.0, 0.0, -4.0};
const navtk::Vector3 PLATFORM_TO_INERTIAL_ARM{0.0, 0.0, 0.0};

// Expected imu data rate, in seconds
constexpr double IMU_RATE = 0.02;

// Error mode to use in the example
const navtk::ErrorMode LOG_LEVEL = navtk::ErrorMode::DIE;

// When true, if at any time within the example (does not filter down to library) an attempt to get
// PVA from the inertial at a specific time fails (returns null), use the latest available instead.
// If example is dying with 'IMU mechanization is x s behind...' and x is very small, setting this
// to true may help
const bool IGNORE_IMU_LAG = false;

// When true, use loosely coupled measurements (if available) rather than pseudoranges.
const bool DEBUG_WITH_LOOSE = false;
}  // namespace

/**
 * The executable created for gps_ins_tightly_coupled_ground in meson.build file uses the path
 * passed to it through the meson run_target in order to "./build/gps_ins_tightly_coupled_ground
 * ../subprojects/navtk-data/ground_sim_tight.log". The data is then processed through an EKF.
 * The tightly coupled filter uses GPS ephemeris messages, GPS pseudorange measurements and IMU
 * measurements. The data file provides GPS position measurements as well, but the position
 * measurements are not used.
 *
 * If data file was not found during meson phase a warning will be generated about the missing file.
 **/
int main(int argc, char* argv[]) {
	navtk::set_global_error_mode(LOG_LEVEL);

	// Note: The data includes a 120 second GPS outage to demonstrate INS drift.
	if (argc < 2) {
		spdlog::error("Usage: {} [input-data-file]\n\nNo input data file specified.", argv[0]);
		return 1;
	}

	char* data_file = argv[1];

	auto plot_results_flag = true;
	if (argc > 2) {
		plot_results_flag = strcmp(argv[2], "1") == 0;
	}

	// The number of seconds (in log file time) to process. A value that is negative, zero, or
	// larger than the length of the log will have no effect.
	int seconds_to_process = -1;
	if (argc > 3) {
		seconds_to_process = atoi(argv[3]);
	}

	// stores the truth data as data is pulled from file
	navtk::exampleutils::TruthResults truth_raw;
	// stores the interpolated truth data
	navtk::exampleutils::TruthPlotResults truth_interp;
	// initialize variables
	aspn_xtensor::TypeTimestamp t_start = aspn_xtensor::to_type_timestamp();

	const Vector pinson_estimate                = zeros(NUM_PINSON_STATES);
	Vector pinson_covariance_diag               = zeros(NUM_PINSON_STATES);
	view(pinson_covariance_diag, range(0, 3))   = ones(3) * 4.0;          // NED position (m)
	view(pinson_covariance_diag, range(3, 6))   = Vector{1.5, 1.5, 3.0};  // NED velocity (m/s)
	view(pinson_covariance_diag, range(6, 9))   = ones(3) * 0.001;        // NED tilts (rad)
	view(pinson_covariance_diag, range(9, 12))  = ones(3) * 9.604e-5;  // Accelerometer bias (m/s^2)
	view(pinson_covariance_diag, range(12, 15)) = ones(3) * 4.841e-12;  // Gyroscope bias (rad/s)
	const Matrix pinson_covariance              = xt::diag(pinson_covariance_diag);

	// Initial PVA results.
	std::shared_ptr<MeasurementPositionVelocityAttitude> starting_pva;

	// stores the filter, sigma, and ins state results
	vector<Vector> filter_output, sigma_output, ins_output;

	// Track which PRNs are in the data set. The key is the PRN and the pair holds a vector of time
	// tags and a vector of PR bias values.
	std::map<int, pair<vector<double>, vector<double>>> pr_bias_output;

	// stores the time tags of filter-corrected solutions
	vector<double> output_filter_time;

	// time tag for last feedback
	aspn_xtensor::TypeTimestamp last_feedback_time = aspn_xtensor::to_type_timestamp();

	// Open LCM log file.
	lcm::LogFile log{data_file, "r"};
	// LCM instructions recommend checking this 'good()' return
	if (!log.good()) {
		spdlog::error("Failed to open logfile. Expected gps_ins.log in data folder.");
		return 1;
	}

	// Gets next LCM message. To generate a usable class instance we first
	// have to detect what type of channel name the data is.
	// Initially, looking for truth message to initialize filter.
	auto next_msg = log.readNextEvent();
	while (true) {
		if (next_msg == NULL) {
			throw std::runtime_error(
			    "Exception Occurred: Could not find truth data in log file. Exiting "
			    "program.\n");
		}
		auto chan = next_msg->channel;
		if (chan.compare(TRUTH_MSG) == 0) {
			// Truth channel contains positionvelocityattitude data type
			// Create the lcm class that matches the channel
			datasources::lcm::messages::aspn::positionvelocityattitude t;

			// Pass in the data from log file into the classes decode function;
			// the class fields will be populated
			// https://lcm-proj.github.io/structlcm_1_1LogEvent.html#details
			// has LogEvent (next_msg) details
			t.decode(next_msg->data, 0, next_msg->datalen);
			// store the truth data to the truth_raw struct
			store_truth_lcm(truth_raw, t);

			// Pull out PVA for initializing filter/inertial
			starting_pva = std::make_shared<MeasurementPositionVelocityAttitude>(
			    navtk::exampleutils::to_aspn(t));
			Matrix C_s_to_n =
			    navtk::dot(navtk::navutils::quat_to_dcm(starting_pva->get_quaternion()),
			               C_INERTIAL_TO_PLATFORM);
			starting_pva->set_quaternion(navtk::navutils::dcm_to_quat(C_s_to_n));
			break;

		} else {
			next_msg = log.readNextEvent();
		}
	}

	// Create the INS object, giving it the initial position and attitude
	auto ins = navtk::inertial::BufferedImu(*starting_pva, nullptr, IMU_RATE);

	// Initialize fusion engine with initial truth time. The fusion engine, fusion strategy, state
	// blocks, and measurement processors act as a navigation filter.
	auto engine = std::make_shared<navtk::filtering::StandardFusionEngine>(
	    starting_pva->get_time_of_validity());

	// Create IMU model
	navtk::filtering::ImuModel imu_model{zeros(3) + 0.095,      // accel_random_walk_sigma
	                                     zeros(3) + .0000873,   // gyro_random_walk_sigma
	                                     zeros(3) + .0098,      // accel_bias_sigma
	                                     zeros(3) + 3600,       // accel_bias_tau
	                                     zeros(3) + 4.8481e-6,  // gyro_bias_sigma
	                                     zeros(3) + 3600,       // gyro_bias_tau
	                                     zeros(3),              // accel_scale_factor
	                                     zeros(3)};             // gyro_scale_factor

	// Create a Pinson15NED stateblock, labeled as "pinson15", and
	// provide the required imu modeling parameters that we just generated.
	shared_ptr<StateBlock<>> imu_block =
	    std::make_shared<navtk::filtering::Pinson15NedBlock>(PINSON15_LABEL, imu_model);
	// Add the IMU state block to the main state block
	engine->add_state_block(imu_block);

	// Add virtual state blocks. For this example inertial sensor/platform frame are identical
	// so no need for a 'SensorToPlatform' before PlatformToSensor block.
	// Individual blocks are chained via their labels; alternatively we could create a
	// ChainedVirtualStateBlock here, or use a PinsonToSensor instance, which numerically calculates
	// the required jacobian
	auto nav_fun = [&](const aspn_xtensor::TypeTimestamp& time) {
		return to_navsolution(
		    *navtk::exampleutils::get_inertial_aux(ins, time, IGNORE_IMU_LAG).first);
	};

	// Lever arm/sensor mounting for the PlatformToSensorEcefQuat VSB that transforms the platform
	// position (origin of the platform frame) to the sensor (gps antenna) frame. Defined in the
	// platform frame (nose, right side, down), this indicates that the antenna is assumed 4 m above
	// the platform frame (negative along down axis) and axes of the 'antenna frame' are aligned
	// with the platform. Only one mounting is provided as it is assumed in this example that the
	// inertial sensor frame and platform frame are coincident.
	auto mount1 = TypeMounting(PLATFORM_TO_INERTIAL_ARM,
	                           zeros(3),
	                           dcm_to_quat(xt::transpose(C_INERTIAL_TO_PLATFORM)),
	                           zeros(3, 3));
	auto mount2 = TypeMounting(PLATFORM_TO_SENSOR_ARM,
	                           zeros(3),
	                           dcm_to_quat(xt::transpose(C_SENSOR_TO_PLATFORM)),
	                           zeros(3, 3));

	auto vsb1 =
	    std::make_shared<navtk::filtering::PinsonErrorToStandardQuat>(PINSON15_LABEL, "A", nav_fun);
	auto vsb2 = std::make_shared<navtk::filtering::StandardToEcefQuat>("A", "B");
	auto vsb3 = std::make_shared<navtk::filtering::SensorToPlatformEcefQuat>("B", "C", mount1);
	auto vsb4 = std::make_shared<navtk::filtering::PlatformToSensorEcefQuat>("C", "D", mount2);
	auto vsb5 = std::make_shared<navtk::filtering::StateExtractor>(
	    "D", ECEF_POS_LABEL, 16, std::vector<navtk::Size>{0, 1, 2});
	auto vsb6 = std::make_shared<navtk::filtering::EcefToStandardQuat>("D", "E");
	auto vsb7 = std::make_shared<navtk::filtering::StateExtractor>(
	    "E", LLH_POS_LABEL, 16, std::vector<navtk::Size>{0, 1, 2});

	engine->add_virtual_state_block(vsb1);
	engine->add_virtual_state_block(vsb2);
	engine->add_virtual_state_block(vsb3);
	engine->add_virtual_state_block(vsb4);
	engine->add_virtual_state_block(vsb5);
	engine->add_virtual_state_block(vsb6);
	engine->add_virtual_state_block(vsb7);

	// Initialize estimate and covariance for the Pinson15 state block
	engine->set_state_block_estimate(PINSON15_LABEL, pinson_estimate);
	engine->set_state_block_covariance(PINSON15_LABEL, pinson_covariance);

	// Create the clock bias state block
	navtk::filtering::ClockModel clock_bias_model = navtk::filtering::COMPENSATED_CRYSTAL_CLOCK;
	shared_ptr<StateBlock<>> clock_bias_state_block =
	    std::make_shared<navtk::filtering::ClockBiasesStateBlock>(CLOCK_BIAS_LABEL,
	                                                              clock_bias_model);

	// Add the clock bias state block to the main state block
	engine->add_state_block(clock_bias_state_block);

	// Initialize estimate for the clock bias state block
	Vector x0_clock_bias = zeros(2);
	engine->set_state_block_estimate(CLOCK_BIAS_LABEL, x0_clock_bias);

	// Initialize prediction error covariance for the clock bias state block
	Matrix p0_clock_bias = zeros(2, 2);
	p0_clock_bias        = xt::diag(Vector({1, 1e-10}));
	engine->set_state_block_covariance(CLOCK_BIAS_LABEL, p0_clock_bias);

	// Create measurement processor object for the GPS pseudorange measurements.
	// This program uses GPS as pseudorange measurements, therefore, use
	// SinglePointPseudorangeProcessor.  This measurement processor will be called
	// "gps_pseudorange", and measurement objects processed by the fusion engine
	// which are labeled "gps_pseudorange" will be sent to this measurement processor.
	auto pr_measurement_processor =
	    std::make_shared<navtk::filtering::SinglePointPseudorangeProcessorEcef>(
	        GPS_LABEL,
	        ECEF_POS_LABEL,
	        CLOCK_BIAS_LABEL,
	        PR_TO_USE,
	        PR_NOISE_COVARIANCE,
	        PR_BIAS_COVARIANCE,
	        PR_BIAS_TIME_CONSTANT,
	        engine,
	        APPLY_TROPO_MODEL,
	        TROPO_REL_HUMIDITY,
	        ELEV_MASK_ANGLE,
	        FORCE_CLOCK_INITIALIZATION);

	auto pos_measurement_processor =
	    std::make_shared<navtk::filtering::GeodeticPos3dMeasurementProcessor>(
	        POS_LABEL, LLH_POS_LABEL, navtk::eye(3));

	engine->add_measurement_processor(pr_measurement_processor);
	engine->add_measurement_processor(pos_measurement_processor);

	// Now that initialization is complete, begin looping through message set and running filter
	while (true) {
		bool updated = false;
		// Read next measurement from log file
		next_msg = log.readNextEvent();
		if (next_msg == NULL) {
			// EOF has been reached; quit loop.
			break;
		} else {
			// process the data
			// This particular file has data on the 'truth', 'imu', 'ephemeris', 'gps'(pseudorange)
			// and 'position' channels
			auto chan = next_msg->channel;
			if (chan.compare(TRUTH_MSG) == 0) {
				// decode LCM truth class
				datasources::lcm::messages::aspn::positionvelocityattitude t;
				t.decode(next_msg->data, 0, next_msg->datalen);
				// store the truth data to the truth_raw struct
				store_truth_lcm(truth_raw, t);
			} else if (chan.compare(IMU_MSG) == 0) {
				// Decode LCM imu class
				datasources::lcm::messages::aspn::imu t;
				t.decode(next_msg->data, 0, next_msg->datalen);
				aspn_xtensor::MeasurementImu imu_meas = navtk::exampleutils::to_aspn(t);

				// Abort processing log if desired playback time has passed
				if (seconds_to_process > 0 &&
				    imu_meas.get_time_of_validity() - starting_pva->get_time_of_validity() >
				        seconds_to_process)
					break;

				ins.mechanize(imu_meas);
			} else if (chan.compare(EPHEM_MSG) == 0) {
				// Decode LCM gpsephemeris class
				datasources::lcm::messages::aspn::gpsephemeris t;
				t.decode(next_msg->data, 0, next_msg->datalen);
				// convert to ASPN gps ephemeris class, pass as aux data to the the processor
				auto eph_aux = std::make_shared<aspn_xtensor::MetadataGpsLnavEphemeris>(
				    navtk::exampleutils::to_aspn(t));
				engine->give_measurement_processor_aux_data(GPS_LABEL, {eph_aux});
			} else if (chan.compare(POSITION_MSG) == 0) {
				// Inner scope to avoid triggering 'Unknown message type warning' from the end of
				// this function
				if (DEBUG_WITH_LOOSE) {
					datasources::lcm::messages::aspn::geodeticposition3d t;
					t.decode(next_msg->data, 0, next_msg->datalen);
					auto gps = std::make_shared<aspn_xtensor::MeasurementPosition>(
					    navtk::exampleutils::to_aspn(t));

					if (!ins.in_range(gps->get_time_of_validity())) {
						spdlog::warn("IMU mechanization is {} s behind the measurement at {}",
						             (gps->get_time_of_validity() - ins.time_span().second),
						             gps->get_time_of_validity());
					}

					while (gps->get_time_of_validity() > engine->get_time()) {
						navtk::exampleutils::propagate_filter(ins,
						                                      *engine,
						                                      gps->get_time_of_validity(),
						                                      PINSON15_LABEL,
						                                      IGNORE_IMU_LAG);
					}

					engine->update(POS_LABEL, gps);
					updated = true;

					vector<string> labels = pos_measurement_processor->get_state_block_labels();
					labels[0]             = PINSON15_LABEL;
					{
						navtk::exampleutils::generate_output(
						    navtk::exampleutils::CouplingType::LOOSE,
						    *engine,
						    ins,
						    labels,
						    filter_output,
						    sigma_output,
						    ins_output,
						    output_filter_time,
						    C_INERTIAL_TO_PLATFORM);
					}
				}
			} else if (chan.compare(GPS_MSG) == 0) {
				if (!DEBUG_WITH_LOOSE) {
					// pseudorange data
					datasources::lcm::messages::aspn::gnss t;
					t.decode(next_msg->data, 0, next_msg->datalen);

					// convert to gnss ASPN class
					auto gnss = std::make_shared<aspn_xtensor::MeasurementSatnav>(
					    navtk::exampleutils::to_aspn(t));

					if (!ins.in_range(gnss->get_time_of_validity())) {
						spdlog::warn("IMU mechanization is {} s behind the measurement at {}",
						             gnss->get_time_of_validity() - ins.time_span().second,
						             gnss->get_time_of_validity());
					}

					while (gnss->get_time_of_validity() > engine->get_time()) {
						navtk::exampleutils::propagate_filter(ins,
						                                      *engine,
						                                      gnss->get_time_of_validity(),
						                                      PINSON15_LABEL,
						                                      IGNORE_IMU_LAG);
					}

					engine->update(GPS_LABEL, gnss);
					updated = true;

					// collect the PINSON15_LABEL, CLOCK_BIAS_LABEL, and pseudorange bias state
					// blocks
					vector<string> labels = pr_measurement_processor->get_state_block_labels();
					labels[0]             = PINSON15_LABEL;

					navtk::exampleutils::generate_output(navtk::exampleutils::CouplingType::TIGHT,
					                                     *engine,
					                                     ins,
					                                     labels,
					                                     filter_output,
					                                     sigma_output,
					                                     ins_output,
					                                     output_filter_time,
					                                     C_INERTIAL_TO_PLATFORM);

					// remove the PINSON15_LABEL and CLOCK_BIAS_LABEL
					labels.erase(labels.begin(), labels.begin() + 2);
					// collect current state of PR bias estimates
					navtk::exampleutils::generate_pr_bias_output(pr_bias_output, labels, *engine);
				}


			} else {
				// Message is not something we are currently processing. Ignore.
				spdlog::warn("Unknown message type: {}. Message will not be processed.", chan);
			}

			if (APPLY_INS_FEEDBACK && updated) {
				auto ins_sol =
				    navtk::exampleutils::get_inertial_aux(ins, engine->get_time(), IGNORE_IMU_LAG)
				        .first;
				Vector x_pinson = engine->get_state_block_estimate(PINSON15_LABEL);
				Vector temp_x   = view(x_pinson, range(0, 3));
				if (navtk::norm(temp_x) > FEEDBACK_THRESHOLD) {
					last_feedback_time = aspn_xtensor::to_type_timestamp(output_filter_time.back());
					auto corrected =
					    navtk::filtering::apply_error_states<navtk::filtering::Pinson15NedBlock>(
					        *ins_sol, x_pinson);
					ins.reset(std::make_shared<MeasurementPositionVelocityAttitude>(corrected));
					// Reset the error states
					view(x_pinson, range(0, 9)) = 0;
					engine->set_state_block_estimate(PINSON15_LABEL, x_pinson);
				}
			}
		}
	}
	// interpolate the truth data and convert to plot-able type navtk
	auto truth_interp_result = navtk::exampleutils::convert_truth(truth_raw, output_filter_time);
	// pull out interpolated truth data
	truth_interp = truth_interp_result.second;
	// pull out unused output_filter_time tags
	vector<size_t> unused_index = truth_interp_result.first;
	// Remove unused index from filter, sigma, ins, and output filter time to match
	// the interpolated truth data
	if (!unused_index.empty()) {
		vector<size_t>::reverse_iterator rit = unused_index.rbegin();
		for (; rit != unused_index.rend(); ++rit) {
			size_t rem_index = *rit;
			filter_output.erase(filter_output.begin() + rem_index);
			sigma_output.erase(sigma_output.begin() + rem_index);
			ins_output.erase(ins_output.begin() + rem_index);
			output_filter_time.erase(output_filter_time.begin() + rem_index);
		}
	}

	// convert from std::vector<navtk::Vector> to navtk::Matrix for plotting
	Matrix filter_results   = std_to_navtk(filter_output);
	Matrix sigma_results    = std_to_navtk(sigma_output);
	Matrix ins_results      = std_to_navtk(ins_output);
	Vector time_tag_results = std_to_navtk(output_filter_time);

	// Calculate the truth tilts and delta between truth and filter tilts
	auto truth_tilt_results = calculate_truth_tilts(
	    truth_raw, truth_interp, filter_results, ins_results, output_filter_time);
	// truth tilt results
	Matrix true_tilts = truth_tilt_results.first;
	// difference between filter and truth tilt results
	Matrix delta_tilts = truth_tilt_results.second;

	// position profile calculation
	auto delta_ned = calculate_position_profile(filter_results, truth_interp);
	// position profile for truth
	Matrix d_ned_true = delta_ned.first;
	// position profile for filter
	Matrix d_ned_filt = delta_ned.second;

	// plot results
	if (plot_results_flag) {

		// Import plotting tools from Python
		pybind11::scoped_interpreter guard{true, argc, argv};
		auto pyplot = pybind11::module::import("matplotlib.pyplot");
		auto show   = pyplot.attr("show");

		// plot the trajectory
		navtk::exampleutils::plot_trajectory(d_ned_true, d_ned_filt);
		// plot the delta truth and filter position, velocity, tilt results
		navtk::exampleutils::plot_delta_results(
		    filter_results, sigma_results, truth_interp, delta_tilts, time_tag_results);
		// plot the position and velocity profile
		navtk::exampleutils::plot_pos_vel_profile(
		    filter_results, d_ned_true, d_ned_filt, truth_interp, time_tag_results);
		// plot the accelerometer and gyroscope bias filter results
		navtk::exampleutils::plot_acc_gyro_err(filter_results, sigma_results, time_tag_results);
		if (!DEBUG_WITH_LOOSE) {
			// plot clock error
			navtk::exampleutils::plot_clock_error(filter_results, time_tag_results);
			// plot the pseudorange bias
			navtk::exampleutils::plot_pr_bias(pr_bias_output);
		}
		// show the figures
		show();
	}

	return 0;
}
