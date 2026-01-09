#include <memory>

#include <spdlog/spdlog.h>

#include <gps_ins/utils/aspn2_lcm_to_aspn.hpp>
#include <gps_ins/utils/plot.hpp>
#include <gps_ins/utils/results.hpp>
#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/containers/TimestampedDataSeries.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/processors/DirectMeasurementProcessor.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/inertial/CoarseDynamicAlignment.hpp>
#include <navtk/inertial/Inertial.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/inertial_functions.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/utils/conversions.hpp>
#include <navtk/utils/interpolation.hpp>

using aspn_xtensor::MeasurementImu;
using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::TypeTimestamp;
using datasources::lcm::messages::aspn::geodeticposition3d;
using datasources::lcm::messages::aspn::imu;
using datasources::lcm::messages::aspn::positionvelocityattitude;
using navtk::eye;
using navtk::Matrix;
using navtk::Matrix3;
using navtk::Vector;
using navtk::Vector3;
using navtk::zeros;
using navtk::exampleutils::std_to_navtk;
using navtk::exampleutils::TruthResults;
using navtk::filtering::NavSolution;
using navtk::filtering::StandardFusionEngine;
using navtk::filtering::TimestampedDataSeries;
using navtk::inertial::Inertial;
using navtk::inertial::InertialPosVelAtt;
using navtk::inertial::StandardPosVelAtt;
using navtk::navutils::PI;
using std::make_shared;
using std::pair;
using std::shared_ptr;
using std::string;
using std::vector;
using xt::range;
using xt::transpose;
using xt::view;

namespace {
// Some constants to control the filter behavior.
// ----------------------------------------------

// How long to wait before processing a GPS message (messages are buffered for this long, to handle
// data arriving out of order.
const aspn_xtensor::TypeTimestamp GPS_PROCESS_DELAY = aspn_xtensor::to_type_timestamp(5, 0);

// Maximum Filter Propagation Time
const aspn_xtensor::TypeTimestamp MAX_PROP_DT = aspn_xtensor::to_type_timestamp(0.25);

// Our IMU in the sample data recorded data at 100Hz.
constexpr double IMU_DT = 0.01;

// INS sensor to platform DCM, required for s3 data file
const Matrix3 C_INS_TO_PLATFORM{{0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}};

// Apply feedback to INS
constexpr bool APPLY_FEEDBACK = true;

// Feedback Threshold, apply feedback after total error reaches this value
constexpr double FEEDBACK_THRESHOLD = 10.0;

// Create string label for creating/calling pinson15 state block
const string PINSON15("pinson15");

// These are the names of the channels in the log file. They'll be used as arguments to LCM's
// subscribeFunction
const string IMU_CHANNEL   = "aspn://vikingnav/novatel/2001.0/0";
const string GPS_CHANNEL   = "aspn://vikingnav/novatel/1001.3/0";
const string TRUTH_CHANNEL = "aspn://vikingnav/novatel/1010.0/0";

// When true reads initial alignment from log file; false uses dynamic alignment algorithm
constexpr bool ALIGN_FROM_TRUTH = false;

// A stationary IMU feels the force of gravity, which we're assuming here is constant.
const Vector3 GRAVITY{0.0, 0.0, -9.81};

// Number of states in a Pinson15 block
constexpr auto NUM_STATES = 15;
}  // namespace

// Helper functions and types
// --------------------------

// Incoming IMU and GPS data will be stored using ordered buffers, since messages may arrive
// out-of-order.
using ImuSeries = TimestampedDataSeries<MeasurementImu>;
using GpsSeries = TimestampedDataSeries<MeasurementPosition>;

// The following three handler functions will be added to the LCM subscriber
// and called by it in order to convert the data from the LCM format to ASPN format
// and a TruthResults struct.
// Function handler for truth data
void handle_truth(const lcm::ReceiveBuffer*,
                  const string&,
                  const positionvelocityattitude* msg,
                  TruthResults* destination_truth_state);

// Container for the output state of the handle_imu function below.
struct IMUHandlerContext {
	ImuSeries* series;
	aspn_xtensor::TypeTimestamp* validity;
};

// Function handler for IMU data
void handle_imu(const lcm::ReceiveBuffer*,
                const string&,
                const imu* msg,
                IMUHandlerContext destination_imu_state);

// Function handler for GPS data
void handle_gps(const lcm::ReceiveBuffer*,
                const string&,
                const geodeticposition3d* msg,
                GpsSeries* destination_gps_series);

// Creates a PinsonAux object based on the current navigation solution and specific force.
AspnBaseVector create_pinson15_ins_aux_data(const Inertial& ins, const MeasurementImu& imu);

AspnBaseVector create_pinson15_ins_aux_data(const NavSolution& nav_sol, const Vector& forces);

// Creates a PVA estimated from the Inertial and Filter, using the Position passed in
shared_ptr<StandardPosVelAtt> create_pva_estimate(const Inertial& ins,
                                                  const StandardFusionEngine& engine,
                                                  const Vector3& use_this_lla);

// Interpolates the INS solution position to the specified time (using an Inertial copy, tmp_ins)
Vector3 interpolate_inertial_lla(const ImuSeries& imu_series,
                                 Inertial tmp_ins,
                                 const aspn_xtensor::TypeTimestamp& time);

/**
 * The executable created for gps_ins_loosely_coupled_flight in meson.build
 * file uses the path passed to it through the meson run_target
 * in order to "./build/gps_ins_loosely_coupled_flight
 * ../subprojects/navtk-data/afrl_s3_flight_20180618_v2.log". The GPS and IMU measurements are
 * stored in time ordered buffers even if the measurements are received out of order. The buffers
 * are processed in close to realtime (5 second delay). Note: the buffers are not thread safe, but
 * this example is single threaded. The filter is loosely coupled, using only position measurements
 * from GPS and IMU measurements.
 *
 * If data file was not found during meson phase a warning will be
 * generated about the missing data file.
 **/
int main(int argc, char* argv[]) {
	// pull out location of data file
	if (argc < 2) {
		spdlog::error("Usage: {} [input-data-file]\n\nNo input data file specified.", argv[0]);
		return 1;
	}
	char* data_file        = argv[1];
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

	// Time ordered buffers for IMU and GPS measurements. IMU is
	// received at ~100Hz and GPS at ~1Hz, so the buffers will hold
	// ~10 seconds of data.
	ImuSeries imu_series(1000);
	GpsSeries gps_series(10);
	// stores truth data as truth position, velocity, attitude is pulled from LCM file
	TruthResults truth_raw;
	// stores interpolated truth data for plotting
	navtk::exampleutils::TruthPlotResults truth_interp;
	// used for time tags
	aspn_xtensor::TypeTimestamp t_val       = aspn_xtensor::to_type_timestamp(),
	                            t_last_prop = aspn_xtensor::to_type_timestamp(),
	                            t_last_imu  = aspn_xtensor::to_type_timestamp(),
	                            t_start     = aspn_xtensor::to_type_timestamp();
	Vector3 lla_initial, attitude_initial, vned_initial, lla_ins;
	Matrix plot_results;
	vector<Vector> filter_output, sigma_output, ins_output;
	vector<double> output_filter_time;

	// OPEN LCM LOG FILE
	// LCM File Location
	std::ostringstream url;
	url << "file://" << data_file << "?speed=0";
	lcm::LCM lcm(url.str());
	if (!lcm.good()) {
		throw std::runtime_error(
		    "Failed to open logfile. Expected afrl_s3_flight_20180618_v2.log in data folder.");
	}

	// Initialize IMU model
	navtk::filtering::ImuModel imu_model{zeros(3) + 0.0095,     // accel_random_walk_sigma
	                                     zeros(3) + 0.0000873,  // gyro_random_walk_sigma
	                                     zeros(3) + 0.0098,     // accel_bias_sigma
	                                     zeros(3) + 3600,       // accel_bias_tau
	                                     zeros(3) + 4.8481e-6,  // gyro_bias_sigma
	                                     zeros(3) + 3600,       // gyro_bias_tau
	                                     zeros(3),              // accel_scale_factor
	                                     zeros(3)};             // gyro_scale_factor

	// subscribe to LCM data, LCM will callback the associated handler as
	// each log entry is processed
	lcm.subscribeFunction(IMU_CHANNEL, handle_imu, {&imu_series, &t_val});
	lcm.subscribeFunction(GPS_CHANNEL, handle_gps, &gps_series);

	// Process log file till we either hit a truth message to align from or the dynamic alignment
	// generates a solution, depending on which method was selected.
	if (ALIGN_FROM_TRUTH) {
		lcm.subscribeFunction(TRUTH_CHANNEL, handle_truth, &truth_raw);
		while (lcm.handleTimeout(10) > 0)
			if (!truth_raw.time.empty()) break;

		if (truth_raw.time.empty()) {
			throw std::runtime_error(
			    "Error: Could not find truth data in log file. Exiting program.");
		}
		t_start          = aspn_xtensor::to_type_timestamp(truth_raw.time[0]);
		t_val            = t_start;
		t_last_prop      = t_start;
		t_last_imu       = t_start;
		lla_initial      = {truth_raw.lat[0], truth_raw.lon[0], truth_raw.alt[0]};
		vned_initial     = {truth_raw.vel_n[0], truth_raw.vel_e[0], truth_raw.vel_d[0]};
		attitude_initial = {truth_raw.att_r[0], truth_raw.att_p[0], truth_raw.att_y[0]};

	} else {
		auto aligner = navtk::inertial::CoarseDynamicAlignment(imu_model);

		while (lcm.handleTimeout(10) > 0) {
			if (aligner.check_alignment_status() ==
			    navtk::inertial::AlignBase::AlignmentStatus::ALIGNED_GOOD)
				break;
			if (!gps_series.empty() && !imu_series.empty()) {
				aspn_xtensor::TypeTimestamp gps_time = gps_series.front()->get_time_of_validity();
				if (imu_series.back()->get_time_of_validity() >= gps_time) {
					auto imu_to_process = imu_series.get_in_range(
					    aspn_xtensor::TypeTimestamp((int64_t)0), gps_time + IMU_DT);
					for (auto imu = imu_to_process.first; imu != imu_to_process.second; imu++) {
						aligner.process(*imu);
					}
					aligner.process(gps_series.front());
					imu_series.erase(imu_to_process.first, imu_to_process.second);
					gps_series.pop_front();
				}
			}
		}
		auto sol         = aligner.get_computed_alignment().second;
		t_start          = sol.time;
		t_val            = t_start;
		t_last_prop      = t_start;
		t_last_imu       = t_start;
		lla_initial      = sol.pos;
		vned_initial     = sol.vel;
		attitude_initial = navtk::navutils::dcm_to_rpy(xt::transpose(sol.rot_mat));

		lcm.subscribeFunction(TRUTH_CHANNEL, handle_truth, &truth_raw);
	}

	// Initialize the fusion engine model. This, along with the fusion strategy (which defaults to
	// an EKF), state blocks, and measurement processors acts as the navigation filter.
	StandardFusionEngine engine(t_start);

	// Create a Pinson15Ned stateblock, labeled as "pinson15", and
	// provide the required IMU modeling parameters generated.
	shared_ptr<navtk::filtering::StateBlock<>> imu_block =
	    make_shared<navtk::filtering::Pinson15NedBlock>(PINSON15, imu_model);
	// Add the IMU state block to the main state block
	engine.add_state_block(imu_block);
	// Initialize estimate and covariance for the pinson15 state block
	Vector x0_pinson         = zeros(NUM_STATES);  // initialize the error to 0
	auto pos_variance        = 9.0;
	auto vel_variance        = 0.1;
	auto tilt_variance       = 0.01;
	auto accel_bias_variance = 9.604e-5;
	auto gyro_bias_variance  = 2.3504074e-11;
	Vector p0_diagonal({pos_variance,
	                    pos_variance,
	                    pos_variance,
	                    vel_variance,
	                    vel_variance,
	                    vel_variance,
	                    tilt_variance,
	                    tilt_variance,
	                    tilt_variance,
	                    accel_bias_variance,
	                    accel_bias_variance,
	                    accel_bias_variance,
	                    gyro_bias_variance,
	                    gyro_bias_variance,
	                    gyro_bias_variance});
	Matrix p0_pinson = xt::diag(p0_diagonal);
	engine.set_state_block_estimate(PINSON15, x0_pinson);
	engine.set_state_block_covariance(PINSON15, p0_pinson);

	// Create measurement processor for GPS position measurements
	Matrix measurement_function_jacobian_gps                          = zeros(3, NUM_STATES);
	view(measurement_function_jacobian_gps, range(0, 3), range(0, 3)) = eye(3);

	// This program uses GPS as position updates only, therefore, use
	// DirectPositionMeasurementProcessor. This measurement processor will be called
	// 'gps', and measurement objects processed by the fusion engine
	// which are labeled 'gps' will be sent to this measurement processor.
	engine.add_measurement_processor(make_shared<navtk::filtering::DirectMeasurementProcessor>(
	    "gps", PINSON15, measurement_function_jacobian_gps));

	// Next, create an ins object, giving it the initial position, attitude and initial velocity.
	// Adjust the initial inertial frame to match truth frame. Required for the s3 file data.
	// Note: INS sensor to platform DCM required for s3 data file
	Matrix C_platform_to_ned = navtk::navutils::rpy_to_dcm(attitude_initial);

	// In contrast to using the truth data, the alignment algorithm returns the
	// IMU sensor to nav DCM directly, meaning we do not need to do the platform to sensor
	// correction (to account for the offset between the platform to nav DCM recorded in truth
	// and the IMU sensor orientation on the platform when aligning from the truth messages)
	Matrix C_sensor_to_ned = C_platform_to_ned;
	if (ALIGN_FROM_TRUTH) {
		C_sensor_to_ned = navtk::dot(C_platform_to_ned, C_INS_TO_PLATFORM);
	}
	NavSolution ins_nav_solution(lla_initial, vned_initial, transpose(C_sensor_to_ned), t_start);
	Inertial ins = Inertial(make_shared<StandardPosVelAtt>(
	    StandardPosVelAtt(t_start, lla_initial, vned_initial, C_sensor_to_ned)));
	engine.give_state_block_aux_data(PINSON15,
	                                 create_pinson15_ins_aux_data(ins_nav_solution, GRAVITY));
	auto imu_to_erase = imu_series.get_in_range(aspn_xtensor::TypeTimestamp((int64_t)0), t_start);
	imu_series.erase(imu_to_erase.first, imu_to_erase.second);

	auto gps_to_erase = gps_series.get_in_range(aspn_xtensor::TypeTimestamp((int64_t)0), t_start);
	gps_series.erase(gps_to_erase.first, gps_to_erase.second);

	while (lcm.handleTimeout(10) > 0) {
		// break out of the example if the specified number of seconds has been processed
		if (seconds_to_process > 0 && t_val - t_start > seconds_to_process) break;
		// propagate any GPS messages (older than 5s for now)
		auto gps_to_process = gps_series.get_in_range(t_last_prop, t_val - GPS_PROCESS_DELAY);
		for (auto gps_it = gps_to_process.first; gps_it != gps_to_process.second; gps_it++) {
			auto& gps       = **gps_it;
			Vector3 lla_gps = {gps.get_term1(), gps.get_term2(), gps.get_term3()};

			// propagate the filter to GPS measurement time at MAX_PROP_DT intervals
			while (t_last_prop < gps.get_time_of_validity()) {
				// If measurements are out of order in the log file then some IMU measurements that
				// are valid before the start time can slip in. This clears them out.
				if (t_last_prop == t_start) {
					auto imu_before_start = imu_series.get_in_range(
					    aspn_xtensor::TypeTimestamp((int64_t)0), t_last_prop);
					imu_series.erase(imu_before_start.first, imu_before_start.second);
				}
				auto t_this_prop = std::min(t_last_prop + MAX_PROP_DT, gps.get_time_of_validity());
				auto imu_to_process = imu_series.get_in_range(t_last_prop, t_this_prop);
				if (imu_to_process.first != imu_to_process.second) {
					// mechanize any IMU messages up to the next interval
					std::for_each(imu_to_process.first, imu_to_process.second, [&ins](auto imu_it) {
						auto& imu = *imu_it;
						ins.mechanize(
						    imu.get_time_of_validity(), imu.get_meas_accel(), imu.get_meas_gyro());
					});
					auto& imu  = **(imu_to_process.second - 1);
					t_last_imu = imu.get_time_of_validity();
					engine.give_state_block_aux_data(PINSON15,
					                                 create_pinson15_ins_aux_data(ins, imu));
					imu_series.erase(imu_to_process.first, imu_to_process.second);
				}
				engine.propagate(t_this_prop);
				t_last_prop = t_this_prop;
			}

			// get the inertial solution position (mechanized at IMU time <= GPS time)
			Vector3 lla_ins     = ins.get_solution()->get_llh();
			Vector3 ned_pos_err = lla_gps - lla_ins;

			// mechanize (using a temporary INS), get the inertial solution, interpolate the LLA
			if (t_last_imu < gps.get_time_of_validity() && !imu_series.empty()) {
				Vector3 interpolated_lla =
				    interpolate_inertial_lla(imu_series, ins, gps.get_time_of_validity());
				ned_pos_err = lla_gps - interpolated_lla;
				lla_ins     = interpolated_lla;
			}

			// Convert latitude-longitude-altitude (rad-rad-meters) to north-east-down in meters
			ned_pos_err[0] =
			    navtk::navutils::delta_lat_to_north(ned_pos_err[0], lla_gps[0], lla_gps[2]);
			ned_pos_err[1] =
			    navtk::navutils::delta_lon_to_east(ned_pos_err[1], lla_gps[0], lla_gps[2]);
			ned_pos_err[2] *= -1;  // negate because the error is in the down direction

			// update the filter using the position (GPS) update
			engine.update("gps",
			              std::make_shared<navtk::filtering::GaussianVectorData>(
			                  gps.get_time_of_validity(), ned_pos_err, gps.get_covariance()));

			// Mechanize an additional IMU sample and propagate the filter to the IMU sample time
			// Mechanization ensures INS state stored in generate_output is at same time tag as
			// filter state
			if (t_last_imu < gps.get_time_of_validity() && !imu_series.empty()) {
				auto& imu = *imu_series.front();
				ins.mechanize(
				    imu.get_time_of_validity(), imu.get_meas_accel(), imu.get_meas_gyro());
				t_last_imu = imu.get_time_of_validity();
				engine.give_state_block_aux_data(PINSON15, create_pinson15_ins_aux_data(ins, imu));
				imu_series.pop_front();
				engine.propagate(t_last_imu);
				t_last_prop = t_last_imu;
				lla_ins     = ins.get_solution()->get_llh();
			}
			// Calculate output results of filter
			navtk::exampleutils::generate_output(navtk::exampleutils::CouplingType::LOOSE,
			                                     engine,
			                                     ins,
			                                     vector<string>({PINSON15}),
			                                     filter_output,
			                                     sigma_output,
			                                     ins_output,
			                                     output_filter_time,
			                                     C_INS_TO_PLATFORM);
			// FEEDBACK
			// Lastly, apply state feedback if desired. State feedback is always needed
			// in the vertical channel, and may be in the horizontal channel. In this
			// example, the full feedback of position, velocity, and attitude is implemented
			// but only when the estimated position error exceeds a predefined threshold
			// (FEEDBACK_THRESHOLD).
			if (APPLY_FEEDBACK) {
				Vector x_pinson = engine.get_state_block_estimate(PINSON15);
				Vector temp_x   = {x_pinson[0], x_pinson[1], x_pinson[2]};
				if (navtk::norm(temp_x) > FEEDBACK_THRESHOLD) {
					// Reset the inertial solution
					ins.reset(create_pva_estimate(ins, engine, lla_ins));
					// Reset the filter error state
					view(x_pinson, range(0, 9)) = 0;
					engine.set_state_block_estimate(PINSON15, x_pinson);
				}
			}
		}
		gps_series.erase(gps_to_process.first, gps_to_process.second);
	}

	// interpolate the truth data and convert to plottable type navtk
	auto truth_interp_result = navtk::exampleutils::convert_truth(truth_raw, output_filter_time);
	// pull out interpolated truth data
	truth_interp = truth_interp_result.second;
	// pull out unused output_filter_time tags indexes
	vector<size_t> unused_index = truth_interp_result.first;
	// Remove unused index from filter, sigma, INS, and output filter time to match
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
	// difference betwee filter and truth tilt results
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
		// plot the attitude profile
		navtk::exampleutils::plot_rpy(truth_raw, filter_results, time_tag_results);
		// plot the accelerometer and gyroscope bias filter results
		navtk::exampleutils::plot_acc_gyro_err(filter_results, sigma_results, time_tag_results);
		// show the figures
		show();
	}
	return 0;
}

// Function handler for truth data
void handle_truth(const lcm::ReceiveBuffer*,
                  const string&,
                  const positionvelocityattitude* msg,
                  TruthResults* destination_truth_state) {

	// This particular datafile stores heading from 0->2*pi, while we use -pi->pi
	auto mod_msg        = *msg;
	mod_msg.attitude[2] = navtk::navutils::wrap_to_pi(msg->attitude[2]);
	store_truth_lcm(*destination_truth_state, mod_msg);
}

// Function handler for IMU data
void handle_imu(const lcm::ReceiveBuffer*,
                const string&,
                const imu* msg,
                IMUHandlerContext destination_imu_state) {
	auto imu = make_shared<MeasurementImu>(navtk::exampleutils::to_aspn(*msg));
	destination_imu_state.series->insert(imu);
	*(destination_imu_state.validity) = imu->get_time_of_validity();
}

// Function handler for GPS data
void handle_gps(const lcm::ReceiveBuffer*,
                const string&,
                const geodeticposition3d* msg,
                GpsSeries* destination_gps_series) {
	auto gps = make_shared<MeasurementPosition>(navtk::exampleutils::to_aspn(*msg));
	destination_gps_series->insert(gps);
}

AspnBaseVector create_pinson15_ins_aux_data(const Inertial& ins, const MeasurementImu& imu) {
	shared_ptr<InertialPosVelAtt> ins_pva = ins.get_solution();

	auto pva = std::make_shared<aspn_xtensor::MeasurementPositionVelocityAttitude>(
	    navtk::utils::to_positionvelocityattitude(*ins_pva));

	Vector3 f_ned = navtk::inertial::calc_force_ned(
	    ins_pva->get_C_s_to_ned(), IMU_DT, imu.get_meas_gyro(), imu.get_meas_accel());
	auto f_and_r = std::make_shared<aspn_xtensor::MeasurementImu>(
	    navtk::utils::to_imu(pva->get_time_of_validity(), f_ned, zeros(3)));

	return AspnBaseVector{pva, f_and_r};
}

AspnBaseVector create_pinson15_ins_aux_data(const NavSolution& nav_sol, const Vector& forces) {
	return navtk::utils::to_inertial_aux(nav_sol, forces, zeros(3));
}

shared_ptr<StandardPosVelAtt> create_pva_estimate(const Inertial& ins,
                                                  const StandardFusionEngine& engine,
                                                  const Vector3& use_this_lla) {
	shared_ptr<InertialPosVelAtt> ins_pva = ins.get_solution();
	auto mod_pva                          = StandardPosVelAtt(
        engine.get_time(), use_this_lla, ins_pva->get_vned(), ins_pva->get_C_s_to_ned());
	Vector x_pinson = (engine).get_state_block_estimate(PINSON15);
	return make_shared<StandardPosVelAtt>(
	    navtk::filtering::apply_error_states<navtk::filtering::Pinson15NedBlock>(mod_pva,
	                                                                             x_pinson));
}

Vector3 interpolate_inertial_lla(const ImuSeries& imu_series,
                                 Inertial tmp_ins,
                                 const aspn_xtensor::TypeTimestamp& time) {
	auto last_pva    = tmp_ins.get_solution();
	Vector3 last_lla = last_pva->get_llh();
	auto& next_imu   = **imu_series.cbegin();
	tmp_ins.mechanize(
	    next_imu.get_time_of_validity(), next_imu.get_meas_accel(), next_imu.get_meas_gyro());
	auto next_pva    = tmp_ins.get_solution();
	Vector3 next_lla = next_pva->get_llh();
	return navtk::utils::linear_interpolate(
	    last_pva->time_validity, last_lla, next_pva->time_validity, next_lla, time);
}
