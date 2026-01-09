#pragma once

#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/inertial/AlignBase.hpp>
#include <navtk/inertial/BasicInsAndFilter.hpp>
#include <navtk/inertial/CoarseDynamicAlignment.hpp>
#include <navtk/inertial/DynData.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/MovementDetectorImu.hpp>
#include <navtk/inertial/MovementStatus.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Class that performs a dynamic alignment using IMU (\f$\delta v\f$ and \f$\delta \theta\f$) and
 * position measurements. Performance is affected by inertial biases, the level of position noise
 * with respect to the platform velocity, and the accuracy of the initial position.
 *
 * As the algorithm is fairly complicated, a high-level description follows.
 *
 * First a period of stationary IMU data is collected to roughly estimate gyro biases and minimize
 * error in the integrated orientation. Then position and IMU data are collected. With two
 * sequential position measurements defining a measurement interval, the delta rotations within the
 * interval are integrated to provide the change in rotation over the interval. These rotations are
 * tracked over the entire alignment period to provide a total estimate of orientation with respect
 * to the initial frame. Then the 2 position measurements are joined with the immediate prior
 * position measurement and double-differenced to provide a reference value of the average specific
 * force in the initial NED frame at the time of the middle position measurement. The delta
 * velocities over the measurement interval are averaged and rotated back into the initial IMU
 * sensor frame and scaled to provide specific forces. These 2 force vectors (NED frame from
 * positions, initial IMU frame from IMU data) are then used to estimate the solution to Wahba's
 * problem, which concerns finding the rotation between 2 vector observations taken in different
 * frames that differ by a fixed rotation. Once the solution has settled (based on the covariance of
 * the last N solutions) this stage is considered complete.
 *
 * However, the method used to solve Wahba's problem may provide more than one solution
 * (theoretically 4, typically 2), so a second stage is used to discriminate between these. Each
 * solution is used to seed a BufferedImu and a position/INS filter estimating Pinson states.
 * These are continuously updated with inertial and position data, and once one judged to be better
 * than the rest (based on absolute and relative measures) the winning inertial and filter solutions
 * are used to supply the alignment values.
 */
class CoarseDynamicAlignment : public AlignBase {

public:
	/**
	 * Constructor.
	 *
	 * @param model Model of the inertial being aligned.
	 * @param static_time Minimum number of seconds of stationary data that is expected at the
	 * start of the run.
	 * @param reset_time How long, in seconds, to continue trying the current stage of alignment,
	 *  with current data before reinitializing and trying again.
	 * @param dcm_integration_method Method to use when integrating DCMs in various places.
	 */
	CoarseDynamicAlignment(
	    const filtering::ImuModel& model             = filtering::stim300_model(),
	    double static_time                           = 30.0,
	    double reset_time                            = 300.0,
	    DcmIntegrationMethods dcm_integration_method = DcmIntegrationMethods::FIRST_ORDER);

	/**
	 * Add a measurement to be used in the alignment.
	 *
	 * @param message Measurement to use. Currently, only aspn_xtensor::MeasurementPosition
	 * and aspn_xtensor::MeasurementImu are recognized. Imu data must be
	 * ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED type. All others will be ignored.
	 *
	 * @return Status of alignment after incorporating the input.
	 *
	 * @throw std::invalid_argument if error mode is DIE and an imu measurement of type other
	 * than ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED is received.
	 */
	AlignmentStatus process(std::shared_ptr<aspn_xtensor::AspnBase> message) override;

	/**
	 * Get computed covariance
	 *
	 * @param format Format for the covariance block.
	 *
	 * @return A pair with a bool representing validity of the attached matrix, and the NxN
	 * covariance matrix (see AlignBase::get_computed_covariance for units).
	 */
	std::pair<bool, Matrix> get_computed_covariance(
	    const CovarianceFormat format = CovarianceFormat::PINSON15NEDBLOCK) const override;

	/**
	 * Get IMU errors
	 *
	 * @return A bool representing the validity of the attached errors, and the estimated accel
	 * (m/s^2) and gyro bias (rad/s) parameters; no other ImuErrors fields are currently estimated.
	 */
	std::pair<bool, ImuErrors> get_imu_errors() const override;

	MotionNeeded motion_needed() const override;

private:
	/* Tracks where in the dynamic alignment process we are. */
	enum class Stage { INITIAL_STATIC, WAHBA_SOLVE, SOLUTION_COMPARE };

	/*
	 * Search align_buffer and pull out any data occuring after t and return in a separate vector
	 * for use in next iteration.
	 *
	 * @param t Time to begin separating IMU data.
	 *
	 * @return Imu data with time_validity > t.
	 */
	std::vector<aspn_xtensor::MeasurementImu> separate_imu_after_time(
	    const aspn_xtensor::TypeTimestamp& t);

	/*
	 * Set members using first position measurement.
	 *
	 * @param pos Initial position measurement.
	 */
	void initialize_with_position(
	    const not_null<std::shared_ptr<aspn_xtensor::MeasurementPosition>> pos);

	/*
	 * Update move status based on current IMU data.
	 *
	 * @param imu Current IMU measurement.
	 */
	void update_move_status(const not_null<std::shared_ptr<aspn_xtensor::MeasurementImu>> imu);

	/*
	 * Update IMU time tracking based on current IMU data.
	 *
	 * @param imu Current IMU measurement.
	 */
	void update_imu_times(const not_null<std::shared_ptr<aspn_xtensor::MeasurementImu>> imu);

	/*
	 * Warn if already aligned or data too old; otherwise mechanize all prospects. If data valid
	 * will push to align_buffer.
	 *
	 * @param imu Current IMU measurement.
	 */
	void mechanize_or_warn(const not_null<std::shared_ptr<aspn_xtensor::MeasurementImu>> imu);

	/*
	 * Determine calibration status and notify user with log messages.
	 */
	void update_calibration_notifications();

	/*
	 * Add new data to the alignment algorithm and estimate a new solution.
	 *
	 * @param pos Position measurement to attempt alignment with.
	 * @param trim Ordered IMU data that has already been buffered by the algorithm but occurred
	 * after the given position.
	 */
	void add_new_meas(const aspn_xtensor::MeasurementPosition& pos,
	                  const std::vector<aspn_xtensor::MeasurementImu>& trim);

	/*
	 * Propagates a DCM using a direct integration of delta rotation values.
	 * Uses integration method chosen at initialization.
	 *
	 * @param C_s_to_n Sensor to nav DCM at time k.
	 *
	 * @return \f$ C^n_{s_{k + 1}} \f$
	 */
	Matrix3 integrate_dth(Matrix3 C_s_to_n);

	/*
	 * Compares the error between filter position estimates and pos, which should be the last
	 * position used to update the filters.
	 */
	void check_prospects(const aspn_xtensor::MeasurementPosition& pos);

	/*
	 * Reset some variables, depending on what stage of alignment we are in, to reduce accumulated
	 * errors and/or attempt to get out of bad state (lots of bad measurements into Wahba solution
	 * inputs, for instance)
	 */
	void reset(const aspn_xtensor::TypeTimestamp& time);

	/* Initialize a temporary inertial with provided PVA and mechanize data include in trim */
	filtering::NavSolution mech_align(const aspn_xtensor::MeasurementPosition& pos,
	                                  const Vector3& vel,
	                                  const Matrix3& C_s_to_n,
	                                  const std::vector<aspn_xtensor::MeasurementImu>& trim);

	/*
	 * Calculate the tilt covariance of the last n solutions to Wahba's problem, used to determine
	 * solution convergence (or just lack of new information from measurements, unfortunately).
	 */
	std::pair<bool, Matrix3> update_last_n(const Matrix3& c_b0_to_n);

	/* Use the last 2 positions received to estimate the avg velocity covariance */
	Matrix update_vel_cov() const;


	/* Estimated delta time of IMU date */
	double approx_dt();

	/* Initialize a new BasicInsAndFilter using provided stats and add to prospects */
	void add_prospect(const filtering::NavSolution& ns,
	                  const Matrix& init_cov,
	                  const Matrix3& initial_cns);

	/*
	 * Assign sol/covariance once a good prospect has been anointed. Calling this means alignment
	 * is finished.
	 *
	 * @param prospect_index Index into #prospects that has the chosen solution.
	 */
	void select_prospect(const Size prospect_index);

	/*
	 * Generates most recent NavSolutions using the given initial conditions. Initializes an
	 * inertial and then mechanizes through all of align_buffer and then all of trim (in other words
	 * all available IMU data).
	 *
	 * @param pos Position at the start of data in align_buffer.
	 * @param vel Velocity at the start of data in align_buffer.
	 * @param C_k_to_start DCM that rotates from the IMU frame at time k (start of data in
	 * align_buffer) to initial IMU frame.
	 * @param C_start_to_n_vec Vector of estimated DCMs that rotate from initial IMU frame to NED
	 * frame.
	 * @param trim Imu data occurring after that in align_buffer.
	 *
	 * @return A vector of NavSolutions at time of last element in /p trim, same size as /p
	 * C_start_to_n_vec.
	 */
	std::vector<filtering::NavSolution> generate_prospective_solutions(
	    const aspn_xtensor::MeasurementPosition& pos,
	    const Vector3& vel,
	    const Matrix3& C_k_to_start,
	    const std::vector<Matrix3>& C_start_to_n_vec,
	    const std::vector<aspn_xtensor::MeasurementImu>& trim);

	/*
	 * Calculate the initial covariance to set Pinson parameters for prospective alignment
	 * solutions.
	 *
	 * @param cnps Estimated initial IMU to NED frame rotations.
	 * @param sol Solutions that covariance will be attached to. These should correspond one-to-one
	 * with the elements of cnps.
	 * @param test_cov Initial tilt covariance estimate. Assumed to be valid for all solutions.
	 *
	 * @return A collection of 15x15 Pinson PVA matrices corresponding to each element of sol.
	 */
	std::vector<Matrix> calc_initial_solution_cov(const std::vector<Matrix3>& cnps,
	                                              const std::vector<filtering::NavSolution>& sol,
	                                              const Matrix3& tilt_cov);

	/*
	 * Use measured (IMU) and reference (pos derived) force measurements to update the
	 * Wahba/Davenport function inputs and estimate the initial IMU to nav DCM.
	 */
	void update_wahba_inputs();

	// Number of position measurements received after movement detected
	Size call_num = 0;

	// DCM from IMU frame at t(k) to initial IMU frame, calculated from integrating dth measurements
	Matrix3 C_k_to_start = eye(3);
	// DCM from IMU frame at t(k - 1) to initial IMU frame, calculated from integrating dth
	// measurements
	Matrix3 C_km1_to_start = eye(3);
	// Sum of outer products of time-synced specific forces in IMU and NED frames; input to Wahba
	// solver (davenport_q method)
	Matrix3 B = zeros(3, 3);

	// Sum of cross terms of time-synced specific forces in IMU and NED frames; i.e. same vectors
	// used to form outer products that form B; used in davenport solver but not original Wahba
	// solver
	Vector3 cross_terms = zeros(3);

	// Vector of estimated initial IMU orientations from Wahba solver
	std::vector<Matrix3> est_cnps;

	// How long to wait, in seconds, before calling reset(). Set by user at initialization.
	double give_up_time;

	// Counter for tracking stored solutions, this time of last_n_solutions
	Size sol_ind = 0;

	// vector of last n (n = num_solutions_in_cov) IMU to nav estimated DCMs
	std::vector<Matrix3> last_n_solutions;

	// vector of full INS + filters, initialized using position measurements and estimated DCMs from
	// Wahba solutions and mechanized dth data the best of these is the source of the computed
	// alignment
	std::vector<BasicInsAndFilter> prospects;

	// Covariance from the best of prospects, returned by get_computed_covariance
	Matrix full_cov = zeros(15, 15);

	// Average delta_v from stationary data collected by movement_detector, used to estimate initial
	// biases
	Vector3 stat_dv_mean = zeros(3);
	// Average delta_theta from stationary data collected by movement_detector, used to estimate
	// initial biases
	Vector3 stat_dth_mean = zeros(3);

	// IMU errors estimated by the best of prospects; empty until one is selected.
	ImuErrors imu_bs;

	// Time of first IMU data received
	aspn_xtensor::TypeTimestamp first_imu_time = aspn_xtensor::to_type_timestamp();
	// Time of latest IMU data received
	aspn_xtensor::TypeTimestamp latest_imu_time = aspn_xtensor::to_type_timestamp();
	// Number of IMU data received; used with imu_time data to estimate IMU dt and to trigger
	// messages to console
	Size num_imu_received = 0;

	// Max value of size_t, to make sure we don't roll over our IMU counter
	Size max_imu_count = 0;

	// Detects movement based on sensor data
	MovementDetectorImu detector;

	// Immediate last status of detection algorithm
	MovementStatus last_status = MovementStatus::NOT_MOVING;

	// Currently set to true when moving_ctr > 100, kicks off main algorithm
	bool movement_detected = false;

	/* Flag for tracking if user has been notified that stationary period has completed */
	bool sustained_notified = false;
	/* Ctor supplied static alignment time */
	double static_time;
	/* How often user is updated on calibration status (s), based on IMU message time. */
	double calib_notify_period = 5.0;
	/* Last time we notified of stationary calibration status */
	aspn_xtensor::TypeTimestamp calib_last_notify = aspn_xtensor::to_type_timestamp();


	/* Magic Numbers */

	// Assumed value of dt if not enough IMU data to actually calculate it. Updated over time.
	double default_dt = 0.01;

	// Threshold for the minimum down tilt sigma needed to declare the Wahba solution 'stable' and
	// to enter the next phase (filtering and testing residuals) (deg)
	double down_tilt_sig_thresh = 5.0;

	// Number of measurements that need to be received before moving on to filtering of possible
	// solutions (Trying to assure variation in Wahba inputs)
	Size min_meas_before_solution_gen = 50;

	// When comparing solutions, at least one error norm must be outside the max (by axis) sigma
	// times this number before further testing is done (in other words making sure at least one
	// solution looks sufficiently bad that another one can be called 'good' by comparison)
	double position_sigma_multiplier = 2.0;

	// Ratio that a 'good' solution must beat all other 'bad' solutions by to be declared the best
	// estimate (in other words, the norm position residual on all bad solutions must be error_ratio
	// times greater than the error norm of teh good solution)
	double error_ratio = 15.0;

	// Dictates size of last_n_solutions
	Size num_solutions_in_cov = 10;

	// Collects stationary data during bias calibration period
	std::vector<Vector3> stationary_dv;
	std::vector<Vector3> stationary_dth;

	// Tracks last time movement detector switched from something other than moving to moving, to
	// allow for monitoring for sustained movement
	aspn_xtensor::TypeTimestamp moving_start_time = aspn_xtensor::to_type_timestamp();

	// Number of seconds of sustained movement required to move from stationary period to wahba
	// solution calculation
	double required_moving_period = 1.0;

	// Function that integrates delta theta measurements so they can be used to update a DCM.
	std::function<Matrix3(const Vector3&)> dtheta_integrator;

	// Convenience class that calculates a bunch of the nav data required to generate Wahba solver
	// inputs
	std::shared_ptr<DynData> iteration_data = nullptr;

	// What stage of dynamic alignment we are in
	Stage current_stage = Stage::INITIAL_STATIC;

	// Time since we last did a hard reset (or changed current_stage)
	aspn_xtensor::TypeTimestamp time_since_last_reset = aspn_xtensor::to_type_timestamp();

	// Collection of time-ordered linearization points over the period that we are attempting to
	// calculate wahba solutions; used to propagate a covariance forward from the initial time to
	// the `current` so we can kick off filters.
	std::vector<
	    std::pair<aspn_xtensor::MeasurementPositionVelocityAttitude, aspn_xtensor::MeasurementImu>>
	    lin_points;
};
}  // namespace inertial
}  // namespace navtk
