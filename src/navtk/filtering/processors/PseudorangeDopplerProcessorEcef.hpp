#pragma once

#include <memory>
#include <string>
#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/PseudorangeDopplerMeasurements.hpp>
#include <navtk/filtering/containers/RangeInfo.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/containers/TrackedGnssObservations.hpp>
#include <navtk/filtering/fusion/StandardFusionEngineBase.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/gnssutils/CarrierPhaseType.hpp>
#include <navtk/gnssutils/GpsEphemerisHandler.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

/*
 * Duplicates much of the code in SinglePointPseudorangeProcessor. Rework to remove
 * code duplication.
 * TODO: PNTOS-550
 */

namespace navtk {
namespace filtering {

/**
 * Updates a 6-state state block (3 states of ECEF position in meters, 3 states of ECEF velocity in
 * meters/second), a ClockBiasesStateBlock, and a processor-managed collection of FogmBlock
 * instances using measurements provided as PairedPva (wrapping a MeasurementSatnav) and associated
 * GpsEphemerisAux. Each PRN's pseudorange bias is estimated using a First-order Gaussian Markov
 * (FOGM) state block. The FOGM state blocks are automatically created/removed from inside
 * PseudorangeDopplerProcessorEcef based on the PRNs available in the pseudorange measurement. Each
 * PRN's pseudorange bias state block label is a concatenation of #pr_bias_sb_label +
 * aspn_xtensor::TypeSatnavObs::get_prn(). As an example label, PRN 32 would have the pseudorange
 * bias state block label `"pr_bias_sv_32"`. This processor corrects pseudoranges for ionospheric
 * delay (if the receiver is dual frequency), group delay (when applicable), satellite clock error,
 * and tropospheric delay.
 */
class PseudorangeDopplerProcessorEcef : public MeasurementProcessor<> {

public:
	/**
	 * Main constructor for the processor.
	 * @param label The identifier for this instance of this processor.
	 * @param pv_ecef_label The label to StateBlock that contains whole-state (corrected)
	 * lever arm-shifted (i.e. at sensor antenna) ECEF position coordinates and velocities.
	 * @param clock_biases_label The label to an instance of ClockBiasesStateBlock.
	 * @param cp_to_use An enum specifying which signal to use.
	 * @param pr_noise_covariance Determines the diagonals of the pseudorange covariance matrix such
	 * that `R = I * pr_noise_covariance`, in meters^2.
	 * @param pr_bias_covariance Determines the covariance for the pseudorange bias First-order
	 * Gauss-Markov state model states, in meters^2.
	 * @param pr_time_constant The pseudorange bias state time constant for the First-order Gauss
	 * Markov model used for all PRNs, in seconds.
	 * @param prr_noise_covariance Determines the diagonals of the pseudorange rate covariance
	 * matrix such that `R = I * prr_noise_covariance`, in meters^2/second^2.
	 * @param engine_in Non-null shared pointer to current fusion engine using
	 * PseudorangeDopplerProcessorEcef.
	 * @param apply_tropo_model When true, applies error in a pseudorange due to tropospheric delay.
	 * @param tropo_rel_humidity (optional) Relative humidity value to use for the tropospheric
	 * delay calculation; fraction between 0 and 1.
	 * @param elevation_mask (optional) Any satellites below this elevation (radians) will not be
	 * used.
	 * @param force_clock_initialization When set `true`, uses the first set of measurements to only
	 * update the clock bias state. Subsequent updates use the full model. Useful in cases when
	 * there is high uncertainty in the receiver clock bias, but initial PVA is roughly known.
	 */
	PseudorangeDopplerProcessorEcef(std::string label,
	                                std::string pv_ecef_label,
	                                std::string clock_biases_label,
	                                gnssutils::CarrierPhaseType cp_to_use,
	                                double pr_noise_covariance,
	                                double pr_bias_covariance,
	                                double pr_time_constant,
	                                double prr_noise_covariance,
	                                not_null<std::shared_ptr<StandardFusionEngineBase>> engine_in,
	                                bool apply_tropo_model          = true,
	                                double tropo_rel_humidity       = 0.5,
	                                double elevation_mask           = -1,
	                                bool force_clock_initialization = true);

	/**
	 * Generates a StandardMeasurementModel instance that maps the input data to the estimated
	 * states.
	 *
	 * @param measurement The raw measurement received from the sensor.
	 * @param gen_x_and_p_func A function that will generate `xhat` (a Vector of estimated states,
	 * constructed from state blocks referenced by `state_block_labels`) and `P` (covariance Matrix
	 * for `xhat`) when called.
	 *
	 * @return A StandardMeasurementModel containing the parameters
	 * required for filter update. In general, the model contains: \n
	 * - `z`: A size M Vector of measurement values. \n
	 * - `h`: A function which maps states to measurements (as `zhat = h(xhat)`), accepting `xhat`
	 *   as an argument and producing an M-length Vector. \n
	 * - `H`: An MxN Matrix relating `xhat` to `z` (as `zhat = H*xhat`, approximately); the Jacobian
	 * of `h`. \n
	 * - `R`: MxM covariance Matrix for measurement Vector `z`. \n
	 *
	 * A normal return will include updates for position, velocity, and clock states. \n
	 * If clock initialization was not forced (`force_clock_initialization` parameter is `false`
	 * in constructor call) then the return from the first call to this function will assume that
	 * all error between inertial and pseudoranges is due to clock. \n
	 * Return will be `nullptr` in any of the following scenarios: \n
	 * - measurement is not of type filtering::PairedPva or MeasurementSatnav \n
	 * - no ephemeris data has been added via aux data \n
	 * - there are no valid ephemerides at the given time for the given PRNs \n
	 * - there are no valid ephemerides available for any of the mobile pseudoranges \n
	 * - the elevation mask eliminates all of the valid measurements.
	 */
	std::shared_ptr<StandardMeasurementModel> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase> measurement,
	    GenXhatPFunction gen_x_and_p_func) override;

	/**
	 * Receive and use arbitrary aux data. This method will be called by the fusion engine when the
	 * fusion engine receives aux data from an external call.
	 *
	 * @param aux_data An AspnBaseVector containing RelativeHumidityAux and/or
	 * aspn_xtensor::MetadataGpsLnavEphemeris messages.
	 */
	void receive_aux_data(const AspnBaseVector &aux_data) override;

	/**
	 * Create a copy of the MeasurementProcessor with the same properties.
	 * @return A shared pointer to a copy of the MeasurementProcessor.
	 */
	not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override;

protected:
	/**
	 * Add/remove pseudorange bias states based on observed SVs, and update `state_block_labels` and
	 * total state count accordingly.
	 *
	 * @param time Time of observation
	 * @param corrected_meas Return value of assemble_measurements()
	 */
	void update_state_block_tracking(const aspn_xtensor::TypeTimestamp &time,
	                                 const PseudorangeDopplerMeasurements &corrected_meas);

	/**
	 * Generate the measurement Jacobian `H`, the partial derivative of `h(x)` w.r.t. `x`, where `x`
	 * is ECEF position/velocity states, receiver clock bias states, and pseudorange bias states.
	 *
	 * @param los Line-of-sight Vectors from receiver to SVs, in same frame as position states.
	 * Order should match the PRN order of current pseudorange bias states.
	 * @param state_count Total number of states contained in all StateBlocks referenced by
	 * `state_block_labels`
	 *
	 * @return Partial derivative of the measurement prediction function; an MxN Matrix where M
	 * is the number of measurements (`ranges.size()`) and N is the number of states
	 * (`state_count`).
	 */
	Matrix generate_h_jacobian(const std::vector<Vector3> &los, Size state_count) const;

	/**
	 * Build a measurement model that updates only the clock bias state using a set of pseudoranges.
	 * Useful as an initial measurement update when there is high uncertainty in receiver clock bias
	 * as compared to position uncertainty.
	 *
	 * @param los Line-of-sight Vectors from receiver to SVs, in same frame as position states.
	 * Order should match the PRN order of current pseudorange bias states.
	 * @param meas Return value of assemble_measurements().
	 * @param xhat State Vector consisting of ECEF position/velocity states (the first cooresponding
	 * to #pv_state_index) and 2 clock bias states (the first corresponding to #clock_state_index).
	 *
	 * @return A MeasurementModel that assumes receiver clock bias is the dominant error source and
	 * updates the state at #clock_state_index with the average of the pseudorange residuals.
	 */
	std::shared_ptr<StandardMeasurementModel> generate_clock_bias_update(
	    const std::vector<Vector3> &los,
	    const PseudorangeDopplerMeasurements &meas,
	    const Vector &xhat) const;

	/**
	 * Build a measurement model that updates ECEF position/velocity states, clock bias and drift,
	 * and pseudorange biases.
	 *
	 * @param los Line-of-sight Vectors from receiver to SVs, in same frame as position states.
	 * Order should match the PRN order of current pseudorange bias states.
	 * @param meas Return value of assemble_measurements().
	 * @param H Measurement function Jacobian mapping ECEF position/velocity states, clock bias
	 * states and pseudorange bias states to a Vector of pseudorange measurements.
	 *
	 * @return A MeasurementModel where \f$ h(x) = \hat{r}_{sv} + \hat{bias}_{sv} + C * bias_{rcvr}
	 * \f$
	 */
	std::shared_ptr<StandardMeasurementModel> generate_standard_update(
	    const std::vector<Vector3> &los,
	    const PseudorangeDopplerMeasurements &meas,
	    const Matrix &H) const;

	/**
	 * Calculates the predicted measurement.
	 *
	 * @param los Line-of-sight Vectors from receiver to SVs, in same frame as position states.
	 * Order should match the PRN order of current pseudorange bias states.
	 * @param xhat A Vector of estimates for the concatenated states.
	 * @param sv_pos An Nx3 Matrix containing the ECEF position (meters) of each satellite.
	 * @param sv_vel An Nx3 Matrix containing the ECEF velocity (meters/second) of each satellite.
	 *
	 * @return A Vector of N predicted measurements where \f$ h(x) = \hat{r}_{sv} + \hat{bias}_{sv}
	 * + C * bias_{rcvr} \f$
	 */
	Vector generate_h(const std::vector<Vector3> &los,
	                  const Vector &xhat,
	                  const Matrix &sv_pos,
	                  const Matrix &sv_vel) const;

	/**
	 * Estimates SV (Space Vehicle) clock error, position, velocity, elevation, and azimuth. Removes
	 * SVs based on mask. Corrects pseudorange measurements for group delay, clock error,
	 * tropospheric delay. Converts dopplers to `pseudorange_rates`.
	 *
	 *  @param prn_data_in Measurements to be corrected
	 *  @param receiver_time Time of the receiver for which measurements are valid
	 *  @param receiver_pos_llh The LLH receiver coordinates (rad, rad, meter)
	 *  @param week_num GPS week number associated with measurement data
	 *
	 *  @return Measurement data.
	 */
	PseudorangeDopplerMeasurements assemble_measurements(
	    const std::vector<aspn_xtensor::TypeSatnavObs> &prn_data_in,
	    double receiver_time,
	    const Vector3 &receiver_pos_llh,
	    int week_num);

	/**
	 * Calculates the range vector and scalar from a receiver to each satellite.
	 *
	 * @param receiver_pos A Vector3 containing the ECEF position (meters) of the receiver.
	 * @param sv_pos An Nx3 Matrix containing the ECEF position (meters) of each satellite.
	 *
	 * @return A list of RangeInfos.
	 */
	std::vector<RangeInfo> calc_range(const Vector3 &receiver_pos, const Matrix &sv_pos);

	/**
	 * Creates a FOGM state block for a single PRN pseudorange bias estimate. Adds the
	 * state block to the PseudorangeDopplerProcessorEcef.
	 *
	 * @param state_block_label string label for the FOGM state block.
	 */
	void set_new_pr_bias_state_block(std::string state_block_label);

	/**
	 * Tracks the PRNs in the measurements
	 */
	TrackedGnssObservations tracked_gnss_observations;

	/**
	 * Specifies what frequency and carrier phase to select measurements from.
	 */
	gnssutils::CarrierPhaseType cp_to_use;

	/**
	 * Determines the pseudorange covariance generated by the MeasurementProcessor.
	 */
	double pr_noise_covariance;

	/**
	 * Determines the covariance for the pseudorange bias first-order Gauss-Markov
	 * state model states.
	 */
	double pr_bias_sigma;

	/**
	 * The pseudorange bias state time constant for the first-order Gauss-Markov
	 * model used for all PRNs.
	 */
	double pr_time_constant;

	/**
	 * Determines the pseudorange rate covariance generated by the MeasurementProcessor.
	 */
	double prr_noise_covariance;

	/**
	 * An object from which the processor gets ephemerides.
	 */
	gnssutils::GpsEphemerisHandler ephemeris_handler;

	/**
	 * Relative humidity of the atmosphere. Used to calculate tropospheric error.
	 */
	double tropo_rel_humidity;

	/**
	 * A threshold to eliminate satellites at low altitudes which might have higher-than-usual error
	 * in their pseudoranges.
	 */
	double elevation_mask;

	/**
	 * A switch to turn on and off the tropospheric correction.
	 */
	bool apply_tropo_model;

	/**
	 * Shared pointer to a fusion engine. Required as input to add and remove the pseudorange bias
	 * FOGM state blocks.
	 */
	std::shared_ptr<StandardFusionEngineBase> engine;

	/**
	 * If `false` then the measurements will be used to construct a single update which only updates
	 * the clock bias state, then it will be set to `true`.
	 */
	bool clock_initialized;

	/**
	 * `true` if ephemerides have been added to the ephemeris handler.
	 */
	bool ephemeris_handler_initialized = false;

	/**
	 * The beginning index of the clock bias/drift states, assumed to be 7th state
	 */
	Size clock_state_index = 6;

	/**
	 * The begining index of the position/velocity states, assumed to be the 1st state
	 */
	Size pv_state_index = 0;

	/**
	 * The beginning index of the carrier phase bias states, assumed to be 9th state.
	 * The end index determined based on the carrier phase measurements observed,
	 * temporarily set to 0.
	 */
	std::pair<Size, Size> xhat_pr_bias_index = std::make_pair(8, 0);

	/**
	 * Label for generating the carrier phase bias FOGM state blocks. The labels
	 * for each PRN will be in the format `"pr_bias_sv_"`+`PRN_NUM`. For example,
	 * PRN 32 will have the pr bias state block label `"pr_bias_sv_32"`.
	 */
	const std::string pr_bias_sb_label = "pr_bias_sv_";
};

}  // namespace filtering
}  // namespace navtk
