#include <navtk/filtering/processors/PseudorangeDopplerProcessorEcef.hpp>

#include <stdexcept>
#include <string>

#include <spdlog/spdlog.h>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xview.hpp>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/containers/RelativeHumidityAux.hpp>
#include <navtk/filtering/fusion/StandardFusionEngineBase.hpp>
#include <navtk/filtering/processors/PseudorangeDopplerProcessorEcef.hpp>
#include <navtk/filtering/stateblocks/FogmBlock.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/gnssutils/assemble_cps.hpp>
#include <navtk/gnssutils/assemble_obs_ephem.hpp>
#include <navtk/gnssutils/assemble_prs.hpp>
#include <navtk/gnssutils/calc_sv_elevation.hpp>
#include <navtk/gnssutils/calc_sv_info.hpp>
#include <navtk/gnssutils/calc_tropo_corr.hpp>
#include <navtk/gnssutils/time_utils.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>

constexpr uint8_t FIRST_PR_BIAS_LABEL_INDEX = 2;

namespace navtk {
namespace filtering {

using aspn_xtensor::MeasurementSatnav;
using aspn_xtensor::TypeSatnavTime;
using navtk::gnssutils::assemble_obs_ephem;
using navtk::gnssutils::CarrierPhaseType;
using navtk::gnssutils::GpsEphemerisHandler;
using navtk::gnssutils::L1_L2_RATIO_2;
using navtk::gnssutils::LIGHT_SPEED;
using std::make_shared;
using std::shared_ptr;
using std::string;
using std::vector;

PseudorangeDopplerProcessorEcef::PseudorangeDopplerProcessorEcef(
    string label,
    string pv_ecef_label,
    string clock_biases_label,
    CarrierPhaseType cp_to_use,
    double pr_noise_covariance,
    double pr_bias_covariance,
    double pr_time_constant,
    double prr_noise_covariance,
    not_null<shared_ptr<StandardFusionEngineBase>> engine_in,
    bool apply_tropo_model,
    double tropo_rel_humidity,
    double elevation_mask,
    bool force_clock_initialization)
    : MeasurementProcessor(label, vector<string>{pv_ecef_label, clock_biases_label}),
      cp_to_use(cp_to_use),
      pr_noise_covariance(pr_noise_covariance),
      pr_bias_sigma(std::sqrt(pr_bias_covariance)),
      pr_time_constant(pr_time_constant),
      prr_noise_covariance(prr_noise_covariance),
      tropo_rel_humidity(tropo_rel_humidity),
      elevation_mask(elevation_mask),
      apply_tropo_model(apply_tropo_model),
      engine(engine_in),
      clock_initialized(!force_clock_initialization) {}

shared_ptr<StandardMeasurementModel> PseudorangeDopplerProcessorEcef::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	shared_ptr<MeasurementSatnav> observation;

	auto data = std::dynamic_pointer_cast<PairedPva>(measurement);

	if (data != nullptr) {
		observation = std::dynamic_pointer_cast<MeasurementSatnav>(data->meas_data);
	} else {
		observation = std::dynamic_pointer_cast<MeasurementSatnav>(measurement);
	}

	if (observation == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (PairedPva or MeasurementSatnav). Unable to "
		    "perform update.");
		return nullptr;
	}

	// Generate the vector of pseudoranges, PRNs
	vector<aspn_xtensor::TypeSatnavObs> meas_data =
	    gnssutils::assemble_cps(*observation, cp_to_use);

	auto xhat = gen_x_and_p_func(state_block_labels);
	if (xhat == nullptr) {
		return nullptr;
	}

	// Pull out position coordinates, assumed in ECEF and corrected for lever arms (ie antenna
	// phase center position).
	Vector3 receiver_pos_ecef =
	    xt::view(xhat->estimate, xt::range(pv_state_index, pv_state_index + 3));
	Vector3 receiver_pos_llh = navutils::ecef_to_llh(receiver_pos_ecef);

	// Correct pseudoranges and calculate satellite position, azimuth, elevation
	auto clock_time = observation->get_receiver_clock_time();
	PseudorangeDopplerMeasurements measurements =
	    assemble_measurements(meas_data,
	                          clock_time.get_seconds_of_week(),
	                          receiver_pos_llh,
	                          clock_time.get_week_number());

	if (measurements.prns.empty()) {
		spdlog::warn("Unable to produce corrected measurements: aborting update.");
		return nullptr;
	}

	update_state_block_tracking(observation->get_time_of_validity(), measurements);

	// Get a fresh xhat to include any reconfigured measurement bias states
	auto xhat_p = gen_x_and_p_func(state_block_labels);
	if (xhat_p == nullptr) {
		return nullptr;
	}

	auto state_count = xhat_pr_bias_index.second;
	auto ranges      = calc_range(receiver_pos_ecef, measurements.sv_position);

	std::vector<Vector3> los_ecef;
	std::transform(
	    ranges.cbegin(),
	    ranges.cend(),
	    std::back_inserter(los_ecef),
	    [](const RangeInfo &range) -> Vector3 { return range.range_vector / range.range_scalar; });

	// Calculate measurement function jacobian
	if (!clock_initialized) {
		clock_initialized = true;
		return generate_clock_bias_update(los_ecef, measurements, xhat_p->estimate);
	}

	auto H = generate_h_jacobian(los_ecef, state_count);
	return generate_standard_update(los_ecef, measurements, H);
}

void PseudorangeDopplerProcessorEcef::update_state_block_tracking(
    const aspn_xtensor::TypeTimestamp &time, const PseudorangeDopplerMeasurements &measurements) {

	tracked_gnss_observations.update(time, measurements.prns);

	if (tracked_gnss_observations.changed()) {
		// remove state blocks
		for (auto prn : tracked_gnss_observations.removed()) {
			string state_block_label = pr_bias_sb_label + std::to_string(prn);
			engine->remove_state_block(state_block_label);
		}

		// add and setup new state blocks
		for (auto prn : tracked_gnss_observations.added()) {
			string state_block_label = pr_bias_sb_label + std::to_string(prn);
			set_new_pr_bias_state_block(state_block_label);
		}
	}

	// rebuild the state block labels based on the measurement order
	state_block_labels.erase(state_block_labels.begin() + FIRST_PR_BIAS_LABEL_INDEX,
	                         state_block_labels.end());
	for (auto prn : measurements.prns)
		state_block_labels.push_back(pr_bias_sb_label + std::to_string(prn));

	// refresh the xhat bias index
	xhat_pr_bias_index.second = xhat_pr_bias_index.first + measurements.prns.size();
}

Matrix PseudorangeDopplerProcessorEcef::generate_h_jacobian(const std::vector<Vector3> &los,
                                                            Size state_count) const {
	auto meas_count = los.size();
	Matrix H        = zeros(meas_count * 2, state_count);
	for (decltype(meas_count) ii = 0; ii < meas_count; ++ii) {
		auto pr_index  = ii;
		auto prr_index = meas_count + ii;
		// Position States
		xt::view(H, pr_index, xt::range(pv_state_index, pv_state_index + 3)) = -los[ii];
		H(pr_index, clock_state_index)                                       = LIGHT_SPEED;
		H(pr_index, xhat_pr_bias_index.first + ii)                           = 1;
		// Velocity States
		xt::view(H, prr_index, xt::range(pv_state_index + 3, pv_state_index + 6)) = -los[ii];
		H(prr_index, clock_state_index + 1)                                       = LIGHT_SPEED;
	}
	return H;
}
std::shared_ptr<StandardMeasurementModel>
PseudorangeDopplerProcessorEcef::generate_clock_bias_update(
    const std::vector<Vector3> &los,
    const PseudorangeDopplerMeasurements &meas,
    const Vector &xhat) const {
	Vector3 xhat_pos = xt::view(xhat, xt::range(pv_state_index, pv_state_index + 3));
	Vector3 xhat_vel = xt::view(xhat, xt::range(pv_state_index + 3, pv_state_index + 6));
	Vector xhat_pr_bias =
	    xt::view(xhat, xt::range(xhat_pr_bias_index.first, xhat_pr_bias_index.second));
	auto rows             = num_rows(meas.sv_position);
	Vector psr_predicted  = zeros(rows);
	Vector psrr_predicted = zeros(rows);
	for (std::size_t row = 0; row < rows; ++row) {
		Vector estimated_range = xt::view(meas.sv_position, row, xt::all()) - xhat_pos;
		psr_predicted(row)     = navtk::norm(estimated_range) + xhat_pr_bias(row) +
		                     LIGHT_SPEED * xhat(clock_state_index);
		auto estimated_range_rate = xt::linalg::vdot(
		    Vector{xt::view(meas.sv_velocity, row, xt::all()) - xhat_vel}, los[row]);
		psrr_predicted(row) = estimated_range_rate - LIGHT_SPEED * xhat(clock_state_index + 1);
	}
	Vector psr_resid      = (meas.pr_corrected - psr_predicted) / LIGHT_SPEED;
	Vector psrr_resid     = -1 * (meas.pr_rate - psrr_predicted) / LIGHT_SPEED;
	Scalar psr_resid_avg  = xt::mean(psr_resid)[0];
	Scalar psrr_resid_avg = xt::mean(psrr_resid)[0];
	Vector z              = Vector{psr_resid_avg, psrr_resid_avg};
	Matrix off_mean       = to_matrix(psr_resid - psr_resid_avg);
	Matrix psr_cov        = dot(xt::transpose(off_mean), off_mean);
	psr_cov /= std::max(static_cast<unsigned long>(num_rows(meas.pr_corrected) - 1), 1ul);
	Matrix psrr_off_mean        = to_matrix(psrr_resid - psrr_resid_avg);
	Matrix psrr_cov             = dot(xt::transpose(psrr_off_mean), psrr_off_mean);
	Matrix R                    = zeros(2, 2);
	R(0, 0)                     = psr_cov(0, 0);
	R(1, 1)                     = psrr_cov(0, 0);
	Matrix H                    = zeros(2, num_rows(xhat));
	H(0, clock_state_index)     = 1.0;
	H(1, clock_state_index + 1) = 1.0;

	auto func = std::function<Vector(const Vector &)>(
	    [tmp = clock_state_index](const Vector &x) { return Vector{x(tmp), x(tmp + 1)}; });
	return make_shared<StandardMeasurementModel>(z, func, H, R);
}

std::shared_ptr<StandardMeasurementModel> PseudorangeDopplerProcessorEcef::generate_standard_update(
    const std::vector<Vector3> &los,
    const PseudorangeDopplerMeasurements &meas,
    const Matrix &H) const {
	Vector z     = xt::concatenate(xt::xtuple(meas.pr_corrected, meas.pr_rate));
	auto num_pr  = num_rows(meas.pr_corrected);
	auto num_prr = num_rows(meas.pr_rate);
	Matrix R     = zeros(num_pr + num_prr, num_pr + num_prr);
	xt::view(R, xt::range(0, num_pr), xt::range(0, num_pr)) = eye(num_pr) * pr_noise_covariance;
	xt::view(R, xt::range(num_pr, num_pr + num_prr), xt::range(num_pr, num_pr + num_prr)) =
	    eye(num_prr) * prr_noise_covariance;
	auto func = [&, los = los, meas = meas](const Vector &x) {
		return generate_h(los, x, meas.sv_position, meas.sv_velocity);
	};
	return make_shared<StandardMeasurementModel>(
	    z, std::function<Vector(const Vector &)>(func), H, R);
}

Vector PseudorangeDopplerProcessorEcef::generate_h(const std::vector<Vector3> &los,
                                                   const Vector &xhat,
                                                   const Matrix &sv_pos,
                                                   const Matrix &sv_vel) const {
	Vector3 xhat_pos = xt::view(xhat, xt::range(pv_state_index, pv_state_index + 3));
	Vector3 xhat_vel = xt::view(xhat, xt::range(pv_state_index + 3, pv_state_index + 6));
	Vector xhat_pr_bias =
	    xt::view(xhat, xt::range(xhat_pr_bias_index.first, xhat_pr_bias_index.second));
	auto rows        = num_rows(sv_pos);
	Vector predicted = zeros(rows * 2);
	for (std::size_t row = 0; row < rows; ++row) {
		auto pr_index        = row;
		auto prr_index       = rows + row;
		auto estimated_range = navtk::norm(Vector{xt::view(sv_pos, row, xt::all()) - xhat_pos});
		predicted(pr_index) =
		    estimated_range + xhat_pr_bias(pr_index) + LIGHT_SPEED * xhat(clock_state_index);
		auto estimated_range_rate =
		    xt::linalg::vdot(Vector{xt::view(sv_vel, row, xt::all()) - xhat_vel}, los[row]);
		predicted(prr_index) = estimated_range_rate - LIGHT_SPEED * xhat(clock_state_index + 1);
	}
	return predicted;
}

not_null<shared_ptr<MeasurementProcessor<>>> PseudorangeDopplerProcessorEcef::clone() {
	return make_shared<PseudorangeDopplerProcessorEcef>(*this);
}

PseudorangeDopplerMeasurements PseudorangeDopplerProcessorEcef::assemble_measurements(
    const vector<aspn_xtensor::TypeSatnavObs> &observations,
    double receiver_time,
    const Vector3 &receiver_pos_llh,
    int week_num) {

	if (!ephemeris_handler_initialized) {
		spdlog::warn(
		    "No ephemeris data has been added to the ephemeris handler. Unable to perform update.");
		return {};
	}

	// Pair observations with ephemerides
	auto o_and_e = assemble_obs_ephem(ephemeris_handler, receiver_time, week_num, observations);
	auto &prns   = o_and_e.prns;
	auto &observations_and_ephemerides = o_and_e.observations_and_ephemerides;
	int meas_count                     = observations_and_ephemerides.size();
	if (meas_count < 1) return {};

	// Calculate and fill data matrices for each measurement
	Vector tgds               = zeros(meas_count);
	Vector pseudoranges       = zeros(meas_count);
	Vector dopplers           = zeros(meas_count);
	Vector sv_clock_error     = zeros(meas_count);
	Vector sv_elevation       = zeros(meas_count);
	Matrix sv_position        = zeros(meas_count, 3);
	Matrix sv_velocity        = zeros(meas_count, 3);
	Vector3 solution_pos_ecef = navutils::llh_to_ecef(receiver_pos_llh);
	for (int ii = 0; ii < meas_count; ++ii) {
		auto &observation = observations_and_ephemerides[ii].first;
		auto &ephemeris   = observations_and_ephemerides[ii].second;

		// Fill tgds and pseudorange lists
		tgds(ii)         = ephemeris.get_t_gd();
		pseudoranges(ii) = observation.get_pseudorange();
		if (observation.get_pseudorange_rate_type() ==
		    ASPN_TYPE_SATNAV_OBS_PSEUDORANGE_RATE_TYPE_PSR_RATE_DOPPLER)
			dopplers(ii) = observation.get_pseudorange_rate();

		// Approximate transmit time and clock error
		auto xmit_time   = receiver_time - pseudoranges(ii) / LIGHT_SPEED;
		auto clock       = ephemeris.get_clock();
		auto delta_time  = gnssutils::correct_for_week_rollover(xmit_time - clock.get_t_oc());
		auto clock_error = clock.get_af_0() + clock.get_af_1() * delta_time +
		                   clock.get_af_2() * delta_time * delta_time;

		// Take SV clock error into account for transmit time calculation
		xmit_time -= clock_error;

		// Calculate satellite position matrix and sv_clock_err vector
		auto sv_data = gnssutils::calc_sv_clock_error(
		    aspn_xtensor::to_type_timestamp(xmit_time), ephemeris, solution_pos_ecef);
		xt::view(sv_position, ii, xt::all()) = sv_data.get_sv_pos();
		xt::view(sv_velocity, ii, xt::all()) = sv_data.get_sv_vel();
		sv_clock_error(ii)                   = sv_data.get_sv_clock_bias();

		// Calculate elevation angle for each satellite
		sv_elevation(ii) = gnssutils::calc_sv_elevation(
		    receiver_pos_llh(0), receiver_pos_llh(1), solution_pos_ecef, sv_data.get_sv_pos());
	}

	// Remove svs below elevation mask, if desired
	if (elevation_mask >= 0) {
		for (int index = num_rows(sv_elevation) - 1; index >= 0; index--) {
			if (sv_elevation(index) < elevation_mask) {
				prns.erase(prns.begin() + index);
				sv_elevation   = xt::view(sv_elevation, xt::drop(index));
				sv_position    = xt::view(sv_position, xt::drop(index), xt::all());
				sv_velocity    = xt::view(sv_velocity, xt::drop(index), xt::all());
				sv_clock_error = xt::view(sv_clock_error, xt::drop(index));
				tgds           = xt::view(tgds, xt::drop(index));
				pseudoranges   = xt::view(pseudoranges, xt::drop(index));
				dopplers       = xt::view(dopplers, xt::drop(index));
			}
		}
		meas_count = pseudoranges.size();
		if (meas_count < 1) {
			spdlog::warn(
			    "The elevation mask has eliminated every measurement. Unable to perform update.");
			return {};
		}
	}

	// Set variables base on frequency
	auto sv_clock_error_with_delay = sv_clock_error;
	auto sv_frequency              = 1.;
	switch (cp_to_use.frequency) {
	case gnssutils::GpsFrequencies::L1:
		sv_clock_error_with_delay -= tgds;
		sv_frequency = 1575.42e6;
		break;
	case gnssutils::GpsFrequencies::L2:
		sv_clock_error_with_delay -= tgds * L1_L2_RATIO_2;
		sv_frequency = 1227.60e6;
		break;
	default:
		// No tgd for dual frequency (L1/L2) users
		break;
	}

	// Generate pseudoranges corrected for sv clock error and group delay
	Vector corrected_prs = pseudoranges + sv_clock_error_with_delay * LIGHT_SPEED;

	// Perform tropospheric corrections if desired
	if (apply_tropo_model) {
		for (int index = 0; index < meas_count; ++index) {
			corrected_prs(index) += gnssutils::calc_tropo_corr(
			    receiver_pos_llh(2), sv_elevation(index), tropo_rel_humidity);
		}
	}

	// Convert dopplers to pseudorange rates
	Vector pr_rates = -1. * dopplers * LIGHT_SPEED / sv_frequency;

	return {corrected_prs, pr_rates, sv_position, sv_velocity, prns};
}

vector<RangeInfo> PseudorangeDopplerProcessorEcef::calc_range(const Vector3 &receiver_pos,
                                                              const Matrix &sv_pos) {

	// Subtract receiver_pos from each row of sv_pos and calculate the magnitude
	vector<RangeInfo> range_info_set;
	for (std::size_t row = 0; row < num_rows(sv_pos); ++row) {
		Vector range_vec = Vector3(xt::view(sv_pos, row, xt::all())) - receiver_pos;
		RangeInfo range  = {range_vec, navtk::norm(range_vec)};
		range_info_set.push_back(range);
	}

	return range_info_set;
}

void PseudorangeDopplerProcessorEcef::set_new_pr_bias_state_block(string state_block_label) {
	// add to state block
	auto pr_bias_block =
	    make_shared<FogmBlock>(state_block_label, pr_time_constant, pr_bias_sigma, 1);
	engine->add_state_block(pr_bias_block);
	engine->set_state_block_estimate(state_block_label, {0.0});
	Matrix sb_cov{{pr_bias_sigma * pr_bias_sigma}};
	engine->set_state_block_covariance(state_block_label, sb_cov);
}

void PseudorangeDopplerProcessorEcef::receive_aux_data(const AspnBaseVector &aux_data) {
	for (auto aux : aux_data) {
		auto ephemeris_data =
		    std::dynamic_pointer_cast<aspn_xtensor::MetadataGpsLnavEphemeris>(aux);
		if (ephemeris_data != nullptr) {
			ephemeris_handler.add_ephemeris(*ephemeris_data);
			ephemeris_handler_initialized = true;
		} else {
			auto rel_humidity_data = std::dynamic_pointer_cast<RelativeHumidityAux>(aux);
			if (rel_humidity_data != nullptr) {
				double aux_value = rel_humidity_data->tropo_rel_humidity;
				if (aux_value >= 0.0 && aux_value <= 1.0) {
					tropo_rel_humidity = aux_value;
				} else {
					spdlog::warn(
					    "Relative humidity value ({}) passed through aux data was rejected. Value "
					    "must be 0-1",
					    aux_value);
				}
			} else {
				MeasurementProcessor::receive_aux_data(aux_data);
			}
		}
	}
}

}  // namespace filtering
}  // namespace navtk
