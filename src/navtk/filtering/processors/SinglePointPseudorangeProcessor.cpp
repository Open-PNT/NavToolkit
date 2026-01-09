#include <navtk/filtering/processors/SinglePointPseudorangeProcessor.hpp>

#include <stdexcept>
#include <string>

#include <spdlog/spdlog.h>
#include <xtensor/xview.hpp>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/containers/RelativeHumidityAux.hpp>
#include <navtk/filtering/fusion/StandardFusionEngineBase.hpp>
#include <navtk/filtering/stateblocks/FogmBlock.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
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
using aspn_xtensor::TypeSatnavObs;
using aspn_xtensor::TypeSatnavTime;
using navtk::gnssutils::assemble_obs_ephem;
using navtk::gnssutils::GpsEphemerisHandler;
using navtk::gnssutils::L1_L2_RATIO_2;
using navtk::gnssutils::LIGHT_SPEED;
using navtk::gnssutils::PseudorangeType;
using std::make_shared;
using std::shared_ptr;
using std::string;
using std::vector;

constexpr std::pair<Size, Size> SinglePointPseudorangeProcessor::XHAT_POS_INDICES;

SinglePointPseudorangeProcessor::SinglePointPseudorangeProcessor(
    string label,
    string pinson15_label,
    string clock_biases_label,
    PseudorangeType pr_to_use,
    double pr_noise_covariance,
    double pr_bias_covariance,
    double pr_time_constant,
    not_null<shared_ptr<StandardFusionEngineBase>> engine_in,
    bool apply_tropo_model,
    double tropo_rel_humidity,
    double elevation_mask,
    bool force_clock_initialization)
    : MeasurementProcessor(label, vector<string>{pinson15_label, clock_biases_label}),
      pr_to_use(pr_to_use),
      pr_noise_covariance(pr_noise_covariance),
      pr_bias_sigma(std::sqrt(pr_bias_covariance)),
      pr_time_constant(pr_time_constant),
      tropo_rel_humidity(tropo_rel_humidity),
      elevation_mask(elevation_mask),
      apply_tropo_model(apply_tropo_model),
      engine(engine_in),
      clock_initialized(!force_clock_initialization) {}

shared_ptr<StandardMeasurementModel> SinglePointPseudorangeProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	auto data = std::dynamic_pointer_cast<PairedPva>(measurement);
	if (data == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (PairedPva). Unable to perform update.");
		return nullptr;
	}
	auto observation = std::dynamic_pointer_cast<MeasurementSatnav>(data->meas_data);
	if (observation == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "PairedPva measurement data is not of correct type (MeasurementSatnav). Unable to "
		    "perform update.");
		return nullptr;
	}

	auto pinson_errors_ptr = gen_x_and_p_func({get_state_block_labels()[0]});
	if (pinson_errors_ptr == nullptr) {
		return nullptr;
	}

	auto pinson_errors     = pinson_errors_ptr->estimate;
	auto receiver_solution = apply_error_states<Pinson15NedBlock>(data->ref_pva, pinson_errors);

	// Generate the vector of pseudoranges, PRNs
	vector<TypeSatnavObs> meas_data = gnssutils::assemble_prs(*observation, pr_to_use);

	// Correct pseudoranges and calculate satellite position, azimuth, elevation
	auto clock_time = observation->get_receiver_clock_time();
	CorrectedGnssPseudorangeMeasurement corrected_meas =
	    correct_measurements(meas_data,
	                         clock_time.get_seconds_of_week(),
	                         receiver_solution.pos,
	                         clock_time.get_week_number());

	if (corrected_meas.prns.empty()) {
		spdlog::warn("Unable to produce corrected measurements: aborting update.");
		return nullptr;
	}

	update_state_block_tracking(observation->get_time_of_validity(), corrected_meas);

	// Get a fresh xhat to include any reconfigured measurement bias states
	auto states_ptr = gen_x_and_p_func(get_state_block_labels());
	if (states_ptr == nullptr) {
		return nullptr;
	}

	Vector3 receiver_pos_ecef = navutils::llh_to_ecef(receiver_solution.pos);

	// Calculate measurement function jacobian
	if (!clock_initialized) {
		clock_initialized = true;
		return generate_clock_bias_update(corrected_meas, receiver_pos_ecef, states_ptr->estimate);
	}

	auto state_count = xhat_pr_bias_index.second;
	auto ranges      = calc_range(receiver_pos_ecef, corrected_meas.sv_position);
	auto los_ned     = calculate_los(ranges, receiver_pos_ecef);
	auto H           = generate_h_jacobian(los_ned, state_count);
	return generate_standard_update(corrected_meas, receiver_pos_ecef, H);
}

void SinglePointPseudorangeProcessor::update_state_block_tracking(
    const aspn_xtensor::TypeTimestamp &time,
    const CorrectedGnssPseudorangeMeasurement &corrected_meas) {

	tracked_gnss_observations.update(time, corrected_meas.prns);

	if (tracked_gnss_observations.changed()) {
		// Adjust the filter state blocks
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

	// rebuild the state_block_labels in measurement order
	state_block_labels.erase(state_block_labels.begin() + FIRST_PR_BIAS_LABEL_INDEX,
	                         state_block_labels.end());
	for (auto prn : corrected_meas.prns) {
		state_block_labels.push_back(pr_bias_sb_label + std::to_string(prn));
	}

	// refresh the xhat bias index
	xhat_pr_bias_index.second = xhat_pr_bias_index.first + corrected_meas.prns.size();
}

Matrix SinglePointPseudorangeProcessor::generate_h_jacobian(const std::vector<Vector3> &los,
                                                            Size state_count) const {
	auto meas_count = los.size();
	Matrix H        = zeros(meas_count, state_count);
	for (decltype(meas_count) ii = 0; ii < meas_count; ++ii) {
		xt::view(H, ii, xt::range(XHAT_POS_INDICES.first, XHAT_POS_INDICES.second)) = -los[ii];
		xt::view(H, ii, xhat_pr_bias_index.first + ii)                              = 1;
	}
	view(H, xt::all(), clock_state_index) = ones(meas_count) * LIGHT_SPEED;
	return H;
}

std::shared_ptr<StandardMeasurementModel>
SinglePointPseudorangeProcessor::generate_clock_bias_update(
    const CorrectedGnssPseudorangeMeasurement &meas,
    const Vector3 &ecef_pos,
    const Vector &xhat) const {

	Vector resid = (meas.pr_corrected - generate_h(xhat, meas.sv_position, ecef_pos)) / LIGHT_SPEED;
	Scalar resid_avg = xt::mean(resid)[0];
	Matrix off_mean  = to_matrix(resid - resid_avg);
	Matrix cov       = dot(xt::transpose(off_mean), off_mean);
	cov /= std::max(static_cast<unsigned long>(num_rows(meas.pr_corrected) - 1), 1ul);
	Matrix empty_h             = zeros(1, num_rows(xhat));
	empty_h(clock_state_index) = 1.0;

	auto func = std::function<Vector(const Vector &)>(
	    [tmp = clock_state_index](const Vector &x) { return Vector{x(tmp)}; });
	return make_shared<StandardMeasurementModel>(Vector{resid_avg}, func, empty_h, cov);
}

std::shared_ptr<StandardMeasurementModel> SinglePointPseudorangeProcessor::generate_standard_update(
    const CorrectedGnssPseudorangeMeasurement &meas,
    const Vector3 &ecef_pos,
    const Matrix &H) const {
	Matrix R  = eye(num_rows(meas.pr_corrected)) * pr_noise_covariance;
	auto func = [&, ecef_pos = ecef_pos, sv_pos = meas.sv_position](const Vector &x) {
		return generate_h(x, sv_pos, ecef_pos);
	};
	return make_shared<StandardMeasurementModel>(
	    meas.pr_corrected, std::function<Vector(const Vector &)>(func), H, R);
}

Vector SinglePointPseudorangeProcessor::generate_h(const Vector &xhat,
                                                   const Matrix &sv_pos,
                                                   const Vector3 &ecef_pos) const {
	Vector xhat_pr_bias =
	    xt::view(xhat, xt::range(xhat_pr_bias_index.first, xhat_pr_bias_index.second));

	// Calculate predicted pseudorange measurement using the satellite position and the
	// inertial position.
	auto num_meas_to_generate = num_rows(sv_pos);
	Vector r_mag              = zeros(num_meas_to_generate);
	for (std::size_t meas_num = 0; meas_num < num_meas_to_generate; ++meas_num) {
		Vector meas_vec = xt::view(sv_pos, meas_num, xt::all()) - ecef_pos;
		r_mag(meas_num) = navtk::norm(meas_vec) + xhat_pr_bias(meas_num);
	}
	return r_mag + LIGHT_SPEED * xhat(clock_state_index);
}

not_null<shared_ptr<MeasurementProcessor<>>> SinglePointPseudorangeProcessor::clone() {
	return make_shared<SinglePointPseudorangeProcessor>(*this);
}

CorrectedGnssPseudorangeMeasurement SinglePointPseudorangeProcessor::correct_measurements(
    const vector<TypeSatnavObs> &observations,
    double receiver_time,
    const Vector3 &receiver_pos_llh,
    int week_num) {

	if (!ephemeris_handler_initialized) {
		spdlog::warn(
		    "No ephemeris data has been added to the ephemeris handler. Unable to perform update.");
		return {};
	}

	auto o_and_e = assemble_obs_ephem(ephemeris_handler, receiver_time, week_num, observations);
	auto &prns   = o_and_e.prns;
	auto &observations_and_ephemerides = o_and_e.observations_and_ephemerides;
	int meas_count                     = observations_and_ephemerides.size();
	if (meas_count < 1) return {};

	// Calculate and fill data matrices for each measurement
	Vector tgds               = zeros(meas_count);
	Vector pseudoranges       = zeros(meas_count);
	Vector sv_clock_error     = zeros(meas_count);
	Vector sv_elevation       = zeros(meas_count);
	Matrix sv_position        = zeros(meas_count, 3);
	Vector3 solution_pos_ecef = navutils::llh_to_ecef(receiver_pos_llh);
	for (int ii = 0; ii < meas_count; ++ii) {
		auto &observation = observations_and_ephemerides[ii].first;
		auto &ephemeris   = observations_and_ephemerides[ii].second;

		// Fill tgds and pseudorange lists
		tgds(ii)         = ephemeris.get_t_gd();
		pseudoranges(ii) = observation.get_pseudorange();

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
		    aspn_xtensor::to_type_timestamp(xmit_time), ephemeris, solution_pos_ecef, false);

		xt::view(sv_position, ii, xt::all()) = sv_data.get_sv_pos();
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
				sv_clock_error = xt::view(sv_clock_error, xt::drop(index));
				tgds           = xt::view(tgds, xt::drop(index));
				pseudoranges   = xt::view(pseudoranges, xt::drop(index));
			}
		}
		meas_count = pseudoranges.size();
		if (meas_count < 1) {
			spdlog::warn(
			    "The elevation mask has eliminated every measurement. Unable to perform update.");
			return {};
		}
	}

	// Generate pseudoranges corrected for sv clock error and group delay
	Vector corrected_prs;
	switch (pr_to_use.frequency) {
	case gnssutils::GpsFrequencies::L1:
		corrected_prs = pseudoranges + (sv_clock_error - tgds) * LIGHT_SPEED;
		break;
	case gnssutils::GpsFrequencies::L2:
		corrected_prs = pseudoranges + (sv_clock_error - tgds * L1_L2_RATIO_2) * LIGHT_SPEED;
		break;
	default:
		// No tgd for dual frequency (L1/L2) users
		corrected_prs = pseudoranges + sv_clock_error * LIGHT_SPEED;
		break;
	}

	// Perform tropospheric corrections if desired
	if (apply_tropo_model) {
		for (int index = 0; index < meas_count; ++index) {
			corrected_prs(index) += gnssutils::calc_tropo_corr(
			    receiver_pos_llh(2), sv_elevation(index), tropo_rel_humidity);
		}
	}

	return {corrected_prs, sv_position, prns};
}

vector<RangeInfo> SinglePointPseudorangeProcessor::calc_range(const Vector3 &receiver_pos,
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

void SinglePointPseudorangeProcessor::set_new_pr_bias_state_block(string state_block_label) {
	// add to state block
	auto pr_bias_block =
	    make_shared<FogmBlock>(state_block_label, pr_time_constant, pr_bias_sigma, 1);
	engine->add_state_block(pr_bias_block);
	engine->set_state_block_estimate(state_block_label, {0.0});
	Matrix sb_cov{{pr_bias_sigma * pr_bias_sigma}};
	engine->set_state_block_covariance(state_block_label, sb_cov);
}

void SinglePointPseudorangeProcessor::receive_aux_data(const AspnBaseVector &aux_data) {
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

std::vector<Vector3> SinglePointPseudorangeProcessor::calculate_los(
    const std::vector<RangeInfo> &ranges, const Vector3 &ecef_pos) const {
	auto C_ecef_to_ned = xt::transpose(navutils::ecef_to_cen(ecef_pos));
	std::vector<Vector3> los_ned;
	std::transform(ranges.cbegin(),
	               ranges.cend(),
	               std::back_inserter(los_ned),
	               [&C_ecef_to_ned](const RangeInfo &range) -> Vector3 {
		               return dot(C_ecef_to_ned, range.range_vector / range.range_scalar);
	               });
	return los_ned;
}

}  // namespace filtering
}  // namespace navtk
