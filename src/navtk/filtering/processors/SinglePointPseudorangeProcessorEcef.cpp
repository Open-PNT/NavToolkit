#include <navtk/filtering/processors/SinglePointPseudorangeProcessorEcef.hpp>

#include <stdexcept>

#include <spdlog/spdlog.h>
#include <xtensor/xview.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/gnssutils/assemble_prs.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>

namespace navtk {
namespace filtering {

using aspn_xtensor::MeasurementSatnav;
using std::make_shared;
using std::shared_ptr;
using std::string;
using std::vector;

SinglePointPseudorangeProcessorEcef::SinglePointPseudorangeProcessorEcef(
    string label,
    string pos_ecef_label,
    string clock_biases_label,
    gnssutils::PseudorangeType pr_to_use,
    double pr_noise_covariance,
    double pr_bias_covariance,
    double pr_time_constant,
    not_null<shared_ptr<StandardFusionEngineBase>> filter_in,
    bool apply_tropo_model,
    double tropo_rel_humidity,
    double elevation_mask,
    bool force_clock_initialization)
    : SinglePointPseudorangeProcessor(label,
                                      "",
                                      clock_biases_label,
                                      pr_to_use,
                                      pr_noise_covariance,
                                      pr_bias_covariance,
                                      pr_time_constant,
                                      filter_in,
                                      apply_tropo_model,
                                      tropo_rel_humidity,
                                      elevation_mask,
                                      force_clock_initialization) {
	state_block_labels = vector<string>{pos_ecef_label, clock_biases_label};
	clock_state_index  = 3;
	xhat_pr_bias_index = std::make_pair(5, 0);
}

shared_ptr<StandardMeasurementModel> SinglePointPseudorangeProcessorEcef::generate_model(
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
	    gnssutils::assemble_prs(*observation, pr_to_use);

	auto est_with_cov = gen_x_and_p_func(get_state_block_labels());
	if (est_with_cov == nullptr) {
		return nullptr;
	}
	Vector xhat = est_with_cov->estimate;

	// Pull out position coordinates, assumed in ECEF and corrected for lever arms (ie antenna
	// phase center position).
	Vector3 receiver_pos_ecef =
	    xt::view(xhat, xt::range(XHAT_POS_INDICES.first, XHAT_POS_INDICES.second));
	Vector3 receiver_pos_llh = navutils::ecef_to_llh(receiver_pos_ecef);

	// Correct pseudoranges and calculate satellite position, azimuth, elevation
	auto clock_time = observation->get_receiver_clock_time();
	CorrectedGnssPseudorangeMeasurement corrected_meas =
	    correct_measurements(meas_data,
	                         clock_time.get_seconds_of_week(),
	                         receiver_pos_llh,
	                         clock_time.get_week_number());

	if (corrected_meas.prns.empty()) {
		spdlog::warn("Unable to produce corrected measurements: aborting update.");
		return nullptr;
	}

	update_state_block_tracking(observation->get_time_of_validity(), corrected_meas);

	// Get a fresh xhat to include any reconfigured measurement bias states
	auto xhat_p = gen_x_and_p_func(get_state_block_labels());
	if (xhat_p == nullptr) {
		return nullptr;
	}

	// Calculate measurement function jacobian
	if (!clock_initialized) {
		clock_initialized = true;
		return generate_clock_bias_update(corrected_meas, receiver_pos_ecef, xhat_p->estimate);
	}

	auto state_count = xhat_pr_bias_index.second;
	auto ranges      = calc_range(receiver_pos_ecef, corrected_meas.sv_position);

	std::vector<Vector3> los_ecef;
	std::transform(
	    ranges.cbegin(),
	    ranges.cend(),
	    std::back_inserter(los_ecef),
	    [](const RangeInfo& range) -> Vector3 { return range.range_vector / range.range_scalar; });

	auto H = generate_h_jacobian(los_ecef, state_count);
	return generate_standard_update(corrected_meas, receiver_pos_ecef, H);
}

not_null<shared_ptr<MeasurementProcessor<>>> SinglePointPseudorangeProcessorEcef::clone() {
	return make_shared<SinglePointPseudorangeProcessorEcef>(*this);
}

}  // namespace filtering
}  // namespace navtk
