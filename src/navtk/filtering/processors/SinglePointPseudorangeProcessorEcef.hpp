#pragma once

#include <memory>
#include <string>
#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/RangeInfo.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/fusion/StandardFusionEngineBase.hpp>
#include <navtk/filtering/processors/SinglePointPseudorangeProcessor.hpp>
#include <navtk/gnssutils/PseudorangeType.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Measurement processor that accepts pseudorange measurements (as MeasurementSatnav or
 * filtering::PairedPva wrapping a MeasurementSatnav) and updates ECEF position, receiver clock
 * bias, and pseudorange bias states.
 */
class SinglePointPseudorangeProcessorEcef : public SinglePointPseudorangeProcessor {

public:
	/**
	 * Main constructor for the processor.
	 * @param label The identifier for this instance of this processor.
	 * @param pos_ecef_label The label to StateBlock that contains whole-state (corrected)
	 * lever arm-shifted (i.e. at sensor antenna) ECEF position coordinates.
	 * @param clock_biases_label The label to an instance of ClockBiasesStateBlock.
	 * @param pr_to_use An enum specifying which signal to use.
	 * @param pr_noise_covariance Determines the diagonals of the measurement covariance matrix such
	 * that `R = I * pr_noise_covariance`, in meters^2.
	 * @param pr_bias_covariance Determines the covariance for the pseudorange bias First-order
	 * Gauss-Markov state model states, in meters^2.
	 * @param pr_time_constant The pseudorange bias state time constant for the First-order Gauss
	 * Markov model used for all PRNs, in seconds.
	 * @param filter_in Non-null shared pointer to current filter using
	 * SinglePointPseudorangeProcessor.
	 * @param apply_tropo_model When true, applies error in a pseudorange due to tropospheric delay.
	 * @param tropo_rel_humidity (optional) Relative humidity value to use for the tropospheric
	 * delay calculation.
	 * @param elevation_mask (optional) Any satellites below this elevation (radians) will not be
	 * used.
	 * @param force_clock_initialization When set `true`, uses the first set of measurements to only
	 * update the clock bias state. Subsequent updates use the full model. Useful in cases when
	 * there is high uncertainty in the receiver clock bias, but initial PVA is roughly known.
	 */
	SinglePointPseudorangeProcessorEcef(
	    std::string label,
	    std::string pos_ecef_label,
	    std::string clock_biases_label,
	    gnssutils::PseudorangeType pr_to_use,
	    double pr_noise_covariance,
	    double pr_bias_covariance,
	    double pr_time_constant,
	    not_null<std::shared_ptr<StandardFusionEngineBase>> filter_in,
	    bool apply_tropo_model          = true,
	    double tropo_rel_humidity       = 0.5,
	    double elevation_mask           = -1,
	    bool force_clock_initialization = true);

	std::shared_ptr<StandardMeasurementModel> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase> measurement,
	    GenXhatPFunction gen_x_and_p_func) override;

	not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override;
};

}  // namespace filtering
}  // namespace navtk
