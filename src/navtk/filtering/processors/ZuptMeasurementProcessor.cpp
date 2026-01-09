
#include <navtk/filtering/processors/ZuptMeasurementProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/inertial/MovementDetector.hpp>
#include <navtk/inertial/MovementDetectorImu.hpp>
#include <navtk/inertial/MovementDetectorPos.hpp>
#include <navtk/inertial/MovementStatus.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

using aspn_xtensor::MeasurementImu;
using aspn_xtensor::MeasurementPosition;

ZuptMeasurementProcessor::ZuptMeasurementProcessor(std::string label,
                                                   const std::string &state_block_label,
                                                   const Matrix3 &cov,
                                                   const Size window,
                                                   const double calib_time,
                                                   const double speed_cutoff,
                                                   const double zero_corr_distance,
                                                   const double mov_detect_imu_weight,
                                                   const double mov_detect_imu_stale,
                                                   const double mov_detect_pos_weight,
                                                   const double mov_detect_pos_stale,
                                                   int state_count)
    : MeasurementProcessor(std::move(label), std::vector<std::string>(1, state_block_label)),
      cov(cov),
      state_count(state_count) {
	detector.add_plugin("zupt_imu",
	                    std::make_shared<navtk::inertial::MovementDetectorImu>(window, calib_time),
	                    mov_detect_imu_weight,
	                    mov_detect_imu_stale);
	detector.add_plugin(
	    "zupt_pos",
	    std::make_shared<navtk::inertial::MovementDetectorPos>(speed_cutoff, zero_corr_distance),
	    mov_detect_pos_weight,
	    mov_detect_pos_stale);
}

std::shared_ptr<StandardMeasurementModel> ZuptMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	if (nullptr != std::dynamic_pointer_cast<MeasurementImu>(measurement)) {
		detector.process({"zupt_imu"}, measurement);
	} else if (nullptr != std::dynamic_pointer_cast<MeasurementPosition>(measurement)) {
		detector.process(std::vector<std::string>{"zupt_pos"}, measurement);
	} else {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (Imu or MeasurementPosition). "
		    "Unable to perform update.");
		return nullptr;
	}

	auto move_status = detector.get_status();

	if (move_status != navtk::inertial::MovementStatus::NOT_MOVING) {
		return nullptr;
	}

	if (state_count == 0) {
		auto xhat_p = gen_x_and_p_func(get_state_block_labels());
		if (xhat_p == nullptr) {
			return nullptr;
		}

		state_count = num_rows(xhat_p->estimate);
	}
	Matrix H   = zeros(3, state_count);
	H.at(0, 3) = 1.0;
	H.at(1, 4) = 1.0;
	H.at(2, 5) = 1.0;
	Vector z   = zeros(3);

	return std::make_shared<StandardMeasurementModel>(z, H, cov);
}

not_null<std::shared_ptr<MeasurementProcessor<>>> ZuptMeasurementProcessor::clone() {
	return std::make_shared<ZuptMeasurementProcessor>(*this);
}

}  // namespace filtering
}  // namespace navtk
