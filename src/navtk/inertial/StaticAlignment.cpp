#include <navtk/inertial/StaticAlignment.hpp>

#include <algorithm>
#include <limits>
#include <memory>

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/inertial/quaternion_static_alignment.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::MeasurementImu;
using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::to_seconds;
using aspn_xtensor::TypeTimestamp;
using navtk::filtering::NavSolution;
using navtk::utils::to_position;

namespace navtk {
namespace inertial {

StaticAlignment::StaticAlignment(const filtering::ImuModel& model,
                                 const double align_time,
                                 const Matrix3& vel_cov)
    : AlignBase(true, false, model), align_time(align_time), vel_cov(vel_cov) {}

StaticAlignment::AlignmentStatus StaticAlignment::process(
    std::shared_ptr<aspn_xtensor::AspnBase> message) {

	if (alignment_status == AlignmentStatus::ALIGNED_GOOD) {
		spdlog::warn("StaticAlignment receiving measurements after aligned");
		return alignment_status;
	}

	auto posdata = std::dynamic_pointer_cast<MeasurementPosition>(message);

	if (posdata == nullptr) {
		auto pvadata = std::dynamic_pointer_cast<MeasurementPositionVelocityAttitude>(message);
		if (pvadata != nullptr) {
			posdata = std::make_shared<aspn_xtensor::MeasurementPosition>(to_position(*pvadata));
		}
	}

	if (posdata != nullptr) {
		// Non-full cov indicates incomplete position data, reject
		if (num_rows((Matrix)posdata->get_covariance()) == 3) {
			gps_buffer.push_back(*posdata);
			if (sufficient_data()) {
				calc_alignment(align_buffer.back().get_time_of_validity());
			}
		} else {
			log_or_throw<std::invalid_argument>(
			    "Position data passed to StaticAlignment appears to contain NANs, ignoring.");
		}
		return alignment_status;
	}


	auto imudata = std::dynamic_pointer_cast<MeasurementImu>(message);
	if (imudata != nullptr) {
		align_buffer.push_back(*imudata);
		if (sufficient_data()) {
			calc_alignment(imudata->get_time_of_validity());
		}
		return alignment_status;
	}
	spdlog::warn("Unused data type in alignment.");
	return alignment_status;
}

bool StaticAlignment::sufficient_data() {
	if (gps_buffer.empty() || align_buffer.empty()) return false;

	int64_t back_t  = align_buffer.back().get_aspn_c()->time_of_validity.elapsed_nsec;
	int64_t front_t = align_buffer[0].get_aspn_c()->time_of_validity.elapsed_nsec;
	return (back_t - front_t) * 1e-9 > align_time;
}

std::pair<Vector3, Matrix3> StaticAlignment::pos_stats() const {

	if (gps_buffer.empty()) {
		return {zeros(3), eye(3) * std::numeric_limits<double>::max()};
	}

	Matrix pos = zeros(3, gps_buffer.size());
	for (Size k = 0; k < gps_buffer.size(); k++) {
		pos(0, k) = gps_buffer[k].get_term1();
		pos(1, k) = gps_buffer[k].get_term2();
		pos(2, k) = gps_buffer[k].get_term3();
	}

	auto pos_mean = xt::mean(pos, {1});
	return {pos_mean, gps_buffer.back().get_covariance()};
}

std::pair<Vector3, Vector3> StaticAlignment::calc_avg() const {
	Vector3 dv_sum  = zeros(3);
	Vector3 dth_sum = zeros(3);

	for (const auto& imu : align_buffer) {
		dv_sum += imu.get_meas_accel();
		dth_sum += imu.get_meas_gyro();
	}

	Vector dv_avg  = dv_sum / align_buffer.size();
	Vector dth_avg = dth_sum / align_buffer.size();
	return {dv_avg, dth_avg};
}

double StaticAlignment::calc_average_delta_time() const {
	int64_t back_t  = align_buffer.back().get_aspn_c()->time_of_validity.elapsed_nsec;
	int64_t front_t = align_buffer.front().get_aspn_c()->time_of_validity.elapsed_nsec;
	return (back_t - front_t) * 1e-9 / (align_buffer.size() - 1);
}

void StaticAlignment::calc_alignment(const aspn_xtensor::TypeTimestamp& imu_time) {
	auto imu_avgs = calc_avg();

	Matrix3 cns = quaternion_static_alignment(imu_avgs.first, imu_avgs.second);
	Matrix3 csn = xt::transpose(cns);

	auto dt = calc_average_delta_time();

	auto stats = pos_stats();
	auto pos   = stats.first;

	Vector imu_as_vec                     = zeros(6);
	xt::view(imu_as_vec, xt::range(0, 3)) = imu_avgs.first;
	xt::view(imu_as_vec, xt::range(3, 6)) = imu_avgs.second;
	Vector imu_sigma                      = zeros(6);

	// Sigma has to be in same units as meas, so integrate
	xt::view(imu_sigma, xt::range(0, 3)) = model.accel_bias_initial_sigma * dt;
	xt::view(imu_sigma, xt::range(3, 6)) = model.gyro_bias_initial_sigma * dt;
	auto imu_cov                         = xt::diag(xt::pow(imu_sigma, 2.0));
	auto imu_as_ec                       = filtering::EstimateWithCovariance(imu_as_vec, imu_cov);

	std::function<Vector(const Vector&)> fx = [](const Vector& x) {
		Matrix3 cns =
		    quaternion_static_alignment(xt::view(x, xt::range(0, 3)), xt::view(x, xt::range(3, 6)));
		return navutils::dcm_to_rpy(cns);
	};
	auto fo  = filtering::first_order_approx_rpy(imu_as_ec, fx);
	tilt_cov = fo.covariance;

	computed_alignment = {true, NavSolution(pos, zeros(3), csn, imu_time)};
	alignment_status   = AlignmentStatus::ALIGNED_GOOD;
}

std::pair<bool, Matrix> StaticAlignment::get_computed_covariance(
    const CovarianceFormat format) const {

	auto cov                                        = zeros(9, 9);
	xt::view(cov, xt::range(0, 3), xt::range(0, 3)) = pos_stats().second;
	xt::view(cov, xt::range(3, 6), xt::range(3, 6)) = vel_cov;
	xt::view(cov, xt::range(6, 9), xt::range(6, 9)) = tilt_cov;

	auto model_terms = bias_stats_from_model(format);

	switch (format) {
	case CovarianceFormat::PINSON15NEDBLOCK: {
		auto out_cov                                          = zeros(15, 15);
		xt::view(out_cov, xt::range(0, 9), xt::range(0, 9))   = cov;
		xt::view(out_cov, xt::range(9, 15), xt::range(9, 15)) = model_terms;
		return {alignment_status == AlignmentStatus::ALIGNED_GOOD, out_cov};
	}
	case CovarianceFormat::PINSON21NEDBLOCK: {
		auto out_cov                                          = zeros(21, 21);
		xt::view(out_cov, xt::range(0, 9), xt::range(0, 9))   = cov;
		xt::view(out_cov, xt::range(9, 21), xt::range(9, 21)) = model_terms;
		return {alignment_status == AlignmentStatus::ALIGNED_GOOD, out_cov};
	}
	default:
		return {false, zeros(1, 1)};
	}
}

MotionNeeded StaticAlignment::motion_needed() const { return MotionNeeded::NO_MOTION; }

}  // namespace inertial
}  // namespace navtk
