#include <navtk/inertial/ManualHeadingAlignment.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/wgs84.hpp>

namespace navtk {
namespace inertial {

ManualHeadingAlignment::ManualHeadingAlignment(const double heading,
                                               const double heading_sigma,
                                               const filtering::ImuModel& model,
                                               const double align_time,
                                               const Matrix3& vel_cov)
    : StaticAlignment(model, align_time, vel_cov), heading(heading), heading_sigma(heading_sigma) {}

void ManualHeadingAlignment::calc_align() {
	auto alignment = StaticAlignment::get_computed_alignment();
	if (alignment.first) {

		auto avgs    = calc_avg();
		auto dv_avg  = avgs.first;
		auto dth_avg = avgs.second;

		auto roll  = atan(dv_avg[1] / dv_avg[2]);
		auto sr    = sin(roll);
		auto cr    = cos(roll);
		auto pitch = atan(-dv_avg[0] / (sr * dv_avg[1] + cr * dv_avg[2]));

		alignment.second.rot_mat =
		    xt::transpose(navutils::rpy_to_dcm(Vector3{roll, pitch, heading}));
		computed_alignment =
		    std::pair<bool, filtering::NavSolution>{alignment.first, alignment.second};
		alignment_status = AlignmentStatus::ALIGNED_GOOD;
	}
}

ManualHeadingAlignment::AlignmentStatus ManualHeadingAlignment::process(
    std::shared_ptr<aspn_xtensor::AspnBase> message) {
	// StaticAlignment parent class will work with either integrated or sampled, but the
	// get_imu_errors() function below assumes integrated
	auto imudata = std::dynamic_pointer_cast<aspn_xtensor::MeasurementImu>(message);
	if (imudata != nullptr) {
		if (imudata->get_imu_type() != ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED) {
			log_or_throw<std::invalid_argument>(
			    "Only ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED currently supported in "
			    "ManualHeadingAlignment.");
			return alignment_status;
		}
	}
	StaticAlignment::process(message);
	calc_align();
	return alignment_status;
}

std::pair<bool, Matrix> ManualHeadingAlignment::get_computed_covariance(
    const CovarianceFormat format) const {

	auto cov = StaticAlignment::get_computed_covariance(format);
	if (cov.first) {
		// Remove correlation between heading and other states since not given
		xt::view(cov.second, 8, xt::all()) = zeros(num_cols(cov.second));
		xt::view(cov.second, xt::all(), 8) = zeros(num_rows(cov.second));

		// Uncertainty in the calculated attitude is a result of accel biases
		double g          = navutils::calculate_gravity_schwartz(gps_buffer.back().get_term3(),
                                                        gps_buffer.back().get_term1())[2];
		auto csn          = computed_alignment.second.rot_mat;
		Matrix3 cov_accel = xt::view(cov.second, xt::range(9, 12), xt::range(9, 12));

		Matrix3 g_rot{{0, 1.0 / g, 0}, {-1.0 / g, 0, 0}, {0, 0, 0}};

		// Turns out tilt variance and cross terms only vary by a single matrix multiply;
		// this is the common bit
		auto partial_mult = dot(xt::transpose(csn), dot(cov_accel, dot(csn, xt::transpose(g_rot))));

		// Assign NE tilt terms
		xt::view(cov.second, xt::range(6, 9), xt::range(6, 9)) = dot(g_rot, partial_mult);
		// Dump in heading sigma
		cov.second(8, 8) = std::pow(heading_sigma, 2);

		// Set cross
		auto cross_terms                                        = dot(csn, partial_mult);
		xt::view(cov.second, xt::range(9, 12), xt::range(6, 9)) = cross_terms;
		xt::view(cov.second, xt::range(6, 9), xt::range(9, 12)) = xt::transpose(cross_terms);
	}
	return cov;
}

std::pair<bool, ImuErrors> ManualHeadingAlignment::get_imu_errors() const {
	ImuErrors imu_bs{};
	if (computed_alignment.first) {
		auto avgs    = calc_avg();
		auto dv_avg  = avgs.first;
		auto dth_avg = avgs.second;
		auto g       = navutils::calculate_gravity_schwartz(gps_buffer.back().get_term3(),
                                                      gps_buffer.back().get_term1());
		auto dt      = calc_average_delta_time();

		Vector3 wie_n{cos(gps_buffer.back().get_term1()) * navutils::ROTATION_RATE,
		              0,
		              -sin(gps_buffer.back().get_term1()) * navutils::ROTATION_RATE};
		auto wie_ins = dot(computed_alignment.second.rot_mat, wie_n);
		imu_bs.gyro_biases += (dth_avg / dt - wie_ins);
		auto dv_rot         = dot(xt::transpose(computed_alignment.second.rot_mat), dv_avg);
		auto est_biases_ned = Vector3{0, 0, g[2] + dv_rot[2] / dt};
		imu_bs.accel_biases = dot(computed_alignment.second.rot_mat, est_biases_ned);
	}
	return {alignment_status == AlignmentStatus::ALIGNED_GOOD, imu_bs};
}

}  // namespace inertial
}  // namespace navtk
