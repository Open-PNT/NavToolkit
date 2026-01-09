#include <navtk/inertial/StaticWahbaAlignment.hpp>

#include <spdlog/spdlog.h>

#include <navtk/filtering/utils.hpp>
#include <navtk/inertial/quaternion_static_alignment.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/wgs84.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::filtering::NavSolution;

namespace navtk {
namespace inertial {

StaticWahbaAlignment::StaticWahbaAlignment(const filtering::ImuModel& model,
                                           const double align_time,
                                           const Matrix3& vel_cov)
    : StaticAlignment(model, align_time, vel_cov) {}

// Basically same as fun in navtk/testing, without rotation into sensor frame
Vector3 perfect_dth(const Vector3& llh, double dt) {
	Vector3 wnie = Vector3{cos(llh[0]), 0, -sin(llh[0])} * navutils::ROTATION_RATE;
	return wnie * dt;
}

Vector3 perfect_dv(const Vector3& llh, double dt) {
	auto g        = navutils::calculate_gravity_schwartz(llh[2], llh[0]);
	auto dth      = perfect_dth(llh, dt);
	auto inv_corr = inverse(eye(3) + 0.5 * navutils::skew(dth));
	return dot(inv_corr, Vector3{0.0, 0.0, -g[2]} * dt);
}

void StaticWahbaAlignment::calc_alignment(const aspn_xtensor::TypeTimestamp& imu_time) {
	Vector3 dv_sum  = zeros(3);
	Vector3 dth_sum = zeros(3);
	for (const auto& imu : align_buffer) {
		dv_sum += imu.get_meas_accel();
		dth_sum += imu.get_meas_gyro();
	}

	auto imu_avgs = calc_avg();

	int64_t back_t  = align_buffer.back().get_aspn_c()->time_of_validity.elapsed_nsec;
	int64_t front_t = align_buffer.front().get_aspn_c()->time_of_validity.elapsed_nsec;
	auto dt         = (back_t - front_t) * 1e-9 / (align_buffer.size() - 1);

	auto stats = pos_stats();
	auto pos   = stats.first;

	// Calculate assumed 'perfect' static dv measurement. Check imu measurement type to determine
	// integration term
	auto int_dt =
	    align_buffer.front().get_imu_type() == ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED ? dt : 1.0;
	Vector3 perfect        = perfect_dv(pos, int_dt);
	Vector3 perfect_dtheta = perfect_dth(pos, int_dt);

	auto to_solve  = xt::linalg::outer(imu_avgs.first, perfect);
	auto to_solve2 = xt::linalg::outer(imu_avgs.second, perfect_dtheta);
	auto to_solve3 = to_solve + to_solve2;

	auto csn3 = solve_wahba_svd(to_solve3);

	computed_alignment = {true, NavSolution(pos, zeros(3), csn3, imu_time)};
	alignment_status   = AlignmentStatus::ALIGNED_GOOD;

	std::function<Vector(const Vector&)> fx = [pos = pos, dt = dt](const Vector& x) {
		Vector3 perfect        = perfect_dv(pos, dt);
		Vector3 perfect_dtheta = perfect_dth(pos, dt);

		auto to_solve  = xt::linalg::outer(xt::view(x, xt::range(0, 3)), perfect);
		auto to_solve2 = xt::linalg::outer(xt::view(x, xt::range(3, 6)), perfect_dtheta);
		auto to_solve3 = to_solve + to_solve2;
		auto csn3      = solve_wahba_svd(to_solve3);
		// Not ideal, but must be a vector
		return navutils::dcm_to_rpy(xt::transpose(csn3));
	};

	Vector biases_sig                     = zeros(6);
	xt::view(biases_sig, xt::range(0, 3)) = model.accel_bias_initial_sigma * dt;
	xt::view(biases_sig, xt::range(3, 6)) = model.gyro_bias_initial_sigma * dt;


	// vstack apparently doesn't work w/ fixed sized inputs
	// xt::vstack(xt::xtuple(imu.delta_v, imu.delta_theta))
	Vector est                     = zeros(6);
	xt::view(est, xt::range(0, 3)) = imu_avgs.first;
	xt::view(est, xt::range(3, 6)) = imu_avgs.second;
	auto ec  = filtering::EstimateWithCovariance(est, xt::diag(xt::pow(biases_sig, 2.0)));
	auto fo  = filtering::first_order_approx_rpy(ec, fx);
	tilt_cov = fo.covariance;
}

std::pair<bool, Matrix> StaticWahbaAlignment::get_computed_covariance(
    const CovarianceFormat format) const {

	auto cov = StaticAlignment::get_computed_covariance(format);
	xt::view(cov.second, xt::range(6, 9), xt::range(6, 9)) = tilt_cov;
	return cov;
}

}  // namespace inertial
}  // namespace navtk
