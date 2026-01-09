#include <navtk/inertial/MovementDetectorImu.hpp>

#include <algorithm>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/factory.hpp>
#include <navtk/linear_algebra.hpp>

namespace {

std::vector<double> to_norms(const std::vector<navtk::Vector3>& v, double sub) {
	std::vector<double> norms;
	std::transform(v.cbegin(), v.cend(), std::back_inserter(norms), [sub](const navtk::Vector& k) {
		return navtk::norm(k) - sub;
	});
	return norms;
}

double mean_norm(const std::vector<double>& norms) {
	return std::accumulate(
	           norms.cbegin(), norms.cend(), 0.0, [](double a, const double b) { return a + b; }) /
	       (norms.size());
}

std::pair<double, double> mean_sig_norm(std::vector<navtk::Vector3>& v, double sub) {
	auto norms = to_norms(v, sub);
	auto mn    = mean_norm(norms);
	auto sm_sq = std::accumulate(norms.cbegin(), norms.cend(), 0.0, [mn](double a, const double b) {
		return a + std::pow(b - mn, 2.0);
	});
	auto sig   = std::sqrt(sm_sq / (v.size()));
	return {mn, sig};
}
}  // namespace

namespace navtk {
namespace inertial {

MovementDetectorImu::MovementDetectorImu(const Size window, const double calib_time)
    : MovementDetectorPlugin(), window(window), calib_time(calib_time) {
	if (window < 1) {
		log_or_throw<std::invalid_argument>(
		    "window must be at least size 1. Attempting to set to default.");
		this->window = 10;
	}
	if (calib_time <= 0) {
		log_or_throw<std::invalid_argument>(
		    "calib_time must be greater than 0. Attempting to set to default.");
		this->calib_time = 30.0;
	}
}

MovementStatus MovementDetectorImu::process(
    not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) {

	auto imu = std::dynamic_pointer_cast<aspn_xtensor::MeasurementImu>(data);

	if (imu != nullptr) {
		dth_extract.push_back(imu->get_meas_gyro());
		time_extract.push_back(imu->get_time_of_validity());

		// Make sure we've filled our bin and have the initial bias estimates from stationary data
		if (ready_to_test && time_extract.size() >= window) {
			auto norms   = to_norms(dth_extract, initial_dth_norm);
			auto mn      = mean_norm(norms);
			auto dth_res = mn <= initial_dth_sig;
			last_status  = dth_res ? MovementStatus::NOT_MOVING : MovementStatus::MOVING;
			last_time    = time_extract.back();
			clear_buffers();
		} else if (!ready_to_test &&
		           (time_extract.back().get_elapsed_nsec() -
		            time_extract.front().get_elapsed_nsec()) >= (int64_t)(calib_time * 1e9)) {
			auto mn_sig      = mean_sig_norm(dth_extract, initial_dth_norm);
			initial_dth_norm = mn_sig.first;
			initial_dth_sig  = mn_sig.second;
			clear_buffers();
			ready_to_test = true;
		}
	}
	return last_status;
}

void MovementDetectorImu::clear_buffers() {
	dth_extract.clear();
	time_extract.clear();
}

aspn_xtensor::TypeTimestamp MovementDetectorImu::get_time() { return last_time; }

}  // namespace inertial
}  // namespace navtk
