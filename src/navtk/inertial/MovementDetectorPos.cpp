#include <navtk/inertial/MovementDetectorPos.hpp>

#include <algorithm>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::utils::extract_pos;

namespace {
navtk::Vector3 calc_delta_pos_ned(
    navtk::not_null<std::shared_ptr<aspn_xtensor::MeasurementPosition>> old,
    navtk::not_null<std::shared_ptr<aspn_xtensor::MeasurementPosition>> cur) {

	auto dn = navtk::navutils::delta_lat_to_north(1.0, old->get_term1(), old->get_term3());
	auto de = navtk::navutils::delta_lon_to_east(1.0, old->get_term1(), old->get_term3());
	navtk::Matrix3 cvt{{dn, 0, 0}, {0, de, 0}, {0, 0, -1}};
	return navtk::dot(cvt, extract_pos(*cur) - extract_pos(*old));
}

double calc_speed(navtk::not_null<std::shared_ptr<aspn_xtensor::MeasurementPosition>> old,
                  navtk::not_null<std::shared_ptr<aspn_xtensor::MeasurementPosition>> cur) {
	int64_t old_time = old->get_aspn_c()->time_of_validity.elapsed_nsec;
	int64_t cur_time = cur->get_aspn_c()->time_of_validity.elapsed_nsec;
	auto dt          = (cur_time - old_time) * 1e-9;
	return navtk::norm((navtk::Vector)calc_delta_pos_ned(old, cur)) / dt;
}

}  // namespace

namespace navtk {
namespace inertial {

MovementDetectorPos::MovementDetectorPos(const double speed_cutoff, const double zero_corr_distance)
    : MovementDetectorPlugin(),
      speed_cutoff(speed_cutoff),
      zero_corr_distance(zero_corr_distance) {}

Vector MovementDetectorPos::calc_sig(
    navtk::not_null<std::shared_ptr<aspn_xtensor::MeasurementPosition>> pos,
    double est_spd,
    double dt) {
	auto corr_scale = 1.0 - std::min(est_spd * dt / zero_corr_distance, 1.0);
	return xt::sqrt(xt::diagonal(last->get_covariance()) + xt::diagonal(pos->get_covariance()) -
	                2 * corr_scale * xt::diagonal(navtk::chol(last->get_covariance())) *
	                    xt::diagonal(navtk::chol(pos->get_covariance())));
}

MovementStatus MovementDetectorPos::process(
    not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) {
	auto pos = std::dynamic_pointer_cast<aspn_xtensor::MeasurementPosition>(data);

	if (pos != nullptr) {
		if (!ready_to_test) {
			last          = pos;
			ready_to_test = true;
		} else {
			int64_t last_time = last->get_aspn_c()->time_of_validity.elapsed_nsec;
			int64_t this_time = pos->get_aspn_c()->time_of_validity.elapsed_nsec;
			if (this_time <= last_time) {
				log_or_throw<std::invalid_argument>(
				    "MovementDetectorPos got out of order position data.");
				return last_status;
			}

			Vector delta_pos = xt::abs(calc_delta_pos_ned(last, pos));
			auto est_spd     = calc_speed(last, pos);
			auto dt          = (this_time - last_time) * 1e-9;

			if (est_spd < 0) {
				last_status = MovementStatus::INVALID;
			} else if (xt::all(delta_pos / dt < speed_cutoff)) {
				last_status = MovementStatus::NOT_MOVING;
			} else if (xt::any(delta_pos > calc_sig(pos, est_spd, dt) * 3.0)) {
				last_status = MovementStatus::MOVING;
			} else {
				last_status = MovementStatus::POSSIBLY_MOVING;
			}
			last = pos;
		}
	}
	return last_status;
}

aspn_xtensor::TypeTimestamp MovementDetectorPos::get_time() {
	if (last != nullptr) {
		return last->get_time_of_validity();
	}
	return TypeTimestamp((int64_t)0);
}

}  // namespace inertial
}  // namespace navtk
