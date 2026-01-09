#include <navtk/inertial/DynData.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/wgs84.hpp>

using aspn_xtensor::TypeTimestamp;

namespace {
/**
 * Given a vector of IMU measurements \p imus,
 * return a vector of the corresponding time of validity deltas with respect to \p t0.
 */
std::vector<double> imu_dts(const aspn_xtensor::TypeTimestamp& t0,
                            const std::vector<aspn_xtensor::MeasurementImu>& imus) {
	std::vector<double> dts;

	if (imus.cbegin() == imus.cend()) return dts;

	int64_t t = imus.cbegin()->get_aspn_c()->time_of_validity.elapsed_nsec;
	dts.push_back((t - t0.get_elapsed_nsec()) * 1e-9);

	for (auto k = imus.cbegin() + 1; k < imus.cend(); ++k) {
		auto current_time = k->get_aspn_c()->time_of_validity.elapsed_nsec;
		dts.push_back((current_time - t) * 1e-9);
		t = current_time;
	}
	return dts;
}

std::pair<navtk::Vector3, navtk::Vector3> split_and_average_forces(
    const std::vector<aspn_xtensor::MeasurementImu>& align_buffer, const std::vector<double>& dts) {
	// Calculate integrated specific force from t(k-1) to t(k-1) + dt/2, and from t(k-1) +
	// dt/2 to t(k).
	auto nh               = std::ceil(align_buffer.size() / 2.0);
	auto f_meas_int       = navtk::zeros(3);
	navtk::Size num_f_int = 0;
	auto f_meas_km1h      = navtk::zeros(3);

	for (navtk::Size k = 0; k < align_buffer.size(); ++k) {
		f_meas_int += (align_buffer[k].get_meas_accel()) / dts[k];
		num_f_int += 1;
		if (k == nh - 1) {
			f_meas_km1h = f_meas_int / num_f_int;
			f_meas_int  = navtk::zeros(3);
			num_f_int   = 0;
		}
	}
	return {f_meas_km1h, f_meas_int / num_f_int};
}
}  // namespace

namespace navtk {
namespace inertial {

DynData::DynData(const aspn_xtensor::MeasurementPosition& origin)
    : origin(origin),
      current(origin),
      prior(origin),
      prior2(origin),
      t_current_prior(origin.get_time_of_validity()),
      t_prior_prior2(origin.get_time_of_validity()) {
	lat_factor = navutils::delta_lat_to_north(1.0, origin.get_term1(), origin.get_term3());
	lon_factor = navutils::delta_lon_to_east(1.0, origin.get_term1(), origin.get_term3());
}

bool DynData::enough_data() { return update_counts > 1; }

std::pair<bool, Vector3> DynData::get_force_from_imu() {
	if (!prior_prior2_imu_valid || !current_prior_imu_valid) return {false, zeros(3)};

	return {true, avg_force_prior_imu};
}

Vector3 DynData::get_force_from_pos() { return avg_force_prior_pos; }

std::pair<aspn_xtensor::TypeTimestamp, Vector3> DynData::get_vel_mid() {
	return {prior.get_time_of_validity(), vel_prior};
}
const aspn_xtensor::MeasurementPosition& DynData::get_origin() const { return origin; }
std::pair<double, double> DynData::get_lat_lon_factors() { return {lat_factor, lon_factor}; }

std::vector<aspn_xtensor::MeasurementPosition> DynData::get_positions() {
	return std::vector<aspn_xtensor::MeasurementPosition>{prior2, prior, current};
}

const aspn_xtensor::MeasurementPosition& DynData::get_position(
    const RecentPositionsEnum& recency) const {
	switch (recency) {
	case RecentPositionsEnum::MOST_RECENT:
		return current;
	case RecentPositionsEnum::SECOND_MOST_RECENT:
		return prior;
	case RecentPositionsEnum::THIRD_MOST_RECENT:
		return prior2;
	default:
		throw std::invalid_argument("Cant get recent position, unhandled enum value!");
	}
}

void DynData::update(const aspn_xtensor::MeasurementPosition& new_pos,
                     const std::vector<aspn_xtensor::MeasurementImu>& align_buffer) {
	prior2  = prior;
	prior   = current;
	current = new_pos;

	prior_prior2_imu_valid = current_prior_imu_valid;
	// Former implementation required > 2, but 2 is possibly actual min
	// Another way to check might be to compare times, looking for large gaps
	current_prior_imu_valid = align_buffer.size() > 2;

	auto delta_current = delta_pos(current);
	auto delta_prior   = delta_pos(prior);
	vel_prior_prior2   = vel_current_prior;

	int64_t current_t = current.get_aspn_c()->time_of_validity.elapsed_nsec;
	int64_t prior_t   = prior.get_aspn_c()->time_of_validity.elapsed_nsec;
	vel_current_prior = (delta_current - delta_prior) / ((current_t - prior_t) * 1e-9);
	t_prior_prior2    = t_current_prior;
	// This method of calculating the mean is needed to prevent a loss of precision. Dividing a
	// delta-aspn_xtensor::TypeTimestamp by 2.0 is safer than adding two times and dividing by 2.0
	// (the more traditional mean calculation) because the division implicitly casts
	// aspn_xtensor::TypeTimestamp to a double.
	t_current_prior = TypeTimestamp(prior_t + ((current_t - prior_t) / 2));
	vel_prior       = (vel_current_prior + vel_prior_prior2) / 2.0;

	split_forces_prior_prior2 = split_forces_current_prior;

	if (current_prior_imu_valid) {
		// Check imu to look for dropped measurements that might invalidate the result
		auto approx_expected_dt =
		    (align_buffer.back().get_aspn_c()->time_of_validity.elapsed_nsec - prior_t) * 1e-9 /
		    (align_buffer.size());
		auto dts = imu_dts(prior.get_time_of_validity(), align_buffer);
		auto avg_imu_dt =
		    std::accumulate(
		        dts.cbegin(), dts.cend(), 0.0, [](double a, double b) { return a + b; }) /
		    align_buffer.size();
		auto ratio = (avg_imu_dt > approx_expected_dt) ? avg_imu_dt / approx_expected_dt
		                                               : approx_expected_dt / avg_imu_dt;
		current_prior_imu_valid = (ratio - 1) < 0.05;

		if (current_prior_imu_valid) {
			split_forces_current_prior = split_and_average_forces(align_buffer, dts);
			avg_force_prior_imu =
			    (split_forces_prior_prior2.second + split_forces_current_prior.first) / 2.0;
			avg_force_prior_pos = force_from_velocity(
			    current, vel_prior_prior2, vel_current_prior, t_prior_prior2, t_current_prior);
		}
	}

	if (!enough_data()) {
		update_counts++;
	}
}

Vector3 DynData::delta_pos(const aspn_xtensor::MeasurementPosition& p) const {
	auto d_north = (p.get_term1() - origin.get_term1()) * lat_factor;
	auto d_east  = (p.get_term2() - origin.get_term2()) * lon_factor;
	auto d_down  = -(p.get_term3() - origin.get_term3());
	return {d_north, d_east, d_down};
}

Vector3 DynData::force_from_velocity(const aspn_xtensor::MeasurementPosition& pos,
                                     const Vector3& v0,
                                     const Vector3& v1,
                                     const aspn_xtensor::TypeTimestamp& t_v0,
                                     const aspn_xtensor::TypeTimestamp& t_v1) {

	int64_t tv0 = t_v0.get_elapsed_nsec();
	int64_t tv1 = t_v1.get_elapsed_nsec();
	// Calculate acceleration and specific force at previous time in navigation frame
	// Old version used most recent pos but time of vel
	auto a_km1 = (v1 - v0) / ((tv1 - tv0) * 1e-9);
	// Apply earth rotation to gravity and add. This method of calculating the mean is needed to
	// prevent a loss of precision. Dividing a delta-aspn_xtensor::TypeTimestamp by 2.0 is safer
	// than adding two times and dividing by 2.0 (the more traditional mean calculation) because the
	// division implicitly casts aspn_xtensor::TypeTimestamp to a double.
	auto rot_comp =
	    (origin.get_aspn_c()->time_of_validity.elapsed_nsec - (tv0 + ((tv1 - tv0) / 2))) * 1e-9 *
	    navutils::ROTATION_RATE * cos(pos.get_term1());
	auto rot_comp_dcm = xt::transpose(navutils::rpy_to_dcm({rot_comp, 0.0, 0.0}));
	auto g            = navutils::calculate_gravity_schwartz(pos.get_term3(), pos.get_term1());
	return a_km1 + dot(rot_comp_dcm, -g);
}
}  // namespace inertial
}  // namespace navtk
