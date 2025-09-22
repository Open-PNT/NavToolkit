#include <navtk/inertial/BufferedPva.hpp>

#include <limits>
#include <memory>

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/inertial/inertial_functions.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>
#include <navtk/utils/interpolation.hpp>

using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using std::make_shared;
using std::shared_ptr;

namespace navtk {
namespace inertial {

BufferedPva::BufferedPva(std::shared_ptr<MeasurementPositionVelocityAttitude> pva,
                         double expected_dt,
                         double buffer_length)
    : pva_buf(buffer_length / expected_dt + 2) {
	if (pva != nullptr) pva_buf.insert(pva);
}

BufferedPva::BufferedPva(const MeasurementPositionVelocityAttitude& pva,
                         double expected_dt,
                         double buffer_length)
    : pva_buf(buffer_length / expected_dt + 2) {
	pva_buf.insert(make_shared<MeasurementPositionVelocityAttitude>(pva));
}

shared_ptr<MeasurementPositionVelocityAttitude> BufferedPva::calc_pva(
    const aspn_xtensor::TypeTimestamp& time) const {
	if (in_range(time)) {
		return calc_pva_no_check(time);
	}
	return nullptr;
}

not_null<shared_ptr<MeasurementPositionVelocityAttitude>> BufferedPva::calc_pva() const {
	return pva_buf.back();
}

std::pair<int64_t, int64_t> BufferedPva::nsec_time_span() const {
	if (pva_buf.empty()) {
		spdlog::warn("Solution buffer empty.");
		return {std::numeric_limits<int>::lowest(), std::numeric_limits<int>::lowest()};
	}
	if (pva_buf.full()) {
		return {(*(pva_buf.cbegin() + 1))->get_aspn_c()->time_of_validity.elapsed_nsec,
		        pva_buf.back()->get_aspn_c()->time_of_validity.elapsed_nsec};
	}
	return {pva_buf.front()->get_aspn_c()->time_of_validity.elapsed_nsec,
	        pva_buf.back()->get_aspn_c()->time_of_validity.elapsed_nsec};
}

std::pair<aspn_xtensor::TypeTimestamp, aspn_xtensor::TypeTimestamp> BufferedPva::time_span() const {
	if (pva_buf.empty()) {
		spdlog::warn("Solution buffer empty.");
		return {to_type_timestamp(std::numeric_limits<double>::lowest()),
		        to_type_timestamp(std::numeric_limits<double>::lowest())};
	}
	if (pva_buf.full()) {
		return {(*(pva_buf.cbegin() + 1))->get_time_of_validity(),
		        pva_buf.back()->get_time_of_validity()};
	}
	return {pva_buf.front()->get_time_of_validity(), pva_buf.back()->get_time_of_validity()};
}

bool BufferedPva::in_range(const aspn_xtensor::TypeTimestamp& t) const {
	auto span = nsec_time_span();
	return t.get_elapsed_nsec() >= span.first && t.get_elapsed_nsec() <= span.second;
}

shared_ptr<aspn_xtensor::MeasurementImu> BufferedPva::calc_force_and_rate(
    const aspn_xtensor::TypeTimestamp& time) const {
	if (in_range(time)) {
		auto nearest = pva_buf.get_nearest_neighbors(time);
		auto before  = nearest.first;
		auto after   = nearest.second;

		// Check if get_nearest_neighbors returned the same result twice (happens when time matches
		// a PVA timestamp in the buffer). If so, then proceed by using the two adjacent PVAs. If
		// there is only one adjacent PVA, then interpolate to get the second.
		if (before == after) {
			// If no valid PVA was returned, then abort.
			if (!safe_deref(pva_buf, before)) {
				spdlog::warn(
				    "There are no PVAs in the buffer around the time {}. Could not "
				    "calculate force and rate",
				    time);
				return nullptr;
			}

			if (before == pva_buf.cbegin()) {
				after += 1;
			} else {
				before -= 1;
			}

			// If there is no surrounding PVA, then abort.
			if (!safe_deref(pva_buf, before) || !safe_deref(pva_buf, after)) {
				spdlog::warn(
				    "There are not two PVAs in the buffer around the time {}. Could not "
				    "calculate force and rate",
				    time);
				return nullptr;
			}
		}
		return calc_force_and_rate((*before)->get_time_of_validity(),
		                           (*after)->get_time_of_validity());
	} else {
		auto span = nsec_time_span();
		spdlog::warn(
		    "aspn_xtensor::TypeTimestamp {} is out of range. The first available time is {} and "
		    "the last available time "
		    "is {}. Could not calculate force and rate",
		    time.get_elapsed_nsec(),
		    span.first,
		    span.second);
		return nullptr;
	}
}

shared_ptr<aspn_xtensor::MeasurementImu> BufferedPva::calc_force_and_rate(
    const aspn_xtensor::TypeTimestamp& time1, const aspn_xtensor::TypeTimestamp& time2) const {

	if (time1 == time2) return calc_force_and_rate(time1);

	// calc_pva does an in_range check which will fail if time1 refers to the first element in the
	// in the pva_buffer (which is perfectly valid and can happen when user asks for forces at
	// time_span().first), so we use a separate check here to allow that first pva element in
	int64_t nsec1 = time1.get_elapsed_nsec();
	int64_t nsec2 = time2.get_elapsed_nsec();

	auto pva1 = (nsec1 >= pva_buf.front()->get_aspn_c()->time_of_validity.elapsed_nsec &&
	             nsec1 <= pva_buf.back()->get_aspn_c()->time_of_validity.elapsed_nsec)
	                ? calc_pva_no_check(time1)
	                : nullptr;
	if (pva1 == nullptr) {
		spdlog::warn("Could not calculate PVA at time1 {}", time1);
		return nullptr;
	}

	auto pva2 = calc_pva(time2);
	if (pva2 == nullptr) {
		spdlog::warn("Could not calculate PVA at time2 {}", time2);
		return nullptr;
	}

	auto force = calc_force_ned(*pva1, *pva2);
	auto rate  = calc_rot_rate(*pva1, *pva2);

	aspn_xtensor::TypeTimestamp mid_time = time1 + ((nsec2 - nsec1) * 1e-9) / 2;

	return make_shared<aspn_xtensor::MeasurementImu>(navtk::utils::to_imu(mid_time, force, rate));
}

shared_ptr<MeasurementPositionVelocityAttitude> BufferedPva::calc_pva_no_check(
    const aspn_xtensor::TypeTimestamp& time) const {

	auto possible = pva_buf.get_nearest_neighbors(time);
	if (safe_deref(pva_buf, possible.first) && safe_deref(pva_buf, possible.second)) {
		return utils::linear_interp_pva(*(possible.first), *(possible.second), time);
	}
	return nullptr;
}

}  // namespace inertial
}  // namespace navtk
