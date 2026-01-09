#include <navtk/get_time.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/inertial/ImuErrors.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>

namespace navtk {


std::pair<bool, aspn_xtensor::TypeTimestamp> get_time(
    std::shared_ptr<aspn_xtensor::AspnBase> meas) {
	if (aspn_xtensor::is_core_message(meas)) {
		auto time = aspn_xtensor::get_time(meas);
		return {true, time};
	} else if (auto gvd = std::dynamic_pointer_cast<filtering::GaussianVectorData>(meas)) {
		return {true, gvd->get_time_of_validity()};
	} else if (auto paired_pva = std::dynamic_pointer_cast<filtering::PairedPva>(meas)) {
		return {true, paired_pva->ref_pva.time};
	} else if (auto pva = std::dynamic_pointer_cast<inertial::InertialPosVelAtt>(meas)) {
		return {true, pva->time_validity};
	} else if (auto imu_errors = std::dynamic_pointer_cast<inertial::ImuErrors>(meas)) {
		return {true, imu_errors->time_validity};
	} else {
		return {false, aspn_xtensor::TypeTimestamp((int64_t)0)};
	}
	// Can add more navtk ASPN extension messages here if needed
}

navtk::Vector to_seconds(const std::vector<aspn_xtensor::TypeTimestamp>& times) {
	auto n                     = times.size();
	navtk::Vector times_double = navtk::zeros(n);
	for (size_t ii = 0; ii < n; ++ii) times_double(ii) = aspn_xtensor::to_seconds(times[ii]);
	return times_double;
}

}  // namespace navtk
