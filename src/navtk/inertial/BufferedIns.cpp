#include <navtk/inertial/BufferedIns.hpp>

#include <navtk/utils/sortable_vectors.hpp>

using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::TypeTimestamp;
using std::make_shared;
using std::shared_ptr;

namespace navtk {
namespace inertial {

BufferedIns::BufferedIns(std::shared_ptr<MeasurementPositionVelocityAttitude> pva,
                         double expected_dt,
                         double buffer_length)
    : BufferedPva(pva, expected_dt, buffer_length) {}

BufferedIns::BufferedIns(const MeasurementPositionVelocityAttitude& pva,
                         double expected_dt,
                         double buffer_length)
    : BufferedPva(pva, expected_dt, buffer_length) {}

void BufferedIns::add_pva(not_null<shared_ptr<MeasurementPositionVelocityAttitude>> pva) {
	int64_t pva_t = pva->get_aspn_c()->time_of_validity.elapsed_nsec;
	if (pva_buf.size() > 0 && pva_t <= nsec_time_span().second) {
		log_or_throw<std::invalid_argument>(
		    "Got stale PVA tagged at {}; latest record at {}.", pva_t, nsec_time_span().second);
		return;
	}
	pva_buf.insert(pva);
}

void BufferedIns::add_pva(const MeasurementPositionVelocityAttitude& pva) {
	add_pva(make_shared<MeasurementPositionVelocityAttitude>(pva));
}

void BufferedIns::add_data(not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) {
	auto pva = std::dynamic_pointer_cast<MeasurementPositionVelocityAttitude>(data);
	if (pva == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "BufferedIns received data type other than MeasurementPositionVelocityAttitude.");
		return;
	}

	add_pva(pva);
}

}  // namespace inertial
}  // namespace navtk
