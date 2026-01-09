#include <navtk/filtering/containers/TimestampedDataSeries.hpp>

#include <navtk/inertial/ImuErrors.hpp>

namespace navtk {
namespace filtering {

template <>
aspn_xtensor::TypeTimestamp get_time_value(
    utils::RingBuffer<std::shared_ptr<inertial::ImuErrors>>::const_iterator it) {
	return (*it)->time_validity;
}

}  // namespace filtering
}  // namespace navtk
