#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/tensors.hpp>

namespace navtk {

/**
 * Extract timestamp from an AspnBase object.
 *
 * @param msg Message containing a timestamp.
 *
 * @return A pair where the first element is a flag indicating if \p msg has a timestamp, and if
 * `true`, the second element is the timestamp. If the flag is `false`, the second element will have
 * a timestamp of 0.
 */
std::pair<bool, aspn_xtensor::TypeTimestamp> get_time(std::shared_ptr<aspn_xtensor::AspnBase> msg);

/**
 * Converts a std::vector of TypeTimestamp to a navtk::Vector.
 *
 * @param times Times to be converted.
 * @return Converted times.
 */
navtk::Vector to_seconds(const std::vector<aspn_xtensor::TypeTimestamp>& times);

}  // namespace navtk
