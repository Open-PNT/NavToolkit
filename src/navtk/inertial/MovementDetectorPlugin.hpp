#pragma once

#include <navtk/aspn.hpp>
#include <navtk/inertial/MovementStatus.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace inertial {

/**
 * Base class for movement detection algorithms
 */
class MovementDetectorPlugin {
public:
	/**
	 * Take the provided data and use to determine whether movement is occurring.
	 *
	 * @param data Movement algorithm data
	 *
	 * @returns An enum with the algorithm's determined movement status.
	 */
	virtual MovementStatus process(not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) = 0;

	/**
	 * Default destructor.
	 */
	virtual ~MovementDetectorPlugin() = default;

	/**
	 * Return latest movement status.
	 *
	 * @returns An enum with the algorithm's determined movement status.
	 */
	virtual MovementStatus get_status() { return last_status; }

	/**
	 * Time of latest measurement.
	 *
	 * @return The time of the last valid measurement received by process(). Only valid if
	 * return of get_status() is not MovementStatus::INVALID.
	 */
	virtual aspn_xtensor::TypeTimestamp get_time() = 0;

protected:
	/**
	 * Last known movement status of the movement algorithm.
	 */
	MovementStatus last_status = MovementStatus::INVALID;
};
}  // namespace inertial
}  // namespace navtk
