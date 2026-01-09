#pragma once

#include <ostream>

namespace navtk {
namespace inertial {

/**
 * Various states of motion reported by MovementDetector and related.
 */
enum class MovementStatus {
	/** Needs more data, or results ambiguous */
	INVALID,
	/** Almost certainly stationary */
	NOT_MOVING,
	/** Too close to call */
	POSSIBLY_MOVING,
	/** Almost certainly moving */
	MOVING
};

/**
 * Define the `ostream` operator for the inertial::MovementStatus enum class; prints string
 * representation of enum.
 * @param os The `std::ostream` reference.
 * @param status The inertial::MovementStatus object to print.
 * @return The `std::ostream` reference.
 */
std::ostream& operator<<(std::ostream& os, const MovementStatus status);

}  // namespace inertial
}  // namespace navtk
