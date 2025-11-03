#pragma once

#include <ostream>

#include <spdlog/fmt/fmt.h>

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

#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND
// Define a custom formatter so fmt (via spdlog) can format MovementStatus.
template <>
struct fmt::formatter<navtk::inertial::MovementStatus> {
	constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

	template <typename FormatContext>
	constexpr auto format(const navtk::inertial::MovementStatus& input, FormatContext& ctx) const {
		switch (input) {
		case navtk::inertial::MovementStatus::INVALID:
			return fmt::format_to(ctx.out(), "INVALID");
		case navtk::inertial::MovementStatus::NOT_MOVING:
			return fmt::format_to(ctx.out(), "NOT_MOVING");
		case navtk::inertial::MovementStatus::POSSIBLY_MOVING:
			return fmt::format_to(ctx.out(), "POSSIBLY_MOVING");
		case navtk::inertial::MovementStatus::MOVING:
			return fmt::format_to(ctx.out(), "MOVING");
		}
		return fmt::format_to(ctx.out(), "Unknown enum value");
	}
};
#endif
