#pragma once

#include <ostream>

namespace navtk {
namespace utils {

/**
 * Indicates the result of a ValidationContext::validate call.
 */
enum class ValidationResult {
	NOT_CHECKED,  //!< Error check was skipped for performance reasons
	GOOD,         //!< Error check was performed and no errors were found.
	BAD           //!< Errors were found
};

/**
 * Print the name-qualified human-readable name of the navtk::ValidationResult, for example,
 * `"ValidationResult::GOOD"`
 *
 * @param os Stream output
 * @param validation_result Value to print to the stream.
 * @return The output stream `os`.
 */
std::ostream& operator<<(std::ostream& os, ValidationResult validation_result);

}  // namespace utils
}  // namespace navtk
