#include <navtk/utils/ValidationResult.hpp>

namespace navtk {
namespace utils {

std::ostream& operator<<(std::ostream& os, ValidationResult validation_result) {
	switch (validation_result) {
	case ValidationResult::NOT_CHECKED:
		return os << "ValidationResult::NOT_CHECKED";
	case ValidationResult::GOOD:
		return os << "ValidationResult::GOOD";
	case ValidationResult::BAD:
		return os << "ValidationResult::BAD";
	default:
		return os << "<INVALID ValidationResult(" << static_cast<int>(validation_result) << ")>";
	}
}

}  // namespace utils
}  // namespace navtk
