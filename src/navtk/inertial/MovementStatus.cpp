#include <navtk/inertial/MovementStatus.hpp>

namespace navtk {
namespace inertial {
std::ostream& operator<<(std::ostream& os, const MovementStatus status) {
	switch (status) {
	case MovementStatus::INVALID:
		os << "INVALID";
		break;
	case MovementStatus::NOT_MOVING:
		os << "NOT_MOVING";
		break;
	case MovementStatus::POSSIBLY_MOVING:
		os << "POSSIBLY_MOVING";
		break;
	case MovementStatus::MOVING:
		os << "MOVING";
		break;
	default:
		os << "NOT MAPPED TO STREAM OP";
	}
	return os;
}
}  // namespace inertial
}  // namespace navtk
