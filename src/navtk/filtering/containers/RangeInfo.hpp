#pragma once

#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {
/**
 * Stores the vector and scalar range between two points.
 */
struct RangeInfo {
	/** The ECEF frame vector (meters) between two points. */
	Vector3 range_vector;
	/** The scalar distance (meters) between two points. */
	double range_scalar;
};
}  // namespace filtering
}  // namespace navtk
