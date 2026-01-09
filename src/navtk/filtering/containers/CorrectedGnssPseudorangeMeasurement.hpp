#pragma once

#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {
/**
 * Stores measurements and SV (Space Vehicle) info.
 */
struct CorrectedGnssPseudorangeMeasurement {
	/** The corrected pseudorange measurements (meters). */
	Vector pr_corrected;
	/** The ECEF coordinates (meters) of each satellite */
	Matrix sv_position;
	/** The PRN of each satellite */
	std::vector<uint16_t> prns;
};
}  // namespace filtering
}  // namespace navtk
