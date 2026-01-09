#pragma once

#include <navtk/tensors.hpp>

#include <vector>

namespace navtk {
namespace filtering {
/**
 * Stores measurements and SV (Space Vehicle) info.
 */
struct PseudorangeDopplerMeasurements {
	/** The corrected pseudorange measurements (meters). */
	Vector pr_corrected;
	/** The converted Doppler measurements (meters/second). */
	Vector pr_rate;
	/** The ECEF coordinates (meters) of each satellite (Nx3), N being the number of satellites */
	Matrix sv_position;
	/** The ECEF velocities (meters/second) of each satellite (Nx3). */
	Matrix sv_velocity;
	/** The PRN of each satellite */
	std::vector<uint16_t> prns;
};
}  // namespace filtering
}  // namespace navtk
