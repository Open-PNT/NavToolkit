#pragma once

#include <navtk/tensors.hpp>

namespace navtk {
namespace navutils {

/**
 * Tags for gravity models to use.
 */
enum class GravModels {
	/** Use the gravity model implemented in calculate_gravity_titterton() */
	TITTERTON,
	/** Use the gravity model implemented in calculate_gravity_schwartz() */
	SCHWARTZ,
	/** Use the gravity model implemented in one of the `calculate_gravity_savage`
	 * functions (such as calculate_gravity_savage_ned()).
	 */
	SAVAGE
};

/**
 * NED frame gravity from Titterton provided gravity model (eq 3.8-3.91 in 2nd ed.).
 * Only the down term is provided (no deflections included).
 *
 * @param alt Ellipsoidal altitude (meters)
 * @param lat Latitude (radians)
 * @param R0 Mean radius of curvature, given just after eq. 3.86 in T+W 2nd ed., meters.
 *
 * @return Gravity at the given latitude and altitude.
 */
Vector3 calculate_gravity_titterton(double alt, double lat, double R0);

/**
 * NED frame gravity from Schwartz gravity model.
 * Only the down term is provided (no deflections included).
 *
 * @param alt Ellipsoidal altitude (meters)
 * @param lat Latitude (radians)
 *
 * @return Gravity at the given latitude and altitude.
 */
Vector3 calculate_gravity_schwartz(double alt, double lat);

/**
 * Gravity Vector from the 'Savage' gravity model as given in Section 5.4
 * of Strapdown Analytics, 2nd ed., N frame version.
 *
 * @see `calculate_gravity_schwartz()`
 * @see `calculate_gravity_savage_ned()`
 * @see `lat_lon_wander_to_C_n_to_e()`
 * @see [Coordinate Frames](../tutorial/coordinate_frames.html) for coordinate frame definitions.
 *
 * @param C_n_to_e DCM that rotates from the N frame
 * to the E frame \f$\textbf{C}_\text{N}^\text{E}\f$ (provides knowledge of horizontal position)
 * @param h Ellipsoidal altitude (meters)
 *
 * @return Gravity at the specified location expressed in the N frame
 */
Vector3 calculate_gravity_savage_n(const Matrix& C_n_to_e, double h);

/**
 * Gravity Vector from the 'Savage' gravity model as given in Section 5.4
 * of Strapdown Analytics, 2nd ed., NED frame version.
 *
 * @see `calculate_gravity_schwartz()`
 * @see `calculate_gravity_savage_n()`
 * @see `lat_lon_wander_to_C_n_to_e()`
 * @see [Coordinate Frames](../tutorial/coordinate_frames.html) for coordinate frame definitions.
 *
 * @param C_n_to_e DCM that rotates from the N frame
 * to the E frame \f$\textbf{C}_\text{N}^\text{E}\f$ (provides knowledge of horizontal position)
 * @param h Ellipsoidal altitude (meters)
 *
 * @return Gravity at the specified location expressed in the NED frame.
 */
Vector3 calculate_gravity_savage_ned(const Matrix& C_n_to_e, double h);
}  // namespace navutils
}  // namespace navtk
