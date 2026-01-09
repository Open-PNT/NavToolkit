#pragma once

namespace navtk {
namespace inertial {

/**
 * Container that holds an aiding altitude, in meters HAE, the integrated altitude error accumulated
 * by an Inertial when mechanizing with an aiding altitude, and the response's time constant.
 */
struct AidingAltData {
	/**
	 * The altitude measurement used in aiding the mechanization (meters HAE).
	 */
	double aiding_alt = 0.0;

	/**
	 * The integrated error resulting from altitude aiding (meters).
	 */
	double integrated_alt_error = 0.0;

	/**
	 * The value of lambda, time constant of the control loop (seconds), to use when calculating
	 * coefficients for altitude aiding.
	 */
	double time_constant = 0.01;
};

}  // namespace inertial
}  // namespace navtk
