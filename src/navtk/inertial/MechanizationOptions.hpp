#pragma once

#include <navtk/navutils/gravity.hpp>

namespace navtk {
namespace inertial {

/**
 * Method for velocity integration.
 */
enum class IntegrationMethods {
	/**
	 * Use rectangular integration \f$ v_0 * \delta t \f$ to calculate new position.
	 */
	RECTANGULAR,

	/**
	 * Use trapezoidal \f$ \frac{(v_0 + v_1)\delta t}{2} \f$ to calculate new position.
	 */
	TRAPEZOIDAL,

	/**
	 * Use Simpson's rule \f$ \frac{(v_prev + 4 * v_0 + v_1)\delta t}{6} \f$ to calculate new
	 * position.
	 */
	SIMPSONS_RULE
};

/**
 * Method to integrate delta rotations into new DCM.
 */
enum class DcmIntegrationMethods {
	/**
	 * First-order integration `C * I + skew(d)`.
	 */
	FIRST_ORDER,

	/**
	 * Use 6th order Taylor series expansion of the rotation vector.
	 */
	SIXTH_ORDER,

	/**
	 * Integrate using matrix exponential.
	 */
	EXPONENTIAL
};

/**
 * Available methods for modelling the Earth in the mechanization.
 */
enum class EarthModels {
	/**
	 * Model the earth as an ellipsoid, using the parameter values found
	 * in WGS84.cpp and/or in Savage table 5.6-1.
	 */
	ELLIPTICAL,

	/**
	 * Model the earth as spherical.
	 */
	SPHERICAL
};

/**
 * Collection of option flags to pass into mechanization functions.
 */
struct MechanizationOptions {
	/**
	 * Select among various gravity providers.
	 */
	navutils::GravModels grav_model = navutils::GravModels::SCHWARTZ;
	/**
	 * Available methods for modelling the Earth in the mechanization.
	 */
	EarthModels earth_model = EarthModels::ELLIPTICAL;
	/**
	 * Method to integrate delta rotations into new DCM.
	 */
	DcmIntegrationMethods dcm_method = DcmIntegrationMethods::SIXTH_ORDER;
	/**
	 * Method of velocity integration.
	 */
	IntegrationMethods int_method = IntegrationMethods::TRAPEZOIDAL;
};

}  // namespace inertial
}  // namespace navtk
