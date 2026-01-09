#pragma once

#include <navtk/inertial/AidingAltData.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/Mechanization.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Class that performs IMU mechanization in the NED/WGS84 frames.
 */
class MechanizationStandard : public Mechanization {

public:
	virtual ~MechanizationStandard() = default;

	/**
	 * Perform a single mechanization of delta velocity and delta rotation
	 * measurements using NED-frame-referenced attitude and velocity and
	 * WGS84 polar coordinates. Essentially a re-implementation of the
	 * mechanization_standard function that uses internal state and minimal
	 * Matrix/Vector operations to be more performant.
	 *
	 * @param dv_s Vector3 of delta velocity measurements in the inertial
	 * sensor frame, m/s.
	 * @param dth_s Vector3 of delta rotations in the inertial sensor
	 * frame, rad.
	 * @param dt Time over which `dv_s` and `dth_s` were collected (and thus
	 * time to mechanize over), seconds.
	 * @param pva Position, velocity and attitude data at the start of
	 * the mechanization interval.
	 * @param old_pva Position, velocity, and attitude data from the start of the
	 * previous mechanization interval.
	 * @param mech_options Mechanization options to use.
	 * @param aiding_alt_data Container holding an external altitude measurement
	 * in m HAE, the accumulated error resulting from the aiding the altitude in
	 * the inertial's mechanization, and additional options for the aiding algorithm.
	 *
	 * @return Non-null `shared_ptr` to a StandardPosVelAtt instance that contains
	 *  the post-mechanization PVA values.
	 */
	not_null<std::shared_ptr<InertialPosVelAtt>> mechanize(
	    const Vector3& dv_s,
	    const Vector3& dth_s,
	    double dt,
	    const not_null<std::shared_ptr<InertialPosVelAtt>> pva,
	    const not_null<std::shared_ptr<InertialPosVelAtt>> old_pva,
	    const MechanizationOptions& mech_options,
	    AidingAltData* aiding_alt_data) override;

private:
	Matrix3 I3 = eye(3);

	// Currently hardcoded as it is in mechanization_standard, and this value
	// required to match some old unit tests we have. Needs to be parameterized
	// somehow- could become a member of mechanization options.
	double omega = 7.292115e-5;

	// Data elements to be stripped out of or computed from the mechanize
	// function arguments
	MechanizationOptions options;
	double lat0 = 0;
	double lon0 = 0;
	double alt0 = 0;
	double vn0  = 0;
	double ve0  = 0;
	double vd0  = 0;
	Matrix3 C_s_to_n0;
	double sin_l = 0;
	double cos_l = 0;
	double sec_l = 0;
	double tan_l = 0;
	double rn;
	double re;
	double r_zero;
	double dt;
	double dv0;
	double dv1;
	double dv2;
	double dth0;
	double dth1;
	double dth2;
	Vector3 g;
	Vector3 v_ned_prev;

	// Force and acceleration values for NED frame
	double fn;
	double fe;
	double fd;
	double an;
	double ae;
	double ad;

	// Change in position (integrated velocity) in the NED frame
	double dpn;
	double dpe;
	double dpd;

	// Aiding altitude scale factor
	double c1;

	// Intermediate variables used in propagating the DCM
	Vector3 sigma = zeros(3);
	Matrix3 A;

	// Containers for the completed mechanized solution elements
	Vector3 llh1  = zeros(3);
	Vector3 vned1 = zeros(3);
	Matrix3 C_s_to_n1;

	// Updates g member based on gravity setting in options
	void calc_grav();

	// Updates sigma member from delta rotation
	void calc_rot_rate();

	// Integrates sigma to obtain A, the DCM that rotates C_S_to_n0 to C_s_to_n1
	void calc_dcm();

	// Updates fn, fe and fd, the specific force elements in the NED frame, m/s^2
	void calc_force_ned();

	// Updates an, ae and ad, the acceleration elements in the NED frame, m/s^2
	void calc_acceleration();

	// Integrates acceleration to obtain new velocity vned1
	void integrate_to_velocity();

	// Integrates velocity to obtain new position, llh1
	void integrate_to_position();

	// Strip out data and perform initial calculations
	void prep(const not_null<std::shared_ptr<InertialPosVelAtt>> pva,
	          const not_null<std::shared_ptr<InertialPosVelAtt>> oldpva,
	          const Vector3& dv,
	          const Vector3& dth,
	          const double dt);

	// Perform the actual mechanization, updating llh1, vned1 and C_s_to_n1
	void compute(AidingAltData* aiding_alt_data);
};

}  // namespace inertial
}  // namespace navtk
