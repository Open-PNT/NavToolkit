#pragma once

#include <memory>

#include <navtk/inertial/AidingAltData.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Base class that performs IMU mechanization. Can be used as an alternative to
 * mechanization function pointers to enable mechanization that carries state.
 */
class Mechanization {

public:
	virtual ~Mechanization() = default;

	/**
	 * Perform a single mechanization of delta velocity and delta rotation
	 * measurements to propagate a position, velocity and attitude forward.
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
	 * @param aiding_alt_data Container holding an external altitude measurement in m HAE, the
	 * accumulated error resulting from the aiding the altitude in the inertial's mechanization, and
	 * additional options for the aiding algorithm.
	 *
	 * @return Non-null `shared_ptr` to an InertialPosVelAtt instance that contains the
	 * post-mechanization PVA values.
	 */
	virtual not_null<std::shared_ptr<InertialPosVelAtt>> mechanize(
	    const Vector3& dv_s,
	    const Vector3& dth_s,
	    double dt,
	    const not_null<std::shared_ptr<InertialPosVelAtt>> pva,
	    const not_null<std::shared_ptr<InertialPosVelAtt>> old_pva,
	    const MechanizationOptions& mech_options,
	    AidingAltData* aiding_alt_data) = 0;

protected:
	Mechanization() = default;
};

}  // namespace inertial
}  // namespace navtk
