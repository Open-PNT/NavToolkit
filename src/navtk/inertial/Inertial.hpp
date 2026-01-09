#pragma once

#include <memory>

#include <navtk/inertial/AidingAltData.hpp>
#include <navtk/inertial/ImuErrors.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/Mechanization.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/MechanizationStandard.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/inertial/WanderPosVelAtt.hpp>
#include <navtk/inertial/mechanization_standard.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
/**
 * NavToolkit namespace for inertial capabilities.
 */
namespace inertial {

/**
 * Shorthand for mechanization function used as Inertial parameter.
 *
 * @param dv Delta velocity measurements in the inertial sensor frame, m/s.
 * @param dth Delta rotation measurement in the inertial sensor frame, rad.
 * @param dt Delta time over which the delta velocity and rotation measurements
 * were collected, seconds.
 * @param ref Starting position/velocity/attitude for mechanization.
 * @param old_ref Position/velocity/attitude from the start of the previous mechanization interval.
 * @param options Selection of options to use during this mechanization interval.
 * @param aiding_alt_data Container holding an external altitude measurement in m HAE, and the
 * accumulated error resulting from the aiding altitude in the inertial's mechanization.
 *
 * @return Position, velocity and attitude after mechanization.
 */
using MechanizationFunction = std::function<not_null<std::shared_ptr<InertialPosVelAtt>>(
    const Vector3& dv,
    const Vector3& dth,
    const double dt,
    const not_null<std::shared_ptr<InertialPosVelAtt>> ref,
    const not_null<std::shared_ptr<InertialPosVelAtt>> old_ref,
    const MechanizationOptions& options,
    AidingAltData* aiding_alt_data)>;

/**
 * Provides an inertial mechanization capability with stored state and
 * the ability to reset the position, velocity and attitude data in
 * various formats or applying measurement error corrections.
 *
 * The class currently supports two methods of mechanization, each drawn
 * from different text sources. Additionally, each of these mechanization
 * algorithms has a 'native' format of position, velocity and attitude
 * (PVA) data. We commonly use shorthands taken from author names to
 * refer to either algorithms or the common navigation quantity
 * groupings that appear in their books.
 *
 * The first group describes position as latitude, longitude and altitude
 * in radians, radians and meters with respect to the WGS84 ellipsoid;
 * velocity is in \f$\frac{m}{s}\f$ in the North, East and Down frame; and
 * attitude is represented as the sensor to NED frame DCM. This grouping
 * is associated with the mechanization_standard function and StandardPosVelAtt
 * class. We refer to this set of algorithms/navigation quantities as
 * 'Titterton'-style, 'TW'-style, or occasionally 'LLH NED', in reference
 * to 'Strapdown Inertial Navigation Technology, 2nd Edition'
 * by D.H. Titterton and J.L. Weston.
 *
 * The second grouping consists of slightly more complicated quantities-
 * a DCM that encodes latitude, longitude and a wander azimuth angle;
 * a separate altitude value; velocity in the 'n' frame (which is like
 * an ENU frame that rotates with the wander angle as opposed to being
 * pegged to North/East); and attitude as a DCM that rotates from the
 * sensor frame to the 'l' frame (which is related to the 'n' frame as
 * the NED frame is related to ENU). This grouping is associated with the
 * mechanization_wander function and WanderPosVelAtt class.
 * We refer to this set of algorithms/navigation quantities as 'Savage'-style
 * or sometimes 'wander frame', after Strapdown Analytics by
 * Paul G. Savage.
 *
 * It is highly recommended that you interact with this class using
 * formats that 'match' the mechanization function supplied through
 * the constructor.
 */
class Inertial final {

public:
	/**
	 * Constructor.
	 *
	 * @param start_pva Initial PVA of this inertial. Defaults to
	 * whatever the StandardPosVelAtt default constructor generates.
	 * If starting PVA is unknown at the time of creation, update it
	 * via one of the reset functions prior to mechanizing.
	 * @param mech_options Collection of options to pass into the
	 * mechanization function.
	 * @param mech_fun Function used to perform the actual mechanization.
	 * Must accept delta velocity(\f$\frac{m}{s}\f$), delta rotation (rad),
	 * delta time, position, velocity and attitude data at beginning of
	 * mechanization, and a MechanizationOptions instance.
	 */
	explicit Inertial(const not_null<std::shared_ptr<InertialPosVelAtt>> start_pva =
	                      std::make_shared<StandardPosVelAtt>(),
	                  const MechanizationOptions& mech_options = MechanizationOptions{},
	                  MechanizationFunction mech_fun =
	                      static_cast<not_null<std::shared_ptr<InertialPosVelAtt>> (*)(
	                          const Vector3&,
	                          const Vector3&,
	                          const double,
	                          const not_null<std::shared_ptr<InertialPosVelAtt>>,
	                          const not_null<std::shared_ptr<InertialPosVelAtt>>,
	                          const MechanizationOptions&,
	                          AidingAltData* aiding_alt_data)>(mechanization_standard));

	/**
	 * Constructor.
	 *
	 * @param mech_class Pointer to a Mechanization instance that will be used
	 * to perform the actual mechanization.
	 * @param start_pva Initial PVA of this inertial. Defaults to
	 * whatever the StandardPosVelAtt default constructor generates.
	 * If starting PVA is unknown at the time of creation, update it
	 * via one of the reset functions prior to mechanizing.
	 * @param mech_options Collection of options to pass into the
	 * mechanization function.
	 */
	explicit Inertial(not_null<std::shared_ptr<Mechanization>> mech_class,
	                  const not_null<std::shared_ptr<InertialPosVelAtt>> start_pva =
	                      std::make_shared<StandardPosVelAtt>(),
	                  const MechanizationOptions& mech_options = MechanizationOptions{});

	/**
	 * Custom deep-copy constructor.
	 *
	 * @param ins The Inertial to copy. Pointers held by ins are deep-copied.
	 */
	Inertial(const Inertial& ins);

	/**
	 * Copy assignment.
	 *
	 * @param ins The Inertial to assign. Pointers held by ins are deep-copied when assigned.
	 * @return `*this` populated with copies of ins members.
	 */
	Inertial& operator=(const Inertial& ins);

	/**
	 * Default move constructor.
	 */
	Inertial(Inertial&&) = default;

	/**
	 * Default move assign.
	 *
	 * @return `*this`.
	 */
	Inertial& operator=(Inertial&&) = default;

	~Inertial() noexcept = default;

	/**
	 * Reset the inertial solution to a new value.
	 *
	 * @param new_pva Position, velocity attitude and time data to set this
	 * inertial to.
	 * @param old_pva Solution one `dt` prior to `new_pva`. Needed for some
	 * higher-order acceleration integration strategies. If not provided
	 * defaults to the same as `new_pva`.
	 */
	void reset(const not_null<std::shared_ptr<InertialPosVelAtt>> new_pva,
	           const std::shared_ptr<InertialPosVelAtt> old_pva = nullptr);

	/**
	 * Reset the inertial solution to a new value.
	 *
	 * @param new_pva Position, velocity attitude and time data to set this
	 * inertial to.
	 */
	void reset(StandardPosVelAtt new_pva);

	/**
	 * Reset the inertial solution to a new value.
	 *
	 * @param new_pva Position, velocity attitude and time data to set this
	 * inertial to.
	 */
	void reset(WanderPosVelAtt new_pva);

	/**
	 * Obtains the current inertial solution, which is the result of the
	 * last mechanization (or the initial values if at the start).
	 *
	 * @return Current solution.
	 */
	not_null<std::shared_ptr<InertialPosVelAtt>> get_solution() const;

	/**
	 * Returns vector of current set of internally-stored gyro bias
	 * values, which are used to correct all incoming gyro measurements.
	 *
	 * @return Vector3 of biases for each gyro (\f$ \frac{rad}{s} \f$),
	 * in the inertial sensor frame.
	 */
	Vector3 get_gyro_biases() const;

	/**
	 * Sets internally-stored gyro bias values, which are used to
	 * correct all incoming gyro measurements.
	 *
	 * @param gyro_biases Vector3 of biases for each gyro axis
	 * \f$ (\frac{rad}{s}) \f$, in the inertial sensor frame such that
	 * \f$ gyro_{meas} = gyro_{true} + gyro_{bias} \f$
	 * in the absence of other errors.
	 */
	void set_gyro_biases(Vector3 gyro_biases);

	/**
	 * Returns vector of current set of internally-stored gyro scale
	 * factor values, which are used to correct all incoming gyro
	 * measurements.
	 *
	 * @return Vector3 of scale factors for each gyro axis (unitless),
	 * in the inertial sensor frame.
	 */
	Vector3 get_gyro_scale_factors() const;

	/**
	 * Sets internally-stored gyro scale factor values, which are used
	 * to correct all incoming gyro measurements.
	 *
	 * @param gyro_scale_factors Vector3 of scale factors for each gyro
	 * axis (unitless), in the inertial sensor frame such that
	 * \f$ gyro_{meas} = (1 + gyro_{scale})gyro_{true} \f$
	 * in the absence of other errors.
	 */
	void set_gyro_scale_factors(Vector3 gyro_scale_factors);

	/**
	 * Returns vector of current set of internally-stored accel bias
	 * values, which are used to correct all incoming accel measurements.
	 *
	 * @return Vector3 of biases for each accel axis (m/sec^2), in the
	 * inertial sensor frame.
	 */
	Vector3 get_accel_biases() const;

	/**
	 * Sets internally-stored accel bias values, which are used to
	 * correct all incoming accel measurements.
	 *
	 * @param accel_biases Vector3 of biases for each accel axis in the
	 * inertial sensor frame, (m/sec^2), such that
	 * \f$ accel_{meas} = accel_{true} + accel_{bias} \f$
	 * in the absence of other errors..
	 */
	void set_accel_biases(Vector3 accel_biases);

	/**
	 * Returns vector of current set of internally-stored accel scale
	 * factor values, which are used to correct all incoming accel
	 * measurements.
	 * @return Vector3 of scale factors for each accel axis, in the
	 * inertial sensor frame (unitless).
	 */
	Vector3 get_accel_scale_factors() const;

	/**
	 * Sets internally-stored accel scale factor values, which are used
	 * to correct all incoming accel measurements.
	 *
	 * @param accel_scale_factors Vector3 of scale factors for each
	 * accel axis in the inertial sensor frame (unitless), such that
	 * \f$ accel_{meas} = (1 + accel_{scale})accel_{true} \f$
	 * in the absence of other errors.
	 */
	void set_accel_scale_factors(Vector3 accel_scale_factors);

	/**
	 * Supply all inertial sensor error corrections.
	 *
	 * @param errors Biases, scale factors, etc.used to correct the raw inertial measurements.
	 */
	void set_imu_errors(const ImuErrors& errors);

	/**
	 * Integrate inertial sensor measurements to obtain an updated
	 * PVA solution.
	 *
	 * @param time Time to integrate to; i.e. the end time of
	 * the measurement integration period for `accel_meas` and `gyro_meas`.
	 * @param accel_meas Vector3 of accelerometer measurements in the inertial sensor frame.
	 * @param gyro_meas Vector3 of gyro measurements in the inertial sensor frame.
	 * @param aiding_alt_data AidingAltData container storing the aiding altitude measurement and
	 * required parameters for using altitude aiding during mechanization.
	 */
	void mechanize(const aspn_xtensor::TypeTimestamp& time,
	               const Vector3& accel_meas,
	               const Vector3& gyro_meas,
	               AidingAltData* aiding_alt_data = nullptr);
	/**
	 * The cumulative error resulting from aiding the mechanization with an altitude measurement.
	 */
	double integrated_alt_error;

private:
	not_null<std::shared_ptr<InertialPosVelAtt>> pva;
	not_null<std::shared_ptr<InertialPosVelAtt>> pva_old1;
	Vector3 gyro_bias;
	Vector3 gyro_scale_factor;
	Vector3 accel_bias;
	Vector3 accel_scale_factor;
	MechanizationFunction _mech_fun;
	MechanizationOptions _mech_options;

	/**
	 * Corrects raw inertial delta-type measurements using currently
	 * stored values of sensor errors (eg biases, scale factors).
	 * Follows error model described in in Titterton and Weston, 2nd edition, eq 8.4 and 8.5
	 * \f$ v_{meas} = (1 + e_{scale})v_{true} + e_{misalignment} + e_{bias} + e_{noise} \f$
	 * such that corrections are applied by reversing this equation:
	 * \f$ v_{true} =\frac{(v_{meas} - e_{misalignment} - e_{bias} - e_{noise})}{1 + e_{scale}}\f$
	 *
	 * @param dt Delta time over which other inputs are valid (seconds).
	 */
	void apply_error_model(double dt);
	Vector3 dv_cpy;
	Vector3 dth_cpy;
};

}  // namespace inertial
}  // namespace navtk
