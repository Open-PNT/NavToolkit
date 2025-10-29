#pragma once

#include <memory>
#include <string>

#include <xtensor/views/xview.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/stateblocks/EarthModel.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/filtering/stateblocks/discretization_strategy.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

typedef aspn_xtensor::MeasurementPositionVelocityAttitude Pva;
typedef aspn_xtensor::MeasurementImu Imu;

namespace navtk {
namespace filtering {

/**
 * A 15-state representation of the error model of an inertial navigation system in NED frame.
 * Based upon the model provided in the Titterton and Weston 2nd edition textbook (pg. 345). The
 * 15-state model is created by combining the original 9x9 state block with the 6x6 G*u block
 * that relates the gyro biases and accelerometer biases to the tilt and velocity error states,
 * respectively. Additional changes include the conversion to North, East and Down position
 * errors in meters as opposed to the latitude (radians), longitude (radians) and altitude
 * (meters) error states in the book model. Note that the error states are additive, meaning in
 * general you add the error state to the uncorrected value to get the corrected value. Tilt
 * states require special handling; see below.
 *
 * Tilt errors: The North, East and Down tilt errors are 3 small angle corrections that when
 * represented in skew-symmetric form and subtracted from an identity matrix may be interpreted as
 * a DCM that rotates a vector from an estimated navigation frame to the 'true' navigation frame,
 * to the extent that the error states are correct. A positive tilt error results in a negative
 * right-handed rotation about the axis to which it is attached. For example, if the sensor frame
 * is aligned with the local vertical with 90 degree heading (sensor x axis is aligned with East
 * axis), and the down tilt value is 1 degree, the corrected heading will be approximately 89
 * degrees.
 *
 * The AspnBaseVector provided to this class should come from the inertial for which it is
 * providing error estimates. Accel and gyro bias states are generally in the inertial sensor frame,
 * but more precisely, they are in the frame that is related to the navigation frame by the
 * NavSolution::rot_mat provided in AspnBaseVector. This means that if the inertial
 * is mechanizing in the inertial sensor frame, then NavSolution::rot_mat should contain
 * `C_nav_to_sensor`, or the nav-to-sensor DCM, and the biases will be in the sensor frame. If the
 * inertial is mechanizing in the platform frame (which is very common), then
 * NavSolution::rot_mat should be `C_nav_to_platform`, the nav-to-platform DCM. The biases will be
 * with respect to that frame.
 *
 * Note that these definitions only hold when using the 'additive error state' formulation
 * (true = estimated + error), the current assumption in all off-the-shelf measurement processors.
 * The opposite formulation will flip the sign on estimated values.
 *
 * Order and description of states:
 *
 *         0 - North position error (m).
 *
 *         1 - East position error (m).
 *
 *         2 - Down position error (m).
 *
 *         3 - North velocity error (m/s).
 *
 *         4 - East velocity error (m/s).
 *
 *         5 - Down velocity error (m/s).
 *
 *         6 - North tilt error (rad).
 *
 *         7 - East tilt error (rad).
 *
 *         8 - Down tilt error (rad).
 *
 *         9 - Accel x-axis bias error (m/s^2) (See note on frame above).
 *
 *         10 - Accel y-axis bias error (m/s^2).
 *
 *         11 - Accel z-axis bias error (m/s^2).
 *
 *         12 - Gyro x-axis bias error (rad/s).
 *
 *         13 - Gyro y-axis bias error (rad/s).
 *
 *         14 - Gyro z-axis bias error (rad/s).
 */
class Pinson15NedBlock : public StateBlock<> {
public:
	/**
	 * A function which, given the current time and the target propagation time, returns the
	 * auxiliary data needed by the StateBlock to propagate.
	 */
	typedef std::function<AspnBaseVector(aspn_xtensor::TypeTimestamp, aspn_xtensor::TypeTimestamp)>
	    LinearizationPointFunction;

	/**
	 * @param label The unique identifier for this StateBlock.
	 * @param imu_model A value to set the active ImuModel.
	 * @param lin_function A value to set the active #LinearizationPointFunction.
	 * @param discretization_strategy A value to set the active #DiscretizationStrategy. Defaults to
	 * a #second_order_discretization_strategy.
	 * @param gravity_model A value to set the active GravityModel. Defaults to a
	 * GravityModelSchwartz.
	 */
	Pinson15NedBlock(
	    const std::string &label,
	    ImuModel imu_model,
	    LinearizationPointFunction lin_function        = nullptr,
	    DiscretizationStrategy discretization_strategy = &second_order_discretization_strategy,
	    not_null<std::shared_ptr<GravityModel>> gravity_model =
	        std::make_shared<GravityModelSchwartz>());

	/**
	 * Custom copy constructor which creates a deep copy.
	 *
	 * @param block The Pinson15NedBlock to copy.
	 */
	Pinson15NedBlock(const Pinson15NedBlock &block);

	/**
	 * A function which is used to give the state block aux data. Note that
	 * it generates Q matrix upon receiving a new ImuModel or a new Pva data, different than
	 * previous ones.
	 *
	 * @param aux_data An AspnBaseVector containing inertial aux (Pva + Imu messages) and/or a
	 * filtering::ImuModel.
	 */
	void receive_aux_data(const AspnBaseVector &aux_data) override;

	/**
	 * Create a copy of the StateBlock with the same properties. Note that
	 * `lin_function` will be shared by the original and the clone.
	 *
	 * @return A shared pointer to a copy of the StateBlock.
	 */
	not_null<std::shared_ptr<StateBlock<>>> clone() override;

	/**
	 * Generates the dynamics model for the fusion engine.
	 *
	 * @param time_from The current filter time.
	 * @param time_to The target propagation time.
	 *
	 * @return The generated dynamics.
	 *
	 * @throw std::runtime_error If the StateBlock has not been provided with inertial aux and the
	 * error mode is ErrorMode::DIE.
	 */
	DynamicsModel generate_dynamics(GenXhatPFunction,
	                                aspn_xtensor::TypeTimestamp time_from,
	                                aspn_xtensor::TypeTimestamp time_to) override;

	/**
	 * Generates the continuous time propagation matrix F, which is the Jacobian of the differential
	 * equations governing inertial error growth. Based upon the model given in Titterton and
	 * Weston, 2nd edition, section 12.3, with some variations as detailed below.
	 *
	 * Errors are defined as 'estimated minus true', or
	 *
	 * \f$ \delta x = \hat{x} - x \f$.
	 *
	 * For DCMs this equates to 'I - estimated * true_transposed':
	 *
	 * \f$ \Psi = I - \hat{C}^n_b C^b_n \f$
	 *
	 * where \f$\Psi\f$ is the skew-symmetric form of tilt errors \f$ \mathbf{\psi} \f$.
	 *
	 * Attitude: The derivation given in the book is correct and used without modification. In
	 * summary,
	 *
	 * \f$ \dot{\Psi} = -\dot{\hat{C^n_b}} C^b_n - \hat{C}^n_b\dot{C}^b_n \f$
	 *
	 * Given the propagation equation of the platform/body-to-ned frame DCM (and a similar equation
	 * for the estimated DCM)
	 *
	 * \f$ \dot{C}^n_b = C^n_b \Omega^b_{ib} - \Omega^n_{in} C^n_b \f$
	 *
	 * and substituting these in to the first equation yields:
	 *
	 * \f$ -\delta\Omega^n_{ib} + \Psi\delta\Omega^n_{ib} + \delta\Omega^n_{in} -
	 * \delta\Omega^n_{in}\Psi -\Omega^n_{in}\Psi + \Psi\Omega^n_{in}\f$
	 *
	 * Dropping error terms and converting to vector form:
	 *
	 * \f$ -\omega^n_{in}\times\psi + \delta\omega^n_{in} - C^n_n\delta\omega^b_{ib} \f$
	 *
	 * Where
	 *
	 * \f$ \omega^n_{ie} = \Omega \begin{bmatrix}\cos{L}, 0, -\sin{L}\end{bmatrix}\f$
	 *
	 * \f$ \delta\omega^n_{ie} = \Omega[\cos{\hat{L}} - \cos{L}, 0, -\sin{\hat{L}} + \sin{L}]\f$
	 *
	 * \f$ = \Omega[\cos{(L + \delta L)} - \cos{L}, 0, -\sin{(L + \delta L)} + \sin{L}]\f$
	 *
	 * \f$ = \Omega[\cos{L}\cos{\delta L} - \sin{L}\sin{\delta L} - \cos{L}, 0, -\sin{L}\cos{\delta
	 * L} - \cos{L}\sin{\delta L} + \sin{L}]\f$
	 *
	 * \f$ \delta L \f$ is assumed to be a 'small' angle, which results in the following
	 * approximations used throughout:
	 *
	 * \f$ \cos{\delta x} \approx 1 \f$
	 *
	 * \f$ \sin{\delta x} \approx \tan{\delta x} \approx \delta x \f$
	 *
	 * Using these approximations gives:
	 *
	 * \f$ \approx \Omega[\cos{L} - \sin{(L)}\delta L - \cos{L}, 0, -\sin{L} - \cos{(L)}\delta L +
	 * \sin{L}]\f$
	 *
	 * \f$ \approx \Omega[-\sin{(L)}\delta L, 0, -\cos{(L)}\delta L]\f$
	 *
	 * For the transport rate term:
	 *
	 * \f$ \omega^n_{en} = [\frac{v_e}{R_e + h}, \frac{-v_n}{R_n + h}, \frac{-v_e\tan{L}}{R_e + h}]
	 * \f$
	 *
	 * \f$ \delta\omega^n_{en} = [\frac{v_e + \delta v_e}{R_e + \delta R_e + h + \delta h}
	 * -\frac{v_e}{R_e + h}, \frac{-(v_n + \delta v_n)}{R_n + \delta R_n + h + \delta h} -
	 * \frac{-v_n}{R_n + h}, \frac{-(v_e + \delta v_e)\tan{(L + \delta L)}}{R_e + \delta R_e + h +
	 * \delta h} - \frac{-v_e \tan{L}}{R_e + h}] \f$
	 *
	 * In this derivation and others, T+W appear to use the Earth radius terms somewhat
	 * inconsistently. While the meters to latitude scaling would normally be \f$ \frac{1}{R_n + h}
	 * \f$, and therefore the estimated scale factor \f$ \frac{1}{R_n + \delta R_n + h + \delta h}
	 * \f$, they effectively use \f$ \frac{1}{R + \delta h} \f$, which is probably meant to be a
	 * 'local navigation on Earth surface' approximation. The terms here are worked fully and these
	 * simplifications explained.
	 *
	 * \f$ \omega^n_{en_0} = \frac{v_e + \delta v_e}{R_e + \delta R_e + h + \delta h}
	 * -\frac{v_e}{R_e + h} \f$
	 *
	 * \f$ = \frac{(v_e + \delta v_e)(R_e + h) - v_e(R_e + \delta R_e + h + \delta h)}{(R_e + \delta
	 * R_e + h + \delta h)(R_e + h)} \f$
	 *
	 * \f$ = \frac{\delta v_e (R_e + h) - v_e (\delta R_e + \delta h) }{(R_e + \delta R_e + h +
	 * \delta h)(R_e + h)} \f$
	 *
	 * \f$ = \frac{\delta v_e}{R_e + \delta R_e + h + \delta h}   -\frac{v_e (\delta R_e + \delta h)
	 * }{(R_e + \delta R_e + h + \delta h)(R_e + h)} \f$
	 *
	 * From here the equations are simplified by neglecting \f$ \delta R_e \f$ and assuming \f$ h
	 * \ll R_e \f$ to get
	 *
	 * \f$ = \frac{\delta v_e}{R_e} - \frac{v_e \delta h }{R_e^2} \f$
	 *
	 * Term 2 is the same format as term 1, with change in sign and velocity term yielding
	 *
	 * \f$ = \frac{-\delta v_n}{R_n} + \frac{v_n \delta h }{R_n^2} \f$
	 *
	 * The third term:
	 *
	 * \f$ \frac{-(v_e + \delta v_e)\tan{(L + \delta L)}}{R_e + \delta R_e + h + \delta h} +
	 * \frac{v_e \tan{L}}{R_e + h} \f$
	 *
	 * \f$ \frac{-(v_e + \delta v_e)(\tan{L} + \tan{\delta L})}{(R_e + \delta R_e + h + \delta
	 * h)(1-\tan{L}\tan{\delta L})} + \frac{v_e \tan{L}}{R_e + h} \f$
	 *
	 * \f$ \frac{-(v_e + \delta v_e)(\tan{L} + \tan{\delta L})(R_e + h) + (v_e \tan{L})(R_e + \delta
	 * R_e + h + \delta h)(1-\tan{L}\tan{\delta L})}{(R_e + \delta R_e + h + \delta
	 * h)(1-\tan{L}\tan{\delta L})(R_e + h)} \f$
	 *
	 * Simplifying with regards to \f$ \delta R_e \f$, \f$ h \f$ and \f$ \delta h \f$ in
	 * denominators as before
	 *
	 * \f$ \frac{-(v_e + \delta v_e)(\tan{L} + \tan{\delta L})(R_e) + (v_e \tan{L})(R_e + \delta
	 * h)(1-\tan{L}\tan{\delta L})}{(R_e^2)(1-\tan{L}\tan{\delta L})} \f$
	 *
	 * Small angle tangent approximation
	 *
	 * \f$ \frac{-(v_e + \delta v_e)(\tan{L} + \delta L)(R_e) + (v_e \tan{L})(R_e + \delta
	 * h)(1-\tan{(L)}\delta L)}{R_e^2(1-\tan{(L)}\delta L)} \f$
	 *
	 * Expanding the numerator and removing error product terms (which includes the \f$ \delta L \f$
	 * term in the denominator):
	 *
	 * \f$ = \frac{(-v_e \delta L - \delta v_e\tan{L} - v_e\tan^2{L} \delta L)(R_e) + v_e \tan{L}
	 * \delta h}{R_e^2}\f$
	 *
	 * \f$ = \frac{-v_e \delta L - \delta v_e\tan{L} - v_e\tan^2{L} \delta L}{R_e} + \frac{v_e
	 * \tan{L} \delta h}{R_e^2}\f$
	 *
	 * \f$ = \frac{-v_e \delta L(1 + \tan^2{L}) - \delta v_e\tan{L}}{R_e} + \frac{v_e \tan{L} \delta
	 * h}{R_e^2}\f$
	 *
	 * \f$ = \frac{-v_e \delta L\sec^2{L} - \delta v_e\tan{L}}{R_e} + \frac{v_e \tan{L} \delta
	 * h}{R_e^2}\f$
	 *
	 * \f$ = \frac{-v_e \delta L}{R_e \cos^2{L}} - \frac{\delta v_e\tan{L}}{R_e}  + \frac{v_e
	 * \tan{L} \delta h}{R_e^2}\f$
	 *
	 * Velocity:
	 *
	 * The velocity derivation begins with the estimated - true propagation:
	 *
	 * \f$ \delta \dot{v} = \hat{C}^n_b\hat{f}^b - C^n_b f^b -(2\hat{\omega}^n_{ie} +
	 * \hat{\omega}^n_{en}) \times \hat{v} + (2\omega^n_{ie} + \omega^n_{en}) \times v + \hat{g_l} -
	 * g_l \f$
	 *
	 * Subbing in for estimated terms and fully expanding:
	 *
	 * \f$ = -\Psi C^n_bf^b + C^n_b\delta f^b + \Psi C^n_b\delta f_b - (2\omega^n_{ie} +
	 * \omega^n_{en})\times \delta v - (2\delta\omega^n_{ie} + \delta\omega^n_{en})\times v
	 * + (2\delta\omega^n_{ie} + \delta\omega^n_{en})\times\delta v + \delta g\f$
	 *
	 * Note the sign on the gravity error term in the text is incorrect. Error product terms are
	 * dropped leaving
	 *
	 *  \f$ = -\Psi C^n_bf^b + C^n_b\delta f^b - (2\omega^n_{ie} + \omega^n_{en})\times \delta v -
	 * (2\delta\omega^n_{ie} + \delta\omega^n_{en})\times v
	 *  + \delta g\f$
	 *
	 * Aside from the gravity error this equation is correct. The \f$ \delta g_l \f$ term is not
	 * provided. This model uses the navtk::filtering::GravityModelSchwartz::calculate_gravity
	 * gravity model as the gravity modeling function. Derivation of the gravity error is as
	 * follows:
	 *
	 * Gravity is modeled as
	 *
	 * \f$ A_1(1 + A_2\sin^2{L} + A_3\sin^4{L}) + (A_4 + A_5\sin^2{L})h + A_6 h^2 \f$
	 *
	 * where each \f$ A \f$ term is a constant. Therefore
	 *
	 * \f$ \delta g = \hat{g} - g = A_1(1 + A_2\sin^2{(L + \delta L)} + A_3\sin^4{(L + \delta L)}) +
	 * (A_4 + A_5\sin^2{(L + \delta L)})(h + \delta h) + A_6(h + \delta h)^2 \\ - A_1(1 +
	 * A_2\sin^2{L} - A_3\sin^4{L}) - (A_4 + A_5\sin^2{L})h - A_6 h^2 \f$
	 *
	 * The latitude error Jacobian term:
	 *
	 * \f$ \frac{\partial\delta g}{\partial\delta L} = 2A_1A_2\sin{(L + \delta L)}\cos{(L + \delta
	 * L)} + 4A_1A_3\sin^3{(L + \delta L)}\cos{(L + \delta L)} + 2A_5(h + \delta h)\sin{(L + \delta
	 * L)}\cos{(L + \delta L)}\f$
	 *
	 * Rearrange and use small angle assumptions
	 *
	 * \f$ [2A_1A_2 + 4A_1A_3(\sin{L} + \cos{(L)\delta L})^2 + 2A_5(h + \delta h)](\sin{L} +
	 * \cos{(L)}\delta L)(\cos{L} - \sin{(L)}\delta L)\f$
	 *
	 * Since this is the derivative, any remaining error terms in the above expression will result
	 * in error products. Ignoring these yields
	 *
	 * \f$ (2A_1A_2 + 4A_1A_3\sin^2{L} + 2A_5h)\sin{L}\cos{L} \f$
	 *
	 * The delta altitude term is easier. By inspection of \f$ \delta g \f$
	 *
	 * \f$ \frac{\partial\delta g}{\partial\delta h} = A_4 + A_5\sin^2{(L + \delta L)} + 2 A_6 (h +
	 * \delta h) \approx A_4 + A_5\sin^2{L} + 2 A_6 h\f$
	 *
	 * Relating these to NED position error states gives
	 *
	 * \f$ \frac{\partial\delta g}{\partial\delta N} \approx (2A_1A_2 + 4A_1A_3\sin^2{L} +
	 * 2A_5h)\sin{L}\cos{L}/(R_n + h) \f$
	 *
	 * \f$ \frac{\partial\delta g}{\partial\delta D} \approx -A_4 - A_5\sin^2{L} - 2 A_6 h\f$
	 *
	 * The position terms as given in the book are derived with similar approximations, from the
	 * relationships
	 *
	 * \f$ \dot{L} = \frac{v_n}{R + h} \f$
	 *
	 * \f$ \dot{l} = \frac{v_e \sec{L}}{R + h} \f$
	 *
	 * \f$ \dot{h} = -v_d \f$
	 *
	 * Of course, the states we use are NED error states, which are just scaled versions of these
	 * values. 'Unscaling' these functions and deriving the error relationship just yields \f$
	 * \dot{\delta} p = \delta v \f$.
	 *
	 * @param pva_aux A MeasurementPositionVelocityAttitude to be used as aux data
	 * @param force_and_rate_aux A MeasurementImu to be used as aux data
	 *
	 * @return The F matrix.
	 */
	Matrix generate_f_pinson15(const Pva &pva_aux, const Imu &force_and_rate_aux);

	/**
	 * Generates the Q matrix.
	 *
	 * @param C_sensor_to_nav The rotation from the sensor frame to the navigation frame as a DCM.
	 *
	 * @return The Q matrix.
	 */
	Matrix generate_q_pinson15(const Matrix &C_sensor_to_nav);

	/**
	 * @return The active ImuModel.
	 */
	ImuModel get_imu_model() const;

	/**
	 * @return The Q matrix.
	 */
	Matrix get_q15_matrix() const;

	/**
	 * @return The active #LinearizationPointFunction.
	 */
	LinearizationPointFunction get_lin_function() const;

	/**
	 * @return The active #DiscretizationStrategy.
	 */
	DiscretizationStrategy get_discretization_strategy() const;

	/**
	 * @return The active GravityModel.
	 */
	not_null<std::shared_ptr<const GravityModel>> get_gravity_model() const;

	/**
	 * @return The most recently passed-in pva aux data.
	 */
	std::shared_ptr<Pva> get_pva_aux();

	/**
	 * @return The most recently passed-in force and rate aux data.
	 */
	std::shared_ptr<Imu> get_force_and_rate_aux();

	/**
	 * Scale first 15 elements of first 2 columns of `phi` such to account for change in rad to
	 * meter scale factors over propagation time.
	 *
	 * @param phi matrix obtained by discretizing the propagation Jacobian.
	 *
	 * @return Input matrix with slight scale adjustment w.r.t. N and E position error states.
	 */
	Matrix scale_phi(Matrix &phi);

private:
	/**
	 * An instance of ImuModel specifying the error model parameters of the INS being used.
	 */
	ImuModel imu_model;

	/**
	 * Keep if all the imu_model's accel_rand_walk elements are the same.
	 */
	bool all_eq_accel_randwalk = false;

	/**
	 * Keep if all the imu_model's gyro_rand_walk elements are the same.
	 */
	bool all_eq_gyro_randwalk = false;

	/**
	 * A Pinson15 Q matrix
	 */
	Matrix q15_matrix = zeros(15, 15);

	/**
	 * Storage for a #LinearizationPointFunction.
	 */
	LinearizationPointFunction lin_function;

	/**
	 * Gravity model used when generating dynamics.
	 */
	not_null<std::shared_ptr<GravityModel>> gravity_model;

	/**
	 * Stores the most recent instance of PVA aux received by the state block. When the copy
	 * constructor for this class is called, a deep copy of this member is made.
	 */
	std::shared_ptr<Pva> new_pva_aux;

	/**
	 * Stores the most recent instance of forces aux received by the state block. When the copy
	 * constructor for this class is called, a deep copy of this member is made.
	 */
	std::shared_ptr<Imu> new_force_and_rate_aux;

	/**
	 * Stores the second-most recent instance of PVA aux received by the state block. When the copy
	 * constructor for this class is called, a deep copy of this member is made.
	 */
	std::shared_ptr<Pva> old_pva_aux;

	/**
	 * Stores the second-most recent instance of forces aux received by the state block. When the
	 * copy constructor for this class is called, a deep copy of this member is made.
	 */
	std::shared_ptr<Imu> old_force_and_rate_aux;
};


}  // namespace filtering
}  // namespace navtk
