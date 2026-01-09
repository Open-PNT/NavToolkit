#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/filtering/stateblocks/discretization_strategy.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * As the Pinson15NedBlock, but with an additional 6 states to model scale factor errors
 * from accel and gyros.
 *
 * Specifically, adds the following states:
 *
 *     15 - Accel x-axis scale factor (unitless).
 *
 *     16 - Accel y-axis scale factor (unitless).
 *
 *     17 - Accel z-axis scale factor (unitless).
 *
 *     18 - Gyro x-axis scale factor (unitless).
 *
 *     19 - Gyro y-axis scale factor (unitless).
 *
 *     20 - Gyro z-axis scale factor (unitless).
 *
 * For both accels and gyros, a single-axis measurement is modeled as
 * `meas = (1 + scale_factor) * true + other error terms`.
 */
class Pinson21NedBlock : public StateBlock<> {
public:
	/**
	 * Constructor.
	 * @param label Label used to refer to this block.
	 * @param imu_model An ImuModel for the inertial whose errors this block models.
	 * @param lin_function Optional function that returns a linearization point.
	 * @param discretization_strategy Strategy to use when discretizing continuous time propagation
	 *  matrices. Defaults to a #second_order_discretization_strategy.
	 * @param gravity_model Gravity model to use. Defaults to a
	 * GravityModelSchwartz.
	 */
	Pinson21NedBlock(
	    const std::string &label,
	    ImuModel imu_model,
	    Pinson15NedBlock::LinearizationPointFunction lin_function = nullptr,
	    DiscretizationStrategy discretization_strategy = &second_order_discretization_strategy,
	    not_null<std::shared_ptr<GravityModel>> gravity_model =
	        std::make_shared<GravityModelSchwartz>());

	/**
	 * Custom copy constructor which creates a deep copy.
	 *
	 * @param block The Pinson21NedBlock to copy.
	 */
	Pinson21NedBlock(const Pinson21NedBlock &block);

	/**
	 * Create a copy of the StateBlock with the same properties. Note that the lin_function of
	 * #p15 will be shared by the original and the clone.
	 *
	 * @return A shared pointer to a copy of the StateBlock.
	 */
	not_null<std::shared_ptr<StateBlock<>>> clone() override;

	/**
	 * A function used to give the state block aux data.
	 *
	 * @param aux_data An AspnBaseVector containing aux data as subclasses of aspn_xtensor::AspnBase
	 */
	void receive_aux_data(const AspnBaseVector &aux_data) override;

	/**
	 * Generates the dynamics model for the filter.
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
	 * Generates the linearized propagation (F) matrix; this is a modification of the
	 * navtk::filtering::Pinson15NedBlock propagation matrix to account for the additional scale
	 * factor terms for the accel and gyro measurements. A portion of the model derivation follows,
	 * concentrating on scale factor related terms.
	 *
	 * Inertial sensor measurements are now modeled as
	 *
	 * \f$ \hat{f}^b = (I + diag(s_{accel}))f^b + \delta f^b\f$
	 *
	 * \f$ \hat{\omega}^b_{ib} = (I + diag(s_{gyro}))\omega^b_{ib} + \delta \omega^b_{ib}\f$
	 *
	 * Paraphrasing Titterton and Weston (T+W),
	 *
	 * \f$ \dot{\Psi} = -\dot{\hat{C}}^n_b C^b_n - \hat{C}^n_b\dot{C}^n_b \f$
	 *
	 * Following the derivation but substituting \f$ \hat{\omega}^b_{ib} \f$ and expanding:
	 *
	 * \f$ -C^n_b s_{gyro} \Omega^b_{ib}  C^b_n - C^n_b \delta\Omega^b_{ib} C^b_n + \Psi
	 * C^n_b s_{gyro}\Omega^b_{ib}C^b_n + \Psi C^n_b\delta\Omega^b_{ib}C^b_n
	 * + \delta\Omega^n_{in} - \delta\Omega^n_{in}\Psi - \Omega^n_{in}\Psi + \Psi\Omega^n_{in}\f$
	 *
	 * We remove all error product terms and are left with
	 *
	 * \f$ -C^n_b s_{gyro} \Omega^b_{ib}  C^b_n - C^n_b \delta\Omega^b_{ib} C^b_n +
	 * \delta\Omega^n_{in} - \Omega^n_{in}\Psi + \Psi\Omega^n_{in}\f$
	 *
	 * Linearization error is then
	 *
	 * \f$ \Psi C^n_b s_{gyro}\Omega^b_{ib}C^b_n + \Psi C^n_b\delta\Omega^b_{ib}C^b_n -
	 * \delta\Omega^n_{in}\Psi \f$
	 *
	 * Refactoring the kept terms into vector form:
	 *
	 * \f$ \dot{\psi} = -C^n_b diag(\omega^b_{ib}) s_{gyro} - C^n_b\delta\omega^b_{ib} +
	 * \delta\omega^n_{in} \times \psi \f$
	 *
	 * Highlights of the velocity term redevelopment with scale factors (note sign error on T+W
	 * original \f$ \delta g \f$):
	 *
	 * \f$ \dot{\delta v} = \hat{C}^n_b\hat{f}^b - C^n_bf^b - (2\hat{\omega}^n_{ie} +
	 * \hat{\omega^n_{en}}) \times \hat{v} + (2\omega^n_ie + \omega^n_{en})\times v + \delta g \f$
	 *
	 * \f$= [I - \Psi]C^n_b[(1 + s_{accel})f^b + \delta f^b] - (2\omega^n_{ie} +
	 * 2\delta\omega^n_{ie}
	 * + \omega^n_{en} + \delta\omega^n_{en}) \times (v + \delta v) - C^n_b f^b + (2\omega^n_{ie} +
	 * \omega^n_{en}) \times v + \delta g\f$
	 *
	 * Expanding with scale factor term:
	 *
	 * \f$ C^n_bs_{accel}f^b + C^n_b\delta f^b - \Psi C^n_bf^b - \Psi C^n_b s_{accel}f^b - \Psi
	 * C^n_b \delta f^b
	 * - (2\omega^n_{ie} + \omega^n_{en})\times \delta v - (2 \delta \omega^n_{ie} + \delta
	 * \omega^n_{en})\times v
	 * -(2 \delta \omega^n_{ie} + \delta \omega^n_{en})\times \delta v
	 * + \delta g \f$
	 *
	 * Of these, the only terms not already in the base Pinson15 model are those containing the
	 * scale factor term
	 *
	 * \f$ C^n_bs_{accel}f^b - \Psi C^n_b s_{accel}f^b \f$
	 *
	 * with the second term being dropped as an error product, as normal.
	 *
	 * @param pva_aux PVA to linearize the state propagation matrix about.
	 * @param force_and_rate_aux Forces and rotation rates to linearize the state propagation matrix
	 * about.
	 *
	 * @return The F matrix.
	 */
	Matrix generate_f_pinson(aspn_xtensor::MeasurementPositionVelocityAttitude pva_aux,
	                         aspn_xtensor::MeasurementImu force_and_rate_aux);

protected:
	/**
	 * A Pinson15 block used to handle aux data and generate the base F and Q vectors.
	 */
	Pinson15NedBlock p15;
};


}  // namespace filtering
}  // namespace navtk
