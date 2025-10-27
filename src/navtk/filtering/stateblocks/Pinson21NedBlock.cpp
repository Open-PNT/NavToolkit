#include <navtk/filtering/stateblocks/Pinson21NedBlock.hpp>

#include <navtk/errors.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>

namespace navtk {
namespace filtering {

Pinson21NedBlock::Pinson21NedBlock(const std::string& label,
                                   ImuModel imu_model,
                                   Pinson15NedBlock::LinearizationPointFunction lin_function,
                                   DiscretizationStrategy discretization_strategy,
                                   not_null<std::shared_ptr<GravityModel>> gravity_model)
    : StateBlock(21, label),
      p15("",
          std::move(imu_model),
          std::move(lin_function),
          std::move(discretization_strategy),
          std::move(gravity_model)) {}

Pinson21NedBlock::Pinson21NedBlock(const Pinson21NedBlock& block)
    : StateBlock(block), p15(block.p15) {}

not_null<std::shared_ptr<StateBlock<>>> Pinson21NedBlock::clone() {
	return std::make_shared<Pinson21NedBlock>(*this);
}

void Pinson21NedBlock::receive_aux_data(const AspnBaseVector& aux_data) {
	p15.receive_aux_data(aux_data);
}

DynamicsModel Pinson21NedBlock::generate_dynamics(GenXhatPFunction,
                                                  aspn_xtensor::TypeTimestamp time_from,
                                                  aspn_xtensor::TypeTimestamp time_to) {
	if (p15.get_lin_function() != nullptr) {
		p15.receive_aux_data(p15.get_lin_function()(time_from, time_to));
	}
	auto pva_aux            = p15.get_pva_aux();
	auto force_and_rate_aux = p15.get_force_and_rate_aux();
	if (pva_aux == nullptr || force_and_rate_aux == nullptr)
		log_or_throw<std::runtime_error>(
		    "Pinson21 Cannot propagate unless it first receives aux_data with a Pose object");

	double dt             = (time_to.get_elapsed_nsec() - time_from.get_elapsed_nsec()) * 1e-9;
	auto F                = generate_f_pinson(*pva_aux, *force_and_rate_aux);
	auto Q15              = p15.get_q15_matrix();
	Matrix Q              = zeros(num_states, num_states);
	size_t p15_num_states = p15.get_num_states();
	xt::view(Q, xt::range(0, p15_num_states), xt::range(0, p15_num_states)) = Q15;
	// See PNTOS-330 for why these have to be min valued to guard against 0 in Q
	const auto min_process_noise = 1e-12;
	const auto ppm_to_unitless   = 1e-6;
	for (Size k = 0; k < 3; k++) {
		Q(p15_num_states + k, p15_num_states + k) = std::max(
		    p15.get_imu_model().accel_scale_factor(k) * ppm_to_unitless, min_process_noise);
		Q(p15_num_states + 3 + k, p15_num_states + 3 + k) =
		    std::max(p15.get_imu_model().gyro_scale_factor(k) * ppm_to_unitless, min_process_noise);
	}
	auto discretized = p15.get_discretization_strategy()(F, eye(num_states), Q, dt);
	auto Phi         = discretized.first;
	auto Qd          = discretized.second;

	Phi = p15.scale_phi(Phi);

	auto g = [Phi = Phi](const Vector& x) { return dot(Phi, x); };
	return DynamicsModel(g, std::move(Phi), std::move(Qd));
}

Matrix Pinson21NedBlock::generate_f_pinson(
    aspn_xtensor::MeasurementPositionVelocityAttitude pva_aux,
    aspn_xtensor::MeasurementImu force_and_rate_aux) {
	Matrix C_sensor_to_nav = navtk::navutils::quat_to_dcm(pva_aux.get_quaternion());

	Matrix F              = zeros(num_states, num_states);
	size_t p15_num_states = p15.get_num_states();
	xt::view(F, xt::range(0, p15_num_states), xt::range(0, p15_num_states)) =
	    p15.generate_f_pinson15(pva_aux, force_and_rate_aux);

	// Fill in scale factor diff eq terms
	Vector force = force_and_rate_aux.get_meas_accel();
	Vector rate  = force_and_rate_aux.get_meas_gyro();
	xt::view(F, xt::range(3, 6), xt::range(15, 18)) =
	    dot(C_sensor_to_nav, diag(dot(transpose(C_sensor_to_nav), force)));
	xt::view(F, xt::range(6, 9), xt::range(18, 21)) = dot(-C_sensor_to_nav, diag(rate));

	return F;
}

}  // namespace filtering
}  // namespace navtk
