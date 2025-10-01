#include <navtk/filtering/stateblocks/ClockBiasesStateBlock.hpp>

#include <spdlog/spdlog.h>

#include <navtk/errors.hpp>
#include <navtk/linear_algebra.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::filtering::DynamicsModel;

namespace navtk {
namespace filtering {

ClockBiasesStateBlock::ClockBiasesStateBlock(const std::string& label,
                                             ClockModel model,
                                             ClockChoice choice,
                                             bool model_frequency_dot)
    : StateBlock(model_frequency_dot ? 3 : 2, label), model(std::move(model)), choice(choice) {
	if (this->model.h_0 < 0 || this->model.h_m1 < 0 || this->model.h_m2 < 0) {
		log_or_throw<std::invalid_argument>("Clock model coefficients cannot be negative");
	}
}


not_null<std::shared_ptr<StateBlock<>>> ClockBiasesStateBlock::clone() {
	return std::make_shared<ClockBiasesStateBlock>(*this);
}

DynamicsModel ClockBiasesStateBlock::generate_dynamics(GenXhatPFunction,
                                                       aspn_xtensor::TypeTimestamp time_from,
                                                       aspn_xtensor::TypeTimestamp time_to) {

	Matrix Phi;

	auto dt = (time_to.get_elapsed_nsec() - time_from.get_elapsed_nsec()) * 1e-9;

	if (dt < 0.0) {
		log_or_throw<std::invalid_argument>("dt should be >= 0 but it is {}", dt);
	}

	double dt2 = dt * dt;

	double rand_walk   = pi_sq * model.h_m2;
	double rand_walk2  = rand_walk * 2;
	double rand_walk23 = rand_walk * (2.0 / 3);

	double white_noise = 0.5 * model.h_0;

	double m00 = (white_noise * dt) + (rand_walk23 * dt * dt2);
	double m11 = rand_walk2 * dt;
	double ct  = pi_sq * model.h_m2 * dt2;

	Matrix Qd;

	// switch between different Qds
	switch (choice) {
	case ClockChoice::QD:
		// defining basic expanded matrix
		Qd = {{m00 + (2 * model.h_m1 * dt2), ct, 0}, {ct, (4 * model.h_m1) + m11, 0}, {0, 0, 0}};
		break;

	case ClockChoice::QD1:
		Qd = {{m00, ct, 0}, {ct, (white_noise / dt) + ((4.0 / 3) * m11), 0}, {0, 0, 0}};
		break;

	case ClockChoice::QD2: {
		double ct1 = ct + model.h_m1 * dt;
		Qd         = {{m00, ct1, 0}, {ct1, m11, 0}, {0, 0, 0}};
	} break;

	case ClockChoice::QD3:
		Qd = {{m00, ct, 0}, {ct, m11, 0}, {0, 0, 0}};
		break;
	}

	if (num_states == 3) {

		Matrix q_three{
		    {(1.0 / 20) * model.q3 * dt2 * dt2 * dt,
		     (1.0 / 8) * model.q3 * dt2 * dt2,
		     (1.0 / 6) * model.q3 * dt2 * dt},
		    {(1.0 / 8) * model.q3 * dt2 * dt2,
		     (1.0 / 3) * model.q3 * dt * dt2,
		     (1.0 / 2) * model.q3 * dt2},
		    {(1.0 / 6) * model.q3 * dt2 * dt, (1.0 / 2) * model.q3 * dt2, model.q3 * dt}};

		Phi    = Matrix{{1.0, dt, 0.5 * dt2}, {0, 1.0, dt}, {0, 0, 1.0}};
		auto g = [Phi = Phi](Vector x) { return dot(Phi, x); };

		return DynamicsModel(g, Phi, Qd + q_three);

	} else {

		Phi    = Matrix{{1.0, dt}, {0.0, 1.0}};
		auto g = [Phi = Phi](Vector x) { return dot(Phi, x); };

		return DynamicsModel(g, Phi, xt::view(Qd, xt::range(0, 2), xt::range(0, 2)));
	}
}

}  // namespace filtering
}  // namespace navtk
