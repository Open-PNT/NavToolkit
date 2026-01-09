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

	if (num_states == 2) {
		Phi = Matrix{{1.0, dt}, {0.0, 1.0}};
	} else {
		Phi = Matrix{{1.0, dt, 0.5 * dt * dt}, {0, 1.0, dt}, {0, 0, 1.0}};
	}

	double dt2 = dt * dt;

	// for both calc_Hwang_Q_no_flicker and original calc_Hwang_Brown_Q
	double ct = pi_sq * model.h_m2 * dt2;

	// for calc_Hwang_Q
	double ct1 = model.h_m1 * dt + pi_sq * model.h_m2 * dt2;

	Matrix Qd;

	// switch between different Qds
	switch (choice) {
	case ClockChoice::QD:
		// defining basic expanded matrix
		Qd = Matrix{
		    {model.h_0 / 2 * dt + 2 * model.h_m1 * dt2 + 2.0 / 3 * pi_sq * model.h_m2 * dt2 * dt,
		     ct,
		     0},
		    {ct, 4 * model.h_m1 + 2 * pi_sq * model.h_m2 * dt, 0},
		    {0, 0, 0}};


		break;

	case ClockChoice::QD1:

		Qd = Matrix{{model.h_0 / 2 * dt + 2.0 / 3 * pi_sq * model.h_m2 * dt2 * dt, ct, 0},
		            {ct, model.h_0 / (2 * dt) + (8.0 / 3) * pi_sq * model.h_m2 * dt, 0},
		            {0, 0, 0}};

		break;

	case ClockChoice::QD2:

		Qd = Matrix{{0.5 * model.h_0 * dt + (2.0 / 3) * pi_sq * model.h_m2 * dt * dt2, ct1, 0},
		            {ct1, 2 * pi_sq * model.h_m2 * dt, 0},
		            {0, 0, 0}};

		break;

	case ClockChoice::QD3:

		Qd = Matrix{{0.5 * model.h_0 * dt + (2.0 / 3) * pi_sq * model.h_m2 * dt * dt2, ct, 0},
		            {ct, 2 * pi_sq * model.h_m2 * dt, 0},
		            {0, 0, 0}};
	}

	auto g = [Phi = Phi](Vector x) { return dot(Phi, x); };
	if (num_states == 3) {


		Matrix q_three{
		    {(1.0 / 20) * model.q3 * dt2 * dt2 * dt,
		     (1.0 / 8) * model.q3 * dt2 * dt2,
		     (1.0 / 6) * model.q3 * dt2 * dt},
		    {(1.0 / 8) * model.q3 * dt2 * dt2,
		     (1.0 / 3) * model.q3 * dt * dt2,
		     (1.0 / 2) * model.q3 * dt2},
		    {(1.0 / 6) * model.q3 * dt2 * dt, (1.0 / 2) * model.q3 * dt2, model.q3 * dt}};

		return DynamicsModel(g, Phi, Qd + q_three);

	} else {

		return DynamicsModel(g, Phi, xt::view(Qd, xt::range(0, 2), xt::range(0, 2)));
	}
}

}  // namespace filtering
}  // namespace navtk
