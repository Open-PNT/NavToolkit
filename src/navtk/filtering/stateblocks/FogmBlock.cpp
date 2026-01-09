#include <navtk/filtering/stateblocks/FogmBlock.hpp>

#include <navtk/factory.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>

namespace navtk {
namespace filtering {

FogmBlock::FogmBlock(const std::string& label,
                     Vector time_constants,
                     Vector state_sigmas,
                     Vector::shape_type::value_type num_states,
                     DiscretizationStrategy discretization_strategy)
    : StateBlock(num_states, label, std::move(discretization_strategy)),
      time_constants(std::move(time_constants)),
      state_sigmas(std::move(state_sigmas)) {}

FogmBlock::FogmBlock(const std::string& label,
                     double time_constant,
                     double state_sigma,
                     Vector::shape_type::value_type num_states,
                     DiscretizationStrategy discretization_strategy)
    : FogmBlock(label,
                zeros(num_states) + time_constant,
                zeros(num_states) + state_sigma,
                num_states,
                std::move(discretization_strategy)) {}

FogmBlock::FogmBlock(const FogmBlock& block)
    : StateBlock(block), time_constants(block.time_constants), state_sigmas(block.state_sigmas) {}

not_null<std::shared_ptr<StateBlock<>>> FogmBlock::clone() {
	return std::make_shared<FogmBlock>(*this);
}

DynamicsModel FogmBlock::generate_dynamics(GenXhatPFunction,
                                           aspn_xtensor::TypeTimestamp time_from,
                                           aspn_xtensor::TypeTimestamp time_to) {
	double dt = (time_to.get_elapsed_nsec() - time_from.get_elapsed_nsec()) * 1e-9;

	Matrix F = zeros(num_states, num_states);
	Matrix Q = zeros(num_states, num_states);

	populate_f_and_q(F, Q);

	auto discretized = discretization_strategy(F, eye(num_rows(Q)), Q, dt);
	auto Phi         = discretized.first;
	auto Qd          = discretized.second;
	auto g           = [Phi = Phi](Vector x) { return dot(Phi, x); };
	return DynamicsModel(g, Phi, Qd);
}

void FogmBlock::populate_f_and_q(Matrix& F, Matrix& Q) {
	for (Size idx = 0; idx < num_cols(F); ++idx) {
		F(idx, idx) = -1.0 / time_constants[idx];
		Q(idx, idx) = 2.0 * std::pow(state_sigmas[idx], 2) / time_constants[idx];
	}
}
}  // namespace filtering
}  // namespace navtk
