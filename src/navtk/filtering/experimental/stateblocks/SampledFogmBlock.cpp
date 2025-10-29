#include <navtk/filtering/experimental/stateblocks/SampledFogmBlock.hpp>

#include <cmath>

#include <navtk/factory.hpp>
#include <navtk/utils/ValidationContext.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/core/xmath.hpp>

using navtk::utils::ValidationContext;
namespace navtk {
namespace filtering {
namespace experimental {

SampledFogmBlock::SampledFogmBlock(
    const std::string& label,
    Vector time_constants,
    Vector state_sigmas,
    size_t num_states,
    navtk::not_null<std::shared_ptr<navtk::experimental::RandomNumberGenerator>> rng)
    : StateBlock<>(num_states, label),
      time_constants(std::move(time_constants)),
      state_sigmas(std::move(state_sigmas)),
      rng(rng) {

	if (ValidationContext validation{}) {
		validation.add_matrix(this->time_constants, "time_constants")
		    .dim(num_states, 1)
		    .add_matrix(this->state_sigmas, "state_sigmas")
		    .dim(num_states, 1)
		    .validate();
	}
}

SampledFogmBlock::SampledFogmBlock(
    const std::string& label,
    double time_constant,
    double state_sigma,
    size_t num_states,
    navtk::not_null<std::shared_ptr<navtk::experimental::RandomNumberGenerator>> rng)
    : SampledFogmBlock(label,
                       zeros(num_states) + time_constant,
                       zeros(num_states) + state_sigma,
                       num_states,
                       rng) {}

not_null<std::shared_ptr<StateBlock<>>> SampledFogmBlock::clone() {
	return std::make_shared<SampledFogmBlock>(*this);
}

StandardDynamicsModel SampledFogmBlock::generate_dynamics(GenXhatPFunction,
                                                          aspn_xtensor::TypeTimestamp time_from,
                                                          aspn_xtensor::TypeTimestamp time_to) {

	auto dt = (time_to.get_elapsed_nsec() - time_from.get_elapsed_nsec()) * 1e-9;

	Vector input_sigmas =
	    xt::eval(state_sigmas * xt::sqrt(1 - xt::exp(-2.0 * dt / time_constants)));
	Vector decays = xt::eval(xt::exp(-dt / time_constants));

	auto g = [this, input_sigmas, decays](const Vector& x) {
		return xt::eval(decays * x + input_sigmas * this->rng->rand_n(this->num_states));
	};

	return StandardDynamicsModel(g, eye(1), zeros(1, 1));
}
}  // namespace experimental
}  // namespace filtering
}  // namespace navtk
