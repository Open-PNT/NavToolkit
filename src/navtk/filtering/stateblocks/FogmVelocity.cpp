#include <navtk/filtering/stateblocks/FogmVelocity.hpp>

#include <navtk/factory.hpp>

namespace navtk {
namespace filtering {

FogmVelocity::FogmVelocity(const std::string& label,
                           Vector time_constants,
                           Vector state_sigmas,
                           size_t num_dimensions,
                           DiscretizationStrategy discretization_strategy)
    : FogmBlock(label,
                std::move(time_constants),
                std::move(state_sigmas),
                num_dimensions * 2,
                std::move(discretization_strategy)) {}

FogmVelocity::FogmVelocity(const std::string& label,
                           double time_constant,
                           double state_sigma,
                           size_t num_dimensions,
                           DiscretizationStrategy discretization_strategy)
    : FogmBlock(label,
                time_constant,
                state_sigma,
                num_dimensions * 2,
                std::move(discretization_strategy)) {}

void FogmVelocity::populate_f_and_q(Matrix& F, Matrix& Q) {
	auto n_dims = num_states / 2;

	Matrix ffogm = zeros(n_dims, n_dims);
	Matrix qfogm = zeros(n_dims, n_dims);

	FogmBlock::populate_f_and_q(ffogm, qfogm);

	auto first_block_idxs  = xt::range(_, n_dims);
	auto second_block_idxs = xt::range(n_dims, n_dims * 2);

	xt::view(F, first_block_idxs, second_block_idxs)  = eye(n_dims);
	xt::view(F, second_block_idxs, second_block_idxs) = ffogm;

	xt::view(Q, second_block_idxs, second_block_idxs) = qfogm;
}

}  // namespace filtering
}  // namespace navtk
