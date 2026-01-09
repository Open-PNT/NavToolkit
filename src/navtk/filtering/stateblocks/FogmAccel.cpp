#include <navtk/filtering/stateblocks/FogmAccel.hpp>

#include <navtk/factory.hpp>

namespace navtk {
namespace filtering {

FogmAccel::FogmAccel(const std::string& label,
                     Vector time_constants,
                     Vector state_sigmas,
                     size_t num_dimensions,
                     DiscretizationStrategy discretization_strategy)
    : FogmBlock(label,
                std::move(time_constants),
                std::move(state_sigmas),
                num_dimensions * 3,
                std::move(discretization_strategy)) {}

FogmAccel::FogmAccel(const std::string& label,
                     double time_constant,
                     double state_sigma,
                     size_t num_dimensions,
                     DiscretizationStrategy discretization_strategy)
    : FogmBlock(label,
                time_constant,
                state_sigma,
                num_dimensions * 3,
                std::move(discretization_strategy)) {}

void FogmAccel::populate_f_and_q(Matrix& F, Matrix& Q) {
	auto n_dims = num_states / 3;

	Matrix ffogm = zeros(n_dims, n_dims);
	Matrix qfogm = zeros(n_dims, n_dims);

	FogmBlock::populate_f_and_q(ffogm, qfogm);

	auto first_block_idxs  = xt::range(_, n_dims);
	auto second_block_idxs = xt::range(n_dims, n_dims * 2);
	auto third_block_idxs  = xt::range(n_dims * 2, n_dims * 3);

	xt::view(F, first_block_idxs, second_block_idxs) = eye(n_dims);
	xt::view(F, second_block_idxs, third_block_idxs) = eye(n_dims);
	xt::view(F, third_block_idxs, third_block_idxs)  = ffogm;

	xt::view(Q, third_block_idxs, third_block_idxs) = qfogm;
}
}  // namespace filtering
}  // namespace navtk
