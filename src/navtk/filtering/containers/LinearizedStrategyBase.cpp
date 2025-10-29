#include <navtk/filtering/containers/LinearizedStrategyBase.hpp>

#include <memory>

#include <xtensor/generators/xbuilder.hpp>

#include <navtk/factory.hpp>
#include <navtk/inspect.hpp>
#include <navtk/transform.hpp>
#include <navtk/utils/ValidationContext.hpp>

using navtk::utils::ValidationContext;

namespace navtk {
namespace filtering {

LinearizedStrategyBase::LinearizedStrategyBase(const FusionStrategy& src)
    : estimate(src.get_estimate()), covariance(src.get_covariance()) {}

Vector LinearizedStrategyBase::get_estimate() const { return estimate; }

void LinearizedStrategyBase::set_estimate_slice_impl(Vector const& estimate_in, Size first_index) {
	auto rows = num_rows(estimate_in);
	if (first_index || rows != num_rows(estimate))
		xt::view(estimate, xt::range(first_index, first_index + rows)) = estimate_in;
	else
		estimate = estimate_in;
}

Matrix LinearizedStrategyBase::get_covariance() const { return covariance; }

void LinearizedStrategyBase::on_fusion_engine_state_block_added_impl(
    Vector const& initial_estimate, Matrix const& initial_covariance) {
	estimate   = xt::concatenate(xt::xtuple(estimate, initial_estimate), 0);
	covariance = block_diag(covariance, initial_covariance);
}

void LinearizedStrategyBase::set_covariance_slice_impl(Matrix const& covariance_in,
                                                       Size first_row,
                                                       Size first_col) {
	auto rows = num_rows(covariance_in);
	auto cols = num_cols(covariance_in);
	if (first_row || first_col || rows != num_rows(covariance) || cols != num_cols(covariance)) {
		auto r                     = xt::range(first_row, first_row + rows);
		auto c                     = xt::range(first_col, first_col + cols);
		xt::view(covariance, r, c) = covariance_in;
	} else
		covariance = covariance_in;
}


void LinearizedStrategyBase::on_fusion_engine_state_block_removed_impl(Size first_index,
                                                                       Size count) {
	if (first_index || count < this->get_num_states()) {
		auto dr    = drop_range(first_index, first_index + count);
		estimate   = xt::view(estimate, dr);
		covariance = xt::view(covariance, dr, dr);
	} else {
		estimate   = Vector{};
		covariance = Matrix{};
	}
}


not_null<std::shared_ptr<FusionStrategy>> LinearizedStrategyBase::clone() const {
	return std::make_shared<LinearizedStrategyBase>(*this);
}

}  // namespace filtering
}  // namespace navtk
