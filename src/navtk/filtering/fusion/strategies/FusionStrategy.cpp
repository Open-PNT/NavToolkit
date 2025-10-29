#include <navtk/filtering/fusion/strategies/FusionStrategy.hpp>

#include <algorithm>
#include <sstream>

#include <spdlog/spdlog.h>
#include <xtensor/generators/xbuilder.hpp>
#include <xtensor/io/xio.hpp>

#include <navtk/errors.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/containers/LinearizedStrategyBase.hpp>
#include <navtk/inspect.hpp>
#include <navtk/utils/ValidationContext.hpp>
#include <navtk/utils/human_readable.hpp>

using navtk::utils::ValidationContext;

namespace navtk {
namespace filtering {

namespace {

template <typename T>
void check_assign(const FusionStrategy& model,
                  const T& incoming,
                  Size (*dim_getter)(const T&),
                  Size first_index,
                  char const* what,
                  char const* where) {
	auto error_mode = get_global_error_mode();
	if (first_index + dim_getter(incoming) > model.get_num_states())
		log_or_throw(
		    error_mode,
		    "Trying to assign {0} beyond the end of the {1}. The {1} has {2} states, but you have "
		    "attempted to set {0} {3} thru {4} with:\n{5}",
		    what,
		    where,
		    model.get_num_states(),
		    first_index,
		    (first_index + dim_getter(incoming) - 1),
		    incoming);
}

template <typename T>
void check_transpose_safety(const T& input, Size x1, Size y1, Size w, Size h) {
	auto error_mode = get_global_error_mode();
	auto y2         = y1 + h - 1;
	auto x2         = x1 + w - 1;
	if (y2 >= x1 && y1 <= x2)
		log_or_throw<std::invalid_argument>(error_mode,
		                                    "Trying to assign {} x {} covariance slice {} at "
		                                    "({}, {}) - ({}, {}) would overlap its own transpose.",
		                                    h,
		                                    w,
		                                    navtk::utils::repr(input),
		                                    x1,
		                                    y1,
		                                    x2,
		                                    y2);
}

}  // namespace


Size FusionStrategy::get_num_states() const { return num_rows(std::move(get_estimate())); }


Size FusionStrategy::on_fusion_engine_state_block_added(Size how_many) {
	return this->on_fusion_engine_state_block_added(zeros(how_many), eye(how_many));
}


Size FusionStrategy::on_fusion_engine_state_block_added(Vector const& initial_estimate,
                                                        Matrix const& initial_covariance) {
	auto state_count = this->get_num_states();
	if (ValidationContext validation{}) {
		validation.add_matrix(initial_estimate, "initial_estimate")
		    .dim('N', 1)
		    .add_matrix(initial_covariance, "initial_covariance")
		    .dim('N', 'N')
		    .validate();
	}
	this->on_fusion_engine_state_block_added_impl(initial_estimate, initial_covariance);
	this->on_state_count_changed();
	return state_count;
}


Size FusionStrategy::on_fusion_engine_state_block_added(Vector const& initial_estimate,
                                                        Matrix const& initial_covariance,
                                                        Matrix const& cross_covariance) {
	auto state_count = this->get_num_states();
	if (ValidationContext validation{}) {
		validation.add_matrix(initial_estimate, "initial_estimate")
		    .dim('N', 1)
		    .add_matrix(initial_covariance, "initial_covariance")
		    .dim('N', 'N')
		    .add_matrix(cross_covariance, "cross_covariance")
		    .dim(state_count, 'N')
		    .validate();
	}
	this->on_fusion_engine_state_block_added(initial_estimate, initial_covariance);
	this->set_covariance_slice_impl(cross_covariance, 0, state_count);
	this->set_covariance_slice_impl(xt::transpose(cross_covariance), state_count, 0);
	this->on_state_count_changed();
	return state_count;
}


void FusionStrategy::on_fusion_engine_state_block_removed(Size first_index, Size count) {
	auto old_size = this->get_num_states();
	if (!count) return;
	if (old_size < count) {
		log_or_throw<std::invalid_argument>("Trying to remove more states than exist.");
		return;
	}
	if (old_size < first_index + count) {
		log_or_throw<std::invalid_argument>("Invalid state indices passed to remove_states");
		return;
	}
	this->on_fusion_engine_state_block_removed_impl(first_index, count);
	this->on_state_count_changed();
}


void FusionStrategy::set_estimate_slice(Vector const& new_estimate, Size first_index) {
	check_assign(*this, new_estimate, num_rows, first_index, "states", "state vector");
	this->set_estimate_slice_impl(new_estimate, first_index);
}


void FusionStrategy::set_covariance_slice(Matrix const& new_covariance, Size first_row) {
	set_covariance_slice(new_covariance, first_row, first_row);
}


void FusionStrategy::set_covariance_slice(Matrix const& new_covariance,
                                          Size first_row,
                                          Size first_col) {
	check_assign(*this, new_covariance, num_rows, first_row, "covariance", "covariance matrix");
	check_assign(*this, new_covariance, num_cols, first_col, "covariance", "covariance matrix");
	auto rows = num_rows(new_covariance);
	auto cols = num_cols(new_covariance);
	bool on_diagonal(first_row == first_col && rows == cols);
	if (!on_diagonal) {
		// Check for an off-diagonal that would overwrite itself.
		check_transpose_safety(new_covariance, first_col, first_row, cols, rows);
		this->set_covariance_slice_impl(xt::transpose(new_covariance), first_col, first_row);
	} else {
		if (ValidationContext validation{}) {
			validation.add_matrix(new_covariance, "new_covariance").symmetric().validate();
		}
	}
	this->set_covariance_slice_impl(new_covariance, first_row, first_col);
}


void FusionStrategy::on_state_count_changed() {}

void FusionStrategy::symmetricize_covariance(double rtol, double atol) {
	auto P = get_covariance();

	if (!is_symmetric(P, rtol, atol)) {
		P = (P + xt::transpose(P)) / 2;
		set_covariance_slice(std::move(P), 0);
	}
}


}  // namespace filtering
}  // namespace navtk
