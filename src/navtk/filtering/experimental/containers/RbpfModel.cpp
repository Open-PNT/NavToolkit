#include <navtk/filtering/experimental/containers/RbpfModel.hpp>

#include <algorithm>
#include <memory>
#include <numeric>
#include <tuple>

#include <xtensor/core/xmath.hpp>
#include <xtensor/generators/xbuilder.hpp>
#include <xtensor/misc/xpad.hpp>

#include <navtk/errors.hpp>
#include <navtk/experimental/random.hpp>
#include <navtk/filtering/experimental/resampling.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/transform.hpp>

using navtk::Matrix;
using navtk::to_matrix;
using navtk::to_vec;
using navtk::Vector;
using navtk::experimental::rand_n;
using xt::all;
using xt::concatenate;
using xt::diagonal;
using xt::drop;
using xt::index_view;
using xt::keep;
using xt::newaxis;
using xt::range;
using xt::sum;
using xt::tile;
using xt::transpose;
using xt::view;
using xt::xtuple;
using xt::zeros;

namespace navtk {
namespace filtering {
namespace experimental {

constexpr Size RbpfModel::DEFAULT_PARTICLE_COUNT;
constexpr Size ONE = 1;

RbpfModel::RbpfModel(Size particle_count, bool calc_single_jacobian)
    : calc_single_jacobian(calc_single_jacobian),
      jitter_levels(0, 0.),
      is_nonlinear(0, false),
      state_particles(zeros(0, particle_count)),
      state_particle_weights(fill(1.0 / particle_count, particle_count)),
      covariance_particles(zeros(particle_count, 0, 0)),
      particle_count_target(particle_count) {
	if (particle_count < 1)
		log_or_throw<std::invalid_argument>("Number of particles must be greater than zero");
}

Size RbpfModel::count_particles() const { return num_rows(state_particle_weights); }

Size RbpfModel::get_particle_count_target() const { return particle_count_target; }

void RbpfModel::set_particle_count_target(Size new_value) {

	auto particle_count = count_particles();

	if (particle_count <= 1 || particle_count == DEFAULT_PARTICLE_COUNT) {
		particle_count_target = new_value;
		this->on_state_marks_changed();
	} else {
		log_or_throw("Exception Occurred: Particle count changed after initialization.");
	}
}
std::vector<double> RbpfModel::get_jitter_scaling() const { return jitter_levels; }

void RbpfModel::set_jitter_scaling(const std::vector<double>& jitter_scales) {
	RbpfModel::StateIndices marked_states = get_marked_states();
	if (validate_jitter_values(marked_states, jitter_scales)) {
		decltype(jitter_levels) new_jitter(is_nonlinear.size(), 0.0);

		// if only one jitter value provided in jitter_scales, set all jitter to the single value
		size_t c = 0;
		for (size_t i = 0; i < is_nonlinear.size(); i++) {
			if (is_nonlinear[i]) {
				new_jitter[i] = jitter_scales[c];
				if (jitter_scales.size() > 1) c++;
			}
		}
		jitter_levels = new_jitter;
	}
}

bool RbpfModel::validate_jitter_values(const RbpfModel::StateIndices& marked_states,
                                       const std::vector<double>& jitter_scales) {
	bool valid_flag = true;
	if (!(jitter_scales.size() == 1 && marked_states.size() > 0) &&
	    jitter_scales.size() != marked_states.size()) {
		log_or_throw<std::invalid_argument>(
		    "There must either be only one jitter value for all states, or as many jitter values "
		    "as marked "
		    "states.");
		valid_flag = false;
	} else {
		if (marked_states.size() == 0) {
			log_or_throw<std::invalid_argument>("Must have at least one marked state.");
			valid_flag = false;
		}
	}
	return valid_flag;
}

void RbpfModel::set_marked_states(const RbpfModel::StateIndices& marked_states,
                                  const std::vector<double>& jitter_scales) {
	auto num_states = is_nonlinear.size();
	decltype(jitter_levels) new_jitter(num_states, 0.0);

	Vector jitter_values = zeros(num_states);

	if (jitter_scales.size() == 1 && marked_states.size() > 0) {
		// set all jitter to single value
		jitter_values += jitter_scales[0];
	} else {
		if (validate_jitter_values(marked_states, jitter_scales)) {
			size_t c = 0;
			for (auto idx : marked_states) {
				jitter_values[idx] = jitter_scales[c];
				c++;
			}
		}
	}

	// Create new_marks vector where each index is true if and only if that index number appears in
	// marked_states.
	decltype(is_nonlinear) new_marks(num_states, false);
	for (auto idx : marked_states) {
		new_marks.at(idx)  = true;
		new_jitter.at(idx) = jitter_values[idx];
	}
	jitter_levels = new_jitter;

	// Check whether the state marks have changed and exit the function if not.
	bool any_changes = false;
	for (decltype(num_states) idx(0); idx < num_states; ++idx) {
		if (is_nonlinear[idx] != new_marks[idx]) {
			any_changes = true;
			break;
		}
	}
	if (!any_changes) return;

	// At this point, we know the marks have changed. Those that have become particles need noise
	// introduced. Those that are no longer particles need the noise removed.
	Vector estimate   = get_estimate();
	Matrix covariance = get_covariance();
	StateIndices marks_added;
	for (decltype(num_states) idx(0); idx < num_states; ++idx) {
		if (is_nonlinear[idx] && !new_marks[idx]) {
			// removed a mark, so set all the particles to the mean position
			view(state_particles, idx, all()) = estimate(idx);
		}
	}

	is_nonlinear = new_marks;

	update_state_mark_helper_variables();

	// Update covariance particles so it still matches the size of the linear states.
	if (any_linear()) {
		covariance_particles =
		    tile(view(covariance, newaxis(), keep_linear, keep_linear), particle_count_target);
	} else {
		covariance_particles = Tensor<3>();
	}

	this->on_state_marks_changed();
}

RbpfModel::StateIndices RbpfModel::get_marked_states() const { return nonlinear_states; }

Size RbpfModel::get_num_states() const { return is_nonlinear.size(); }

void RbpfModel::on_fusion_engine_state_block_added_impl(Vector const& initial_estimate,
                                                        Matrix const& initial_covariance) {
	Size starting_state_count = this->get_num_states();
	Size how_many             = num_rows(initial_estimate);
	Size final_state_count    = starting_state_count + how_many;
	Size particle_count       = count_particles();

	// If nothing is being added, no changes need to be made
	if (how_many == 0) return;

	// Default the new states to "unmarked"
	is_nonlinear.resize(final_state_count, false);

	// Default the new states to "no jitter"
	jitter_levels.resize(final_state_count, 0.0);

	// do not allow tiling of zero size array
	if (particle_count == 0) {
		particle_count         = 1;
		state_particle_weights = ones(1);
	}
	// Initialize new state particles to zero.
	auto new_particles = tile(view(initial_estimate, all(), newaxis()), {ONE, particle_count});
	// Make particle_count
	Tensor<3> p0_tiles;
	if (any_linear()) {
		p0_tiles =
		    tile(view(initial_covariance, newaxis(), all(), all()), {particle_count, ONE, ONE});
	} else {
		p0_tiles = Tensor<3>();
	}
	// If we're starting from zero states, trying to concatenate will cause odd overflow errors from
	// xtensor, so short-circuit that case
	if (!starting_state_count) {
		state_particles      = std::move(new_particles);
		covariance_particles = p0_tiles;
		covariance           = initial_covariance;
		estimate             = initial_estimate;
		return;
	}
	// Expand state_particles by how_many x particle_count
	state_particles = concatenate(xtuple(state_particles, new_particles), 0);
	estimate        = concatenate(xtuple(estimate, initial_estimate));
	// Update covariance
	covariance = block_diag(covariance, initial_covariance);
	// Initialize new covariance particles by replicating the identity matrix
	auto num_linear_states = linear_states.size();
	auto new_cov_particles = tile(view(eye(num_linear_states + how_many), newaxis(), all(), all()),
	                              {particle_count, ONE, ONE});
	// Overwrite the new covariance particle array with the existing particles.
	if (num_linear_states > 0) {
		view(new_cov_particles, all(), range(0, num_linear_states), range(0, num_linear_states)) =
		    std::move(covariance_particles);
		// Overwrite the bottom-right corners of each covariance particle with the newly-created
		// tiles.
		auto new_range                                       = range(num_linear_states, _);
		view(new_cov_particles, all(), new_range, new_range) = std::move(p0_tiles);
	}
	// Set covariance_particles member to the newly-computed value
	covariance_particles = new_cov_particles;
}

void RbpfModel::on_fusion_engine_state_block_removed_impl(Size first_index, Size count) {
	auto dr         = drop_range(first_index, first_index + count);
	state_particles = view(state_particles, dr, all());

	// Remove linear states in drop range from covariance particles.
	auto relative_indices = get_relative_linear_indices(first_index, count);
	if (relative_indices.size() > 0)
		covariance_particles =
		    view(covariance_particles, all(), drop(relative_indices), drop(relative_indices));

	estimate   = view(estimate, dr);
	covariance = view(covariance, dr, dr);

	is_nonlinear.erase(is_nonlinear.begin() + first_index,
	                   is_nonlinear.begin() + first_index + count);
	jitter_levels.erase(jitter_levels.begin() + first_index,
	                    jitter_levels.begin() + first_index + count);
}

Vector RbpfModel::get_estimate() const { return this->estimate; }

void RbpfModel::set_estimate_slice_impl(Vector const& slice, Size first_index) {

	if (slice.size() == 0) return;

	// Non-inclusive end index, compatible with xt::range()
	Size slice_rows = num_rows(slice);
	Size stop_index = slice_rows + first_index;

	view(this->estimate, range(first_index, stop_index)) = slice;
	auto particle_count                                  = count_particles();

	// Update states by setting all to the same value, then adding gaussian noise
	view(state_particles, range(first_index, stop_index), all()) =
	    tile(view(slice, all(), newaxis()), {ONE, particle_count});

	apply_state_particle_noise(first_index, stop_index);

	// Reset weights
	state_particle_weights = fill(1.0 / particle_count, particle_count);
}

Matrix RbpfModel::get_covariance() const {
	// If propagation or update has occurred and covariance has not been calculated since, then
	// calculate.
	if (covariance_stale) {
		auto num_states       = this->get_num_states();
		Matrix new_covariance = zeros(num_states, num_states);

		if (any_nonlinear()) {
			new_covariance = calc_cov_weighted(state_particles, state_particle_weights);
		}

		if (any_linear()) {
			// calculate covariance linear estimate
			auto particle_count = count_particles();

			double sum_weights = sum(state_particle_weights)[0];

			Vector estimate_new = estimate;

			// calculate covariance linear estimate
			Matrix linear_covariance;

			if (calc_single_jacobian) {
				linear_covariance = view(this->covariance_particles, 0, all(), all());
			} else {
				linear_covariance = zeros(linear_states.size(), linear_states.size());
				for (Size i = 0; i < particle_count; i++) {
					Matrix covariance_sum = view(this->covariance_particles, i, all(), all());

					linear_covariance +=
					    (this->state_particle_weights(i) / sum_weights) * covariance_sum;
				}
			}

			view(new_covariance, keep_linear, keep_linear) = linear_covariance;
		}
		this->covariance = symmetricize_covariance(new_covariance);
		covariance_stale = false;
	}
	return this->covariance;
}

not_null<std::shared_ptr<FusionStrategy>> RbpfModel::clone() const {
	return std::make_shared<RbpfModel>(*this);
}

bool RbpfModel::any_nonlinear() const { return !nonlinear_states.empty(); }

bool RbpfModel::any_linear() const { return !linear_states.empty(); }

void RbpfModel::symmetricize_covariance() {
	Matrix cov = get_covariance();
	covariance = symmetricize_covariance(cov);
}

Matrix RbpfModel::symmetricize_covariance(Matrix& temp_covariance) const {
	if (!is_symmetric(temp_covariance)) {
		temp_covariance = (temp_covariance + transpose(temp_covariance)) / 2;
	}
	return temp_covariance;
}

void RbpfModel::set_covariance_slice_impl(Matrix const& new_covariance,
                                          Size first_row,
                                          Size first_col) {

	if (new_covariance.size() == 0) return;

	auto row_range                         = range(first_row, first_row + num_rows(new_covariance));
	auto col_range                         = range(first_col, first_col + num_cols(new_covariance));
	view(covariance, row_range, col_range) = new_covariance;
	if (first_row != first_col) {
		view(covariance, col_range, row_range) = transpose(new_covariance);
	} else {
		if (any_nonlinear() && num_rows(new_covariance) > 0) {
			Size stop_row = first_row + num_rows(new_covariance);
			auto index    = range(first_row, stop_row);
			for (auto idx = count_particles(); idx > 0; idx--) {
				view(state_particles, index, idx - 1) = view(estimate, index);
			}
			apply_state_particle_noise(first_row, stop_row);
		}
	}
	if (any_linear()) {
		Size goal            = any_nonlinear() ? particle_count_target : 1;
		covariance_particles = tile(view(covariance, newaxis(), keep_linear, keep_linear), goal);
	}
}

void RbpfModel::on_state_count_changed() {
	FusionStrategy::on_state_count_changed();
	this->on_state_marks_changed();
}

void RbpfModel::on_state_marks_changed() {
	update_state_mark_helper_variables();
	update_particle_count();
}

void RbpfModel::update_particle_count() {
	Size goal = any_nonlinear() ? particle_count_target : 1;
	Size cols = num_cols(state_particles);

	if (cols != goal) {
		ResamplingResult res;

		if (goal != particle_count_target && goal != 1) {
			log_or_throw("Exception Occurred: Particle count has changed unexpectedly.");
		}

		std::vector<size_t> index;
		res = systematic_resampling(state_particle_weights, &goal);

		// update weights via resampling
		state_particle_weights = index_view(state_particle_weights, res.index);

		double sum_weights = sum(state_particle_weights)[0];

		if (sum_weights > 0) state_particle_weights /= sum_weights;

		// update particle state and covariance information via resampling
		Matrix state_particles_copy         = state_particles;
		Tensor<3> covariance_particles_copy = covariance_particles;

		// apply resampling of particles
		auto num_states = this->get_num_states();
		state_particles = zeros(num_states, goal);

		covariance_particles = zeros(goal, this->linear_states.size(), this->linear_states.size());
		if (num_states > 0) {
			if (any_nonlinear()) {
				for (decltype(goal) i = 0; i < goal; i++) {
					view(state_particles, all(), i) = this->estimate;
					view(covariance_particles, i, all(), all()) =
					    view(covariance_particles_copy, res.index[i], all(), all());
				}
				apply_state_particle_noise(0, num_rows(state_particles));
			} else {
				for (decltype(goal) i = 0; i < goal; i++) {
					view(state_particles, all(), i) =
					    view(state_particles_copy, all(), res.index[i]);
					view(covariance_particles, i, all(), all()) =
					    view(covariance_particles_copy, res.index[i], all(), all());
				}
			}
		}
	}
}

void RbpfModel::update_state_mark_helper_variables() {
	Size idx = 0;
	nonlinear_states.clear();
	linear_states.clear();
	for (bool mark : this->is_nonlinear) {
		(mark ? nonlinear_states : linear_states).push_back(idx++);
	}

	drop_linear = drop(linear_states);
	keep_linear = keep(linear_states);
}

Vector RbpfModel::compute_state_noise_weights() const {
	if (num_rows(covariance) > 1) return diagonal(chol(std::move(covariance)));
	if (num_rows(covariance) == 1) return to_vec(sqrt(covariance));
	return {1};
}

void RbpfModel::apply_state_particle_noise(Size first_index, Size stop_index) {
	auto particle_count = count_particles();
	if (!has_zero_size(state_particles) && particle_count > 1 && stop_index > first_index) {
		auto ind_range = range(first_index, stop_index);

		Matrix chol_m = chol(view(covariance, ind_range, ind_range));

		Matrix rng = rand_n(num_rows(chol_m), particle_count);

		for (Size ii = 0; ii < particle_count; ii++) {
			view(state_particles, ind_range, ii) +=
			    to_vec(dot(chol_m, to_matrix(view(rng, all(), ii), 1)));
		}
	}
}

RbpfModel::StateIndices RbpfModel::get_relative_linear_indices(Size first_index, Size count) const {
	if (linear_states.size() == 0) return RbpfModel::StateIndices();

	// Create a vector with the indices [first_index, first_index+count)
	StateIndices states(count);
	std::iota(std::begin(states), std::end(states), first_index);

	// Create a vector of the intersection between the linear states and the requested states
	StateIndices intersected_states;
	std::set_intersection(states.begin(),
	                      states.end(),
	                      linear_states.begin(),
	                      linear_states.end(),
	                      back_inserter(intersected_states));
	if (intersected_states.size() == 0) return intersected_states;

	// Map the absolute intersected states to their equivalent relative states
	Size relative_linear_idx    = 0;
	Size intersected_states_idx = 0;
	StateIndices absolute_to_relative_linear_states;
	for (Size idx = 0; idx < get_num_states(); ++idx) {
		if (!is_nonlinear[idx]) {
			if (idx >= first_index && idx < first_index + count) {
				intersected_states[intersected_states_idx] = relative_linear_idx;
				intersected_states_idx++;
			}
			relative_linear_idx++;
		}
	}
	return intersected_states;
}

}  // namespace experimental
}  // namespace filtering
}  // namespace navtk
