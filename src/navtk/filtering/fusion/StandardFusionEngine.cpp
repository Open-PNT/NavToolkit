#include <navtk/filtering/fusion/StandardFusionEngine.hpp>

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>

#include <spdlog/spdlog.h>
#include <xtensor/views/xview.hpp>

#include <navtk/errors.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/get_time.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/ValidationContext.hpp>

using navtk::utils::ValidationContext;
using navtk::utils::ValidationResult;
using std::pair;
using std::string;
using std::vector;
using xt::newaxis;
using xt::range;
using xt::view;


namespace navtk {
namespace filtering {

std::invalid_argument bad_block(string const &label);
std::invalid_argument bad_processor(string const &label);
std::invalid_argument bad_vsb(string const &label);

StandardFusionEngine::StandardFusionEngine(
    const aspn_xtensor::TypeTimestamp &cur_time,
    not_null<std::shared_ptr<StandardModelStrategy>> strategy)
    : cur_time(cur_time), strategy(std::move(strategy)) {
	auto num_states = this->strategy->get_num_states();
	if (this->strategy->get_num_states() > 0) {
		this->strategy->on_fusion_engine_state_block_removed(0, num_states);
		spdlog::warn(
		    "The strategy passed into the fusion engine already contains states. All states should "
		    "be added by adding state blocks to the fusion engine. The current strategy states "
		    "have now been removed.");
	}
}

StandardFusionEngine::StandardFusionEngine(const aspn_xtensor::TypeTimestamp &cur_time)
    : StandardFusionEngine(cur_time, std::make_shared<EkfStrategy>()) {}

StandardFusionEngine::StandardFusionEngine(const StandardFusionEngine &other)
    : StandardFusionEngine() {
	cur_time = other.cur_time;
	strategy = std::dynamic_pointer_cast<StandardModelStrategy>(other.strategy->clone());
	for (auto block : other.blocks) {
		// todo (PNTOS-302) may break if add_state_block behaviors changes
		blocks.push_back(block->clone());
		set_state_block_estimate(block->get_label(), get_state_block_estimate(block->get_label()));
		set_state_block_covariance(block->get_label(),
		                           get_state_block_covariance(block->get_label()));
	}

	vsb_man = other.vsb_man;

	for (auto processor : other.processors) {
		add_measurement_processor(processor->clone());
	}

	for (auto const &it : other.process_covariance_cross_terms) {
		set_cross_term_process_covariance(it.label1, it.label2, it.term);
	}
}

StandardFusionEngine &StandardFusionEngine::operator=(const StandardFusionEngine &other) {
	if (this != &other) {
		*this = StandardFusionEngine(other);
	}
	return *this;
}

void StandardFusionEngine::set_time(const aspn_xtensor::TypeTimestamp &time) { cur_time = time; }

aspn_xtensor::TypeTimestamp StandardFusionEngine::get_time() const { return cur_time; }

vector<string> StandardFusionEngine::get_state_block_names_list() const {
	vector<string> out(blocks.size());
	std::transform(
	    blocks.begin(),
	    blocks.end(),
	    out.begin(),
	    [](not_null<std::shared_ptr<StateBlock<>>> const &m) -> string { return m->get_label(); });
	return out;
}

not_null<std::shared_ptr<const StateBlock<>>> StandardFusionEngine::get_state_block(
    string const &label) const {
	for (auto const &block : blocks)
		if (block->get_label() == label)
			return std::const_pointer_cast<const StateBlock<>>(block.get());
	throw bad_block(label);
}

not_null<std::shared_ptr<StateBlock<>>> StandardFusionEngine::get_state_block(string const &label) {
	for (auto const &block : blocks)
		if (block->get_label() == label) return block;
	throw bad_block(label);
}

Matrix StandardFusionEngine::get_state_block_covariance(string const &label) const {
	auto block_found = has_block(label);
	auto real_label  = block_found ? label : get_real_block_labels({label}).front();
	auto indices     = get_mat_indices(real_label);
	auto state_range = range(indices.first, indices.second);
	auto pre_cov     = view(strategy->get_covariance(), state_range, state_range);
	if (block_found) {
		return pre_cov;
	}
	auto pre_x = view(strategy->get_estimate(), state_range);
	auto jac   = vsb_man.jacobian(pre_x, real_label, label, cur_time);
	return dot(dot(jac, pre_cov), xt::transpose(jac));
}

Vector StandardFusionEngine::get_state_block_estimate(string const &label) const {
	auto block_found = has_block(label);
	auto real_label  = block_found ? label : get_real_block_labels({label}).front();
	auto indices     = get_mat_indices(real_label);
	auto pre_x       = view(strategy->get_estimate(), range(indices.first, indices.second));
	if (block_found) {
		return pre_x;
	}
	return vsb_man.convert_estimate(pre_x, real_label, label, cur_time);
}

EstimateWithCovariance StandardFusionEngine::get_state_block_est_and_cov(
    string const &label) const {
	auto block_found = has_block(label);
	auto real_label  = block_found ? label : get_real_block_labels({label}).front();
	auto real_ec     = EstimateWithCovariance(get_state_block_estimate(real_label),
                                          get_state_block_covariance(real_label));
	return block_found ? real_ec : vsb_man.convert(real_ec, real_label, label, cur_time);
}

void StandardFusionEngine::add_state_block(not_null<std::shared_ptr<StateBlock<>>> block) {
	strategy->on_fusion_engine_state_block_added(block->get_num_states());
	blocks.push_back(block);
	clear_cache();
}

void StandardFusionEngine::set_state_block_covariance(string const &label,
                                                      Matrix const &covariance) {
	auto indices = get_mat_indices(label);

	if (ValidationContext validation{}) {
		auto num_states = get_state_block(label)->get_num_states();
		validation.add_matrix(covariance, "covariance").dim(num_states, num_states).validate();
	}

	strategy->set_covariance_slice(covariance, indices.first, indices.first);
	clear_cache();
}

void StandardFusionEngine::set_cross_term_process_covariance(string const &label1,
                                                             string const &label2,
                                                             Matrix const &block) {
	if (ValidationContext validation{}) {
		auto num_states1 = get_state_block(label1)->get_num_states();
		auto num_states2 = get_state_block(label2)->get_num_states();

		auto vr = validation.add_matrix(block, "newQd").dim(num_states1, num_states2).validate();
		if (ValidationResult::BAD == vr) return;
	}

	process_covariance_cross_terms.erase(
	    std::remove_if(process_covariance_cross_terms.begin(),
	                   process_covariance_cross_terms.end(),
	                   [&](ProcessCovarianceCrossTerm const &t) -> bool {
		                   return t.label1 == label1 && t.label2 == label2;
	                   }),
	    process_covariance_cross_terms.end());
	process_covariance_cross_terms.push_back({label1, label2, block});
}

void StandardFusionEngine::set_cross_term_covariance(string const &label1,
                                                     string const &label2,
                                                     Matrix const &block) {
	auto a = get_mat_indices(label1);
	auto b = get_mat_indices(label2);

	if (ValidationResult::BAD == ValidationContext{}
	                                 .add_matrix(block, "block")
	                                 .dim(a.second - a.first, b.second - b.first)
	                                 .validate())
		return;

	strategy->set_covariance_slice(block, a.first, b.first);
	strategy->set_covariance_slice(xt::transpose(block), b.first, a.first);
	clear_cache();
}

Matrix StandardFusionEngine::get_cross_term_covariance(string const &label1,
                                                       string const &label2) const {

	auto block1_found = has_block(label1);
	auto block2_found = has_block(label2);
	auto real_label1  = block1_found ? label1 : get_real_block_labels({label1}).front();
	auto real_label2  = block2_found ? label2 : get_real_block_labels({label2}).front();
	auto indices1     = get_mat_indices(real_label1);
	auto indices2     = get_mat_indices(real_label2);
	auto state_range1 = range(indices1.first, indices1.second);
	auto state_range2 = range(indices2.first, indices2.second);
	auto pre_cov      = view(strategy->get_covariance(), state_range1, state_range2);

	if (block1_found && block2_found) {
		return pre_cov;
	}

	// Could be more efficient for cases where retrieving the estimate is expensive.  Possibly:
	// auto jac1 = block1_found ? eye(indices1.second - indices1.first) :
	// vsb_man.jacobian(view(strategy->get_estimate(), state_range1), real_label1, label1,
	// cur_time);

	auto pre_x1 = view(strategy->get_estimate(), state_range1);
	auto pre_x2 = view(strategy->get_estimate(), state_range2);
	auto jac1   = vsb_man.jacobian(pre_x1, real_label1, label1, cur_time);
	auto jac2   = vsb_man.jacobian(pre_x2, real_label2, label2, cur_time);
	return dot(dot(jac1, pre_cov), xt::transpose(jac2));
}

void StandardFusionEngine::set_state_block_estimate(string const &label, Vector const &estimate) {
	auto indices = get_mat_indices(label);

	if (ValidationContext validation{}) {
		auto num_states = get_state_block(label)->get_num_states();
		validation.add_matrix(estimate, "estimate").dim(num_states, 1).validate();
	}

	strategy->set_estimate_slice(estimate, indices.first);
	clear_cache();
}

// TODO PNTOS-246 Throw an error if label does not belong to any StateBlock
void StandardFusionEngine::remove_state_block(string const &label) {
	Size start_index = 0;
	for (auto itr = blocks.begin(); itr != blocks.end(); ++itr) {
		if ((*itr)->get_label() == label) {
			strategy->on_fusion_engine_state_block_removed(start_index, (*itr)->get_num_states());
			itr = blocks.erase(itr);
			if (itr == blocks.end()) break;
		} else
			start_index += (*itr)->get_num_states();
	}
	clear_cache();
}


void StandardFusionEngine::give_state_block_aux_data(string const &label,
                                                     AspnBaseVector const &data) {
	auto block = get_state_block(label);
	block->receive_aux_data(std::move(data));
}


void StandardFusionEngine::add_measurement_processor(
    not_null<std::shared_ptr<MeasurementProcessor<>>> processor) {
	processors.push_back(processor);
}

vector<string> StandardFusionEngine::get_measurement_processor_names_list() const {
	vector<string> out(processors.size());
	std::transform(processors.begin(), processors.end(), out.begin(), [](const auto &m) -> string {
		return m->get_label();
	});
	return out;
}

bool StandardFusionEngine::has_processor(string const &label) const {
	for (const auto &processor : processors)
		if (processor->get_label() == label) return true;
	return false;
}

not_null<std::shared_ptr<const MeasurementProcessor<>>>
StandardFusionEngine::get_measurement_processor(string const &label) const {
	for (auto const &proc : processors)
		if (proc->get_label() == label)
			return std::const_pointer_cast<const MeasurementProcessor<>>(proc.get());
	throw bad_processor(label);
}


not_null<std::shared_ptr<MeasurementProcessor<>>> StandardFusionEngine::get_measurement_processor(
    string const &label) {
	for (auto const &proc : processors)
		if (proc->get_label() == label) return proc;
	throw bad_processor(label);
}


// TODO PNTOS-246 Throw an error if label does not belong to any MeasurementProcessor
void StandardFusionEngine::remove_measurement_processor(string const &label) {
	processors.erase(
	    std::remove_if(processors.begin(),
	                   processors.end(),
	                   [&](not_null<std::shared_ptr<MeasurementProcessor<>>> const &it) -> bool {
		                   return it->get_label() == label;
	                   }),
	    processors.end());
}


void StandardFusionEngine::give_measurement_processor_aux_data(string const &label,
                                                               AspnBaseVector const &data) {
	auto proc = get_measurement_processor(label);
	proc->receive_aux_data(std::move(data));
}

std::shared_ptr<EstimateWithCovariance> StandardFusionEngine::peek_ahead(
    aspn_xtensor::TypeTimestamp time, std::vector<std::string> const &mixed_block_labels) const {

	if (mixed_block_labels.empty()) {
		spdlog::warn("peek_ahead with empty mixed_block_labels does nothing.");
		return nullptr;
	}

	StandardFusionEngine copy(*this);
	// Propagate over any remaining difference between last meas time and
	// requested output time
	copy.propagate(time);
	return copy.generate_x_and_p(mixed_block_labels);
}

EstimateWithCovariance StandardFusionEngine::reset_state_estimate(aspn_xtensor::TypeTimestamp time,
                                                                  string const &label,
                                                                  vector<size_t> const &indices) {
	if (time < cur_time) {
		log_or_throw<std::invalid_argument>(
		    "Cannot reset filter states in past. Filter at time {}, reset requested at {}",
		    cur_time,
		    time);
	}

	if (indices.empty())
		log_or_throw<std::invalid_argument>("Indices into state vector argument cannot be empty");

	auto cur_est = get_state_block_estimate(label);

	for (auto index : indices)
		if (index > num_rows(cur_est))
			log_or_throw<std::invalid_argument>(
			    "Given indices exceed the maximum state block index available.");

	propagate(time);

	cur_est        = get_state_block_estimate(label);
	auto cur_cov   = get_state_block_covariance(label);
	Vector est_out = zeros(num_rows(cur_est));

	for (auto index : indices) {
		est_out[index] = cur_est[index];
		cur_est[index] = 0;
	}
	set_state_block_estimate(label, cur_est);
	return {est_out, cur_cov};
}

vector<string> StandardFusionEngine::get_virtual_state_block_target_labels() const {
	return vsb_man.get_virtual_state_block_target_labels();
}

void StandardFusionEngine::add_virtual_state_block(not_null<std::shared_ptr<VirtualStateBlock>> v) {
	vsb_man.add_virtual_state_block(v);
}

void StandardFusionEngine::remove_virtual_state_block(std::string const &target) {
	vsb_man.remove_virtual_state_block(target);
}

void StandardFusionEngine::give_virtual_state_block_aux_data(std::string const &target_label,
                                                             AspnBaseVector const &data) {

	auto vsb = vsb_man.get_virtual_state_block(target_label);
	if (!vsb) {
		throw bad_vsb(target_label);
	}
	vsb->receive_aux_data(std::move(data));
}

size_t StandardFusionEngine::get_num_states() const {
	size_t out = 0;
	for (auto const &block : blocks) out += block->get_num_states();
	return out;
}

// protected

pair<size_t, size_t> StandardFusionEngine::get_mat_indices(string const &label) const {
	return get_mat_indices(find_block_idx_or_bail(label));
}

pair<size_t, size_t> StandardFusionEngine::get_mat_indices(size_t idx) const {
	size_t state_begin = 0, ii = 0;
	for (auto const &block : blocks) {
		if (ii == idx) return {state_begin, state_begin + block->get_num_states()};
		state_begin += block->get_num_states();
		++ii;
	}

	log_or_throw<std::invalid_argument>(
	    "No StateBlock numbered {} exists. There are only {} StateBlocks", idx, ii);
	return {};
}

vector<pair<size_t, size_t>> StandardFusionEngine::get_mat_indices_list() const {
	vector<pair<size_t, size_t>> out(blocks.size());
	size_t offset = 0;
	transform(blocks.begin(),
	          blocks.end(),
	          out.begin(),
	          [&](not_null<std::shared_ptr<StateBlock<>>> block) -> pair<size_t, size_t> {
		          size_t start = offset;
		          return {start, (offset += block->get_num_states())};
	          });
	return out;
}

size_t StandardFusionEngine::find_block_idx_or_bail(string const &label) const {
	for (size_t ii = 0; ii < blocks.size(); ++ii)
		if (blocks[ii]->get_label() == label) return ii;
	throw bad_block(label);
}

size_t StandardFusionEngine::find_processor_idx_or_bail(string const &label) const {
	for (size_t ii = 0; ii < processors.size(); ++ii)
		if (processors[ii]->get_label() == label) return ii;
	throw bad_processor(label);
}

bool StandardFusionEngine::has_block(string const &label) const {
	for (const auto &block : blocks)
		if (block->get_label() == label) return true;
	return false;
}

bool StandardFusionEngine::has_virtual_state_block(string const &target_label) const {

	auto vsb_link = vsb_man.get_start_block_label(target_label);

	if (vsb_link.first) {
		for (const auto &block : blocks)
			if (block->get_label() == vsb_link.second) return true;
	}
	return false;
}

vector<string> StandardFusionEngine::get_real_block_labels(
    vector<string> const &mixed_block_labels) const {
	vector<string> real_labels;
	for (auto i = mixed_block_labels.begin(); i != mixed_block_labels.end(); i++) {
		if (has_block(*i))
			real_labels.push_back(*i);
		else {
			auto label_result = vsb_man.get_start_block_label(*i);
			if (label_result.first)
				real_labels.push_back(label_result.second);
			else
				throw bad_vsb(*i);
		}
	}
	return real_labels;
}

std::function<Vector(Vector)> StandardFusionEngine::calc_full_transform(
    vector<string> const &mixed_block_labels) const {
	// Need a function that takes 'real' state vector and transforms it into 'virtual' vector
	auto labels = get_real_block_labels(mixed_block_labels);
	vector<int> sizes;
	for (size_t i = 0; i < labels.size(); i++) {
		sizes.push_back(get_state_block(labels[i])->get_num_states());
	}
	return
	    [&, labels = labels, sizes = sizes, mixed_block_labels = mixed_block_labels](Vector x_hat) {
		    vector<Vector> tx_x;
		    int out_size = 0;
		    int ind      = 0;

		    for (size_t i = 0; i < labels.size(); i++) {
			    auto real_x = view(x_hat, range(ind, ind + sizes[i]));
			    ind += sizes[i];
			    tx_x.emplace_back(
			        vsb_man.convert_estimate(real_x, labels[i], mixed_block_labels[i], cur_time));
			    out_size += num_rows(tx_x.back());
		    }

		    auto out = zeros(out_size);
		    ind      = 0;

		    for (auto x = tx_x.begin(); x != tx_x.end(); x++) {
			    auto sz                         = num_rows(*x);
			    view(out, range(ind, ind + sz)) = *x;
			    ind += sz;
		    }
		    return out;
	    };
}

Matrix StandardFusionEngine::calc_transform_jacobian(
    vector<string> const &mixed_block_labels) const {

	vector<Matrix> jacs;
	Size all_rows = 0;
	Size all_cols = 0;

	for (auto i = mixed_block_labels.begin(); i != mixed_block_labels.end(); i++) {
		if (has_block(*i)) {
			jacs.emplace_back(eye(get_state_block(*i)->get_num_states()));
		} else {
			auto real_label_result = vsb_man.get_start_block_label(*i);
			if (real_label_result.first) {
				auto start = get_state_block_estimate(real_label_result.second);
				jacs.emplace_back(vsb_man.jacobian(start, real_label_result.second, *i, cur_time));
			} else
				throw bad_vsb(*i);
		}
		all_rows += num_rows(jacs.back());
		all_cols += num_cols(jacs.back());
	}

	// build big Jacobian matrix; may not be square
	auto big_jac = zeros(all_rows, all_cols);
	Size col     = 0;
	Size row     = 0;
	for (auto jac = jacs.begin(); jac != jacs.end(); jac++) {
		auto x                                                  = num_cols(*jac);
		auto y                                                  = num_rows(*jac);
		view(big_jac, range(row, row + y), range(col, col + x)) = *jac;
		col += x;
		row += y;
	}
	return big_jac;
}

std::shared_ptr<EstimateWithCovariance> StandardFusionEngine::generate_x_and_p(
    vector<string> const &mixed_block_labels) const {

	if (mixed_block_labels.empty()) {
		spdlog::warn("No labels provided, nothing to do.");
		return nullptr;
	}

	if (mixed_block_labels == last_gen_xp_args) {
		return last_gen_xp_results;
	}

	auto mat_idx_list = get_mat_indices_list();
	if (mixed_block_labels.size() == 1)
		return std::make_shared<EstimateWithCovariance>(
		    get_state_block_est_and_cov(mixed_block_labels[0]));

	auto act_lab    = get_real_block_labels(mixed_block_labels);
	auto xt_keepers = xt::keep(get_all_state_indices(act_lab));
	auto xHat       = xt::view(strategy->get_estimate(), xt_keepers);
	auto P          = xt::view(strategy->get_covariance(), xt_keepers, xt_keepers);

	if (act_lab != mixed_block_labels) {
		last_tx             = calc_full_transform(mixed_block_labels);
		last_jac            = calc_transform_jacobian(mixed_block_labels);
		last_gen_xp_results = std::make_shared<EstimateWithCovariance>(
		    last_tx(xHat), dot(dot(last_jac, P), transpose(last_jac)));
	} else {
		last_gen_xp_results = std::make_shared<EstimateWithCovariance>(xHat, P);
	}

	last_gen_xp_args = mixed_block_labels;
	return last_gen_xp_results;
}

not_null<std::shared_ptr<StandardMeasurementModel>> StandardFusionEngine::expand_update_model(
    StandardMeasurementModel const &model, StandardMeasurementProcessor const &proc) {
	return expand_update_model(model, proc.get_state_block_labels());
}

not_null<std::shared_ptr<StandardMeasurementModel>> StandardFusionEngine::expand_update_model(
    StandardMeasurementModel const &model, vector<string> const &state_block_labels) {
	auto out = std::make_shared<StandardMeasurementModel>(model);

	size_t num_meas                   = num_rows(model.z);
	auto bigH                         = zeros(num_meas, get_num_states());
	auto xt_keepers                   = xt::keep(get_all_state_indices(state_block_labels));
	view(bigH, xt::all(), xt_keepers) = model.H;

	out->H = bigH;
	out->h = [xt_keepers = xt_keepers, model = model](Vector xLarge) -> Vector {
		return model.h(xt::view(xLarge, xt_keepers));
	};
	return out;
}

std::invalid_argument bad_block(string const &label) {
	return std::invalid_argument("Couldn't find block named " + label);
}

std::invalid_argument bad_processor(string const &label) {
	return std::invalid_argument("Couldn't find processor named " + label);
}

std::invalid_argument bad_vsb(string const &label) {
	return std::invalid_argument("Couldn't find block nor VirtualStateBlock for name " + label);
}

std::vector<Size> StandardFusionEngine::get_all_state_indices(
    const std::vector<std::string> &block_labels) const {
	std::vector<Size> all_indices;

	for (const auto &label : block_labels) {
		auto start_stop_indices = get_mat_indices(label);
		for (size_t idx = start_stop_indices.first; idx < start_stop_indices.second; idx++) {
			all_indices.push_back(idx);
		}
	}
	return all_indices;
}

void StandardFusionEngine::propagate(aspn_xtensor::TypeTimestamp time) {
	if (cur_time == time)
		return;
	else if (cur_time > time) {
		log_or_throw<std::invalid_argument>(
		    "Reverse propagate requested: propagation requested at time {} but filter is at time "
		    "{}. Possible out of order measurements!",
		    time,
		    cur_time);
		return;
	}

	auto mat_idx_list = get_mat_indices_list();

	vector<StandardDynamicsModel> dynamics;
	dynamics.reserve(blocks.size());

	for (auto const &blk : blocks) {
		auto label = blk->get_label();
		auto dyn   = blk->generate_dynamics(gen_x_and_p_func, cur_time, time);
		if (ValidationContext validation{}) {
			auto context      = "State block " + label + "'s";
			auto expectedSize = blk->get_num_states();

			if (ValidationResult::BAD == validation.add_matrix(dyn.Phi, context + " Phi")
			                                 .dim(expectedSize, expectedSize)
			                                 .add_matrix(dyn.Qd, context + " Qd")
			                                 .dim(expectedSize, expectedSize)
			                                 .validate()) {
				clear_cache();
				return;
			}
		}
		dynamics.emplace_back(std::move(dyn));
	}

	auto g = [&](Vector xLarge) -> Vector {
		Vector out = zeros(num_rows(xLarge));
		size_t ii  = 0;
		size_t start(0);
		for (auto const &dyn : dynamics) {
			auto fOut = dyn.g(view(xLarge, range(mat_idx_list[ii].first, mat_idx_list[ii].second)));
			view(out, range(start, start + num_rows(fOut))) = fOut;
			start += num_rows(fOut);
			++ii;
		}
		return out;
	};

	auto num_states = strategy->get_num_states();
	Matrix Phi      = zeros(num_states, num_states);
	Matrix Qd       = zeros(num_states, num_states);
	size_t ii       = 0;
	for (auto const &dyn : dynamics) {
		size_t start = mat_idx_list[ii].first;
		size_t end   = mat_idx_list[ii].second;

		view(Phi, range(start, end), range(start, end)) = dyn.Phi;
		view(Qd, range(start, end), range(start, end))  = dyn.Qd;
		++ii;
	}
	for (auto const &it : process_covariance_cross_terms) {
		auto ind1 = get_mat_indices(it.label1);
		auto ind2 = get_mat_indices(it.label2);
		view(Qd, range(ind1.first, ind1.second), range(ind2.first, ind2.second)) = it.term;
		view(Qd, range(ind2.first, ind2.second), range(ind1.first, ind1.second)) =
		    xt::transpose(it.term);
	}
	strategy->propagate(StandardDynamicsModel(g, Phi, Qd));
	cur_time = time;
	clear_cache();
}

void StandardFusionEngine::update(string const &processor_label,
                                  std::shared_ptr<aspn_xtensor::AspnBase> measurement,
                                  std::shared_ptr<aspn_xtensor::TypeTimestamp> timestamp) {

	if (timestamp) {
		propagate(*timestamp);
	} else {
		auto time = navtk::get_time(measurement);
		if (!time.first) {
			log_or_throw<std::invalid_argument>(
			    "Cannot process measurement for processor {}. Does not contain a timestamp.",
			    processor_label);
			return;
		}
		propagate(time.second);
	}

	auto processor = get_measurement_processor(processor_label);

	auto pre_proc_labels = processor->get_state_block_labels();

	auto pre_model = processor->generate_model(measurement, gen_x_and_p_func);
	if (pre_model == nullptr) {
		clear_cache();
		return;
	};

	auto proc_labels = processor->get_state_block_labels();
	auto real_labels = get_real_block_labels(proc_labels);
	auto model       = std::make_shared<StandardMeasurementModel>(*pre_model);
	if (proc_labels != real_labels) {
		// We have allowed for the fact that an MP might add/remove
		// state blocks (and thus modify their own stateBlocksNames
		// list) as part of the generate_model process. This means that
		// we must re-calculate the transform functions/jacobians
		// in case the model they returned is against a modified
		// block list- otherwise we could compute much of this
		// earlier and just capture it
		if (proc_labels != last_gen_xp_args) {
			last_tx  = calc_full_transform(proc_labels);
			last_jac = calc_transform_jacobian(proc_labels);
		}
		// Now we have to make the Virtual model work with real
		// states we'll be passing in. Since h() is a nonlinear function
		// expecting one or more VirtualStateBlocks(VSB) we are stuck
		// evaluating it as-is, so we have yet another mapping of x.
		// H is also a mapping of the VSBs, so we have to stick the
		// Jacobian between it and the VSBs
		model->h = [&, virtual_fun = std::move(pre_model->h)](Vector x) {
			return virtual_fun(this->last_tx(std::move(x)));
		};
		model->H = dot(pre_model->H, last_jac);
	}

	// Update the state blocks that might have been edited in the Measurement Processor
	auto big_model = expand_update_model(*model, real_labels);
	strategy->update(*big_model);
	clear_cache();
}

void StandardFusionEngine::clear_cache() const {
	last_gen_xp_results = nullptr;
	last_gen_xp_args.clear();
	last_tx  = nullptr;
	last_jac = Matrix{};
}

}  // namespace filtering
}  // namespace navtk
