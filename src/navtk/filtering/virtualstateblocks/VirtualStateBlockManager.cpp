#include <navtk/filtering/virtualstateblocks/VirtualStateBlockManager.hpp>

#include <algorithm>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/virtualstateblocks/ChainedVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/inspect.hpp>

namespace navtk {
namespace filtering {

VirtualStateBlockManager::VirtualStateBlockManager(const VirtualStateBlockManager& other)
    : relationships(other.relationships) {
	for (auto map_element : other.vsb_map) {
		auto labels = map_element.first;
		auto block  = map_element.second;
		auto it     = vsb_map.find({labels.first, labels.second});
		if (it != vsb_map.end())
			it->second = block->clone();
		else
			vsb_map.insert({{labels.first, labels.second}, block->clone()});
	}

	for (auto map_element : other.gen_vsb_map) {
		auto labels = map_element.first;
		auto block  = map_element.second;
		auto clone  = std::dynamic_pointer_cast<ChainedVirtualStateBlock>(block->clone());
		auto it     = gen_vsb_map.find({labels.first, labels.second});
		if (it != gen_vsb_map.end())
			it->second = clone;
		else
			gen_vsb_map.insert({{labels.first, labels.second}, clone});
	}
}

VirtualStateBlockManager& VirtualStateBlockManager::operator=(
    VirtualStateBlockManager const& other) {
	if (this == &other) return *this;

	relationships = other.relationships;

	for (auto map_element : other.vsb_map) {
		auto labels = map_element.first;
		auto block  = map_element.second;
		auto it     = vsb_map.find({labels.first, labels.second});
		if (it != vsb_map.end())
			it->second = block->clone();
		else
			vsb_map.insert({{labels.first, labels.second}, block->clone()});
	}

	for (auto map_element : other.gen_vsb_map) {
		auto labels = map_element.first;
		auto block  = map_element.second;
		auto clone  = std::dynamic_pointer_cast<ChainedVirtualStateBlock>(block->clone());
		auto it     = gen_vsb_map.find({labels.first, labels.second});
		if (it != gen_vsb_map.end())
			it->second = clone;
		else
			gen_vsb_map.insert({{labels.first, labels.second}, clone});
	}

	return *this;
}

std::vector<std::string> VirtualStateBlockManager::get_virtual_state_block_target_labels() const {
	std::vector<std::string> out;
	for (auto iter = vsb_map.begin(); iter != vsb_map.end(); iter++) {
		out.push_back(iter->first.second);
	}
	return out;
}

void VirtualStateBlockManager::add_virtual_state_block(
    not_null<std::shared_ptr<VirtualStateBlock>> trans) {
	if (trans->get_current() == trans->get_target()) {
		log_or_throw<std::invalid_argument>("Current and target tags should not be the same.");
		return;
	}
	if (duplicate(trans)) {
		log_or_throw<std::invalid_argument>("Already have a target with this tag");
		return;
	}
	auto it = vsb_map.find({trans->get_current(), trans->get_target()});
	if (it != vsb_map.end())
		it->second = trans;  // std::move?
	else
		vsb_map.insert({{trans->get_current(), trans->get_target()}, trans});
	gen_vsb_map.clear();
	relationships.clear();
}

std::shared_ptr<VirtualStateBlock> VirtualStateBlockManager::get_virtual_state_block(
    const std::string& target_label) {
	for (const auto& vsb : vsb_map) {
		if (vsb.first.second == target_label) {
			return vsb.second;
		}
	}
	return nullptr;
}

void VirtualStateBlockManager::remove_virtual_state_block(std::string const& target) {
	for (auto itr = vsb_map.begin(); itr != vsb_map.end(); ++itr) {
		if ((*itr).first.second == target) {
			itr = vsb_map.erase(itr);
			break;
		}
	}
	gen_vsb_map.clear();
	relationships.clear();
}

bool VirtualStateBlockManager::duplicate(not_null<std::shared_ptr<VirtualStateBlock>> trans) {
	std::vector<std::string> starts;
	std::vector<std::string> stops;
	std::tie(starts, stops) = starts_and_stops();
	std::vector<std::string> wrapped;
	wrapped.push_back(trans->get_target());
	return std::includes(stops.begin(), stops.end(), wrapped.begin(), wrapped.end());
}

std::pair<std::vector<std::string>, std::vector<std::string>>
VirtualStateBlockManager::starts_and_stops() const {
	std::vector<std::string> starts;
	std::vector<std::string> stops;
	for (auto iter = vsb_map.begin(); iter != vsb_map.end(); iter++) {
		starts.push_back(iter->first.first);
		stops.push_back(iter->first.second);
	}
	std::sort(starts.begin(), starts.end());
	std::sort(stops.begin(), stops.end());
	return {starts, stops};
}

// Get a sorted vector of all VirtualStateBlock.current that are not
// also VirtualStateBlock.target (and are either guaranteed state blocks
// or dangling transforms), and vice versa. Note that any circular
// VirtualStateBlocks (if VirtualStatBlock("A", "B") and
// VirtualStateBlock("B", "A") have both been registered) neither "A"
// nor "B" will appear in either list.
std::pair<std::vector<std::string>, std::vector<std::string>>
VirtualStateBlockManager::get_terminating() const {
	// Peel out all targets from registered VirtualBlocks
	// Remove duplicates and that are current == targets, return the rest
	std::vector<std::string> starts;
	std::vector<std::string> stops;
	std::tie(starts, stops) = starts_and_stops();

	// Each has to be unique before diffing, and sorted for unique to
	// pull out non-consecutive matches
	auto it_starts = std::unique(starts.begin(), starts.end());
	auto it_stops  = std::unique(stops.begin(), stops.end());

	starts.resize(it_starts - starts.begin());
	stops.resize(it_stops - stops.begin());

	std::vector<std::string> current_not_targets(starts.size());
	std::vector<std::string> targets_not_current(stops.size());

	it_starts = set_difference(
	    starts.begin(), starts.end(), stops.begin(), stops.end(), current_not_targets.begin());
	it_stops = set_difference(
	    stops.begin(), stops.end(), starts.begin(), starts.end(), targets_not_current.begin());
	current_not_targets.resize(it_starts - current_not_targets.begin());
	targets_not_current.resize(it_stops - targets_not_current.begin());
	return {current_not_targets, targets_not_current};
}

std::pair<bool, std::string> VirtualStateBlockManager::get_start_block_label(
    std::string const& target) const {
	if (relationships.count(target) == 0) {

		// Collect all the 'current' fields from every available VSB that aren't also targets and
		// check each one
		auto all_starts = get_terminating().first;

		for (auto start_node = all_starts.begin(); start_node != all_starts.end(); start_node++) {

			auto tx = pathfinder(*start_node, target);
			// nullptr when no VirtualStateBlocks are registered or if there is no path from the
			// start node to the target node, in which case we just want to move on to the next one.
			if (tx != nullptr) {
				relationships[target] = tx->get_current();
				break;
			}
		}

		if (relationships.count(target) == 0) {
			return {false, {}};
		}
	}
	return {true, relationships.at(target)};
}

EstimateWithCovariance VirtualStateBlockManager::convert(
    const EstimateWithCovariance& orig,
    const std::string& start,
    const std::string& target,
    const aspn_xtensor::TypeTimestamp& time) const {

	if (vsb_map.empty()) {
		log_or_throw<std::out_of_range>(
		    "No VirtualStateBlocks have been registered. Please add using the "
		    "add_virtual_state_block function.");
	}

	if (start == target) return orig;
	auto tx = pathfinder(start, target);
	if (tx == nullptr)
		log_or_throw<std::out_of_range>("Exhausted node search, no path for transform available.");
	auto transformed = tx->convert(orig, time);
	return transformed;
}

Vector VirtualStateBlockManager::convert_estimate(const Vector& orig,
                                                  const std::string& start,
                                                  const std::string& target,
                                                  const aspn_xtensor::TypeTimestamp& time) const {
	if (vsb_map.empty()) {
		log_or_throw<std::out_of_range>(
		    "No VirtualStateBlocks have been registered. Please add using the "
		    "add_virtual_state_block function.");
	}

	if (start == target) return orig;
	auto tx = pathfinder(start, target);
	if (tx == nullptr)
		log_or_throw<std::out_of_range>("Exhausted node search, no path for transform available.");
	auto transformed = tx->convert_estimate(orig, time);
	return transformed;
}

Matrix VirtualStateBlockManager::jacobian(const EstimateWithCovariance& orig,
                                          const std::string& start,
                                          const std::string& target,
                                          const aspn_xtensor::TypeTimestamp& time) const {
	return jacobian(orig.estimate, start, target, time);
}

Matrix VirtualStateBlockManager::jacobian(const Vector& orig,
                                          const std::string& start,
                                          const std::string& target,
                                          const aspn_xtensor::TypeTimestamp& time) const {
	if (start == target) return eye(num_rows(orig));
	auto tx = pathfinder(start, target);
	if (tx == nullptr)
		log_or_throw<std::out_of_range>("Exhausted node search, no path for transform available.");
	return tx->jacobian(orig, time);
}

std::shared_ptr<VirtualStateBlock> VirtualStateBlockManager::pathfinder(
    const std::string& current, const std::string& target) const {

	if (vsb_map.empty()) {
		log_or_throw<std::out_of_range>(
		    "No VirtualStateBlocks have been registered. Please add using the "
		    "add_virtual_state_block function.");
		return nullptr;
	}

	std::vector<std::pair<std::string, std::string>> found_path{};

	if (gen_vsb_map.find({current, target}) == gen_vsb_map.end()) {
		found_path = link_or_fail(current, target, found_path);
		if (found_path.size() > 1) {
			std::vector<not_null<std::shared_ptr<VirtualStateBlock>>> lumps;
			for (auto iter = found_path.begin(); iter != found_path.end(); iter++) {
				lumps.push_back(vsb_map.at(*iter));
			}
			auto block = std::make_shared<ChainedVirtualStateBlock>(lumps);
			auto it    = gen_vsb_map.find({current, target});
			if (it != gen_vsb_map.end())
				it->second = block;
			else
				gen_vsb_map.insert({{current, target}, block});
			return gen_vsb_map.at({current, target}).get();
		}
		if (vsb_map.count({current, target}) == 1)
			return vsb_map.at({current, target}).get();
		else
			return nullptr;

	} else {
		return gen_vsb_map.at({current, target}).get();
	}
}

std::vector<std::pair<std::string, std::string>> VirtualStateBlockManager::link_or_fail(
    const std::string& current,
    const std::string& target,
    std::vector<std::pair<std::string, std::string>>& origin_to_node) const {

	for (auto ele = vsb_map.begin(); ele != vsb_map.end(); ele++) {
		if (ele->first.second == target) {
			origin_to_node.insert(origin_to_node.begin(), {ele->first.first, ele->first.second});
			if (ele->first.first == current)
				return origin_to_node;
			else
				return link_or_fail(current, ele->first.first, origin_to_node);
		}
	}
	return {};
}
}  // namespace filtering
}  // namespace navtk
