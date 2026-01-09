#pragma once

#include <memory>
#include <utility>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/virtualstateblocks/ChainedVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace filtering {
/**
 * Utility class for managing VirtualStateBlocks and using them to
 * convert state representations.
 */
class VirtualStateBlockManager {
public:
	/**
	 * Default constructor.
	 */
	VirtualStateBlockManager() = default;

	/**
	 * Default destructor.
	 */
	~VirtualStateBlockManager() = default;

	/**
	 * Copy constructor.
	 *
	 * @param other The VirtualStateBlockManager to be copied.
	 */
	VirtualStateBlockManager(const VirtualStateBlockManager& other);

	/**
	 * Copy assignment operator.
	 *
	 * @param other Another instance of this class whose fields are copied.
	 *
	 * @return A copy of \p other .
	 */
	VirtualStateBlockManager& operator=(VirtualStateBlockManager const& other);

	/**
	 * Default move constructor.
	 *
	 * @param other Another instance of this class.
	 */
	VirtualStateBlockManager(VirtualStateBlockManager&& other) = default;

	/**
	 * Default move assignment operator.
	 *
	 * @param other Another instance of this class.
	 *
	 * @return An instance of VirtualStateBlockManager with the data from \p other .
	 */
	VirtualStateBlockManager& operator=(VirtualStateBlockManager&& other) = default;

	/**
	 * Returns a list of the target labels of all virtual state blocks managed in this instance.
	 *
	 * A label being returned by this list is not a guarantee that the virtual state block has a
	 * valid source.
	 *
	 * @returns A list of the target labels of virtual state blocks managed in this instance.
	 */
	std::vector<std::string> get_virtual_state_block_target_labels() const;

	/**
	 * Registers a VirtualStateBlock that can convert an EstimateWithCovariance from a given
	 * representation to another. Users may supply multiple 'single hop' VirtualStateBlocks (i.e.
	 * a->b, b->c, c->d, where 'a' is a real StateBlock) which the manager can then use to go from
	 * a->d, for instance. No VirtualStateBlock should use more than one other block as a source
	 * (i.e. adding a c->e to the above is ok, as both d and e could unambiguously be derived from
	 * a, but an e->d VSB would cause ambiguity as to whether a or e should the the starting point
	 * for the transform to d).
	 *
	 * @param trans Pointer to a VirtualStateBlock instance that converts from representation A to
	 * B, as indicated by the VirtualStateBlocks `current` and `target` fields.
	 *
	 * @throw std::invalid_argument if trans.current and trans.target fields are identical, or if
	 * trans.target is identical to the trans.target of any other previously added
	 * VirtualStateBlock, but only if the error mode is ErrorMode::DIE for either case.
	 */
	void add_virtual_state_block(not_null<std::shared_ptr<VirtualStateBlock>> trans);

	/**
	 * Removes a virtual state block that was added with add_virtual_state_block.
	 *
	 * @param target target label of the virtual state block to be removed
	 */
	void remove_virtual_state_block(std::string const& target);

	/**
	 * Get a pointer to a VSB with the given \p target_label.
	 *
	 * @param target_label The target label for the VSB to request
	 *
	 * @return A pointer to the requested VSB.  May be nullptr, if \p target_label does not exist.
	 */
	std::shared_ptr<VirtualStateBlock> get_virtual_state_block(const std::string& target_label);

	/**
	 * Get the StateBlock label of the starting node, assumed to be a 'real' StateBlock that should
	 * be used to convert to the requested \p target.
	 *
	 * @param target VirtualStateBlock representation that the user would like to request.
	 *
	 * @return A pair of values, the first a bool which will be true if there is a valid path from
	 * a real state block to \p target.  The second value is the label of the real state block's
	 * EstimateWithCovariance that should be supplied to the convert() function to get a
	 * VirtualStateBlock that converts to the \p target representation.
	 */
	std::pair<bool, std::string> get_start_block_label(std::string const& target) const;

	/**
	 * Convert a StateBlock EstimateWithCovariance from its current representation (as indicated by
	 * its label) to a new representation.
	 *
	 * @param orig Estimate and covariance in starting format.
	 * @param start Label that refers to \p orig.
	 * @param target Label that refers to data format post-conversion.
	 * @param time Time of validity for \p orig.
	 *
	 * @return A converted EstimateWithCovariance in the \p target representation.
	 *
	 * @throw std::out_of_range If no VirtualStateBlocks have been registered prior to this function
	 * call, or if there is no chain of VirtualStateBlocks that can be used to effect a
	 * transformation from the starting representation to the \p target parameter representation but
	 * only if the error mode is ErrorMode::DIE for either case.
	 */
	EstimateWithCovariance convert(const EstimateWithCovariance& orig,
	                               const std::string& start,
	                               const std::string& target,
	                               const aspn_xtensor::TypeTimestamp& time) const;

	/**
	 * Convert a StateBlock estimate from its current representation (as indicated by its label) to
	 * a new representation.
	 *
	 * @param orig Estimate in starting format.
	 * @param start Label that refers to \p orig.
	 * @param target Label that refers to data format post-conversion.
	 * @param time Time of validity for \p orig.
	 *
	 * @return A converted estimate in the \p target representation.
	 *
	 * @throw std::out_of_range If no VirtualStateBlocks have been registered prior to this function
	 * call, or if there is no chain of VirtualStateBlocks that can be used to effect a
	 * transformation from the starting representation to the \p target parameter representation but
	 * only if the error mode is ErrorMode::DIE for either case.
	 */
	Vector convert_estimate(const Vector& orig,
	                        const std::string& start,
	                        const std::string& target,
	                        const aspn_xtensor::TypeTimestamp& time) const;

	/**
	 * Get the Jacobian of the transform function from A to B.
	 *
	 * @param orig Estimate and covariance in starting format.
	 * @param start Label that refers to \p orig.
	 * @param target Representation that the Jacobian should approximately transform orig to ('B').
	 * @param time Time of validity for \p orig.
	 *
	 * @return MxN Jacobian that when pre-multiplied by N-length `orig.x` generates an M-length
	 * vector in \p target representation.
	 *
	 * @throw std::out_of_range If no VirtualStateBlocks have been registered prior to this function
	 * call, or if there is no chain of VirtualStateBlocks that can be used to effect a
	 * transformation from the starting representation to the \p target parameter representation but
	 * only if the error mode is ErrorMode::DIE for either case.
	 */
	Matrix jacobian(const EstimateWithCovariance& orig,
	                const std::string& start,
	                const std::string& target,
	                const aspn_xtensor::TypeTimestamp& time) const;

	/**
	 * Get the Jacobian of the transform function from A to B.
	 *
	 * @param orig Estimate in starting format.
	 * @param start Label that refers to \p orig.
	 * @param target Representation that the Jacobian should approximately transform orig to ('B').
	 * @param time Time of validity for \p orig.
	 *
	 * @return MxN Jacobian that when pre-multiplied by N-length `orig.x` generates an M-length
	 * vector in \p target representation.
	 *
	 * @throw std::out_of_range If no VirtualStateBlocks have been registered prior to this function
	 * call, or if there is no chain of VirtualStateBlocks that can be used to effect a
	 * transformation from the starting representation to the \p target parameter representation but
	 * only if the error mode is ErrorMode::DIE for either case.
	 */
	Matrix jacobian(const Vector& orig,
	                const std::string& start,
	                const std::string& target,
	                const aspn_xtensor::TypeTimestamp& time) const;

private:
	/**
	 * Find a path through all VirtualStateBlocks from current to target. Recursive function,
	 * working backwards from target until {current, target} match the tags on an existing VSB.
	 *
	 * @param current VirtualStateBlock to begin with.
	 * @param target VirtualStateBlock to transform to.
	 * @param origin_to_node Vector of {current, target} pairs that build a path from some other
	 * current value up to this \p current value.
	 *
	 * @return Vector of pairs that contain the current and target values of VirtualStateBlocks that
	 * when chained together will convert the EstimateWithCovariance of a StateBlock with the label
	 * \p current to a mapped value with the label \p target.
	 *
	 * Example: If 5 VirtualStateBlocks were added with current and target fields of {a, b}, {b, c},
	 * {c, d}, {b, e}, {e, f}, and this function was originally called with (a, d, {}), then the
	 * first recursion arguments would be (a, c, {{c, d}}), and the second (a, b, {{b, c}, {c, d}})
	 * and so on until the final return value of {{a, b}, {b, c}, {c, d}}, which allows for direct
	 * lookup of VSBs in vsbMap for chaining.
	 */
	std::vector<std::pair<std::string, std::string>> link_or_fail(
	    const std::string& current,
	    const std::string& target,
	    std::vector<std::pair<std::string, std::string>>& origin_to_node) const;

	/**
	 * Check if the `target` field on \p trans is the same as the `target` field on any previously
	 * added VirtualStateBlock.
	 */
	bool duplicate(not_null<std::shared_ptr<VirtualStateBlock>> trans);

	/**
	 * Creates sorted vectors of the `current` and `target` fields on all added VirtualStateBlocks.
	 *
	 * @return Pair with the first element containing all `current` values (duplicates may be
	 * present), and the second element containing all `target` values.
	 */
	std::pair<std::vector<std::string>, std::vector<std::string>> starts_and_stops() const;

	/**
	 * Iterates over the vsbMap and returns vectors of
	 * a) VirtualStateBlock.current fields that do not match any VirtualStateBlock.target
	 * b) VirtualStateBlock.target fields that do not match any VirtualStateBlock.current
	 */
	std::pair<std::vector<std::string>, std::vector<std::string>> get_terminating() const;

	/**
	 * Attempts to find the shortest path through registered VirtualStateBlocks to get from
	 * \p current to \p target. If such a conversion requires using more than one VirtualStateBlock,
	 * a ChainedVirtualStateBlock is created that goes directly from \p current to \p target and
	 * registered in a map, enabling the path search to be skipped in favor of a direct lookup.
	 */
	std::shared_ptr<VirtualStateBlock> pathfinder(const std::string& current,
	                                              const std::string& target) const;

	/**
	 * A map of VirtualStateBlocks, where keys are pairs created from the mapped VSBs current and
	 * target fields.
	 */
	std::map<std::pair<std::string, std::string>, not_null<std::shared_ptr<VirtualStateBlock>>>
	    vsb_map;

	/**
	 * As vsb_map, but only contains synthesized ChainedVirtualStateBlocks that have been created in
	 * response to convert() calls that require more than one VirtualStateBlock from vsb_map to be
	 * used.
	 */
	mutable std::map<std::pair<std::string, std::string>,
	                 not_null<std::shared_ptr<ChainedVirtualStateBlock>>>
	    gen_vsb_map;

	/**
	 * All known direct links between start labels and target labels; i.e. if user wants something
	 * in `target` representation we know that the user should provide something in 'start'
	 * representation. `target` are the keys and 'start' the values.
	 */
	mutable std::map<std::string, std::string> relationships;
};

}  // namespace filtering
}  // namespace navtk
