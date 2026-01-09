#include <navtk/filtering/virtualstateblocks/ChainedVirtualStateBlock.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/linear_algebra.hpp>

namespace navtk {
namespace filtering {

ChainedVirtualStateBlock::ChainedVirtualStateBlock(
    std::vector<not_null<std::shared_ptr<VirtualStateBlock>>> to_chain)
    : VirtualStateBlock(to_chain.front()->get_current(), to_chain.back()->get_target()),
      chained(std::move(to_chain)),
      last_x(zeros(2)),
      last_jac(zeros(1, 1)),
      last_x_conv(zeros(2)),
      conv_synced(false) {}

ChainedVirtualStateBlock::ChainedVirtualStateBlock(const ChainedVirtualStateBlock& other)
    : VirtualStateBlock(other.chained.front()->get_current(), other.chained.back()->get_target()),
      last_x(other.last_x),
      last_jac(other.last_jac),
      last_x_conv(other.last_x_conv),
      conv_synced(other.conv_synced) {
	for (auto block : other.chained) chained.push_back(block->clone());
}

ChainedVirtualStateBlock& ChainedVirtualStateBlock::operator=(
    ChainedVirtualStateBlock const& other) {
	if (this == &other) return *this;
	current     = other.current;
	target      = other.target;
	last_x      = other.last_x;
	last_jac    = other.last_jac;
	last_x_conv = other.last_x_conv;
	conv_synced = other.conv_synced;
	chained.clear();
	for (auto block : other.chained) chained.push_back(block->clone());
	return *this;
}

not_null<std::shared_ptr<VirtualStateBlock>> ChainedVirtualStateBlock::clone() {
	return std::make_shared<ChainedVirtualStateBlock>(*this);
}

bool ChainedVirtualStateBlock::cache_safe(const Vector& x,
                                          const aspn_xtensor::TypeTimestamp& time) {
	return conv_synced && num_rows(last_x) == num_cols(last_jac) &&
	       xt::allclose(x, last_x, 1e-12, 0.0) && time == last_time;
}

EstimateWithCovariance ChainedVirtualStateBlock::convert(const EstimateWithCovariance& ec,
                                                         const aspn_xtensor::TypeTimestamp& time) {
	// TODO Decide if we keep. Can generalize state and jac caching to all VSBs potentially,
	// but not the convert() function generally unless we restrict to 1st order mappings
	// (ie no Hessians). Doing so here is something of a API violation.
	// Especially useful for Chained, as convert_estimate() is calculated as a side product
	// of jacobian().
	auto jac     = jacobian(ec.estimate, time);
	auto x_out   = convert_estimate(ec.estimate, time);
	auto cov_out = dot(dot(jac, ec.covariance), transpose(jac));
	last_x       = ec.estimate;
	last_x_conv  = x_out;
	last_jac     = jac;
	return EstimateWithCovariance(x_out, cov_out);
}

Vector ChainedVirtualStateBlock::convert_estimate(const Vector& x,
                                                  const aspn_xtensor::TypeTimestamp& time) {
	if (!cache_safe(x, time)) {
		Vector out{x};
		for (auto trans = chained.begin(); trans != chained.end(); trans++) {
			out = (*trans)->convert_estimate(out, time);
		}
		last_time   = time;
		last_x      = x;
		last_x_conv = out;
		conv_synced = false;
		last_jac    = zeros(1, 1);
	}
	return last_x_conv;
}

Matrix ChainedVirtualStateBlock::jacobian(const Vector& x,
                                          const aspn_xtensor::TypeTimestamp& time) {
	if (!cache_safe(x, time)) {
		Matrix jac = eye(num_rows(x));
		Vector mut = x;
		for (auto trans = chained.begin(); trans != chained.end(); trans++) {
			jac = dot((*trans)->jacobian(mut, time), jac);
			mut = (*trans)->convert_estimate(mut, time);
		}
		last_time   = time;
		last_jac    = jac;
		last_x      = x;
		last_x_conv = mut;
		conv_synced = true;
	}

	return last_jac;
}

}  // namespace filtering
}  // namespace navtk
