#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/linear_algebra.hpp>

namespace navtk {
namespace filtering {

using xt::transpose;

VirtualStateBlock::VirtualStateBlock(std::string current, std::string target)
    : current(std::move(current)), target(std::move(target)) {}

EstimateWithCovariance VirtualStateBlock::convert(const EstimateWithCovariance& ec,
                                                  const aspn_xtensor::TypeTimestamp& time) {
	auto state = convert_estimate(ec.estimate, time);
	auto jac   = jacobian(ec.estimate, time);
	auto cov   = dot(dot(jac, ec.covariance), transpose(jac));
	return EstimateWithCovariance(state, cov);
}

void VirtualStateBlock::receive_aux_data(const AspnBaseVector&) {
	spdlog::warn("The virtual state block labeled {} does not utilize this type of aux data.",
	             target);
}

std::string VirtualStateBlock::get_current() const { return current; }
std::string VirtualStateBlock::get_target() const { return target; }

}  // namespace filtering
}  // namespace navtk
