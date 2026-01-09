#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <navtk/filtering/containers/EstimateWithCovariance.hpp>

namespace navtk {
namespace filtering {

/**
 * A function which, given a set of StateBlock labels, returns the estimate and covariance
 * associated with the states of those blocks. This is used to lazily evaluate `xhat` and `P`.
 */
using GenXhatPFunction =
    std::function<std::shared_ptr<EstimateWithCovariance>(const std::vector<std::string> &)>;

/**
 * Warning: this implementation of GenXhatPFunction should be used with care. It which ignores the
 * input parameter and always returns nullptr. This can be passed to StateBlock::generate_dynamics
 * or MeasurementProcessor::generate_model but only when the method does not need the state estimate
 * or covariance to generate its model.
 */
static GenXhatPFunction NULL_GEN_XHAT_AND_P_FUNCTION =
    [](const std::vector<std::string> &) -> std::shared_ptr<EstimateWithCovariance> {
	return nullptr;
};

}  // namespace filtering
}  // namespace navtk
