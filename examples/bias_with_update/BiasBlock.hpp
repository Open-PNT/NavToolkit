#pragma once

#include <memory>

#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

class BiasBlock : public navtk::filtering::StateBlock<> {
public:
	virtual ~BiasBlock() = default;

	BiasBlock(const std::string& label) : navtk::filtering::StateBlock<>(1, label) {}

	navtk::not_null<std::shared_ptr<StateBlock<>>> clone() override {
		return std::make_shared<BiasBlock>(get_label());
	}

	// This function takes in the current state estimate and time
	// and produces the needed dynamics equation parameters (g, Phi, Qd)
	navtk::filtering::StandardDynamicsModel generate_dynamics(
	    navtk::filtering::GenXhatPFunction,
	    aspn_xtensor::TypeTimestamp time_from,
	    aspn_xtensor::TypeTimestamp time_to) override {

		// Define the g(x) propagation function as g(x_(k)) = x_(k-1)
		auto g = [](navtk::Vector x) { return x; };

		auto delta_time = to_seconds(time_to - time_from);

		navtk::Matrix Phi = {{1}};
		navtk::Matrix Qd  = {{delta_time}};

		return navtk::filtering::StandardDynamicsModel(g, Phi, Qd);
	}
};
