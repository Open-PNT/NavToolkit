#pragma once

#include <memory>

#include <navtk/filtering/fusion/strategies/FusionStrategy.hpp>
#include <navtk/inspect.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

// A subclass of the linearized strategies that adds states and sets an estimate and covariance.
template <typename Base>
class ConfiguredStrategy : public Base {
public:
	ConfiguredStrategy(navtk::Vector const& x0, navtk::Matrix const& p0) : Base() {
		Base::on_fusion_engine_state_block_added(navtk::num_rows(x0));
		Base::set_estimate_slice(x0);
		Base::set_covariance_slice(p0);
	}

	navtk::not_null<std::shared_ptr<navtk::filtering::FusionStrategy>> clone() const override {
		return std::make_shared<ConfiguredStrategy>(*this);
	}
};
