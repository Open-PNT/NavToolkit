// INCLUDES
#include <memory>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/experimental/fusion/strategies/RbpfStrategy.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/tensors.hpp>
// END

int main() {

	// CREATE ENGINE
	auto fusion_strategy = std::make_shared<navtk::filtering::experimental::RbpfStrategy>();
	auto time            = aspn_xtensor::TypeTimestamp((int64_t)0);
	auto engine          = navtk::filtering::StandardFusionEngine(time, fusion_strategy);
	// END

	// ADD BLOCK
	auto model = navtk::filtering::hg1700_model();
	auto block =
	    std::make_shared<navtk::filtering::Pinson15NedBlock>(std::string("pinson15"), model);
	engine.add_state_block(block);
	// END

	// MARK STATE
	fusion_strategy->set_marked_states(std::vector<size_t>{0, 1});
	// END

	// MARK STATE WITH JITTER
	fusion_strategy->set_marked_states(std::vector<size_t>{0, 1}, {0.075});
	// END

	// SET NUM PARTICLES
	fusion_strategy->set_particle_count_target(5000);
	// END

	// SET CALC SINGLE JACOBIAN
	fusion_strategy->calc_single_jacobian = true;
	// END

	// SET RESAMPLING THRESHOLD
	fusion_strategy->resampling_threshold = 0.8;
	// END
}
