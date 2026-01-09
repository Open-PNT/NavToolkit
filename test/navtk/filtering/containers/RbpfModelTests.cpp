#include <memory>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>

#include <navtk/factory.hpp>
#include <navtk/filtering/experimental/containers/RbpfModel.hpp>

using namespace navtk::filtering;
using navtk::filtering::experimental::RbpfModel;
using namespace navtk;

struct RbpfModelTests : ::testing::Test {};

TEST(RbpfModelTests, StateMarksParticleCountSurviveCloneReset_SLOW) {
	RbpfModel target(123);
	target.on_fusion_engine_state_block_added(3);
	target.set_marked_states({1, 2});
	auto clone = std::dynamic_pointer_cast<RbpfModel>(target.clone());
	ASSERT_EQ(123, clone->count_particles());
	ASSERT_EQ(std::vector<size_t>({1, 2}), clone->get_marked_states());
}

TEST(RbpfModelTests, MarkingStateEnablesParticles) {
	RbpfModel target(20);
	target.on_fusion_engine_state_block_added(3);
	ASSERT_EQ(20, target.get_particle_count_target());
	ASSERT_EQ(1, target.count_particles());
	target.set_marked_states({0, 2});
	ASSERT_EQ(20, target.get_particle_count_target());
	ASSERT_EQ(20, target.count_particles());

	RbpfModel slow_init;
	slow_init.on_fusion_engine_state_block_added(zeros(3), eye(3));
	ASSERT_EQ(RbpfModel::DEFAULT_PARTICLE_COUNT, slow_init.get_particle_count_target());
	ASSERT_EQ(1, slow_init.count_particles());
	slow_init.set_particle_count_target(25);
	ASSERT_EQ(1, slow_init.count_particles());
	ASSERT_EQ(25, slow_init.get_particle_count_target());
	slow_init.set_marked_states({0, 2});
	ASSERT_EQ(25, slow_init.count_particles());
}

ERROR_MODE_SENSITIVE_TEST(TEST, RbpfModelTests, BadConstruct) {
	EXPECT_HONORS_MODE_EX(RbpfModel(0), "Number of particles", std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST, RbpfModelTests, BadParticleCountChange) {
	RbpfModel target(123);
	EXPECT_HONORS_MODE_EX(
	    target.set_particle_count_target(3), "Particle count changed", std::runtime_error);
}

// TODO: (PNTOS-295) MOAR TESTS:
// - forbid setting cross terms on P
// - warn about setting after initialize ... but ... is there even a way to test that warnings
// occur?
