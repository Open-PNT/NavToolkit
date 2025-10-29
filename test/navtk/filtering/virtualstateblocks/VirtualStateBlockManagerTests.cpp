#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>
#include <xtensor/views/xview.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/filtering/virtualstateblocks/FirstOrderVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/ScaleVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlockManager.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::Matrix;
using navtk::num_rows;
using navtk::Vector;
using navtk::zeros;
using navtk::filtering::EstimateWithCovariance;
using navtk::filtering::FirstOrderVirtualStateBlock;
using navtk::filtering::ScaleVirtualStateBlock;
using navtk::filtering::VirtualStateBlock;
using navtk::filtering::VirtualStateBlockManager;
using xt::diag;
using xt::range;
using xt::view;

Vector get_nominal() { return Vector{10.0, 20.0, 30.0, 40.0, 50.0, 60.0}; }

Vector err_trans(const Vector& x) { return x + get_nominal(); }

Matrix err_jac(const Vector& x) { return navtk::eye(num_rows(x)); }

struct VirtualStateBlockManagerTests : public ::testing::Test {
	// Use a 6 state vector representing position and tilt errors.
	// States within the real state block are in km and mrad
	Vector defX;
	// Equivalent to 10 mrad and 10 meter std
	Matrix defP;

	class DummyVirtualStateBlock : public ScaleVirtualStateBlock {
	public:
		DummyVirtualStateBlock(const std::string& current,
		                       const std::string& target,
		                       double mult            = 0.0,
		                       navtk::Size num_states = 6)
		    : ScaleVirtualStateBlock(current, target, navtk::ones(num_states) * mult) {}
	};

	EstimateWithCovariance base;
	std::unique_ptr<VirtualStateBlockManager> manager;
	VirtualStateBlockManager manager_copy_ctr;
	VirtualStateBlockManager manager_copy_assign;

	// To go from state (km, mrad) to normal (m, rad)
	Vector scale_factors;
	Vector post_scale_x;
	Matrix post_scale_p;
	Vector post_whole_x;
	Matrix post_whole_p;

	VirtualStateBlockManagerTests()
	    : ::testing::Test(),
	      defX({0.01, 0.02, 0.03, 4.0, 5.0, 6.0}),
	      defP(diag(defX / 2.0)),
	      base({EstimateWithCovariance(defX, defP)}),
	      scale_factors({1e3, 1e3, 1e3, 1e-3, 1e-3, 1e-3}),
	      post_scale_x({10.0, 20.0, 30.0, 4e-3, 5e-3, 6e-3}),
	      post_scale_p(diag(Vector{5e3, 1e4, 1.5e4, 2e-6, 2.5e-6, 3e-6})),
	      post_whole_x(post_scale_x + get_nominal()),
	      post_whole_p(post_scale_p) {}

	void test_manager_and_copies(const std::function<void(VirtualStateBlockManager&)>& test) {
		// Test the original
		test(*manager);

		// Destroy the original and test the copies
		manager.reset();
		test(manager_copy_ctr);
		test(manager_copy_assign);
	}

	virtual void SetUp() override {
		manager = std::make_unique<VirtualStateBlockManager>();
		manager->add_virtual_state_block(std::make_shared<ScaleVirtualStateBlock>(
		    ScaleVirtualStateBlock("base", "unscaled", scale_factors)));
		manager->add_virtual_state_block(std::make_shared<FirstOrderVirtualStateBlock>(
		    FirstOrderVirtualStateBlock("unscaled", "whole", err_trans, err_jac)));
		manager_copy_ctr    = VirtualStateBlockManager(*manager);
		manager_copy_assign = *manager;
	}
};

TEST_F(VirtualStateBlockManagerTests, failsOnUnregisteredStop) {
	// Define the test.
	auto test = [this](VirtualStateBlockManager& manager) {
		EXPECT_UB_OR_DIE(
		    manager.convert(base, "base", "unknown", aspn_xtensor::TypeTimestamp((int64_t)0)),
		    "Exhausted node search, no path for transform available.",
		    std::out_of_range);
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, failsOnUnregisteredStart) {
	// Define the test.
	auto test = [this](VirtualStateBlockManager& manager) {
		EXPECT_UB_OR_DIE(
		    manager.convert(base, "nope", "unscaled", aspn_xtensor::TypeTimestamp((int64_t)0)),
		    "Exhausted node search, no path for transform available.",
		    std::out_of_range);
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, failsOnUnregisteredBoth) {
	// Define the test.
	auto test = [this](VirtualStateBlockManager& manager) {
		EXPECT_UB_OR_DIE(
		    manager.convert(base, "nope", "otherNope", aspn_xtensor::TypeTimestamp((int64_t)0)),
		    "Exhausted node search, no path for transform available.",
		    std::out_of_range);
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, failsNoStartBlock) {
	// Define the test.
	auto test = [](VirtualStateBlockManager& manager) {
		auto result = manager.get_start_block_label("fake");
		ASSERT_FALSE(result.first);
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, noOp) {
	// Define the test.
	auto test = [this](VirtualStateBlockManager& manager) {
		auto converted =
		    manager.convert(base, "base", "base", aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(converted.estimate, base.estimate);
		ASSERT_ALLCLOSE(converted.covariance, base.covariance);
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, emptyTxTags) {
	// Define the test.
	auto test = [this](VirtualStateBlockManager& manager) {
		manager.add_virtual_state_block(std::make_shared<ScaleVirtualStateBlock>(
		    ScaleVirtualStateBlock("", "unscaled2", scale_factors)));
		auto converted =
		    manager.convert(base, "", "unscaled2", aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(converted.estimate, post_scale_x);
		ASSERT_ALLCLOSE(converted.covariance, post_scale_p);
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, VirtualStateBlockManagerTests, failDuplicateTargets) {
	// Define the test.
	auto test_func = [this](VirtualStateBlockManager& manager) {
		EXPECT_HONORS_MODE_EX(
		    manager.add_virtual_state_block(std::make_shared<ScaleVirtualStateBlock>(
		        ScaleVirtualStateBlock("", "unscaled", this->test.scale_factors))),
		    "Already have a target with this tag",
		    std::invalid_argument);
	};

	// Execute the test on the original and copies.
	test.test_manager_and_copies(test_func);
}

TEST_F(VirtualStateBlockManagerTests, failSameTag) {
	// Define the test.
	auto test = [this](VirtualStateBlockManager& manager) {
		EXPECT_UB_OR_DIE(manager.add_virtual_state_block(std::make_shared<ScaleVirtualStateBlock>(
		                     ScaleVirtualStateBlock("a", "a", scale_factors))),
		                 "Current and target tags should not be the same.",
		                 std::invalid_argument);
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, convertsOnce) {
	// Define the test.
	auto test = [this](VirtualStateBlockManager& manager) {
		auto converted =
		    manager.convert(base, "base", "unscaled", aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(converted.estimate, post_scale_x);
		ASSERT_ALLCLOSE(converted.covariance, post_scale_p);
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, convertsTwiceManual) {
	// Define the test.
	auto test = [this](VirtualStateBlockManager& manager) {
		auto converted = manager.convert(
		    manager.convert(base, "base", "unscaled", aspn_xtensor::TypeTimestamp((int64_t)0)),
		    "unscaled",
		    "whole",
		    aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(converted.estimate, post_whole_x);
		ASSERT_ALLCLOSE(converted.covariance, post_whole_p);
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, manualIsSameAsAuto) {
	// Define the test.
	auto test = [this](VirtualStateBlockManager& manager) {
		auto manual = manager.convert(
		    manager.convert(base, "base", "unscaled", aspn_xtensor::TypeTimestamp((int64_t)0)),
		    "unscaled",
		    "whole",
		    aspn_xtensor::TypeTimestamp((int64_t)0));
		auto magic =
		    manager.convert(base, "base", "whole", aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(manual.estimate, magic.estimate);
		ASSERT_ALLCLOSE(manual.covariance, magic.covariance);
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, emptyManager) {
	// This test is unlikely to detect problems in the custom initializers, so just test "manager".
	VirtualStateBlockManager manager;
	EXPECT_UB_OR_DIE(
	    manager.convert(base, "base", "unscaled", aspn_xtensor::TypeTimestamp((int64_t)0)),
	    "No VirtualStateBlocks have been registered. Please add using the "
	    "add_virtual_state_block function.",
	    std::out_of_range);
}

TEST_F(VirtualStateBlockManagerTests, longsingle) {
	// If no startBlocks or relationships are added but there a single,
	// unique path from start to target we should be able to return.
	// Currently only works if there is a single VirtualStateBlock that
	// goes from start to target.
	manager = std::make_unique<VirtualStateBlockManager>();
	manager->add_virtual_state_block(std::make_shared<DummyVirtualStateBlock>("base", "p1"));
	manager->add_virtual_state_block(std::make_shared<DummyVirtualStateBlock>("p1", "p2"));
	manager_copy_ctr    = VirtualStateBlockManager(*manager);
	manager_copy_assign = VirtualStateBlockManager();
	manager_copy_assign = *manager;

	// Define the test.
	auto test = [](VirtualStateBlockManager& manager) {
		auto start_label = manager.get_start_block_label("p2");
		ASSERT_TRUE(start_label.first);
		ASSERT_TRUE(start_label.second == "base");
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, GetJacobianSingle) {
	// Define the test.
	auto test = [this](VirtualStateBlockManager& manager) {
		manager.add_virtual_state_block(std::make_shared<DummyVirtualStateBlock>("base", "p1"));
		auto res = manager.jacobian(base, "base", "p1", aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(res, zeros(6, 6));
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, GetJacobianDouble) {
	// Define the test.
	auto test = [this](VirtualStateBlockManager& manager) {
		manager.add_virtual_state_block(
		    std::make_shared<DummyVirtualStateBlock>("base", "p1", 2.0));
		manager.add_virtual_state_block(std::make_shared<DummyVirtualStateBlock>("p1", "p2", 3.0));

		auto res1 = manager.jacobian(base, "base", "p1", aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(res1, navtk::eye(6) * 2.0);

		auto cvt1 = manager.convert(base, "base", "p1", aspn_xtensor::TypeTimestamp((int64_t)0));
		auto res2 = manager.jacobian(cvt1, "p1", "p2", aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(res2, navtk::eye(6) * 3.0);

		auto res_both =
		    manager.jacobian(base, "base", "p2", aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(res_both, navtk::dot(res2, res1));
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, Reducing) {

	// Keep first 4
	std::function<Vector(const Vector&)> fx1 = [](const Vector& x) { return view(x, range(0, 4)); };

	std::function<Matrix(const Vector&)> jx1 = [](const Vector&) {
		return Matrix{
		    {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}};
	};

	// Keep last 2
	std::function<Vector(const Vector&)> fx2 = [](const Vector& x) { return view(x, range(2, 4)); };

	std::function<Matrix(const Vector&)> jx2 = [](const Vector&) {
		return Matrix{{0, 0, 1, 0}, {0, 0, 0, 1}};
	};

	// Define the test.
	auto test = [this, &fx1, &fx2, &jx1, &jx2](VirtualStateBlockManager& manager) {
		manager.add_virtual_state_block(std::make_shared<FirstOrderVirtualStateBlock>(
		    FirstOrderVirtualStateBlock("base", "p1", fx1, jx1)));
		manager.add_virtual_state_block(std::make_shared<FirstOrderVirtualStateBlock>(
		    FirstOrderVirtualStateBlock("p1", "p2", fx2, jx2)));

		auto cvt1 = manager.convert(base, "base", "p1", aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(view(defX, range(0, 4)), cvt1.estimate);
		ASSERT_ALLCLOSE(view(defP, range(0, 4), range(0, 4)), cvt1.covariance);

		auto jac_both =
		    manager.jacobian(base, "base", "p2", aspn_xtensor::TypeTimestamp((int64_t)0));
		Matrix exp_jac{{0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}};
		ASSERT_ALLCLOSE(exp_jac, jac_both);

		auto cvt_both =
		    manager.convert(base, "base", "p2", aspn_xtensor::TypeTimestamp((int64_t)0));
		ASSERT_ALLCLOSE(view(defX, range(2, 4)), cvt_both.estimate);
		ASSERT_ALLCLOSE(view(defP, range(2, 4), range(2, 4)), cvt_both.covariance);
	};

	// Execute the test on the original and copies.
	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, RemoveVirtualStateBlock1) {

	auto test = [](VirtualStateBlockManager& manager) {
		auto start_label = manager.get_start_block_label("unscaled");
		ASSERT_TRUE(start_label.first);
		ASSERT_EQ(start_label.second, "base");

		manager.remove_virtual_state_block("unscaled");

		EXPECT_FALSE(manager.get_start_block_label("unscaled").first);
	};

	test_manager_and_copies(test);
}

TEST_F(VirtualStateBlockManagerTests, RemoveVirtualStateBlock2) {

	auto test = [](VirtualStateBlockManager& manager) {
		manager.remove_virtual_state_block("whole");

		auto unscaled_vsb = manager.get_virtual_state_block("unscaled");
		EXPECT_NE(unscaled_vsb, nullptr);

		auto whole_vsb = manager.get_virtual_state_block("whole");
		EXPECT_EQ(whole_vsb, nullptr);
	};

	test_manager_and_copies(test);
}
