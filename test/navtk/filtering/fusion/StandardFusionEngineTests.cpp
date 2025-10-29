#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>
#include <xtensor/views/xview.hpp>

#include <navtk/errors.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/fusion/strategies/EkfStrategy.hpp>
#include <navtk/filtering/fusion/strategies/UkfStrategy.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/filtering/virtualstateblocks/ScaleVirtualStateBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using namespace xt::placeholders;
using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::ErrorMode;
using navtk::ErrorModeLock;
using navtk::eye;
using navtk::Matrix;
using navtk::MatrixT;
using navtk::not_null;
using navtk::num_cols;
using navtk::num_rows;
using navtk::ones;
using navtk::Scalar;
using navtk::Tensor;
using navtk::to_vec;
using navtk::Vector;
using navtk::zeros;
using navtk::filtering::GaussianVectorData;
using navtk::filtering::GenXhatPFunction;
using navtk::filtering::MeasurementProcessor;
using navtk::filtering::StandardDynamicsModel;
using navtk::filtering::StandardFusionEngine;
using navtk::filtering::StandardMeasurementModel;
using navtk::filtering::StateBlock;
using std::make_shared;
using std::unique_ptr;
using std::vector;
using xt::range;
using xt::view;

template <typename T>
T *anonymous() {
	return new T();
}


class FakeAux : public aspn_xtensor::AspnBase {
public:
	double d;
	FakeAux(double din) : aspn_xtensor::AspnBase(ASPN_EXTENDED_BEGIN, 0, 0, 0, 0) { d = din; }
};


class TestableStandardFusionEngine : public StandardFusionEngine {
public:
	using StandardFusionEngine::StandardFusionEngine;

	// Bring protected parent members into public
	using StandardFusionEngine::blocks;
	using StandardFusionEngine::cur_time;
	using StandardFusionEngine::expand_update_model;
	using StandardFusionEngine::find_block_idx_or_bail;
	using StandardFusionEngine::find_processor_idx_or_bail;
	using StandardFusionEngine::generate_x_and_p;
	using StandardFusionEngine::get_mat_indices;
	using StandardFusionEngine::get_mat_indices_list;
	using StandardFusionEngine::get_num_states;
	using StandardFusionEngine::process_covariance_cross_terms;
	using StandardFusionEngine::processors;
	using StandardFusionEngine::strategy;
	using StandardFusionEngine::vsb_man;
};


struct StandardFusionEngineTests : public ::testing::Test {

	class TestableProcessor : public MeasurementProcessor<> {
	public:
		TestableProcessor(std::string label, std::vector<std::string> state_block_labels)
		    : MeasurementProcessor(std::move(label), std::move(state_block_labels)) {}
	};

	class TestableBlock : public StateBlock<> {
	public:
		TestableBlock(size_t num_states, std::string label)
		    : StateBlock(num_states, std::move(label)) {}
	};

	class Processor : public TestableProcessor {
	public:
		std::shared_ptr<FakeAux> aux;
		Processor() : TestableProcessor("myprocessor", std::vector<std::string>{"myblock"}) {}
		virtual void receive_aux_data(const AspnBaseVector &r) override {
			aux = std::dynamic_pointer_cast<FakeAux>(r[0]);
		}
		virtual not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override {
			return make_shared<Processor>();
		}
		virtual std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase> meas,
		    GenXhatPFunction gen_x_and_p_func) override {
			auto xhat_p = gen_x_and_p_func(this->get_state_block_labels());
			auto shape  = xhat_p->estimate.size();
			auto data   = std::dynamic_pointer_cast<GaussianVectorData>(meas);

			return std::shared_ptr<StandardMeasurementModel>(new StandardMeasurementModel(
			    data->estimate,
			    [](Vector it) -> Vector { return it; },
			    eye(shape, shape),
			    data->covariance));
		}
	};

	class Processor2 : public TestableProcessor {
	public:
		Processor2()
		    : TestableProcessor("myprocessor2", std::vector<std::string>{"myblock2", "myblock3"}) {}
		virtual not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override {
			return make_shared<Processor2>();
		}
		virtual std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase>, GenXhatPFunction) override {
			return std::shared_ptr<StandardMeasurementModel>(new StandardMeasurementModel(
			    Vector{1},
			    [](Vector it) -> Vector { return Vector{xt::sum(it)(0)}; },
			    Matrix{{1, 1, 1, 1, 1}},
			    Matrix{{3}}));
		}
	};

	class Processor3 : public TestableProcessor {
	public:
		Processor3() : TestableProcessor("myprocessor3", std::vector<std::string>{"myblock2"}) {}
		virtual not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override {
			return make_shared<Processor3>();
		}
		virtual std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase>, GenXhatPFunction) override {
			return std::shared_ptr<StandardMeasurementModel>(new StandardMeasurementModel(
			    Vector({1, 1}),
			    [](Vector it) -> Vector { return it; },
			    eye(2, 2),
			    eye(2, 2) * 3.0));
		}
	};

	class NullModelProcessor : public TestableProcessor {
	public:
		NullModelProcessor(const std::string &block_label)
		    : TestableProcessor("myprocessor", {block_label}) {}
		virtual not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override {
			return make_shared<NullModelProcessor>(get_state_block_labels().front());
		}

		virtual std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase>, GenXhatPFunction) override {
			return nullptr;
		}
	};

	class Block : public TestableBlock {
	public:
		std::shared_ptr<FakeAux> aux;
		Block(size_t num_states = 1) : TestableBlock(num_states, "myblock") {}
		virtual void receive_aux_data(const AspnBaseVector &r) override {
			aux = std::dynamic_pointer_cast<FakeAux>(r[0]);
		}
		virtual not_null<std::shared_ptr<StateBlock<>>> clone() override {
			return make_shared<Block>();
		}
		virtual StandardDynamicsModel generate_dynamics(GenXhatPFunction,
		                                                aspn_xtensor::TypeTimestamp,
		                                                aspn_xtensor::TypeTimestamp) override {
			return StandardDynamicsModel(
			    [](Vector it) -> Vector { return it * 3.0; }, Matrix({{3}}), Matrix({{2}}));
		}
	};

	class Block2 : public TestableBlock {
	public:
		Block2() : TestableBlock(2, "myblock2") {}
		virtual not_null<std::shared_ptr<StateBlock<>>> clone() override {
			return make_shared<Block2>();
		}
		virtual StandardDynamicsModel generate_dynamics(
		    GenXhatPFunction,
		    aspn_xtensor::TypeTimestamp time_from,
		    aspn_xtensor::TypeTimestamp time_to) override {
			auto dt = to_seconds(time_to - time_from);
			return StandardDynamicsModel(
			    [=](Vector it) -> Vector { return 0.5 * 2 * it * pow(dt, 2); },
			    eye(2, 2) * 2 * dt,
			    eye(2, 2));
		}
	};

	class Block3 : public TestableBlock {
	public:
		Block3() : TestableBlock(3, "myblock3") {}
		virtual not_null<std::shared_ptr<StateBlock<>>> clone() override {
			return make_shared<Block3>();
		}
		virtual StandardDynamicsModel generate_dynamics(GenXhatPFunction,
		                                                aspn_xtensor::TypeTimestamp,
		                                                aspn_xtensor::TypeTimestamp) override {
			return StandardDynamicsModel(
			    [](Vector it) -> Vector { return it; }, eye(3, 3), eye(3, 3) * 1.5);
		}
	};

	void check_copy(TestableStandardFusionEngine &engine,
	                const TestableStandardFusionEngine &engine_copy) {
		// Verify that all basic fields on the engine are deeply copied.
		ASSERT_EQ(engine.cur_time, engine_copy.cur_time);
		engine.cur_time = engine.cur_time + 1.0;
		ASSERT_NE(engine.cur_time, engine_copy.cur_time);

		// Verify that the strategy is copied by checking the strategy type, replacing it on one
		// engine, and rechecking the type.
		ASSERT_NE(std::dynamic_pointer_cast<navtk::filtering::EkfStrategy>(engine.strategy),
		          nullptr);
		ASSERT_NE(std::dynamic_pointer_cast<navtk::filtering::EkfStrategy>(engine_copy.strategy),
		          nullptr);
		engine.strategy = make_shared<navtk::filtering::UkfStrategy>();
		ASSERT_EQ(std::dynamic_pointer_cast<navtk::filtering::EkfStrategy>(engine.strategy),
		          nullptr);
		ASSERT_NE(std::dynamic_pointer_cast<navtk::filtering::EkfStrategy>(engine_copy.strategy),
		          nullptr);


		// Call getter functions to make sure underlying members weren't invalidated by any copy
		// actions
		for (size_t ii = 0; ii < engine.blocks.size(); ++ii) {
			ASSERT_STREQ(engine.blocks[ii]->get_label().c_str(),
			             engine_copy.blocks[ii]->get_label().c_str());
		}
		for (size_t ii = 0; ii < engine.processors.size(); ++ii) {
			ASSERT_STREQ(engine.processors[ii]->get_label().c_str(),
			             engine_copy.processors[ii]->get_label().c_str());
			auto labs1 = engine.processors[ii]->get_state_block_labels();
			auto labs2 = engine_copy.processors[ii]->get_state_block_labels();
			for (size_t k = 0; k < labs1.size(); ++k) {
				ASSERT_STREQ(labs1[k].c_str(), labs2[k].c_str());
			}
		}
		auto strategy = engine.strategy;

		// Verify that the engines are deeply copied by adding a StateBlock to one engine and
		// asserting that their counts are not equal.
		engine.add_state_block(block3);
		ASSERT_NE(engine.blocks.size(), engine_copy.blocks.size());

		// Verify that cross terms are deeply copied by asserting that they are equal, changing one,
		// and asserting that they are not equal.
		ASSERT_EQ(engine.process_covariance_cross_terms[0].term,
		          engine_copy.process_covariance_cross_terms[0].term);
		engine.process_covariance_cross_terms[0].term += 1;
		ASSERT_NE(engine.process_covariance_cross_terms[0].term,
		          engine_copy.process_covariance_cross_terms[0].term);

		// Verify that the virtual state block manager is deeply copied by asserting that they are
		// equal, changing one, and asserting that they are not equal.
		ASSERT_EQ(engine.vsb_man.get_start_block_label("unscaled"),
		          engine_copy.vsb_man.get_start_block_label("unscaled"));
		engine.vsb_man = navtk::filtering::VirtualStateBlockManager();
		engine.add_virtual_state_block(
		    make_shared<navtk::filtering::ScaleVirtualStateBlock>("base2", "unscaled", zeros(6)));
		ASSERT_NE(engine.vsb_man.get_start_block_label("unscaled"),
		          engine_copy.vsb_man.get_start_block_label("unscaled"));
	}

	StandardFusionEngineTests()
	    : block(make_shared<Block>()),
	      block2(make_shared<Block2>()),
	      block3(make_shared<Block3>()),
	      invalid_block(make_shared<Block>(2)),
	      processor(make_shared<Processor>()),
	      processor2(make_shared<Processor2>()),
	      processor3(make_shared<Processor3>()) {
		for (size_t ii = 0; ii < 5; ++ii)
			measurements.push_back(std::make_shared<GaussianVectorData>(
			    to_type_timestamp(ii), Vector{1}, Matrix{{8}}));
	}

	not_null<std::shared_ptr<StateBlock<>>> block;
	not_null<std::shared_ptr<StateBlock<>>> block2;
	not_null<std::shared_ptr<StateBlock<>>> block3;
	// same as block, but with num_states set to 2, even though g(x) only propagates 1 state
	not_null<std::shared_ptr<StateBlock<>>> invalid_block;
	not_null<std::shared_ptr<MeasurementProcessor<>>> processor;
	not_null<std::shared_ptr<MeasurementProcessor<>>> processor2;
	not_null<std::shared_ptr<MeasurementProcessor<>>> processor3;
	vector<std::shared_ptr<aspn_xtensor::AspnBase>> measurements;
};

TEST_F(StandardFusionEngineTests, NoVsb) {
	StandardFusionEngine engine;
	auto guard = ErrorModeLock(ErrorMode::DIE);
	ASSERT_FALSE(engine.has_virtual_state_block("test"));
}


TEST_F(StandardFusionEngineTests, SimplePropagate) {
	StandardFusionEngine engine;
	engine.add_state_block(block);
	engine.set_state_block_estimate("myblock", Vector{5.5});
	engine.set_state_block_covariance("myblock", Matrix{{1.0}});
	engine.propagate(to_type_timestamp(3.5));
	auto out     = engine.get_state_block_estimate("myblock");
	auto out_var = engine.get_state_block_covariance("myblock");
	ASSERT_ALLCLOSE(Vector{16.5}, out);
	ASSERT_ALLCLOSE(Matrix{{11.0}}, out_var);
}

TEST_F(StandardFusionEngineTests, SimpleUpdate) {
	StandardFusionEngine engine;
	engine.add_state_block(block);
	engine.set_state_block_estimate("myblock", Vector{9});
	engine.set_state_block_covariance("myblock", Matrix{{8}});
	engine.add_measurement_processor(processor);
	engine.update("myprocessor", measurements[0]);

	auto out     = engine.get_state_block_estimate("myblock");
	auto out_var = engine.get_state_block_covariance("myblock");
	ASSERT_ALLCLOSE(Vector{5}, out);
	ASSERT_ALLCLOSE(Matrix{{4}}, out_var);
}

TEST_F(StandardFusionEngineTests, UpdateConverges) {
	StandardFusionEngine engine;
	engine.add_state_block(block);
	engine.set_state_block_estimate("myblock", Vector{9});
	engine.set_state_block_covariance("myblock", Matrix{{8}});
	engine.add_measurement_processor(processor);
	for (auto &meas : measurements) {
		auto data        = std::dynamic_pointer_cast<GaussianVectorData>(meas);
		data->covariance = Matrix{{0.0001}};
		engine.update("myprocessor", meas);
	}

	auto out     = engine.get_state_block_estimate("myblock");
	auto out_var = engine.get_state_block_covariance("myblock");
	ASSERT_ALLCLOSE_EX(Vector{1}, out, 1e-3, 1e-8);
	ASSERT_ALLCLOSE_EX(Matrix{{0}}, out_var, 0, 1e-3);
}

TEST_F(StandardFusionEngineTests, UpdateNoTimestamp) {
	StandardFusionEngine engine;
	auto timestampless_meas = make_shared<FakeAux>(1.0);
	EXPECT_UB_OR_DIE(engine.update("myprocessor", timestampless_meas),
	                 "Does not contain a timestamp.",
	                 std::invalid_argument);
}

TEST_F(StandardFusionEngineTests, UpdateAbortIfNull) {
	StandardFusionEngine engine;
	engine.add_state_block(block);
	engine.set_state_block_estimate("myblock", Vector{9});
	engine.set_state_block_covariance("myblock", Matrix{{8}});
	engine.add_measurement_processor(std::make_shared<NullModelProcessor>("myblock"));
	engine.update("myprocessor", measurements[0]);
	EXPECT_ALLCLOSE(Vector{9}, engine.get_state_block_estimate("myblock"));
	EXPECT_ALLCLOSE(Matrix{{8}}, engine.get_state_block_covariance("myblock"));
}

TEST_F(StandardFusionEngineTests, UpdateAbortIfNullWithVSB) {
	StandardFusionEngine engine;
	engine.add_state_block(block);
	engine.set_state_block_estimate("myblock", Vector{9});
	engine.set_state_block_covariance("myblock", Matrix{{8}});
	engine.add_virtual_state_block(
	    make_shared<navtk::filtering::ScaleVirtualStateBlock>("myblock", "vsb", zeros(6)));
	engine.add_measurement_processor(std::make_shared<NullModelProcessor>("vsb"));
	engine.update("myprocessor", measurements[0]);
	EXPECT_ALLCLOSE(Vector{9}, engine.get_state_block_estimate("myblock"));
	EXPECT_ALLCLOSE(Matrix{{8}}, engine.get_state_block_covariance("myblock"));
}

TEST_F(StandardFusionEngineTests, ProcessorCaching) {
	StandardFusionEngine engine;
	engine.add_measurement_processor(processor);
	auto out = engine.get_measurement_processor("myprocessor");
	EXPECT_EQ(processor, out);
	// The rest of this test in the Java version serves no purpose.
}

TEST_F(StandardFusionEngineTests, ProcessorLabelInvalid) {
	StandardFusionEngine engine;
	engine.add_measurement_processor(processor);

	EXPECT_THROW(engine.get_measurement_processor("wrongname"), std::invalid_argument);
	engine.get_measurement_processor("myprocessor");
	EXPECT_THROW(engine.get_measurement_processor("wrongnameagain"), std::invalid_argument);
}

TEST_F(StandardFusionEngineTests, ProcessorWithNoStateBlocks) {
	StandardFusionEngine engine;
	engine.add_measurement_processor(processor);
	EXPECT_THROW(engine.update("myprocessor", measurements[0]), std::invalid_argument);
}

// The next few tests expects an xtensor exception due to an assertion that is enabled by
// XTENSOR_ENABLE_ASSERT.
#ifdef XTENSOR_ENABLE_ASSERT
TEST_F(StandardFusionEngineTests, ProcessorBadDimensions) {
	StandardFusionEngine engine;
	class MyProcessor : public Processor {
	public:
		virtual std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase>, GenXhatPFunction) override {
			return std::shared_ptr<StandardMeasurementModel>(new StandardMeasurementModel(
			    eye(1, 1),
			    [](Vector it) -> Vector { return 4 * it; },
			    4 * eye(2, 2),
			    3 * eye(1, 1)));
		}
	};

	engine.add_measurement_processor(make_shared<MyProcessor>());
	engine.add_state_block(block);

	EXPECT_THROW(engine.update("myprocessor", measurements[0]), std::runtime_error);
}

TEST_F(StandardFusionEngineTests, ProcessorWrong_h_Dims) {
	StandardFusionEngine engine;
	class MyProcessor : public Processor {
	public:
		virtual std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase>, GenXhatPFunction) override {
			return std::shared_ptr<StandardMeasurementModel>(new StandardMeasurementModel(
			    zeros(2, 1),
			    [](MatrixT<Scalar>) -> MatrixT<Scalar> { return eye(2, 2); },
			    4 * ones(2, 2),
			    3 * eye(2)));
		}
	};

	engine.add_measurement_processor(make_shared<MyProcessor>());
	engine.add_state_block(block);

	EXPECT_THROW(engine.update("myprocessor", measurements[0]), std::runtime_error);
}

TEST_F(StandardFusionEngineTests, ProcessorWrongNumStates) {
	StandardFusionEngine engine;
	class MyProcessor : public Processor {
	public:
		virtual std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase>, GenXhatPFunction) override {
			return std::shared_ptr<StandardMeasurementModel>(new StandardMeasurementModel(
			    zeros(2, 1),
			    [](Vector it) -> Vector { return 3 * it; },
			    4 * eye(2, 2),
			    3 * eye(2, 2)));
		}
	};

	engine.add_measurement_processor(make_shared<MyProcessor>());
	engine.add_state_block(block);

	EXPECT_THROW(engine.update("myprocessor", measurements[0]), std::runtime_error);
}
#endif

TEST_F(StandardFusionEngineTests, StateBlockCaching) {
	StandardFusionEngine engine;
	engine.add_state_block(block);
	auto out = engine.get_state_block("myblock");
	EXPECT_EQ(block, out);
	// The rest of this test in the Java version serves no purpose
}

TEST_F(StandardFusionEngineTests, StateBlockLabelInvalid) {
	StandardFusionEngine engine;
	engine.add_state_block(block);

	EXPECT_THROW(engine.get_state_block("wrongname"), std::invalid_argument);
	engine.get_state_block("myblock");
	EXPECT_THROW(engine.get_state_block("wrongnameagain"), std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, StandardFusionEngineTests, StateBlockWrongNumStates) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_measurement_processor(test.processor);
	engine.add_state_block(test.invalid_block);

	EXPECT_HONORS_MODE(engine.propagate(to_type_timestamp(2.0)), "invalid");
}

class BadBlock : public StateBlock<> {
public:
	BadBlock() : StateBlock(1, "myblock") {}
	virtual void receive_aux_data(const AspnBaseVector &) override {}
	virtual not_null<std::shared_ptr<StateBlock<>>> clone() override {
		return make_shared<BadBlock>();
	}
	virtual StandardDynamicsModel generate_dynamics(GenXhatPFunction,
	                                                aspn_xtensor::TypeTimestamp,
	                                                aspn_xtensor::TypeTimestamp) override {
		return StandardDynamicsModel(
		    [](Vector it) -> Vector { return it * 3.0; }, zeros(3, 1), zeros(3, 1));
	}
};

ERROR_MODE_SENSITIVE_TEST(TEST_F, StandardFusionEngineTests, StateBlockBadDimensions) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_measurement_processor(test.processor);
	engine.add_state_block(make_shared<BadBlock>());
	EXPECT_HONORS_MODE(engine.propagate(to_type_timestamp(2.0)), "invalid");
}

TEST_F(StandardFusionEngineTests, GenerateXAndP) {
	TestableStandardFusionEngine engine(to_type_timestamp(1.0));
	Vector x1{1, 2};
	Vector x2{3, 4, 5};
	Matrix cov1{{1, -.5}, {-.5, 2}};
	Matrix cov2 = eye(3) * 3;
	Vector expX = xt::concatenate(xt::xtuple(x1, x2));
	Matrix expP = zeros(5, 5);
	view(expP, range(0, num_rows(cov1)), range(0, num_cols(cov1))) = cov1;
	view(expP, range(num_rows(cov1), _), range(num_cols(cov1), _)) = cov2;

	engine.add_measurement_processor(processor2);
	engine.add_state_block(block2);
	engine.add_state_block(block);
	engine.add_state_block(block3);
	engine.set_state_block_estimate(block2->get_label(), x1);
	engine.set_state_block_estimate(block3->get_label(), x2);
	engine.set_state_block_covariance(block2->get_label(), cov1);
	engine.set_state_block_covariance(block3->get_label(), cov2);

	auto x_and_p_pair = engine.generate_x_and_p(processor2->get_state_block_labels());

	ASSERT_ALLCLOSE(expX, x_and_p_pair->estimate);
	ASSERT_ALLCLOSE(expP, x_and_p_pair->covariance);
}

TEST_F(StandardFusionEngineTests, SetStateBlockEst) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block2);
	engine.set_state_block_estimate(block2->get_label(), ones(2));
	ASSERT_ALLCLOSE(ones(2), engine.get_state_block_estimate(block2->get_label()));
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, StandardFusionEngineTests, SetInvalidStateBlockEst) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(test.block2);
	auto label2 = test.block2->get_label();
	engine.set_state_block_estimate(label2, ones(2));

	EXPECT_HONORS_MODE(engine.set_state_block_estimate(label2, Vector{1.0}), "invalid");

// This xtensor exception is due to an assertion that is enabled by XTENSOR_ENABLE_ASSERT.
#ifdef XTENSOR_ENABLE_ASSERT
	// Because our declared input type is Vector, these trigger xtensor's "cannot change dimension"
	// exception. Consequently they don't honor the error mode :(
	EXPECT_THROW(engine.set_state_block_estimate(label2, ones(1, 2)), std::runtime_error);
	EXPECT_THROW(engine.set_state_block_estimate(label2, ones(3, 1)), std::runtime_error);
#endif

	// When error checking is disabled, behavior is undefined, otherwise we are prevented from
	// writing the invalid estimate.
	if (mode != ErrorMode::OFF) {
		ASSERT_ALLCLOSE(ones(2), engine.get_state_block_estimate(label2));
	}
}

TEST_F(StandardFusionEngineTests, SetStateBlockCov) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	Matrix cov1({{1.0, -0.5}, {-0.5, 2.0}});
	engine.add_state_block(block2);

// This xtensor exception is due to an assertion that is enabled by XTENSOR_ENABLE_ASSERT.
#ifdef XTENSOR_ENABLE_ASSERT
	EXPECT_THROW(engine.set_state_block_covariance(block2->get_label(), Vector{1.0}),
	             std::runtime_error);
#endif

	engine.set_state_block_covariance(block2->get_label(), cov1);
	ASSERT_ALLCLOSE(cov1, engine.get_state_block_covariance(block2->get_label()));
	EXPECT_THROW(engine.set_state_block_covariance(block2->get_label(), eye(3, 3)),
	             std::runtime_error);
}

TEST_F(StandardFusionEngineTests, RemoveStateBlock) {
	auto x0 = Vector{0};
	Vector x1{2, 2};
	Vector x2{3, 3, 3};
	Matrix p0 = eye(1, 1);
	Matrix p1 = eye(2, 2) * 2;
	Matrix p2 = eye(3, 3) * 3;

	StandardFusionEngine engine = StandardFusionEngine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.add_state_block(block3);
	engine.set_state_block_estimate(block->get_label(), x0);
	engine.set_state_block_estimate(block2->get_label(), x1);
	engine.set_state_block_estimate(block3->get_label(), x2);
	engine.set_state_block_covariance(block->get_label(), p0);
	engine.set_state_block_covariance(block2->get_label(), p1);
	engine.set_state_block_covariance(block3->get_label(), p2);

	ASSERT_ALLCLOSE(p0, engine.get_state_block_covariance(block->get_label()));
	ASSERT_ALLCLOSE(p1, engine.get_state_block_covariance(block2->get_label()));
	ASSERT_ALLCLOSE(p2, engine.get_state_block_covariance(block3->get_label()));

	engine.remove_state_block(block2->get_label());

	ASSERT_ALLCLOSE(p0, engine.get_state_block_covariance(block->get_label()));
	ASSERT_ALLCLOSE(p2, engine.get_state_block_covariance(block3->get_label()));
	EXPECT_THROW(engine.get_state_block_covariance(block2->get_label()), std::invalid_argument);
	EXPECT_THROW(engine.get_state_block(block2->get_label()), std::invalid_argument);
	EXPECT_EQ(2, engine.get_state_block_names_list().size());
	ASSERT_ALLCLOSE(x0, engine.get_state_block_estimate(block->get_label()));
	ASSERT_ALLCLOSE(x2, engine.get_state_block_estimate(block3->get_label()));
	EXPECT_THROW(engine.get_state_block_estimate(block2->get_label()), std::invalid_argument);
	EXPECT_THROW(engine.get_state_block_covariance(block2->get_label()), std::invalid_argument);


	// Remove another block
	engine.remove_state_block(block->get_label());
	EXPECT_THROW(engine.get_state_block(block->get_label()), std::invalid_argument);
	EXPECT_EQ(1, engine.get_state_block_names_list().size());
	ASSERT_ALLCLOSE(x2, engine.get_state_block_estimate(block3->get_label()));
	EXPECT_THROW(engine.get_state_block_estimate(block->get_label()), std::invalid_argument);
	EXPECT_THROW(engine.get_state_block_covariance(block->get_label()), std::invalid_argument);
}

TEST_F(StandardFusionEngineTests, SetCrossTermPMatrix) {
	TestableStandardFusionEngine engine(to_type_timestamp(1.0));
	auto cov    = Matrix{{10}};
	Matrix cov1 = Matrix({{1, -0.5}, {-0.5, 2}});
	Matrix cov2 = eye(3, 3) * 3;
	Matrix cross_term1({{0.5, -0.5}});
	Matrix cross_term2({{0.1, 0.2, 0.3}, {-0.1, -0.2, -0.3}});
	Matrix exp({{10.0, 0.5, -.5, 0.0, 0.0, 0.0},
	            {0.5, 1.0, -.5, 0.1, 0.2, 0.3},
	            {-.5, -.5, 2.0, -.1, -.2, -.3},
	            {0.0, 0.1, -.1, 3.0, 0.0, 0.0},
	            {0.0, 0.2, -.2, 0.0, 3.0, 0.0},
	            {0.0, 0.3, -.3, 0.0, 0.0, 3.0}});
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.add_state_block(block3);
	engine.set_state_block_covariance(block->get_label(), cov);
	engine.set_state_block_covariance(block2->get_label(), cov1);
	engine.set_state_block_covariance(block3->get_label(), cov2);

	// When error checking is enabled, setting invalid cross terms has no effect.
	{
		ErrorModeLock guard{ErrorMode::LOG};
		EXPECT_ERROR(
		    engine.set_cross_term_covariance(block->get_label(), block3->get_label(), cross_term1),
		    "invalid");
	}
	engine.set_cross_term_covariance(block->get_label(), block2->get_label(), cross_term1);
	engine.set_cross_term_covariance(block2->get_label(), block3->get_label(), cross_term2);
	ASSERT_ALLCLOSE(exp, engine.generate_x_and_p(engine.get_state_block_names_list())->covariance);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, StandardFusionEngineTests, SetInvalidCrossTermPMatrix) {
	TestableStandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(test.block);
	engine.add_state_block(test.block2);
	engine.add_state_block(test.block3);
	Matrix cross_term1({{0.5, -0.5}});
	Matrix cross_term2({{0.1, 0.2, 0.3}, {-0.1, -0.2, -0.3}});
	auto label1 = test.block->get_label();
	auto label2 = test.block2->get_label();

	engine.set_cross_term_covariance(label1, label2, cross_term1);
	EXPECT_HONORS_MODE(engine.set_cross_term_covariance(label1, label2, cross_term2),
	                   "invalid matrix dimension");

	// When error checking is off, the behavior is undefined, but if error checking was on, we
	// were prevented from setting a covariance of invalid size.
	if (mode != ErrorMode::OFF) {
		ASSERT_ALLCLOSE(cross_term1, engine.get_cross_term_covariance(label1, label2));
	}
}

TEST_F(StandardFusionEngineTests, ExpandUpdateModel) {
	TestableStandardFusionEngine engine(to_type_timestamp(1.0));
	auto x0 = Vector{1};
	Vector x1{2, 2};
	Vector x2{3, 3, 3};
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.add_state_block(block3);
	engine.set_state_block_estimate(block->get_label(), x0);
	engine.set_state_block_estimate(block2->get_label(), x1);
	engine.set_state_block_estimate(block3->get_label(), x2);
	engine.add_measurement_processor(processor);
	engine.add_measurement_processor(processor2);
	engine.add_measurement_processor(processor3);

	auto gen_x_and_p_func = [&engine](const std::vector<std::string> &labels)
	    -> std::shared_ptr<navtk::filtering::EstimateWithCovariance> {
		return engine.generate_x_and_p(labels);
	};
	auto model1 = engine.expand_update_model(
	    *(processor->generate_model(
	        std::make_shared<GaussianVectorData>(to_type_timestamp(0), Vector{1}, Matrix{{1}}),
	        gen_x_and_p_func)),
	    *processor);
	auto model2 = engine.expand_update_model(
	    *(processor2->generate_model(
	        std::make_shared<GaussianVectorData>(to_type_timestamp(0), Vector{1}, Matrix{{1}}),
	        gen_x_and_p_func)),
	    *processor2);
	auto model3 = engine.expand_update_model(
	    *(processor3->generate_model(
	        std::make_shared<GaussianVectorData>(to_type_timestamp(0), ones(2), eye(2, 2)),
	        gen_x_and_p_func)),
	    *processor3);
	auto x_and_p_all = engine.generate_x_and_p(engine.get_state_block_names_list());
	// First block
	ASSERT_ALLCLOSE((Matrix{{1, 0, 0, 0, 0, 0}}), model1->H);
	ASSERT_ALLCLOSE(x0, model1->h(x_and_p_all->estimate));

	// Last 2 blocks
	ASSERT_ALLCLOSE((Matrix{{0, 1, 1, 1, 1, 1}}), model2->H);
	Vector expected = to_vec(sum(x1) + sum(x2));
	Vector actual   = model2->h(x_and_p_all->estimate);
	ASSERT_ALLCLOSE(expected, actual);

	// Block sandwiched between 2 others
	ASSERT_ALLCLOSE((Matrix{{0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}}), model3->H);
	ASSERT_ALLCLOSE(x1, model3->h(x_and_p_all->estimate));
}

TEST_F(StandardFusionEngineTests, AnotherPropTest) {
	StandardFusionEngine engine(aspn_xtensor::TypeTimestamp((int64_t)0));
	engine.add_state_block(block2);
	engine.set_state_block_estimate(block2->get_label(), ones(2));
	engine.set_state_block_covariance(block2->get_label(), eye(2, 2));
	engine.propagate(to_type_timestamp(3.0));
	ASSERT_ALLCLOSE((Matrix{{9}, {9}}), engine.get_state_block_estimate(block2->get_label()));
}

TEST_F(StandardFusionEngineTests, ResetFilterStates) {
	TestableStandardFusionEngine engine(to_type_timestamp(1.0));
	Vector x0{1};
	Vector x1{2, 2};
	Vector x2{3, 3, 3};
	auto cov0 = Matrix{{10}};
	auto cov1 = Matrix{{1, -0.5}, {-0.5, 2}};
	auto cov2 = eye(3, 3) * 3.0;
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.add_state_block(block3);
	engine.set_state_block_estimate(block->get_label(), x0);
	engine.set_state_block_estimate(block2->get_label(), x1);
	engine.set_state_block_estimate(block3->get_label(), x2);
	engine.set_state_block_covariance(block->get_label(), cov0);
	engine.set_state_block_covariance(block2->get_label(), cov1);
	engine.set_state_block_covariance(block3->get_label(), cov2);

	// Do some resets
	auto reset_vals = engine.reset_state_estimate(to_type_timestamp(1.0), block->get_label(), {0});
	ASSERT_ALLCLOSE(x0, reset_vals.estimate);
	ASSERT_ALLCLOSE(cov0, reset_vals.covariance);
	ASSERT_ALLCLOSE(Vector{0}, engine.get_state_block_estimate(block->get_label()));
	ASSERT_ALLCLOSE(cov0, engine.get_state_block_covariance(block->get_label()));
	ASSERT_ALLCLOSE(concatenate(xtuple(Vector{0}, x1, x2)),
	                engine.generate_x_and_p(engine.get_state_block_names_list())->estimate);

	// Middle of block
	reset_vals = engine.reset_state_estimate(to_type_timestamp(1.0), block3->get_label(), {1});
	ASSERT_ALLCLOSE((Matrix{{0}, {x2(1)}, {0}}), reset_vals.estimate);
	ASSERT_ALLCLOSE(cov2, reset_vals.covariance);
	ASSERT_ALLCLOSE((Vector{x2[0], 0, x2[2]}),
	                engine.get_state_block_estimate(block3->get_label()));
	auto names_list = engine.get_state_block_names_list();
	auto ewc        = engine.generate_x_and_p(names_list);
	ASSERT_ALLCLOSE((Vector{0, x1[0], x1[0], x2[0], 0, x2[2]}), ewc->estimate);
}

TEST_F(StandardFusionEngineTests, ResetFilterStatesErrors) {
	TestableStandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);

	// Reset in past
	EXPECT_UB_OR_DIE(engine.reset_state_estimate(
	                     aspn_xtensor::TypeTimestamp((int64_t)0), block->get_label(), {0}),
	                 "Cannot reset filter states in past. Filter at time 1.000000000s, reset "
	                 "requested at 0.000000000s",
	                 std::invalid_argument);

	// Bad indices
	EXPECT_UB_OR_DIE(engine.reset_state_estimate(to_type_timestamp(1.0), block->get_label(), {}),
	                 "Indices into state vector argument cannot be empty",
	                 std::invalid_argument);
	EXPECT_UB_OR_DIE(
	    engine.reset_state_estimate(to_type_timestamp(1.0), block->get_label(), {0, 2}),
	    "Given indices exceed the maximum state block index available",
	    std::invalid_argument);
}


TEST_F(StandardFusionEngineTests, SetCrossTermQdMat) {
	TestableStandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.add_state_block(block3);
	auto nl         = engine.get_state_block_names_list();
	auto expected   = engine.peek_ahead(to_type_timestamp(2.0), nl)->covariance;
	auto expected_2 = engine.peek_ahead(to_type_timestamp(2.0), {block2->get_label()})->covariance;
	auto expected_1_3 =
	    engine.peek_ahead(to_type_timestamp(2.0), {block->get_label(), block3->get_label()})
	        ->covariance;

	expected(0, 2)++;
	expected(2, 0)++;
	engine.set_cross_term_process_covariance(
	    block->get_label(), block2->get_label(), Matrix{{0, 1}});

	// setting an invalid covariance while error checking is enabled doesn't impact the result
	{
		ErrorModeLock guard{ErrorMode::LOG};
		EXPECT_ERROR(engine.set_cross_term_process_covariance(
		                 block->get_label(), block2->get_label(), ones(2, 2)),
		             "invalid matrix dimensions");
	}

	engine.propagate(to_type_timestamp(2.0));
	ASSERT_ALLCLOSE(expected, engine.generate_x_and_p(nl)->covariance);
	ASSERT_ALLCLOSE(expected_2, engine.generate_x_and_p({block2->get_label()})->covariance);
	ASSERT_ALLCLOSE(expected_1_3,
	                engine.generate_x_and_p({block->get_label(), block3->get_label()})->covariance);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, StandardFusionEngineTests, SetInvalidCrossTermQdMat) {
	TestableStandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(test.block);
	engine.add_state_block(test.block2);
	engine.add_state_block(test.block3);
	EXPECT_HONORS_MODE(engine.set_cross_term_process_covariance(
	                       test.block->get_label(), test.block2->get_label(), ones(2, 2)),
	                   "invalid matrix dimensions");
}

TEST_F(StandardFusionEngineTests, SetCrossTermQdMatAgain) {
	TestableStandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.add_state_block(block3);
	auto nl       = engine.get_state_block_names_list();
	auto expected = engine.peek_ahead(to_type_timestamp(2.0), nl)->covariance;
	expected(0, 2)++;
	expected(2, 0)++;
	engine.set_cross_term_process_covariance(
	    block->get_label(), block2->get_label(), Matrix{{0, 2}});
	engine.set_cross_term_process_covariance(
	    block->get_label(), block2->get_label(), Matrix{{0, 1}});
	engine.propagate(to_type_timestamp(2.0));
	ASSERT_TRUE(allclose(expected, engine.generate_x_and_p(nl)->covariance));
}

TEST_F(StandardFusionEngineTests, GetCrossTermPSingleFail) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	EXPECT_THROW(engine.get_cross_term_covariance(block->get_label(), block2->get_label()),
	             std::invalid_argument);
}

TEST_F(StandardFusionEngineTests, GetCrossTermPSingleSameLabel) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	engine.set_state_block_covariance(block->get_label(), Matrix{{9}});
	ASSERT_ALLCLOSE(Matrix{{9}},
	                engine.get_cross_term_covariance(block->get_label(), block->get_label()));
}

TEST_F(StandardFusionEngineTests, GetCrossTermPDoubleSameLabel) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.set_state_block_covariance(block->get_label(), Matrix{{1}});
	engine.set_state_block_covariance(block2->get_label(), Matrix{{1.5, 3.5}, {3.5, 4.5}});
	engine.set_cross_term_covariance(block->get_label(), block2->get_label(), Matrix{{3.0, 4.0}});
	ASSERT_ALLCLOSE(Matrix{{1}},
	                engine.get_cross_term_covariance(block->get_label(), block->get_label()));
}

TEST_F(StandardFusionEngineTests, GetCrossTermPDouble) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.set_state_block_covariance(block->get_label(), Matrix{{1}});
	engine.set_state_block_covariance(block2->get_label(), Matrix({{1.5, 3.5}, {3.5, 4.5}}));
	engine.set_cross_term_covariance(block->get_label(), block2->get_label(), Matrix({{3.0, 4.0}}));
	ASSERT_ALLCLOSE((Matrix{{3, 4}}),
	                engine.get_cross_term_covariance(block->get_label(), block2->get_label()));
}


TEST_F(StandardFusionEngineTests, GetCrossTermPDoubleWrongLabelFail) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	EXPECT_THROW(engine.get_cross_term_covariance(block->get_label(), block3->get_label()),
	             std::invalid_argument);
	EXPECT_THROW(engine.get_cross_term_covariance(block3->get_label(), block->get_label()),
	             std::invalid_argument);
}

TEST_F(StandardFusionEngineTests, GetCrossTermPVsbAndActual) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.set_state_block_covariance(block->get_label(), Matrix{{1}});
	engine.set_state_block_covariance(block2->get_label(), Matrix({{1.5, 3.5}, {3.5, 4.5}}));

	auto vsb = make_shared<navtk::filtering::ScaleVirtualStateBlock>(
	    block->get_label(), "Scaled", Vector({3.0}));
	engine.add_virtual_state_block(vsb);

	engine.set_cross_term_covariance(block->get_label(), block2->get_label(), Matrix({{3.0, 4.0}}));

	ASSERT_ALLCLOSE((Matrix{{9.0, 12.0}}),
	                engine.get_cross_term_covariance(vsb->get_target(), block2->get_label()));
}

TEST_F(StandardFusionEngineTests, GetCrossTermPActualAndVsb) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.set_state_block_covariance(block->get_label(), Matrix{{1}});
	engine.set_state_block_covariance(block2->get_label(), Matrix({{1.5, 3.5}, {3.5, 4.5}}));

	auto vsb = make_shared<navtk::filtering::ScaleVirtualStateBlock>(
	    block2->get_label(), "Scaled", Vector({2.0, 2.0}));
	engine.add_virtual_state_block(vsb);

	engine.set_cross_term_covariance(block->get_label(), block2->get_label(), Matrix({{3.0, 4.0}}));

	ASSERT_ALLCLOSE((Matrix{{6.0, 8.0}}),
	                engine.get_cross_term_covariance(block->get_label(), vsb->get_target()));
}

TEST_F(StandardFusionEngineTests, GetCrossTermPVsbAndVsb) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.set_state_block_covariance(block->get_label(), Matrix{{1}});
	engine.set_state_block_covariance(block2->get_label(), Matrix({{1.5, 3.5}, {3.5, 4.5}}));

	auto vsb1 = make_shared<navtk::filtering::ScaleVirtualStateBlock>(
	    block->get_label(), "Scaled1", Vector({3.0}));
	engine.add_virtual_state_block(vsb1);
	auto vsb2 = make_shared<navtk::filtering::ScaleVirtualStateBlock>(
	    block2->get_label(), "Scaled2", Vector({2.0, 2.0}));
	engine.add_virtual_state_block(vsb2);

	engine.set_cross_term_covariance(block->get_label(), block2->get_label(), Matrix({{3.0, 4.0}}));

	ASSERT_ALLCLOSE((Matrix{{18.0, 24.0}}),
	                engine.get_cross_term_covariance(vsb1->get_target(), vsb2->get_target()));
}

TEST_F(StandardFusionEngineTests, GetCrossTermPThreeBlock) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.add_state_block(block3);
	engine.set_state_block_covariance(block->get_label(), Matrix{{2}});
	engine.set_state_block_covariance(block2->get_label(), Matrix{{1.5, 3.5}, {3.5, 4.5}});
	engine.set_state_block_covariance(block3->get_label(), eye(3, 3));
	engine.set_cross_term_covariance(block->get_label(), block2->get_label(), (Matrix{{1.1, 2.2}}));
	engine.set_cross_term_covariance(
	    block->get_label(), block3->get_label(), Matrix{{3.3, 4.4, 5.5}});
	engine.set_cross_term_covariance(
	    block2->get_label(), block3->get_label(), Matrix{{6.6, 7.7, 8.8}, {9.9, 10, 11.11}});


	ASSERT_ALLCLOSE((Matrix{{1.1, 2.2}}),
	                engine.get_cross_term_covariance(block->get_label(), block2->get_label()));
	ASSERT_TRUE(
	    allclose(Matrix{{3.3, 4.4, 5.5}},
	             engine.get_cross_term_covariance(block->get_label(), block3->get_label())));
	ASSERT_TRUE(
	    allclose(Matrix{{6.6, 7.7, 8.8}, {9.9, 10, 11.11}},
	             engine.get_cross_term_covariance(block2->get_label(), block3->get_label())));

	Matrix whole_p{{2.0, 1.1, 2.2, 3.3, 4.4, 5.5},
	               {1.1, 1.5, 3.5, 6.6, 7.7, 8.8},
	               {2.2, 3.5, 4.5, 9.9, 10.0, 11.11},
	               {3.3, 6.6, 9.9, 1.0, 0.0, 0.0},
	               {4.4, 7.7, 10.0, 0.0, 1.0, 0.0},
	               {5.5, 8.8, 11.11, 0.0, 0.0, 1.0}};

	ASSERT_ALLCLOSE(whole_p,
	                engine.generate_x_and_p(engine.get_state_block_names_list())->covariance);
}

TEST_F(StandardFusionEngineTests, blockValidAux) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	auto aux = make_shared<FakeAux>(1.0);
	AspnBaseVector aux_vec{aux};
	engine.add_state_block(block);
	engine.give_state_block_aux_data(block->get_label(), aux_vec);
	ASSERT_TRUE(std::dynamic_pointer_cast<Block>(block)->aux == aux);
}

TEST_F(StandardFusionEngineTests, blockInvalidAux) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	auto aux = make_shared<FakeAux>(1.0);
	ASSERT_THROW(engine.give_state_block_aux_data("bloop", {aux}), std::invalid_argument);
}

TEST_F(StandardFusionEngineTests, removeProcessor) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_measurement_processor(processor);
	engine.add_measurement_processor(processor2);
	engine.add_measurement_processor(processor3);

	engine.get_measurement_processor(processor->get_label());
	engine.get_measurement_processor(processor2->get_label());
	engine.get_measurement_processor(processor3->get_label());

	engine.remove_measurement_processor(processor2->get_label());
	engine.get_measurement_processor(processor->get_label());
	engine.get_measurement_processor(processor3->get_label());
	ASSERT_THROW(engine.get_measurement_processor(processor2->get_label()), std::invalid_argument);

	engine.remove_measurement_processor(processor->get_label());
	engine.get_measurement_processor(processor3->get_label());
	ASSERT_THROW(engine.get_measurement_processor(processor->get_label()), std::invalid_argument);

	engine.remove_measurement_processor(processor3->get_label());
	ASSERT_THROW(engine.get_measurement_processor(processor3->get_label()), std::invalid_argument);
}

TEST_F(StandardFusionEngineTests, procValidAux) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_measurement_processor(processor);
	auto aux = make_shared<FakeAux>(1.0);
	AspnBaseVector aux_vec{aux};
	engine.add_measurement_processor(processor);
	engine.give_measurement_processor_aux_data(processor->get_label(), aux_vec);
	ASSERT_TRUE(std::dynamic_pointer_cast<Processor>(processor)->aux == aux);
}

TEST_F(StandardFusionEngineTests, procInvalidAux) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_measurement_processor(processor);
	auto aux = make_shared<aspn_xtensor::AspnBase>(ASPN_UNDEFINED, 1, 2, 3, 4);
	AspnBaseVector aux_vec{aux};
	ASSERT_THROW(engine.give_measurement_processor_aux_data("bloop", aux_vec),
	             std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, StandardFusionEngineTests, reversePropagate) {
	StandardFusionEngine engine(to_type_timestamp(1.0));
	engine.add_state_block(test.block);
	ASSERT_HONORS_MODE_EX(
	    engine.propagate(to_type_timestamp(0.9)),
	    "Reverse propagate requested: propagation requested at time 0.900000000s but "
	    "filter is at time 1.000000000s. Possible out of order measurements!",
	    std::invalid_argument);
}

TEST_F(StandardFusionEngineTests, get_mat_indices_list) {
	auto engine = TestableStandardFusionEngine{};

	// Empty blocks
	auto empty_list = engine.get_mat_indices_list();
	EXPECT_EQ(0, empty_list.size());
	EXPECT_UB_OR_DIE(engine.get_mat_indices(1),
	                 "No StateBlock numbered 1 exists. There are only 0 StateBlocks",
	                 std::invalid_argument);

	// One block
	engine.add_state_block(block);
	auto one_list = engine.get_mat_indices_list();
	EXPECT_EQ(1, one_list.size());
	auto first_entry = one_list.back();
	EXPECT_EQ(0, first_entry.first);
	EXPECT_EQ(1, first_entry.second);
	EXPECT_UB_OR_DIE(engine.get_mat_indices(2),
	                 "No StateBlock numbered 2 exists. There are only 1 StateBlocks",
	                 std::invalid_argument);

	// Two blocks
	engine.add_state_block(block2);
	auto two_list = engine.get_mat_indices_list();
	EXPECT_EQ(2, two_list.size());
	first_entry = two_list.front();
	EXPECT_EQ(0, first_entry.first);
	EXPECT_EQ(1, first_entry.second);
	auto second_entry = two_list.back();
	EXPECT_EQ(1, second_entry.first);
	EXPECT_EQ(3, second_entry.second);
	EXPECT_UB_OR_DIE(engine.get_mat_indices(3),
	                 "No StateBlock numbered 3 exists. There are only 2 StateBlocks",
	                 std::invalid_argument);

	// Three blocks
	engine.add_state_block(block3);
	auto three_list = engine.get_mat_indices_list();
	EXPECT_EQ(3, three_list.size());
	first_entry = three_list.front();
	EXPECT_EQ(0, first_entry.first);
	EXPECT_EQ(1, first_entry.second);
	second_entry = three_list[1];
	EXPECT_EQ(1, second_entry.first);
	EXPECT_EQ(3, second_entry.second);
	auto third_entry = three_list.back();
	EXPECT_EQ(3, third_entry.first);
	EXPECT_EQ(6, third_entry.second);
	EXPECT_UB_OR_DIE(engine.get_mat_indices(5),
	                 "No StateBlock numbered 5 exists. There are only 3 StateBlocks",
	                 std::invalid_argument);
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, StandardFusionEngineTests, get_mat_indices_errors) {
	auto engine = TestableStandardFusionEngine{};

	// Empty blocks
	EXPECT_HONORS_MODE_EX(engine.get_mat_indices(1),
	                      "No StateBlock numbered 1 exists. There are only 0 StateBlocks",
	                      std::invalid_argument);

	// One block
	engine.add_state_block(test.block);
	EXPECT_HONORS_MODE_EX(engine.get_mat_indices(2),
	                      "No StateBlock numbered 2 exists. There are only 1 StateBlocks",
	                      std::invalid_argument);

	// Two blocks
	engine.add_state_block(test.block2);
	EXPECT_HONORS_MODE_EX(engine.get_mat_indices(3),
	                      "No StateBlock numbered 3 exists. There are only 2 StateBlocks",
	                      std::invalid_argument);

	// Three blocks
	engine.add_state_block(test.block3);
	EXPECT_HONORS_MODE_EX(engine.get_mat_indices(5),
	                      "No StateBlock numbered 5 exists. There are only 3 StateBlocks",
	                      std::invalid_argument);
}

TEST_F(StandardFusionEngineTests, copy) {
	// Setup a engine with SBs, MPs, and VSB.
	TestableStandardFusionEngine engine(aspn_xtensor::TypeTimestamp((int64_t)0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.add_measurement_processor(processor);
	engine.add_measurement_processor(processor2);
	engine.set_cross_term_process_covariance(
	    block->get_label(),
	    block2->get_label(),
	    ones(block->get_num_states(), block2->get_num_states()));
	engine.add_virtual_state_block(
	    make_shared<navtk::filtering::ScaleVirtualStateBlock>("base", "unscaled", zeros(6)));

	auto engine_copy = TestableStandardFusionEngine(engine);

	// Copy and scope-delete, should help expose any shallow-copy issues if it isn't compiled away
	{ auto trash_copy = TestableStandardFusionEngine(engine); }
	check_copy(engine, engine_copy);
}

TEST_F(StandardFusionEngineTests, copy_assign) {
	TestableStandardFusionEngine engine(aspn_xtensor::TypeTimestamp((int64_t)0));
	engine.add_state_block(block);
	engine.add_state_block(block2);
	engine.add_measurement_processor(processor);
	engine.add_measurement_processor(processor2);
	engine.set_cross_term_process_covariance(
	    block->get_label(),
	    block2->get_label(),
	    ones(block->get_num_states(), block2->get_num_states()));
	engine.add_virtual_state_block(
	    make_shared<navtk::filtering::ScaleVirtualStateBlock>("base", "unscaled", zeros(6)));

	TestableStandardFusionEngine engine_copy(to_type_timestamp(1.0));
	engine_copy = engine;
	{
		TestableStandardFusionEngine trash_copy(to_type_timestamp(1.0));
		trash_copy = engine;
	}
	check_copy(engine, engine_copy);
}

TEST_F(StandardFusionEngineTests, warnWhenStrategyHasStates) {
	auto strategy = make_shared<navtk::filtering::EkfStrategy>();
	strategy->on_fusion_engine_state_block_added(1);
	EXPECT_WARN(StandardFusionEngine(to_type_timestamp(1.0), strategy), "already contains states");
}

TEST_F(StandardFusionEngineTests, empty_labels) {
	StandardFusionEngine engine;
	engine.add_state_block(block);
	engine.set_state_block_estimate("myblock", Vector{5.5});
	engine.set_state_block_covariance("myblock", Matrix{{1.0}});
	std::vector<std::string> empty_labels;
	EXPECT_WARN(engine.generate_x_and_p(empty_labels), "No labels provided, nothing to do.");
	EXPECT_WARN(engine.peek_ahead(to_type_timestamp(3.0), empty_labels),
	            "peek_ahead with empty mixed_block_labels does nothing.");
}

TEST_F(StandardFusionEngineTests, can_get_vsb_through_gen_xp) {
	StandardFusionEngine engine;
	engine.add_state_block(block);
	engine.set_state_block_estimate("myblock", Vector{5.5});
	engine.set_state_block_covariance("myblock", Matrix{{1.0}});
	engine.add_virtual_state_block(
	    make_shared<navtk::filtering::ScaleVirtualStateBlock>("myblock", "half", Vector{0.5}));
	auto xp = engine.generate_x_and_p({"half"});
	ASSERT_ALLCLOSE(xp->estimate, Vector{2.75});
	ASSERT_ALLCLOSE(xp->covariance, Matrix{{0.25}});
}
