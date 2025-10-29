#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <tensor_assert.hpp>
#include <xtensor/views/xview.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/filtering/stateblocks/discretization_strategy.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::eye;
using navtk::Matrix;
using navtk::not_null;
using navtk::num_rows;
using navtk::Vector;
using navtk::zeros;
using navtk::filtering::DiscretizationStrategy;
using navtk::filtering::EstimateWithCovariance;
using navtk::filtering::GaussianVectorData;
using navtk::filtering::GenXhatPFunction;
using navtk::filtering::MeasurementProcessor;
using navtk::filtering::second_order_discretization_strategy;
using navtk::filtering::StandardDynamicsModel;
using navtk::filtering::StandardFusionEngine;
using navtk::filtering::StandardMeasurementModel;
using navtk::filtering::StateBlock;
using navtk::filtering::VirtualStateBlock;

struct AliasedFilterTests : public ::testing::Test {
	static double c2f(double c) { return c * 9.0 / 5.0 + 32.0; }
	static double f2c(double f) { return (f - 32.0) * 5.0 / 9.0; }
	std::string f_block{"fahrenheitBlock"};
	std::string c_block{"celsiusBlock"};
	std::string f_proc{"fahrenheitProc"};
	std::string c_proc{"celsiusProc"};
	std::string a_block{"magicA"};
	std::string b_block{"magicB"};
	std::string magic1_proc{"magic1_proc"};
	std::string magic2_proc{"magic2_proc"};
	std::string dumb_name = "frumpf";
	Vector x0F{212.0};
	Matrix p0F{{4.0}};
	Vector x0C{100.0};
	Matrix p0C{{100.0 / 81.0}};

	std::shared_ptr<GaussianVectorData> meas_f = std::make_shared<GaussianVectorData>(
	    to_type_timestamp(3600.0), Vector{10.0}, Matrix{{3.0}});
	std::shared_ptr<GaussianVectorData> meas_c = std::make_shared<GaussianVectorData>(
	    to_type_timestamp(3600.0), Vector{10.0}, Matrix{{3.0}});
	std::shared_ptr<GaussianVectorData> meas1 =
	    std::make_shared<GaussianVectorData>(to_type_timestamp(3600.0), Vector{4.0}, Matrix{{9.0}});
	std::shared_ptr<GaussianVectorData> meas2 =
	    std::make_shared<GaussianVectorData>(to_type_timestamp(3600.0), Vector{4.0}, Matrix{{9.0}});
	StandardFusionEngine ff, fc, fa;  // straight fahrenheit, celsius, and aliased
	std::logic_error fixme{"TEST invalid; desired behavior unresolved"};
	std::string as_one{"as_one"};
	Vector x0D{2.0, 9.9};
	Matrix p0D{{1.5, -0.7}, {-0.7, 9.2}};


	class Constant2State : public StateBlock<> {
	public:
		Constant2State(std::string lab) : StateBlock(2, lab) {}
		virtual not_null<std::shared_ptr<StateBlock<>>> clone() override {
			return std::make_shared<Constant2State>(get_label());
		}
		virtual StandardDynamicsModel generate_dynamics(GenXhatPFunction,
		                                                aspn_xtensor::TypeTimestamp,
		                                                aspn_xtensor::TypeTimestamp) override {
			return StandardDynamicsModel{
			    [](Vector it) -> Vector { return it; }, eye(2), zeros(2, 2)};
		}
	};

	class FahrenheitBlock : public StateBlock<> {
	public:
		FahrenheitBlock(std::string lab) : StateBlock(1, lab) {}
		virtual not_null<std::shared_ptr<StateBlock<>>> clone() override {
			return std::make_shared<FahrenheitBlock>(get_label());
		}
		virtual StandardDynamicsModel generate_dynamics(GenXhatPFunction,
		                                                aspn_xtensor::TypeTimestamp t0,
		                                                aspn_xtensor::TypeTimestamp t1) override {
			auto dt = to_seconds(t1 - t0);
			Matrix F{{dt / 3600.0 * change_per_hour_f}};
			Matrix Q         = diag(Vector{pow(1.0 / 100.0, 2.0)});
			auto discretized = strat(F, eye(num_rows(Q), num_rows(Q)), Q, dt);
			auto Phi         = discretized.first;
			auto Qd          = discretized.second;
			return StandardDynamicsModel{
			    [t0 = t0, t1 = t1, changePerHour = change_per_hour_f](Vector it) -> Vector {
				    return (it - 32.0) * (1.0 + to_seconds(t1 - t0) / 3600.0 * changePerHour) +
				           32.0;
			    },
			    Phi,
			    Qd};
		}

	private:
		double change_per_hour_f{0.01};  // Temp increases by 1 deg F/hour, linearly perHour
		DiscretizationStrategy strat{&second_order_discretization_strategy};
	};

	class CelsiusBlock : public StateBlock<> {
	public:
		CelsiusBlock(std::string lab) : StateBlock(1, lab) {}
		virtual not_null<std::shared_ptr<StateBlock<>>> clone() override {
			return std::make_shared<CelsiusBlock>(get_label());
		}
		virtual StandardDynamicsModel generate_dynamics(GenXhatPFunction,
		                                                aspn_xtensor::TypeTimestamp t0,
		                                                aspn_xtensor::TypeTimestamp t1) override {
			auto dt = to_seconds(t1 - t0);
			Matrix F{{dt / 3600.0 * change_per_hour_c}};
			Matrix Q         = diag(Vector{pow(5.0 / 9.0 / 100.0, 2.0)});
			auto discretized = strat(F, eye(num_rows(Q), num_rows(Q)), Q, dt);
			auto Phi         = discretized.first;
			auto Qd          = discretized.second;
			return StandardDynamicsModel{
			    [t0 = t0, t1 = t1, changePerHour = change_per_hour_c](Vector it) -> Vector {
				    return it * (1.0 + to_seconds(t1 - t0) / 3600.0 * changePerHour);
			    },
			    Phi,
			    Qd};
		}

	private:
		double change_per_hour_c{0.01};
		DiscretizationStrategy strat{&second_order_discretization_strategy};
	};

	// All measurements arrive as F
	class FahrenheitProc : public MeasurementProcessor<> {
	public:
		FahrenheitProc(std::string proc_lab, std::string state_lab)
		    : MeasurementProcessor(proc_lab, state_lab) {}
		not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override {
			return std::make_shared<FahrenheitProc>(get_label(), get_state_block_labels()[0]);
		}
		virtual std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase> meas, GenXhatPFunction) override {

			auto data = std::dynamic_pointer_cast<GaussianVectorData>(meas);

			return std::shared_ptr<StandardMeasurementModel>(new StandardMeasurementModel(
			    data->estimate,
			    [](Vector it) -> Vector { return it; },
			    Matrix{{1.0}},
			    data->covariance));
		}
	};

	class CelsiusProc : public MeasurementProcessor<> {
	public:
		CelsiusProc(std::string proc_lab, std::string state_lab)
		    : MeasurementProcessor(proc_lab, state_lab) {}
		not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override {
			return std::make_shared<CelsiusProc>(get_label(), get_state_block_labels()[0]);
		}
		virtual std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase> meas, GenXhatPFunction) override {
			auto data = std::dynamic_pointer_cast<GaussianVectorData>(meas);

			// Oh no, my measurements are in F, and my state in C... transform input meas to match
			Vector conv_m{f2c(data->estimate[0])};
			Matrix conv_r{{std::pow((std::sqrt(data->covariance(0, 0)) * 5.0 / 9.0),
			                        2.0)}};  // Not really how we'd do this, but ok for scalar

			return std::shared_ptr<StandardMeasurementModel>(new StandardMeasurementModel(
			    conv_m, [](Vector it) -> Vector { return Vector{it[0]}; }, Matrix{{1.0}}, conv_r));
		}
	};

	// All measurements arrive as F
	class Magic1State : public MeasurementProcessor<> {
	public:
		Magic1State(std::string proc_lab, std::string state_lab)
		    : MeasurementProcessor(proc_lab, state_lab) {}
		not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override {
			return std::make_shared<Magic1State>(get_label(), get_state_block_labels()[0]);
		}
		virtual std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase> meas, GenXhatPFunction) override {
			auto data = std::dynamic_pointer_cast<GaussianVectorData>(meas);

			return std::shared_ptr<StandardMeasurementModel>(new StandardMeasurementModel(
			    data->estimate,
			    [](Vector it) -> Vector { return it; },
			    Matrix{{1.0}},
			    data->covariance));
		}
	};

	class Magic2State : public MeasurementProcessor<> {
	public:
		Magic2State(std::string proc_lab, std::string state_lab)
		    : MeasurementProcessor(proc_lab, state_lab) {}
		not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override {
			return std::make_shared<Magic2State>(get_label(), get_state_block_labels()[0]);
		}
		virtual std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase> meas, GenXhatPFunction) override {
			auto data = std::dynamic_pointer_cast<GaussianVectorData>(meas);

			return std::shared_ptr<StandardMeasurementModel>(new StandardMeasurementModel(
			    data->estimate,
			    [](Vector it) -> Vector { return Vector{it[0] + it[1]}; },
			    Matrix{{1.0, 1.0}},
			    data->covariance));
		}
	};


	class CFTransformer : public VirtualStateBlock {
	public:
		CFTransformer(std::string from, std::string to) : VirtualStateBlock(from, to) {}

		CFTransformer(const CFTransformer& other)
		    : VirtualStateBlock(other.current, other.target), jac(other.jac) {}

		not_null<std::shared_ptr<VirtualStateBlock>> clone() override {
			return std::make_shared<CFTransformer>(*this);
		}

		Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override {
			return Vector{c2f(x[0])};
		};

		virtual Matrix jacobian(const Vector&, const aspn_xtensor::TypeTimestamp&) override {
			return jac;
		};

	private:
		Matrix jac = Matrix{{9.0 / 5.0}};
	};

	class TwoToOne : public VirtualStateBlock {
	public:
		TwoToOne(std::string from, std::string to) : VirtualStateBlock(from, to) {}

		TwoToOne(const TwoToOne& other)
		    : VirtualStateBlock(other.current, other.target), jac(other.jac) {}

		not_null<std::shared_ptr<VirtualStateBlock>> clone() override {
			return std::make_shared<TwoToOne>(*this);
		}

		Vector convert_estimate(const Vector& x, const aspn_xtensor::TypeTimestamp&) override {
			return Vector{x[0] + x[1]};
		};

		virtual Matrix jacobian(const Vector&, const aspn_xtensor::TypeTimestamp&) override {
			return jac;
		};

	private:
		Matrix jac = Matrix{{1.0, 1.0}};
	};

	/*
	 * Helper function for setting up test filters with common blocks and processors.
	 *
	 * @param engine StandardFusionEngine to test.
	 * @param blk Filter specific block to be added.
	 * @param x Estimate for blk.
	 * @param m Covariance for blk.
	 */
	void init_filter(StandardFusionEngine& engine,
	                 not_null<std::shared_ptr<StateBlock<>>> blk,
	                 const Vector& x,
	                 const Matrix& m) {
		engine.add_state_block(std::make_shared<Constant2State>(a_block));
		engine.set_state_block_estimate(a_block, x0D);
		engine.set_state_block_covariance(a_block, p0D);

		engine.add_state_block(std::make_shared<Constant2State>(b_block));
		engine.set_state_block_estimate(b_block, x0D);
		engine.set_state_block_covariance(b_block, p0D);

		engine.add_state_block(blk);
		engine.set_state_block_estimate(blk->get_label(), x);
		engine.set_state_block_covariance(blk->get_label(), m);

		engine.add_measurement_processor(std::make_shared<FahrenheitProc>(f_proc, f_block));
		engine.add_measurement_processor(std::make_shared<CelsiusProc>(c_proc, c_block));
		engine.add_measurement_processor(std::make_shared<Magic2State>(magic2_proc, a_block));
	}

	virtual void SetUp() override {
		init_filter(fa, std::make_shared<CelsiusBlock>(c_block), x0C, p0C);
		init_filter(fc, std::make_shared<CelsiusBlock>(c_block), x0C, p0C);
		init_filter(ff, std::make_shared<FahrenheitBlock>(f_block), x0F, p0F);

		fa.add_measurement_processor(std::make_shared<Magic1State>(magic1_proc, as_one));
		fa.add_virtual_state_block(std::make_shared<CFTransformer>(c_block, f_block));
		fa.add_virtual_state_block(std::make_shared<TwoToOne>(a_block, as_one));
	}
};

TEST_F(AliasedFilterTests, TransformerOK) {
	CFTransformer cf         = CFTransformer(c_block, a_block);
	EstimateWithCovariance a = EstimateWithCovariance(Vector{12.0}, Matrix{{3.0}});
	auto ct                  = cf.convert(a, aspn_xtensor::TypeTimestamp((int64_t)0));
}

TEST_F(AliasedFilterTests, getAliasedBlockCovariance) {
	auto cov = fa.get_state_block_covariance(f_block);
	ASSERT_ALLCLOSE(cov, p0F);
	ASSERT_ALLCLOSE(fa.get_state_block_covariance(a_block), p0D);
	ASSERT_ALLCLOSE(fa.get_state_block_covariance(b_block), p0D);
}

TEST_F(AliasedFilterTests, get_state_block_estimate) {

	auto est = fa.get_state_block_estimate(f_block);
	ASSERT_ALLCLOSE(est, x0F);
	ASSERT_ALLCLOSE(fa.get_state_block_estimate(a_block), x0D);
	ASSERT_ALLCLOSE(fa.get_state_block_estimate(b_block), x0D);
	ASSERT_ALLCLOSE(fa.get_state_block_estimate(as_one), Vector{11.9});
}

TEST_F(AliasedFilterTests, propAliased) {
	ff.propagate(to_type_timestamp(2200.0));
	fa.propagate(to_type_timestamp(2200.0));
	fc.propagate(to_type_timestamp(2200.0));

	ASSERT_ALLCLOSE(ff.get_state_block_estimate(f_block), fa.get_state_block_estimate(f_block));
	ASSERT_ALLCLOSE(ff.get_state_block_covariance(f_block), fa.get_state_block_covariance(f_block));

	auto tx = CFTransformer(c_block, f_block);
	EstimateWithCovariance c_alias{fc.get_state_block_estimate(c_block),
	                               fc.get_state_block_covariance(c_block)};
	auto cvt = tx.convert(c_alias, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(cvt.estimate, fa.get_state_block_estimate(f_block));
	ASSERT_ALLCLOSE(cvt.covariance, fa.get_state_block_covariance(f_block));
}

TEST_F(AliasedFilterTests, updateAliased) {
	ff.update(f_proc, meas_f);
	fa.update(f_proc, meas_f);
	fc.update(c_proc, meas_c);
	ff.update(magic2_proc, meas2);
	fa.update(magic1_proc, meas1);
	fc.update(magic2_proc, meas2);

	ASSERT_ALLCLOSE(ff.get_state_block_estimate(f_block), fa.get_state_block_estimate(f_block));
	ASSERT_ALLCLOSE(ff.get_state_block_covariance(f_block), fa.get_state_block_covariance(f_block));
	ASSERT_ALLCLOSE(ff.get_state_block_estimate(a_block), fa.get_state_block_estimate(a_block));
	ASSERT_ALLCLOSE(ff.get_state_block_covariance(a_block), fa.get_state_block_covariance(a_block));

	auto tx = CFTransformer(c_block, f_block);
	EstimateWithCovariance c_alias{fc.get_state_block_estimate(c_block),
	                               fc.get_state_block_covariance(c_block)};
	auto cvt = tx.convert(c_alias, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(cvt.estimate, fa.get_state_block_estimate(f_block));
	ASSERT_ALLCLOSE(cvt.covariance, fa.get_state_block_covariance(f_block));
}

ERROR_MODE_SENSITIVE_TEST(TEST_F, AliasedFilterTests, getStateBlockEstimateAliasedMultiple) {
	auto bf = std::make_shared<AliasedFilterTests::FahrenheitBlock>(test.dumb_name);
	test.fa.add_state_block(bf);
	auto tx = std::make_shared<AliasedFilterTests::CFTransformer>(test.dumb_name, test.f_block);
	EXPECT_HONORS_MODE_EX(test.fa.add_virtual_state_block(tx),
	                      "Already have a target with this tag",
	                      std::invalid_argument);
}

TEST_F(AliasedFilterTests, getStateBlockEstimateNoSourceBlock) {
	auto tx = std::make_shared<AliasedFilterTests::CFTransformer>(dumb_name, "notblock");
	fa.add_virtual_state_block(tx);
	EXPECT_THROW(fa.get_state_block_estimate("notblock"), std::invalid_argument);
}

TEST_F(AliasedFilterTests, getStateBlockEstimateFailNone) {
	EXPECT_THROW(fa.get_state_block_estimate(dumb_name), std::invalid_argument);
}

/**
 * Assuming that
 * 1) Only shared StateBlocks should ever be Aliased (ie no sensor
 * specific MP added blocks)
 * 2) A knowing overlord of some kind (user, Scarab equivalent) has added
 * all to-be-aliased StateBlocks (but not necessarily all Transformers)
 * and initialized them
 * 3) (A little wobbly) Said overlord will be the only one implementing
 * any Aliased block resets (ie no MP will try to reset a block with some
 * aliased value), then the following filter API calls should only work
 * with no-kidding StateBlock labels, and would this throw a
 * std::invalid_argument otherwise. At a minimum we'd need the inverse
 * transform provided to even begin to make this work.
 *
 * This means that we will generally not 'advertise' Aliases via
 * get_state_block_names_list (but may provide a separate function to do so.)
 */

TEST_F(AliasedFilterTests, reset_state_estimate) {
	EXPECT_THROW({ fa.reset_state_estimate(fa.get_time(), f_block, {0}); }, std::invalid_argument);
}

TEST_F(AliasedFilterTests, setAliasedBlockEstimate) {
	EXPECT_THROW({ fa.set_state_block_estimate(f_block, Vector{0.0}); }, std::invalid_argument);
}

TEST_F(AliasedFilterTests, setAliasedBlockCovariance) {
	EXPECT_THROW({ fa.set_state_block_covariance(f_block, Matrix{{0.0}}); }, std::invalid_argument);
}

TEST_F(AliasedFilterTests, getAliasedBlock) {
	EXPECT_THROW({ fa.get_state_block(f_block); }, std::invalid_argument);
}

TEST_F(AliasedFilterTests, removeAliasedBlock) {
	fa.remove_state_block(f_block);
	fa.get_state_block_estimate(c_block);
}

TEST_F(AliasedFilterTests, has_virtual_block) {
	// Linked vsb already exists
	ASSERT_TRUE(fa.has_virtual_state_block(f_block));
	ASSERT_FALSE(fa.has_virtual_state_block("blerp"));
	// Add VSB, but no base state linked
	fa.add_virtual_state_block(std::make_shared<CFTransformer>("the_beginning", "iz"));
	ASSERT_FALSE(fa.has_virtual_state_block("iz"));
	// Add base state, should now work
	fa.add_state_block(std::make_shared<Constant2State>("the_beginning"));
	ASSERT_TRUE(fa.has_virtual_state_block("iz"));
	// Add another VSB that is missing another VSB bridge, fails
	fa.add_virtual_state_block(std::make_shared<CFTransformer>("before", "the_end"));
	ASSERT_FALSE(fa.has_virtual_state_block("the_end"));
	// Add bridge vsb, should pass
	fa.add_virtual_state_block(std::make_shared<CFTransformer>("iz", "before"));
	ASSERT_TRUE(fa.has_virtual_state_block("iz"));
	ASSERT_TRUE(fa.has_virtual_state_block("before"));
	ASSERT_TRUE(fa.has_virtual_state_block("the_end"));
	// Remove base SB, all should fail
	fa.remove_state_block("the_beginning");
	ASSERT_FALSE(fa.has_virtual_state_block("iz"));
	ASSERT_FALSE(fa.has_virtual_state_block("before"));
	ASSERT_FALSE(fa.has_virtual_state_block("the_end"));
}

// // Not that anyone uses this, but if a user wanted to set cross Qd
// // terms between an aliased and non-aliased block (or 2 aliased blocks)
// // We'd only be able to do a first order tx using the inverse Jacobian(s)
// // which are currently not forced to be provided.
// TEST_F(AliasedFilterTests, set_cross_term_qd_mat) { throw fixme; }

// // This could be a tricky one... does a Aliased block 'inherit' the functions
// // of the original? Probably? But everything will be 'as is' ie there
// // is no possibility of converting anything without letting the user
// // tweak more things
// TEST_F(AliasedFilterTests, give_state_block_aux_data) {}

// Other testing needed
// Non-scalar state blocks
// Mixed aliased and non alias block prop/updates
// multiple prop/update cycles
// non-linear tx
