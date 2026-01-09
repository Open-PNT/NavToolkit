#include <iostream>
#include <memory>
#include <vector>

#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <xtensor/xrandom.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/ClockModel.hpp>
#include <navtk/filtering/containers/CorrectedGnssPseudorangeMeasurement.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/containers/GaussianVectorData.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/containers/LinearizedStrategyBase.hpp>
#include <navtk/filtering/containers/MeasurementBuffer.hpp>
#include <navtk/filtering/containers/MeasurementBuffer3d.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/containers/Pose.hpp>
#include <navtk/filtering/containers/PseudorangeDopplerMeasurements.hpp>
#include <navtk/filtering/containers/RangeInfo.hpp>
#include <navtk/filtering/containers/RelativeHumidityAux.hpp>
#include <navtk/filtering/containers/SampledDynamicsModel.hpp>
#include <navtk/filtering/containers/SampledMeasurementModel.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/filtering/containers/TimestampedDataSeries.hpp>
#include <navtk/filtering/containers/TrackedGnssObservations.hpp>
#include <navtk/filtering/experimental/containers/RbpfModel.hpp>
#include <navtk/filtering/experimental/fusion/strategies/RbpfStrategy.hpp>
#include <navtk/filtering/experimental/processors/NonlinearAltitudeProcessor.hpp>
#include <navtk/filtering/experimental/resampling.hpp>
#include <navtk/filtering/experimental/stateblocks/SampledFogmBlock.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/fusion/strategies/EkfStrategy.hpp>
#include <navtk/filtering/fusion/strategies/FusionStrategy.hpp>
#include <navtk/filtering/fusion/strategies/SampledModelStrategy.hpp>
#include <navtk/filtering/fusion/strategies/StandardModelStrategy.hpp>
#include <navtk/filtering/fusion/strategies/UkfStrategy.hpp>
#include <navtk/filtering/processors/AltitudeMeasurementProcessor.hpp>
#include <navtk/filtering/processors/AltitudeMeasurementProcessorWithBias.hpp>
#include <navtk/filtering/processors/Attitude3dMeasurementProcessor.hpp>
#include <navtk/filtering/processors/BiasedRangeProcessor.hpp>
#include <navtk/filtering/processors/DeltaPositionMeasurementProcessor.hpp>
#include <navtk/filtering/processors/DirectMeasurementProcessor.hpp>
#include <navtk/filtering/processors/DirectionToPoints3dMeasurementProcessor.hpp>
#include <navtk/filtering/processors/GeodeticPos2dMeasurementProcessor.hpp>
#include <navtk/filtering/processors/GeodeticPos3dMeasurementProcessor.hpp>
#include <navtk/filtering/processors/MagneticFieldMagnitudeMeasurementProcessor.hpp>
#include <navtk/filtering/processors/MagnetometerToHeadingMeasurementProcessor.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/filtering/processors/PinsonPositionMeasurementProcessor.hpp>
#include <navtk/filtering/processors/PositionVelocityAttitudeMeasurementProcessor.hpp>
#include <navtk/filtering/processors/VelocityMeasurementProcessor.hpp>
#include <navtk/filtering/processors/ZuptMeasurementProcessor.hpp>
#include <navtk/filtering/stateblocks/ClockBiasesStateBlock.hpp>
#include <navtk/filtering/stateblocks/EarthModel.hpp>
#include <navtk/filtering/stateblocks/FogmAccel.hpp>
#include <navtk/filtering/stateblocks/FogmBlock.hpp>
#include <navtk/filtering/stateblocks/FogmVelocity.hpp>
#include <navtk/filtering/stateblocks/GravityModel.hpp>
#include <navtk/filtering/stateblocks/GravityModelSchwartz.hpp>
#include <navtk/filtering/stateblocks/GravityModelTittertonAndWeston.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/Pinson21NedBlock.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/filtering/stateblocks/discretization_strategy.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/filtering/virtualstateblocks/ChainedVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/EcefToStandard.hpp>
#include <navtk/filtering/virtualstateblocks/EcefToStandardQuat.hpp>
#include <navtk/filtering/virtualstateblocks/FirstOrderVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/NumericalVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonErrorToStandard.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonErrorToStandardQuat.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonToSensor.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonToSensorLlh.hpp>
#include <navtk/filtering/virtualstateblocks/PlatformToSensorCartesianVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/PlatformToSensorEcef.hpp>
#include <navtk/filtering/virtualstateblocks/PlatformToSensorEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/QuatToRpyPva.hpp>
#include <navtk/filtering/virtualstateblocks/ScaleVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/SensorToPlatformCartesianVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/SensorToPlatformEcef.hpp>
#include <navtk/filtering/virtualstateblocks/SensorToPlatformEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/ShiftVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/StandardToEcef.hpp>
#include <navtk/filtering/virtualstateblocks/StandardToEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/StateExtractor.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/VirtualStateBlockManager.hpp>
#include <navtk/geospatial/providers/SimpleElevationProvider.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include "binding_helpers.hpp"

using aspn_xtensor::MeasurementAltitude;
using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::MeasurementSatnav;
using aspn_xtensor::MeasurementVelocity;
using aspn_xtensor::MetadataGpsLnavEphemeris;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeMounting;
using aspn_xtensor::TypeTimestamp;
using navtk::Matrix;
using navtk::Matrix3;
using navtk::not_null;
using navtk::Scalar;
using navtk::Size;
using navtk::Vector;
using navtk::Vector3;
using navtk::experimental::RandomNumberGenerator;
using navtk::filtering::AltitudeMeasurementProcessor;
using navtk::filtering::AltitudeMeasurementProcessorWithBias;
using navtk::filtering::apply_error_states;
using navtk::filtering::Attitude3dMeasurementProcessor;
using navtk::filtering::BiasedRangeProcessor;
using navtk::filtering::calc_numerical_hessians;
using navtk::filtering::calc_numerical_jacobian;
using navtk::filtering::ChainedVirtualStateBlock;
using navtk::filtering::ClockBiasesStateBlock;
using navtk::filtering::ClockChoice;
using navtk::filtering::ClockModel;
using navtk::filtering::COMPENSATED_CRYSTAL_CLOCK;
using navtk::filtering::CorrectedGnssPseudorangeMeasurement;
using navtk::filtering::DeltaPositionMeasurementProcessor;
using navtk::filtering::DirectionToPoints3dMeasurementProcessor;
using navtk::filtering::DirectMeasurementProcessor;
using navtk::filtering::DiscretizationStrategy;
using navtk::filtering::EarthModel;
using navtk::filtering::EcefToStandard;
using navtk::filtering::EcefToStandardQuat;
using navtk::filtering::EkfStrategy;
using navtk::filtering::EstimateWithCovariance;
using navtk::filtering::FirstOrderVirtualStateBlock;
using navtk::filtering::FogmAccel;
using navtk::filtering::FogmBlock;
using navtk::filtering::FogmVelocity;
using navtk::filtering::full_order_discretization_strategy;
using navtk::filtering::FusionStrategy;
using navtk::filtering::GaussianVectorData;
using navtk::filtering::GeodeticPos2dMeasurementProcessor;
using navtk::filtering::GeodeticPos3dMeasurementProcessor;
using navtk::filtering::get_time_value;
using navtk::filtering::GravityModel;
using navtk::filtering::GravityModelSchwartz;
using navtk::filtering::GravityModelTittertonAndWeston;
using navtk::filtering::hg1700_model;
using navtk::filtering::hg9900_model;
using navtk::filtering::ideal_imu_model;
using navtk::filtering::ImuModel;
using navtk::filtering::LinearizedStrategyBase;
using navtk::filtering::MagneticFieldMagnitudeMeasurementProcessor;
using navtk::filtering::MagnetometerToHeadingMeasurementProcessor;
using navtk::filtering::MeasurementBuffer;
using navtk::filtering::MeasurementBuffer3d;
using navtk::filtering::MeasurementProcessor;
using navtk::filtering::NavSolution;
using navtk::filtering::NumericalVirtualStateBlock;
using navtk::filtering::OVENIZED_CRYSTAL_CLOCK;
using navtk::filtering::PairedPva;
using navtk::filtering::Pinson15NedBlock;
using navtk::filtering::Pinson21NedBlock;
using navtk::filtering::PinsonErrorToStandard;
using navtk::filtering::PinsonErrorToStandardQuat;
using navtk::filtering::PinsonPositionMeasurementProcessor;
using navtk::filtering::PinsonToSensor;
using navtk::filtering::PinsonToSensorLlh;
using navtk::filtering::PlatformToSensorCartesianVirtualStateBlock;
using navtk::filtering::PlatformToSensorEcef;
using navtk::filtering::PlatformToSensorEcefQuat;
using navtk::filtering::Pose;
using navtk::filtering::PositionVelocityAttitudeMeasurementProcessor;
using navtk::filtering::PseudorangeDopplerMeasurements;
using navtk::filtering::QuatToRpyPva;
using navtk::filtering::RangeInfo;
using navtk::filtering::RelativeHumidityAux;
using navtk::filtering::RUBIDIUM_CLOCK;
using navtk::filtering::sagem_primus200_model;
using navtk::filtering::SampledDynamicsModel;
using navtk::filtering::SampledMeasurementModel;
using navtk::filtering::SampledModelStrategy;
using navtk::filtering::ScaleVirtualStateBlock;
using navtk::filtering::second_order_discretization_strategy;
using navtk::filtering::SensorToPlatformCartesianVirtualStateBlock;
using navtk::filtering::SensorToPlatformEcef;
using navtk::filtering::SensorToPlatformEcefQuat;
using navtk::filtering::ShiftVirtualStateBlock;
using navtk::filtering::StandardDynamicsModel;
using navtk::filtering::StandardFusionEngine;
using navtk::filtering::StandardMeasurementModel;
using navtk::filtering::StandardMeasurementProcessor;
using navtk::filtering::StandardModelStrategy;
using navtk::filtering::StandardStateBlock;
using navtk::filtering::StandardToEcef;
using navtk::filtering::StandardToEcefQuat;
using navtk::filtering::StateBlock;
using navtk::filtering::StateExtractor;
using navtk::filtering::stim300_model;
using navtk::filtering::TimestampedDataPointerCompare;
using navtk::filtering::TimestampedDataSeries;
using navtk::filtering::TrackedGnssObservations;
using navtk::filtering::UkfStrategy;
using navtk::filtering::VelocityMeasurementProcessor;
using navtk::filtering::VirtualStateBlock;
using navtk::filtering::VirtualStateBlockManager;
using navtk::filtering::ZuptMeasurementProcessor;
using navtk::filtering::experimental::NonlinearAltitudeProcessor;
using navtk::filtering::experimental::RbpfModel;
using navtk::filtering::experimental::RbpfStrategy;
using navtk::filtering::experimental::ResamplingFunction;
using navtk::filtering::experimental::ResamplingResult;
using navtk::filtering::experimental::residual_resample_with_replacement;
using navtk::filtering::experimental::SampledFogmBlock;
using navtk::filtering::experimental::systematic_resampling;
using navtk::geospatial::SimpleElevationProvider;
using std::vector;

using namespace pybind11::literals;

namespace py = pybind11;

template <class FusionStrategyBase = FusionStrategy>
class PyFusionStrategy : public FusionStrategyBase {
public:
	using FusionStrategyBase::FusionStrategyBase;

	Size get_num_states() const override {
		PYBIND11_OVERRIDE(Size, FusionStrategyBase, get_num_states, );
	}

	Vector get_estimate() const override {
		PYBIND11_OVERRIDE_PURE(Vector, FusionStrategyBase, get_estimate, );
	}

	Matrix get_covariance() const override {
		PYBIND11_OVERRIDE_PURE(Matrix, FusionStrategyBase, get_covariance, );
	}

	not_null<std::shared_ptr<FusionStrategy>> clone() const override {
		PYBIND11_OVERRIDE_PURE(
		    not_null<std::shared_ptr<FusionStrategy>>, FusionStrategyBase, clone, );
	}

	void on_fusion_engine_state_block_added_impl(Vector const &initial_estimate,
	                                             Matrix const &initial_covariance) override {
		PYBIND11_OVERRIDE_PURE(void,
		                       FusionStrategyBase,
		                       on_fusion_engine_state_block_added_impl,
		                       initial_estimate,
		                       initial_covariance);
	}

	void set_covariance_slice_impl(Matrix const &new_covariance,
	                               Size first_row,
	                               Size first_col) override {
		PYBIND11_OVERRIDE_PURE(void,
		                       FusionStrategyBase,
		                       set_covariance_slice_impl,
		                       new_covariance,
		                       first_row,
		                       first_col);
	}

	void set_estimate_slice_impl(Vector const &new_estimate, Size first_index) override {
		PYBIND11_OVERRIDE_PURE(
		    void, FusionStrategyBase, set_estimate_slice_impl, new_estimate, first_index);
	}

	void on_fusion_engine_state_block_removed_impl(Size first_index, Size count) override {
		PYBIND11_OVERRIDE_PURE(void,
		                       FusionStrategyBase,
		                       on_fusion_engine_state_block_removed_impl,
		                       first_index,
		                       count);
	}

	void on_state_count_changed() override {
		PYBIND11_OVERRIDE(void, FusionStrategyBase, on_state_count_changed, );
	}
};

template <class StandardModelStrategyBase = StandardModelStrategy,
          class PyBase                    = PyFusionStrategy<StandardModelStrategyBase>>
class PyStandardModelStrategy : public PyBase {
public:
	using PyBase::PyBase;

	void propagate(const StandardDynamicsModel &dynamics_model) override {
		PYBIND11_OVERRIDE_PURE(void, StandardModelStrategyBase, propagate, dynamics_model);
	}

	void update(const StandardMeasurementModel &measurement_model) override {
		PYBIND11_OVERRIDE_PURE(void, StandardModelStrategyBase, update, measurement_model);
	}

	using StandardModelStrategyBase::check_update_args;
	using StandardModelStrategyBase::validate_linearized_propagate;
	using StandardModelStrategyBase::validate_linearized_update;
};

template <class SampledModelStrategyBase = SampledModelStrategy>
class PySampledModelStrategy : public PyFusionStrategy<SampledModelStrategyBase> {
public:
	using PyFusionStrategy<SampledModelStrategyBase>::PyFusionStrategy;

	void propagate(const SampledDynamicsModel &dynamics_model) override {
		PYBIND11_OVERRIDE_PURE(void, SampledModelStrategyBase, propagate, dynamics_model);
	}

	void update(const SampledMeasurementModel &measurement_model) override {
		PYBIND11_OVERRIDE_PURE(void, SampledModelStrategyBase, update, measurement_model);
	}
};

template <class LinearizedStrategyBaseBase = LinearizedStrategyBase,
          class PyBase                     = PyFusionStrategy<LinearizedStrategyBaseBase>>
class PyLinearizedStrategyBase : public PyBase {
public:
	using PyBase::PyBase;

	Vector get_estimate() const override {
		PYBIND11_OVERRIDE(Vector, LinearizedStrategyBaseBase, get_estimate, );
	}

	Matrix get_covariance() const override {
		PYBIND11_OVERRIDE(Matrix, LinearizedStrategyBaseBase, get_covariance, );
	}

	not_null<std::shared_ptr<FusionStrategy>> clone() const override {
		PYBIND11_OVERRIDE(
		    not_null<std::shared_ptr<FusionStrategy>>, LinearizedStrategyBaseBase, clone, );
	}

	void on_fusion_engine_state_block_added_impl(Vector const &initial_estimate,
	                                             Matrix const &initial_covariance) override {
		PYBIND11_OVERRIDE(void,
		                  LinearizedStrategyBaseBase,
		                  on_fusion_engine_state_block_added_impl,
		                  initial_estimate,
		                  initial_covariance);
	}

	void set_covariance_slice_impl(Matrix const &new_covariance,
	                               Size first_row,
	                               Size first_col) override {
		PYBIND11_OVERRIDE(void,
		                  LinearizedStrategyBaseBase,
		                  set_covariance_slice_impl,
		                  new_covariance,
		                  first_row,
		                  first_col);
	}

	void set_estimate_slice_impl(Vector const &new_estimate, Size first_index) override {
		PYBIND11_OVERRIDE(
		    void, LinearizedStrategyBaseBase, set_estimate_slice_impl, new_estimate, first_index);
	}

	void on_fusion_engine_state_block_removed_impl(Size first_index, Size count) override {
		PYBIND11_OVERRIDE(void,
		                  LinearizedStrategyBaseBase,
		                  on_fusion_engine_state_block_removed_impl,
		                  first_index,
		                  count);
	}
};

template <class EkfStrategyBase = EkfStrategy,
          class PyBase =
              PyLinearizedStrategyBase<EkfStrategyBase, PyStandardModelStrategy<EkfStrategyBase>>>
class PyEkfStrategy : public PyBase {
public:
	using PyBase::PyBase;

	void propagate(const StandardDynamicsModel &dynamics_model) override {
		PYBIND11_OVERRIDE(void, EkfStrategyBase, propagate, dynamics_model);
	}

	void update(const StandardMeasurementModel &measurement_model) override {
		PYBIND11_OVERRIDE(void, EkfStrategyBase, update, measurement_model);
	}

	not_null<std::shared_ptr<FusionStrategy>> clone() const override {
		PYBIND11_OVERRIDE(not_null<std::shared_ptr<FusionStrategy>>, EkfStrategyBase, clone, );
	}
};

void add_filtering_experimental_functions(pybind11::module &m) {

	m.doc() = "Bindings to the NavToolkit Experimental Filtering Work.";

	CLASS(RbpfModel, FusionStrategy)
	CTOR(RbpfModel, PARAMS(Size, bool), "particle_count"_a = 100, "calc_single_jacobian"_a = true)
	METHOD_VOID(RbpfModel, count_particles)
	METHOD_VOID(RbpfModel, get_particle_count_target)
	METHOD(RbpfModel, set_particle_count_target, "particle_count_target"_a)
	METHOD(RbpfModel,
	       set_marked_states,
	       "marked_states"_a,
	       "jitter_scales"_a = std::vector<double>{0.0})
	METHOD_VOID(RbpfModel, get_marked_states)
	METHOD_VOID(RbpfModel, any_nonlinear)
	METHOD_VOID(RbpfModel, any_linear)
	METHOD_OVERLOAD_VOID(RbpfModel, symmetricize_covariance, )
	METHOD_OVERLOAD_CONST(RbpfModel, symmetricize_covariance, Matrix &, _2, "temp_covariance"_a)
	FIELD(RbpfModel, calc_single_jacobian)
	METHOD_VOID(RbpfModel, get_jitter_scaling)
	METHOD(RbpfModel, set_jitter_scaling, "jitter_scales"_a)
	CDOC(RbpfModel);

	CLASS(RbpfStrategy, RbpfModel, StandardModelStrategy)
	CTOR(RbpfStrategy,
	     PARAMS(int, double, bool, ResamplingFunction),
	     "num_particles"_a        = 100,
	     "resampling_threshold"_a = 0.75,
	     "calc_single_jacobian"_a = true,
	     "_resampling_fun"_a      = ResamplingFunction{&residual_resample_with_replacement})
	METHOD_VOID(RbpfStrategy, get_particle_state_marks)
	METHOD_VOID(RbpfStrategy, get_state_particles)
	METHOD_VOID(RbpfStrategy, get_state_particles_cov)
	METHOD_VOID(RbpfStrategy, get_state_particles_weights)
	PROPERTY(RbpfStrategy, particle_count_target)
	FIELD(RbpfStrategy, resampling_threshold)
	CDOC(RbpfStrategy);

	CLASS(ResamplingResult)
	CTOR_NODOC_DEFAULT
	FIELD(ResamplingResult, index)
	FIELD(ResamplingResult, index_count)
	CDOC(ResamplingResult);

	FUNCTION(systematic_resampling, "weights"_a, "M"_a)
	FUNCTION(residual_resample_with_replacement, "weights"_a, "m_arg"_a)

	CLASS(SampledFogmBlock, StateBlock<>)
	CTOR(SampledFogmBlock,
	     PARAMS(const std::string &,
	            Vector,
	            Vector,
	            size_t,
	            not_null<std::shared_ptr<RandomNumberGenerator>>),
	     "label"_a,
	     "time_constants"_a,
	     "state_sigmas"_a,
	     "num_states"_a,
	     "rng"_a)
	CTOR_OVERLOAD(SampledFogmBlock,
	              PARAMS(const std::string &,
	                     double,
	                     double,
	                     size_t,
	                     not_null<std::shared_ptr<RandomNumberGenerator>>),
	              _2,
	              "label"_a,
	              "time_constant"_a,
	              "state_sigma"_a,
	              "num_states"_a,
	              "rng"_a)
	CDOC(SampledFogmBlock);

	CLASS(NonlinearAltitudeProcessor, MeasurementProcessor<>)
	CTOR(NonlinearAltitudeProcessor,
	     PARAMS(std::string,
	            std::vector<std::string>,
	            std::vector<unsigned long>,
	            not_null<std::shared_ptr<SimpleElevationProvider>>,
	            size_t,
	            int),
	     "label"_a,
	     "state_block_labels"_a,
	     "marked_state_indices"_a,
	     "elevation_provider"_a,
	     "state_vector_length"_a,
	     "warning_threshold"_a = 10000)
	CDOC(NonlinearAltitudeProcessor);

	py::class_<MeasurementBuffer, PYBIND11_SH_DEF(MeasurementBuffer)>(m, "MeasurementBuffer")
	    .def(
	        "__iter__",
	        [](MeasurementBuffer &buf) { return py::make_iterator(buf.begin(), buf.end()); },
	        py::keep_alive<0, 1>())
	    .def(py::init<>())
	    .def("add_measurement",
	         &MeasurementBuffer::add_measurement,
	         PROCESS_DOC(MeasurementBufferBase_add_measurement),
	         "time"_a,
	         "measurement"_a,
	         "covariance"_a)
	    .def("get_measurement",
	         &MeasurementBuffer::get_measurement,
	         PROCESS_DOC(MeasurementBufferBase_get_measurement),
	         "time"_a)
	    .def("get_covariance",
	         &MeasurementBuffer::get_covariance,
	         PROCESS_DOC(MeasurementBufferBase_get_covariance),
	         "time"_a)
	    .def("remove_old_measurements",
	         &MeasurementBuffer::remove_old_measurements,
	         PROCESS_DOC(MeasurementBufferBase_remove_old_measurements),
	         "time"_a)
	    .def("covers_time",
	         &MeasurementBuffer::covers_time,
	         PROCESS_DOC(MeasurementBufferBase_covers_time),
	         "time"_a)
	    .def("get_times",
	         &MeasurementBuffer::get_times,
	         PROCESS_DOC(MeasurementBufferBase_get_times))
	    .def("get_last_time",
	         &MeasurementBuffer::get_last_time,
	         PROCESS_DOC(MeasurementBufferBase_get_last_time))
	    .def("is_empty", &MeasurementBuffer::is_empty, PROCESS_DOC(MeasurementBufferBase_is_empty))
	    .def("clear", &MeasurementBuffer::clear, PROCESS_DOC(MeasurementBufferBase_clear))
	    .def("get_measurements_around",
	         &MeasurementBuffer::get_measurements_around,
	         PROCESS_DOC(MeasurementBuffer_get_measurements_around),
	         "t_0"_a,
	         "t_1"_a)
	    .def("get_average_variance",
	         &MeasurementBuffer::get_average_variance,
	         PROCESS_DOC(MeasurementBuffer_get_average_variance),
	         "t_0"_a,
	         "t_1"_a) CDOC(MeasurementBuffer);

	py::class_<MeasurementBuffer3d, PYBIND11_SH_DEF(MeasurementBuffer3d)>(m, "MeasurementBuffer3d")
	    .def(py::init<>())
	    .def("add_measurement",
	         &MeasurementBuffer3d::add_measurement,
	         PROCESS_DOC(MeasurementBufferBase_add_measurement),
	         "time"_a,
	         "measurement"_a,
	         "covariance"_a)
	    .def("get_measurement",
	         &MeasurementBuffer3d::get_measurement,
	         PROCESS_DOC(MeasurementBufferBase_get_measurement),
	         "time"_a)
	    .def("get_covariance",
	         &MeasurementBuffer3d::get_covariance,
	         PROCESS_DOC(MeasurementBufferBase_get_covariance),
	         "time"_a)
	    .def("remove_old_measurements",
	         &MeasurementBuffer3d::remove_old_measurements,
	         PROCESS_DOC(MeasurementBufferBase_remove_old_measurements),
	         "time"_a)
	    .def("covers_time",
	         &MeasurementBuffer3d::covers_time,
	         PROCESS_DOC(MeasurementBufferBase_covers_time),
	         "time"_a)
	    .def("get_times",
	         &MeasurementBuffer3d::get_times,
	         PROCESS_DOC(MeasurementBufferBase_get_times))
	    .def("get_last_time",
	         &MeasurementBuffer3d::get_last_time,
	         PROCESS_DOC(MeasurementBufferBase_get_last_time))
	    .def(
	        "is_empty", &MeasurementBuffer3d::is_empty, PROCESS_DOC(MeasurementBufferBase_is_empty))
	    .def("clear", &MeasurementBuffer3d::clear, PROCESS_DOC(MeasurementBufferBase_clear))
	        CDOC(MeasurementBuffer3d);
}

void add_filtering_functions(pybind11::module &m) {
	m.doc() = "Bindings to the NavToolkit/C++ Sensor Fusion Framework";

	CLASS(EstimateWithCovariance)
	CTOR(EstimateWithCovariance, PARAMS(Vector, Matrix), "estimate"_a, "covariance"_a)
	FIELD(EstimateWithCovariance, estimate)
	FIELD(EstimateWithCovariance, covariance)
	CDOC(EstimateWithCovariance);


	class PublicFusionStrategy : public FusionStrategy {
	public:
		using FusionStrategy::on_fusion_engine_state_block_added_impl;
		using FusionStrategy::on_fusion_engine_state_block_removed_impl;
		using FusionStrategy::on_state_count_changed;
		using FusionStrategy::set_covariance_slice_impl;
		using FusionStrategy::set_estimate_slice_impl;
	};

	CLASS(FusionStrategy, PyFusionStrategy<>)
	CTOR_NODOC_DEFAULT
	METHOD_VOID(FusionStrategy, get_num_states)
	METHOD_OVERLOAD(FusionStrategy, on_fusion_engine_state_block_added, Size, , "how_many"_a)
	METHOD_OVERLOAD(FusionStrategy,
	                on_fusion_engine_state_block_added,
	                PARAMS(Vector const &, Matrix const &),
	                _2,
	                "initial_estimate"_a,
	                "initial_covariance"_a)
	METHOD_OVERLOAD(FusionStrategy,
	                on_fusion_engine_state_block_added,
	                PARAMS(Vector const &, Matrix const &, Matrix const &),
	                _3,
	                "initial_estimate"_a,
	                "initial_covariance"_a,
	                "cross_covariance"_a)
	METHOD(FusionStrategy, on_fusion_engine_state_block_removed, "first_index"_a, "count"_a)
	METHOD_VOID(FusionStrategy, get_estimate)
	METHOD(FusionStrategy, set_estimate_slice, "new_estimate"_a, "first_index"_a = 0)
	METHOD_VOID(FusionStrategy, get_covariance)
	METHOD_OVERLOAD(FusionStrategy,
	                set_covariance_slice,
	                PARAMS(Matrix const &, Size, Size),
	                ,
	                "new_covariance"_a,
	                "first_row"_a,
	                "first_col"_a)
	METHOD_OVERLOAD(FusionStrategy,
	                set_covariance_slice,
	                PARAMS(Matrix const &, Size),
	                _2,
	                "new_covariance"_a,
	                "first_state"_a = 0)
	METHOD_VOID(FusionStrategy, clone)
	METHOD(FusionStrategy, symmetricize_covariance, "rtol"_a = 1e-5, "atol"_a = 1e-8)
	METHOD_PROTECTED(FusionStrategy,
	                 on_fusion_engine_state_block_added_impl,
	                 PublicFusionStrategy,
	                 "initial_estimate"_a,
	                 "initial_covariance"_a)
	METHOD_PROTECTED(FusionStrategy,
	                 set_covariance_slice_impl,
	                 PublicFusionStrategy,
	                 "new_covariance"_a,
	                 "first_row"_a,
	                 "first_col"_a)
	METHOD_PROTECTED(FusionStrategy,
	                 set_estimate_slice_impl,
	                 PublicFusionStrategy,
	                 "new_estimate"_a,
	                 "first_index"_a)
	METHOD_PROTECTED(FusionStrategy,
	                 on_fusion_engine_state_block_removed_impl,
	                 PublicFusionStrategy,
	                 "first_index"_a,
	                 "count"_a)
	METHOD_PROTECTED_VOID(FusionStrategy, on_state_count_changed, PublicFusionStrategy)
	CDOC(FusionStrategy);

	CLASS(SampledDynamicsModel)
	CTOR(SampledDynamicsModel, PARAMS(SampledDynamicsModel::SampledPropagationFunction), "g"_a)
	FIELD(SampledDynamicsModel, g)
	CDOC(SampledDynamicsModel);

	CLASS(SampledMeasurementModel)
	CTOR(SampledMeasurementModel,
	     PARAMS(Vector, SampledMeasurementModel::SampledUpdateFunction),
	     "z"_a,
	     "h"_a)
	FIELD(SampledMeasurementModel, z)
	FIELD(SampledMeasurementModel, h)
	CDOC(SampledMeasurementModel);

	CLASS(SampledModelStrategy, FusionStrategy, PySampledModelStrategy<>)
	CTOR_NODOC_DEFAULT
	METHOD(SampledModelStrategy, propagate, "dynamics_model"_a)
	METHOD(SampledModelStrategy, update, "measurement_model"_a)
	CDOC(SampledModelStrategy);

	CLASS(StandardDynamicsModel)
	CTOR(StandardDynamicsModel,
	     PARAMS(StandardDynamicsModel::StateTransitionFunction, Matrix, Matrix),
	     "g"_a,
	     "Phi"_a,
	     "Qd"_a)
	CTOR_OVERLOAD(StandardDynamicsModel, PARAMS(Matrix, Matrix), _2, "Phi"_a, "Qd"_a)
	FIELD(StandardDynamicsModel, g)
	FIELD(StandardDynamicsModel, Phi)
	FIELD(StandardDynamicsModel, Qd)
	CDOC(StandardDynamicsModel);

	CLASS(StandardMeasurementModel)
	CTOR(StandardMeasurementModel,
	     PARAMS(Vector, StandardMeasurementModel::MeasurementFunction, Matrix, Matrix),
	     "z"_a,
	     "h"_a,
	     "H"_a,
	     "R"_a)
	CTOR_OVERLOAD(StandardMeasurementModel, PARAMS(Vector, Matrix, Matrix), _2, "z"_a, "H"_a, "R"_a)
	FIELD(StandardMeasurementModel, z)
	FIELD(StandardMeasurementModel, h)
	FIELD(StandardMeasurementModel, H)
	FIELD(StandardMeasurementModel, R)
	CDOC(StandardMeasurementModel);

	CLASS(StandardModelStrategy, PyStandardModelStrategy<>)
	CTOR_NODOC_DEFAULT
	METHOD(StandardModelStrategy, propagate, "dynamics_model"_a)
	METHOD(StandardModelStrategy, update, "measurement_model"_a)
	METHOD_PROTECTED(
	    StandardModelStrategy, check_update_args, PyStandardModelStrategy<>, "measurement_model"_a)
	METHOD_PROTECTED(StandardModelStrategy,
	                 validate_linearized_propagate,
	                 PyStandardModelStrategy<>,
	                 "dynamics_jacobian"_a,
	                 "dynamics_noise_covariance"_a)
	METHOD_PROTECTED(StandardModelStrategy,
	                 validate_linearized_update,
	                 PyStandardModelStrategy<>,
	                 "measurement_jacobian"_a,
	                 "measurement_noise_covariance"_a,
	                 "measurement"_a,
	                 "hx"_a)
	CDOC(StandardModelStrategy);

	// TimestampedDataSeries, TimestampedDataTimeIterator, and
	// TimestampedDataPointerCompareRingBufferIterator are templated classes, and require separate
	// bindings for each type implementation. Could implement common types, but isn't really worth
	// it. If we want to bind these later, we could pass a type name to a function that implements
	// the class for that type.

	CLASS(GaussianVectorData, aspn_xtensor::AspnBase, EstimateWithCovariance)
	CTOR(GaussianVectorData,
	     PARAMS(aspn_xtensor::TypeTimestamp, Vector, Matrix, AspnMessageType),
	     "time_of_validity"_a,
	     "estimate"_a,
	     "covariance"_a,
	     "message_type"_a = ASPN_EXTENDED_BEGIN)
	CDOC(GaussianVectorData);

	CLASS(PairedPva, aspn_xtensor::AspnBase)
	CTOR(PairedPva,
	     PARAMS(std::shared_ptr<aspn_xtensor::AspnBase>, NavSolution, AspnMessageType),
	     "md"_a,
	     "pva"_a,
	     "message_type"_a = ASPN_EXTENDED_BEGIN)
	FIELD(PairedPva, meas_data)
	FIELD(PairedPva, ref_pva)
	CDOC(PairedPva);

	CLASS(CorrectedGnssPseudorangeMeasurement)
	CTOR_NODOC(
	    PARAMS(Vector, Matrix, std::vector<uint16_t>), "pr_corrected"_a, "sv_position"_a, "prns"_a)
	FIELD(CorrectedGnssPseudorangeMeasurement, pr_corrected)
	FIELD(CorrectedGnssPseudorangeMeasurement, sv_position)
	FIELD(CorrectedGnssPseudorangeMeasurement, prns)
	CDOC(CorrectedGnssPseudorangeMeasurement);

	CLASS(PseudorangeDopplerMeasurements)
	CTOR_NODOC(PARAMS(Vector, Vector, Matrix, Matrix, std::vector<uint16_t>),
	           "pr_corrected"_a,
	           "pr_rate"_a,
	           "sv_position"_a,
	           "sv_velocity"_a,
	           "prns"_a)
	FIELD(PseudorangeDopplerMeasurements, pr_corrected)
	FIELD(PseudorangeDopplerMeasurements, pr_rate)
	FIELD(PseudorangeDopplerMeasurements, sv_position)
	FIELD(PseudorangeDopplerMeasurements, sv_velocity)
	FIELD(PseudorangeDopplerMeasurements, prns)
	CDOC(PseudorangeDopplerMeasurements);

	CLASS(TrackedGnssObservations)
	CTOR_NODOC_DEFAULT
	METHOD(TrackedGnssObservations, update, "time"_a, "observation_prns"_a)
	METHOD_VOID(TrackedGnssObservations, changed)
	METHOD_VOID(TrackedGnssObservations, tracked)
	METHOD_VOID(TrackedGnssObservations, added)
	METHOD_VOID(TrackedGnssObservations, removed)
	CDOC(TrackedGnssObservations);

	CLASS(RangeInfo)
	CTOR_NODOC(PARAMS(Vector3, double), "range_vector"_a, "range_scalar"_a)
	FIELD(RangeInfo, range_vector)
	FIELD(RangeInfo, range_scalar)
	CDOC(RangeInfo);

	class PyStandardStateBlock : public StateBlock<> {
	public:
		using StateBlock::StateBlock;
		StandardDynamicsModel generate_dynamics(navtk::filtering::GenXhatPFunction gen_x_and_p_func,
		                                        aspn_xtensor::TypeTimestamp time_from,
		                                        aspn_xtensor::TypeTimestamp time_to) override {
			PYBIND11_OVERRIDE_PURE(StandardDynamicsModel,
			                       StateBlock<>,
			                       generate_dynamics,
			                       gen_x_and_p_func,
			                       time_from,
			                       time_to);
		}

		void receive_aux_data(const AspnBaseVector &aux_data) override {
			PYBIND11_OVERRIDE(void, StandardStateBlock, receive_aux_data, aux_data);
		}

		not_null<std::shared_ptr<StateBlock<>>> clone() override {
			PYBIND11_OVERRIDE_PURE(not_null<std::shared_ptr<StateBlock<>>>, StateBlock, clone, );
		}
	};

	// Allow bindings to access these protected members.
	class PyStateBlockPublic : public StateBlock<> {
	public:
		using StateBlock::discretization_strategy;
		using StateBlock::num_states;
	};

	// clang-format off
	CLASST(StateBlock, PyStandardStateBlock)
	CTOR_OVERLOAD(
	    StateBlock,
	    PARAMS(size_t, std::string, DiscretizationStrategy, navtk::Matrix),
	    _2,
	    "num_states"_a,
	    "label"_a,
	    "discretization_strategy"_a = DiscretizationStrategy{&full_order_discretization_strategy},
	    "Q"_a                       = navtk::zeros(1, 1))
	METHODT(StateBlock, generate_dynamics, "xhat"_a, "time_from"_a, "time_to"_a)
	METHODT(StateBlock, receive_aux_data, "aux_data"_a)
	METHODT_VOID(StateBlock, clone)
	METHODT_VOID(StateBlock, get_label)
	METHODT_VOID(StateBlock, get_num_states)
	.def_readwrite("num_states",
					&PyStateBlockPublic::num_states,
					PROCESS_DOC(StateBlock_num_states))
	.def_readwrite("discretization_strategy",
					&PyStateBlockPublic::discretization_strategy,
					PROCESS_DOC(StateBlock_discretization_strategy))
	CDOC(StateBlock);
	// clang-format on

	class PyMeasurementProcessor : public MeasurementProcessor<> {
	public:
		PyMeasurementProcessor(std::string label, std::string state_block_label)
		    : MeasurementProcessor<>(label, state_block_label) {}

		PyMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels)
		    : MeasurementProcessor<>(label, state_block_labels) {}

		std::shared_ptr<StandardMeasurementModel> generate_model(
		    std::shared_ptr<aspn_xtensor::AspnBase> measurement,
		    navtk::filtering::GenXhatPFunction gen_x_and_p_func) override {
			PYBIND11_OVERRIDE_PURE(std::shared_ptr<StandardMeasurementModel>,
			                       MeasurementProcessor<>,
			                       generate_model,
			                       measurement,
			                       gen_x_and_p_func);
		}
		void receive_aux_data(const AspnBaseVector &aux_data) override {
			PYBIND11_OVERRIDE(void, MeasurementProcessor, receive_aux_data, aux_data);
		}

		not_null<std::shared_ptr<MeasurementProcessor<>>> clone() override {
			PYBIND11_OVERRIDE_PURE(
			    not_null<std::shared_ptr<MeasurementProcessor<>>>, MeasurementProcessor, clone, );
		}
	};

	CLASS(Pose)
	CTOR(
	    Pose, PARAMS(Vector3, Matrix3, aspn_xtensor::TypeTimestamp), "pos"_a, "rot_mat"_a, "time"_a)
	FIELD(Pose, pos)
	FIELD(Pose, rot_mat)
	FIELD(Pose, time)
	CDOC(Pose);

	CLASS(NavSolution, Pose)
	CTOR(NavSolution, PARAMS(Pose, Vector3), "pose"_a, "vel"_a)
	CTOR_OVERLOAD(NavSolution,
	              PARAMS(Vector3, Vector3, Matrix3, aspn_xtensor::TypeTimestamp),
	              _2,
	              "pos"_a,
	              "vel"_a,
	              "rot_mat"_a,
	              "time"_a)
	FIELD(NavSolution, vel)
	CDOC(NavSolution);

	CLASST(MeasurementProcessor, PyMeasurementProcessor)
	CTOR_NODOC(PARAMS(std::string, std::string), "label"_a, "state_block_label"_a)
	CTOR_NODOC(PARAMS(std::string, std::vector<std::string>), "label"_a, "state_block_labels"_a)
	METHODT(MeasurementProcessor, generate_model, "measurement"_a, "gen_x_and_p_func"_a)
	METHODT(MeasurementProcessor, receive_aux_data, NOT_NONE("aux_data"))
	METHODT_VOID(MeasurementProcessor, clone)
	METHODT_VOID(MeasurementProcessor, get_label)
	METHODT_VOID(MeasurementProcessor, get_state_block_labels)
	CDOC(MeasurementProcessor);

	// Allow bindings to access these protected members.
	class PyLinearizedStrategyBasePublic : public LinearizedStrategyBase {
	public:
		using LinearizedStrategyBase::LinearizedStrategyBase;

		using LinearizedStrategyBase::covariance;
		using LinearizedStrategyBase::estimate;
		using LinearizedStrategyBase::on_fusion_engine_state_block_added_impl;
		using LinearizedStrategyBase::on_fusion_engine_state_block_removed_impl;
		using LinearizedStrategyBase::set_covariance_slice_impl;
		using LinearizedStrategyBase::set_estimate_slice_impl;
	};

	// clang-format off
	CLASS(LinearizedStrategyBase,
	      PyLinearizedStrategyBase<>,
	      FusionStrategy)
	CTOR_NODOC_DEFAULT
	CTOR(LinearizedStrategyBase, PARAMS(const FusionStrategy &), "src"_a)
    .def_readwrite("estimate",
                    &PyLinearizedStrategyBasePublic::estimate,
                    PROCESS_DOC(LinearizedStrategyBase_estimate))
    .def_readwrite("covariance",
                    &PyLinearizedStrategyBasePublic::covariance,
                    PROCESS_DOC(LinearizedStrategyBase_covariance))
    CDOC(LinearizedStrategyBase);
	// clang-format on

	CLASS(EkfStrategy, PyEkfStrategy<>, StandardModelStrategy, LinearizedStrategyBase)
	CTOR_NODOC_DEFAULT
	CDOC(EkfStrategy);

	using PyUkfStrategy =
	    PyEkfStrategy<UkfStrategy,
	                  PyLinearizedStrategyBase<UkfStrategy, PyStandardModelStrategy<UkfStrategy>>>;
	CLASS(UkfStrategy, PyUkfStrategy, StandardModelStrategy, LinearizedStrategyBase)
	CTOR_NODOC_DEFAULT
	CDOC(UkfStrategy);

	NAMESPACE_FUNCTION(
	    first_order_discretization_strategy, navtk::filtering, "F"_a, "G"_a, "Q"_a, "dt"_a);
	NAMESPACE_FUNCTION(
	    second_order_discretization_strategy, navtk::filtering, "F"_a, "G"_a, "Q"_a, "dt"_a);
	NAMESPACE_FUNCTION(
	    full_order_discretization_strategy, navtk::filtering, "F"_a, "G"_a, "Q"_a, "dt"_a);

	CLASS(StandardFusionEngine)
	CTOR(StandardFusionEngine,
	     PARAMS(const aspn_xtensor::TypeTimestamp &,
	            not_null<std::shared_ptr<StandardModelStrategy>>),
	     "cur_time"_a,
	     NOT_NONE("strategy"))
	CTOR_OVERLOAD(StandardFusionEngine,
	              PARAMS(const aspn_xtensor::TypeTimestamp &),
	              _2,
	              "cur_time"_a = aspn_xtensor::TypeTimestamp((int64_t)0))
	CTOR_OVERLOAD(StandardFusionEngine, PARAMS(const StandardFusionEngine &), _3, "other"_a)
	METHOD_VOID(StandardFusionEngine, get_time)
	METHOD(StandardFusionEngine, set_time, "time"_a)
	METHOD_VOID(StandardFusionEngine, get_state_block_names_list)
	METHOD(StandardFusionEngine, has_block, "label"_a)
	METHOD(StandardFusionEngine, has_virtual_state_block, "label"_a)
	METHOD_OVERLOAD(StandardFusionEngine, get_state_block, const std::string &, , "label"_a)
	METHOD_OVERLOAD_CONST(StandardFusionEngine, get_state_block, const std::string &, _2, "label"_a)
	METHOD(StandardFusionEngine, get_state_block_covariance, "label"_a)
	METHOD(StandardFusionEngine, set_state_block_covariance, "label"_a, "covariance"_a)
	METHOD(StandardFusionEngine, get_state_block_estimate, "label"_a)
	METHOD(StandardFusionEngine, set_state_block_estimate, "label"_a, "estimate"_a)
	METHOD(StandardFusionEngine, add_state_block, NOT_NONE("block"))
	METHOD(StandardFusionEngine, remove_state_block, "label"_a)
	METHOD(
	    StandardFusionEngine, set_cross_term_process_covariance, "label1"_a, "label2"_a, "block"_a)
	METHOD(StandardFusionEngine, get_cross_term_covariance, "label1"_a, "label2"_a)
	METHOD(StandardFusionEngine, set_cross_term_covariance, "label1"_a, "label2"_a, "block"_a)
	METHOD(StandardFusionEngine, give_state_block_aux_data, "label"_a, NOT_NONE("data"))
	METHOD(StandardFusionEngine, give_measurement_processor_aux_data, "label"_a, NOT_NONE("data"))
	METHOD(StandardFusionEngine, add_measurement_processor, NOT_NONE("processor"))
	METHOD(StandardFusionEngine, remove_measurement_processor, "label"_a)
	METHOD_VOID(StandardFusionEngine, get_measurement_processor_names_list)
	METHOD(StandardFusionEngine, has_processor, "label"_a)
	METHOD_OVERLOAD(
	    StandardFusionEngine, get_measurement_processor, const std::string &, , "label"_a)
	METHOD_OVERLOAD_CONST(
	    StandardFusionEngine, get_measurement_processor, const std::string &, _2, "label"_a)
	METHOD(StandardFusionEngine, propagate, "time"_a)
	METHOD(StandardFusionEngine,
	       update,
	       "processor_label"_a,
	       NOT_NONE("measurement"),
	       "timestamp"_a = nullptr)
	METHOD(StandardFusionEngine, peek_ahead, "time"_a, "state_block_labels"_a)
	METHOD(StandardFusionEngine, reset_state_estimate, "time"_a, "label"_a, "indices"_a)
	METHOD_VOID(StandardFusionEngine, get_virtual_state_block_target_labels)
	METHOD(StandardFusionEngine, add_virtual_state_block, NOT_NONE("v"))
	METHOD(StandardFusionEngine, remove_virtual_state_block, "target"_a)
	METHOD(StandardFusionEngine, generate_x_and_p, "state_block_labels"_a)
	METHOD_VOID(StandardFusionEngine, get_num_states)
	CDOC(StandardFusionEngine);

	CLASS(GravityModel)
	METHOD(GravityModel, calculate_gravity, "earth_model"_a, "alt_msl"_a)
	CDOC(GravityModel);

	CLASS(GravityModelSchwartz, GravityModel)
	CTOR_NODOC_DEFAULT
	METHOD(GravityModelSchwartz, calculate_gravity, "earth_model"_a, "alt_msl"_a)
	CDOC(GravityModelSchwartz);

	CLASS(GravityModelTittertonAndWeston, GravityModel)
	CTOR_NODOC_DEFAULT
	METHOD(GravityModelTittertonAndWeston, calculate_gravity, "earth_model"_a, "alt_msl"_a)
	CDOC(GravityModelTittertonAndWeston);

	CLASS(Pinson15NedBlock, StateBlock<>)
	CTOR(
	    Pinson15NedBlock,
	    PARAMS(const std::string &,
	           ImuModel,
	           Pinson15NedBlock::LinearizationPointFunction,
	           DiscretizationStrategy,
	           not_null<std::shared_ptr<GravityModel>>),
	    "label"_a,
	    "imu_model"_a,
	    "lin_function"_a            = nullptr,
	    "discretization_strategy"_a = DiscretizationStrategy{&second_order_discretization_strategy},
	    NOT_NONE("gravity_model")   = std::make_shared<GravityModelSchwartz>())
	CTOR_OVERLOAD(Pinson15NedBlock, const Pinson15NedBlock &, _2, "block"_a)
	METHOD_VOID(Pinson15NedBlock, generate_f_pinson15)
	METHOD_VOID(Pinson15NedBlock, generate_q_pinson15)
	METHOD_VOID(Pinson15NedBlock, get_imu_model)
	METHOD_VOID(Pinson15NedBlock, get_lin_function)
	METHOD_VOID(Pinson15NedBlock, get_discretization_strategy)
	METHOD_VOID(Pinson15NedBlock, get_gravity_model)
	METHOD_VOID(Pinson15NedBlock, get_pva_aux)
	METHOD_VOID(Pinson15NedBlock, get_force_and_rate_aux)
	METHOD(Pinson15NedBlock, scale_phi, "phi"_a)
	CDOC(Pinson15NedBlock);

	CLASS(EarthModel)
	CTOR(EarthModel,
	     PARAMS(Vector3, Vector3, const GravityModel &),
	     "pos"_a,
	     "vel"_a,
	     "gravity"_a = GravityModelSchwartz())
	FIELD(EarthModel, lat)
	FIELD(EarthModel, alt_msl)
	FIELD(EarthModel, v_n)
	FIELD(EarthModel, v_e)
	FIELD(EarthModel, sin_l)
	FIELD(EarthModel, cos_l)
	FIELD(EarthModel, tan_l)
	FIELD(EarthModel, sec_l)
	FIELD(EarthModel, sin_2l)
	FIELD(EarthModel, r_n)
	FIELD(EarthModel, r_e)
	FIELD(EarthModel, r_zero)
	FIELD(EarthModel, lat_factor)
	FIELD(EarthModel, lon_factor)
	FIELD(EarthModel, omega_en_n)
	FIELD(EarthModel, omega_ie_n)
	FIELD(EarthModel, omega_in_n)
	FIELD(EarthModel, g_n)
	    .def_readonly("ecc", &EarthModel::ecc, PROCESS_DOC(EarthModel_ecc)) CDOC(EarthModel);

	CLASS(Pinson21NedBlock, StateBlock<>)
	CTOR(
	    Pinson21NedBlock,
	    PARAMS(const std::string &,
	           ImuModel,
	           Pinson15NedBlock::LinearizationPointFunction,
	           DiscretizationStrategy,
	           not_null<std::shared_ptr<GravityModel>>),
	    "label"_a,
	    "imu_model"_a,
	    "lin_function"_a            = nullptr,
	    "discretization_strategy"_a = DiscretizationStrategy{&second_order_discretization_strategy},
	    NOT_NONE("gravity_model")   = std::make_shared<GravityModelSchwartz>())
	CTOR_OVERLOAD(Pinson21NedBlock, const Pinson21NedBlock &, _2, "block"_a)
	METHOD_VOID(Pinson21NedBlock, generate_f_pinson)
	CDOC(Pinson21NedBlock);

	CLASS(ClockModel)
	CTOR_NODOC(PARAMS(double, double, double, double), "h_0"_a, "h_m1"_a, "h_m2"_a, "q3"_a)
	FIELD(ClockModel, h_0)
	FIELD(ClockModel, h_m1)
	FIELD(ClockModel, h_m2)
	FIELD(ClockModel, q3)
	CDOC(ClockModel);

	ENUM(ClockChoice)
	CHOICE(ClockChoice, QD)
	CHOICE(ClockChoice, QD1)
	CHOICE(ClockChoice, QD2)
	CHOICE(ClockChoice, QD3);

	ATTR(RUBIDIUM_CLOCK);
	ATTR(OVENIZED_CRYSTAL_CLOCK);
	ATTR(COMPENSATED_CRYSTAL_CLOCK);

	CLASS(ClockBiasesStateBlock, StateBlock<>)
	CTOR(ClockBiasesStateBlock,
	     PARAMS(const std::string &, ClockModel, ClockChoice, bool),
	     "label"_a,
	     "clock_model"_a,
	     "clock_choice"_a        = navtk::filtering::ClockChoice::QD,
	     "model_frequency_dot"_a = false)
	CDOC(ClockBiasesStateBlock);

	CLASS(FogmBlock, StateBlock<>)
	CTOR(FogmBlock,
	     PARAMS(const std::string &, Vector, Vector, size_t, DiscretizationStrategy),
	     "label"_a,
	     "time_constants"_a,
	     "state_sigmas"_a,
	     "num_states"_a,
	     "discretization_strategy"_a = DiscretizationStrategy{&full_order_discretization_strategy})
	CTOR_OVERLOAD(
	    FogmBlock,
	    PARAMS(const std::string &, double, double, size_t, DiscretizationStrategy),
	    _2,
	    "label"_a,
	    "time_constant"_a,
	    "state_sigma"_a,
	    "num_states"_a,
	    "discretization_strategy"_a = DiscretizationStrategy{&full_order_discretization_strategy})
	CTOR_OVERLOAD(FogmBlock, PARAMS(const FogmBlock &), _3, "block"_a)
	CDOC(FogmBlock);

	CLASS(FogmAccel, FogmBlock)
	CTOR(FogmAccel,
	     PARAMS(const std::string &, Vector, Vector, size_t, DiscretizationStrategy),
	     "label"_a,
	     "time_constants"_a,
	     "state_sigmas"_a,
	     "num_dimensions"_a,
	     "discretization_strategy"_a = DiscretizationStrategy{&full_order_discretization_strategy})
	CTOR_OVERLOAD(
	    FogmAccel,
	    PARAMS(const std::string &, double, double, size_t, DiscretizationStrategy),
	    _2,
	    "label"_a,
	    "time_constant"_a,
	    "state_sigma"_a,
	    "num_dimensions"_a,
	    "discretization_strategy"_a = DiscretizationStrategy{&full_order_discretization_strategy})
	CDOC(FogmAccel);

	CLASS(FogmVelocity, FogmBlock)
	CTOR(FogmVelocity,
	     PARAMS(const std::string &, Vector, Vector, size_t, DiscretizationStrategy),
	     "label"_a,
	     "time_constants"_a,
	     "state_sigmas"_a,
	     "num_dimensions"_a,
	     "discretization_strategy"_a = DiscretizationStrategy{&full_order_discretization_strategy})
	CTOR_OVERLOAD(
	    FogmVelocity,
	    PARAMS(const std::string &, double, double, size_t, DiscretizationStrategy),
	    _2,
	    "label"_a,
	    "time_constant"_a,
	    "state_sigma"_a,
	    "num_dimensions"_a,
	    "discretization_strategy"_a = DiscretizationStrategy{&full_order_discretization_strategy})
	CDOC(FogmVelocity);

	CLASS(DirectMeasurementProcessor, MeasurementProcessor<>)
	CTOR(DirectMeasurementProcessor,
	     PARAMS(std::string, const std::string &, Matrix),
	     "label"_a,
	     "state_block_label"_a,
	     "measurement_matrix"_a)
	CTOR_OVERLOAD(DirectMeasurementProcessor,
	              PARAMS(std::string, vector<std::string>, Matrix),
	              _2,
	              "label"_a,
	              "state_block_labels"_a,
	              "measurement_matrix"_a)
	CTOR_OVERLOAD(
	    DirectMeasurementProcessor, PARAMS(const DirectMeasurementProcessor &), _3, "processor"_a)
	METHOD_VOID(DirectMeasurementProcessor, get_measurement_matrix)
	CDOC(DirectMeasurementProcessor);

	CLASS(RelativeHumidityAux, aspn_xtensor::AspnBase)
	CTOR(RelativeHumidityAux, double, "t"_a)
	FIELD(RelativeHumidityAux, tropo_rel_humidity)
	CDOC(RelativeHumidityAux);

	CLASS(AltitudeMeasurementProcessor, MeasurementProcessor<>)
	CTOR(AltitudeMeasurementProcessor,
	     PARAMS(std::string, const std::string &),
	     "label"_a,
	     "state_block_label"_a)
	CTOR_OVERLOAD(AltitudeMeasurementProcessor,
	              PARAMS(std::string, vector<std::string>),
	              _2,
	              "label"_a,
	              "state_block_labels"_a)
	CDOC(AltitudeMeasurementProcessor);

	CLASS(AltitudeMeasurementProcessorWithBias, MeasurementProcessor<>)
	CTOR(AltitudeMeasurementProcessorWithBias,
	     PARAMS(std::string, const std::string &, const std::string &),
	     "label"_a,
	     "pinson_label"_a,
	     "altitude_bias_label"_a)
	CTOR_OVERLOAD(AltitudeMeasurementProcessorWithBias,
	              PARAMS(std::string, vector<std::string>),
	              _2,
	              "label"_a,
	              "state_block_labels"_a)
	CDOC(AltitudeMeasurementProcessorWithBias);

	CLASS(Attitude3dMeasurementProcessor, MeasurementProcessor<>)
	CTOR(Attitude3dMeasurementProcessor,
	     PARAMS(std::string, const std::string &),
	     "label"_a,
	     "state_block_label"_a)
	CTOR_OVERLOAD(Attitude3dMeasurementProcessor,
	              PARAMS(std::string, vector<std::string>),
	              _2,
	              "label"_a,
	              "state_block_labels"_a)
	CDOC(Attitude3dMeasurementProcessor);

	CLASS(BiasedRangeProcessor, MeasurementProcessor<>)
	CTOR(BiasedRangeProcessor,
	     PARAMS(std::string, const std::string &, const std::string &),
	     "label"_a,
	     "position_label"_a,
	     "bias_label"_a)
	CTOR_OVERLOAD(BiasedRangeProcessor,
	              PARAMS(std::string, vector<std::string>),
	              _2,
	              "label"_a,
	              "state_block_labels"_a)
	CDOC(BiasedRangeProcessor);

	CLASS(DeltaPositionMeasurementProcessor, MeasurementProcessor<>)
	CTOR(DeltaPositionMeasurementProcessor,
	     PARAMS(std::string,
	            const std::string &,
	            Matrix,
	            bool,
	            bool,
	            bool,
	            AspnMeasurementDeltaPositionReferenceFrame),
	     "label"_a,
	     "state_block_label"_a,
	     "measurement_matrix"_a,
	     "use_term1"_a,
	     "use_term2"_a,
	     "use_term3"_a,
	     "expected_frame"_a = ASPN_MEASUREMENT_DELTA_POSITION_REFERENCE_FRAME_NED)
	CTOR_OVERLOAD(DeltaPositionMeasurementProcessor,
	              PARAMS(std::string,
	                     vector<std::string>,
	                     Matrix,
	                     bool,
	                     bool,
	                     bool,
	                     AspnMeasurementDeltaPositionReferenceFrame),
	              _2,
	              "label"_a,
	              "state_block_labels"_a,
	              "measurement_matrix"_a,
	              "use_term1"_a,
	              "use_term2"_a,
	              "use_term3"_a,
	              "expected_frame"_a)
	CDOC(DeltaPositionMeasurementProcessor);

	CLASS(DirectionToPoints3dMeasurementProcessor, MeasurementProcessor<>)
	CTOR(DirectionToPoints3dMeasurementProcessor,
	     PARAMS(std::string, const std::string &),
	     "label"_a,
	     "state_block_label"_a)
	CDOC(DirectionToPoints3dMeasurementProcessor);

	CLASS(GeodeticPos3dMeasurementProcessor, MeasurementProcessor<>)
	CTOR(GeodeticPos3dMeasurementProcessor,
	     PARAMS(std::string, const std::string &, Matrix),
	     "label"_a,
	     "state_block_label"_a,
	     "measurement_matrix"_a)
	CTOR_OVERLOAD(GeodeticPos3dMeasurementProcessor,
	              PARAMS(std::string, vector<std::string>, Matrix),
	              _2,
	              "label"_a,
	              "state_block_labels"_a,
	              "measurement_matrix"_a)
	CDOC(GeodeticPos3dMeasurementProcessor);

	CLASS(GeodeticPos2dMeasurementProcessor, MeasurementProcessor<>)
	CTOR(GeodeticPos2dMeasurementProcessor,
	     PARAMS(std::string, const std::string &, Matrix),
	     "label"_a,
	     "state_block_label"_a,
	     "measurement_matrix"_a)
	CTOR_OVERLOAD(GeodeticPos2dMeasurementProcessor,
	              PARAMS(std::string, vector<std::string>, Matrix),
	              _2,
	              "label"_a,
	              "state_block_labels"_a,
	              "measurement_matrix"_a)
	CDOC(GeodeticPos2dMeasurementProcessor);

	CLASS(MagnetometerToHeadingMeasurementProcessor, MeasurementProcessor<>)
	CTOR(MagnetometerToHeadingMeasurementProcessor,
	     PARAMS(std::string,
	            const std::string &,
	            const std::shared_ptr<MagnetometerCalibration> &,
	            double,
	            double,
	            const Matrix &),
	     "label"_a,
	     "state_block_label"_a,
	     "calibration"_a,
	     "heading_var"_a          = -1.0,
	     "magnetic_declination"_a = 0.0,
	     "dcm"_a                  = Matrix{{1., 0, 0}, {0, 1., 0}, {0, 0, 1.}})
	CTOR_OVERLOAD(MagnetometerToHeadingMeasurementProcessor,
	              PARAMS(std::string,
	                     vector<std::string>,
	                     const std::shared_ptr<MagnetometerCalibration> &,
	                     double,
	                     double,
	                     const Matrix &),
	              _2,
	              "label"_a,
	              "state_block_labels"_a,
	              "calibration"_a,
	              "heading_var"_a          = -1.0,
	              "magnetic_declination"_a = 0.0,
	              "dcm"_a                  = Matrix{{1., 0, 0}, {0, 1., 0}, {0, 0, 1.}})
	CDOC(MagnetometerToHeadingMeasurementProcessor);

	CLASS(MagneticFieldMagnitudeMeasurementProcessor, MeasurementProcessor<>)
	CTOR(MagneticFieldMagnitudeMeasurementProcessor,
	     PARAMS(std::string, const std::string &, Vector, Vector, Matrix),
	     "label"_a,
	     "state_block_label"_a,
	     "x_vec"_a,
	     "y_vec"_a,
	     "map"_a)
	CTOR_OVERLOAD(MagneticFieldMagnitudeMeasurementProcessor,
	              PARAMS(std::string, vector<std::string>, Vector, Vector, Matrix),
	              _2,
	              "label"_a,
	              "state_block_labels"_a,
	              "x_vec"_a,
	              "y_vec"_a,
	              "map"_a)
	CDOC(MagneticFieldMagnitudeMeasurementProcessor);

	CLASS(PinsonPositionMeasurementProcessor, MeasurementProcessor<>)
	CTOR(PinsonPositionMeasurementProcessor,
	     PARAMS(const std::string &, std::vector<std::string>, TypeMounting, TypeMounting),
	     "label"_a,
	     "state_block_label"_a,
	     "inertial_mount"_a,
	     "sensor_mount"_a)
	CTOR_OVERLOAD(PinsonPositionMeasurementProcessor,
	              PARAMS(const PinsonPositionMeasurementProcessor &),
	              _2,
	              "processor"_a)
	CDOC(PinsonPositionMeasurementProcessor);

	CLASS(PositionVelocityAttitudeMeasurementProcessor, MeasurementProcessor<>)
	CTOR(PositionVelocityAttitudeMeasurementProcessor,
	     PARAMS(std::string,
	            const std::string &,
	            Matrix,
	            bool,
	            bool,
	            bool,
	            bool,
	            bool,
	            bool,
	            bool,
	            AspnMeasurementPositionVelocityAttitudeReferenceFrame),
	     "label"_a,
	     "state_block_label"_a,
	     "measurement_matrix"_a,
	     "use_p1"_a,
	     "use_p2"_a,
	     "use_p3"_a,
	     "use_v1"_a,
	     "use_v2"_a,
	     "use_v3"_a,
	     "use_quaternion"_a,
	     "expected_frame"_a)
	CTOR_OVERLOAD(PositionVelocityAttitudeMeasurementProcessor,
	              PARAMS(std::string,
	                     vector<std::string>,
	                     Matrix,
	                     bool,
	                     bool,
	                     bool,
	                     bool,
	                     bool,
	                     bool,
	                     bool,
	                     AspnMeasurementPositionVelocityAttitudeReferenceFrame),
	              _2,
	              "label"_a,
	              "state_block_labels"_a,
	              "measurement_matrix"_a,
	              "use_p1"_a,
	              "use_p2"_a,
	              "use_p3"_a,
	              "use_v1"_a,
	              "use_v2"_a,
	              "use_v3"_a,
	              "use_quaternion"_a,
	              "expected_frame"_a)
	CDOC(PositionVelocityAttitudeMeasurementProcessor);

	CLASS(VelocityMeasurementProcessor, MeasurementProcessor<>)
	CTOR(VelocityMeasurementProcessor,
	     PARAMS(std::string,
	            const std::string &,
	            Matrix,
	            bool,
	            bool,
	            bool,
	            AspnMeasurementVelocityReferenceFrame),
	     "label"_a,
	     "state_block_label"_a,
	     "measurement_matrix"_a,
	     "use_x"_a,
	     "use_y"_a,
	     "use_z"_a,
	     "expected_frame"_a)
	CTOR_OVERLOAD(VelocityMeasurementProcessor,
	              PARAMS(std::string,
	                     vector<std::string>,
	                     Matrix,
	                     bool,
	                     bool,
	                     bool,
	                     AspnMeasurementVelocityReferenceFrame),
	              _2,
	              "label"_a,
	              "state_block_labels"_a,
	              "measurement_matrix"_a,
	              "use_x"_a,
	              "use_y"_a,
	              "use_z"_a,
	              "expected_frame"_a)
	CDOC(VelocityMeasurementProcessor);

	CLASS(ZuptMeasurementProcessor, MeasurementProcessor<>)
	CTOR(ZuptMeasurementProcessor,
	     PARAMS(std::string, const std::string &, const Matrix3),
	     "label"_a,
	     "state_block_label"_a,
	     "cov"_a)
	CDOC(ZuptMeasurementProcessor);

	CLASS(ImuModel, aspn_xtensor::AspnBase)
	CTOR_NODOC(PARAMS(Vector3,
	                  Vector3,
	                  Vector3,
	                  Vector3,
	                  Vector3,
	                  Vector3,
	                  Vector3,
	                  Vector3,
	                  Vector3,
	                  Vector3),
	           "accel_random_walk_sigma"_a,
	           "gyro_random_walk_sigma"_a,
	           "accel_bias_sigma"_a,
	           "accel_bias_tau"_a,
	           "gyro_bias_sigma"_a,
	           "gyro_bias_tau"_a,
	           "accel_scale_factor"_a       = navtk::zeros(3),
	           "gyro_scale_factor"_a        = navtk::zeros(3),
	           "accel_bias_initial_sigma"_a = navtk::zeros(3),
	           "gyro_bias_initial_sigma"_a  = navtk::zeros(3))
	FIELD(ImuModel, accel_random_walk_sigma)
	FIELD(ImuModel, gyro_random_walk_sigma)
	FIELD(ImuModel, accel_bias_sigma)
	FIELD(ImuModel, accel_bias_tau)
	FIELD(ImuModel, gyro_bias_sigma)
	FIELD(ImuModel, gyro_bias_tau)
	FIELD(ImuModel, accel_scale_factor)
	FIELD(ImuModel, gyro_scale_factor)
	FIELD(ImuModel, accel_bias_initial_sigma)
	FIELD(ImuModel, gyro_bias_initial_sigma)
	CDOC(ImuModel);

	FUNCTION_VOID(hg9900_model);
	FUNCTION_VOID(hg1700_model);
	FUNCTION_VOID(sagem_primus200_model);
	FUNCTION_VOID(stim300_model);
	FUNCTION_VOID(ideal_imu_model);

	CLASS(VirtualStateBlockManager)
	CTOR_CLSDOC_DEFAULT(VirtualStateBlockManager)
	METHOD(VirtualStateBlockManager, add_virtual_state_block, NOT_NONE("trans"))
	METHOD(VirtualStateBlockManager, remove_virtual_state_block, "target"_a)
	METHOD(VirtualStateBlockManager, get_start_block_label, "target"_a)
	METHOD(VirtualStateBlockManager, convert, "orig"_a, "start"_a, "target"_a, "time"_a)
	METHOD(VirtualStateBlockManager, convert_estimate, "orig"_a, "start"_a, "target"_a, "time"_a)
	METHOD_OVERLOAD_CONST(VirtualStateBlockManager,
	                      jacobian,
	                      PARAMS(const EstimateWithCovariance &,
	                             const std::string &,
	                             const std::string &,
	                             const aspn_xtensor::TypeTimestamp &),
	                      ,
	                      "orig"_a,
	                      "start"_a,
	                      "target"_a,
	                      "time"_a)
	METHOD_OVERLOAD_CONST(VirtualStateBlockManager,
	                      jacobian,
	                      PARAMS(const Vector &,
	                             const std::string &,
	                             const std::string &,
	                             const aspn_xtensor::TypeTimestamp &),
	                      ,
	                      "orig"_a,
	                      "start"_a,
	                      "target"_a,
	                      "time"_a)
	CDOC(VirtualStateBlockManager);

	class PyVirtualStateBlock : public VirtualStateBlock {
	public:
		using VirtualStateBlock::VirtualStateBlock;

		not_null<std::shared_ptr<VirtualStateBlock>> clone() override {
			PYBIND11_OVERRIDE_PURE(
			    not_null<std::shared_ptr<VirtualStateBlock>>, VirtualStateBlock, clone, );
		}
		EstimateWithCovariance convert(EstimateWithCovariance const &ec,
		                               aspn_xtensor::TypeTimestamp const &time) override {
			PYBIND11_OVERRIDE(EstimateWithCovariance, VirtualStateBlock, convert, ec, time);
		}
		Vector convert_estimate(Vector const &x, aspn_xtensor::TypeTimestamp const &time) override {
			PYBIND11_OVERRIDE_PURE(Vector, VirtualStateBlock, convert_estimate, x, time);
		}
		Matrix jacobian(Vector const &x, aspn_xtensor::TypeTimestamp const &time) override {
			PYBIND11_OVERRIDE_PURE(Matrix, VirtualStateBlock, jacobian, x, time);
		}
	};

	CLASS(VirtualStateBlock, PyVirtualStateBlock)
	CTOR(VirtualStateBlock, PARAMS(std::string, std::string), "current"_a, "target"_a)
	METHOD_VOID(VirtualStateBlock, clone)
	METHOD_VOID(VirtualStateBlock, get_current)
	METHOD_VOID(VirtualStateBlock, get_target)
	METHOD(VirtualStateBlock, convert, "ec"_a, "time"_a)
	METHOD(VirtualStateBlock, convert_estimate, "x"_a, "time"_a)
	METHOD(VirtualStateBlock, jacobian, "x"_a, "time"_a)
	CDOC(VirtualStateBlock);

	class PyNumericalVirtualStateBlock : public NumericalVirtualStateBlock {
	public:
		using NumericalVirtualStateBlock::NumericalVirtualStateBlock;

		not_null<std::shared_ptr<VirtualStateBlock>> clone() override {
			PYBIND11_OVERRIDE_PURE(
			    not_null<std::shared_ptr<VirtualStateBlock>>, VirtualStateBlock, clone, );
		}

		EstimateWithCovariance convert(EstimateWithCovariance const &ec,
		                               aspn_xtensor::TypeTimestamp const &time) override {
			PYBIND11_OVERRIDE(
			    EstimateWithCovariance, NumericalVirtualStateBlock, convert, ec, time);
		}
		Vector convert_estimate(Vector const &x, aspn_xtensor::TypeTimestamp const &time) override {
			PYBIND11_OVERRIDE(Vector, NumericalVirtualStateBlock, convert_estimate, x, time);
		}
		Matrix jacobian(Vector const &x, aspn_xtensor::TypeTimestamp const &time) override {
			PYBIND11_OVERRIDE(Matrix, NumericalVirtualStateBlock, jacobian, x, time);
		}
		Vector fx(Vector const &x, aspn_xtensor::TypeTimestamp const &time) override {
			PYBIND11_OVERRIDE_PURE(Vector, NumericalVirtualStateBlock, fx, x, time);
		}
	};

	class PublicNumericalVirtualStateBlock : public NumericalVirtualStateBlock {
	public:
		using NumericalVirtualStateBlock::fx;
	};

	CLASS(NumericalVirtualStateBlock, PyNumericalVirtualStateBlock, VirtualStateBlock)
	CTOR(NumericalVirtualStateBlock,
	     PARAMS(const std::string &, const std::string &),
	     "current"_a,
	     "target"_a)
	METHOD_PROTECTED(
	    NumericalVirtualStateBlock, fx, PublicNumericalVirtualStateBlock, "x"_a, "time"_a)
	CDOC(NumericalVirtualStateBlock);

	CLASS(ScaleVirtualStateBlock, VirtualStateBlock)
	CTOR(ScaleVirtualStateBlock,
	     PARAMS(const std::string &, const std::string &, const Vector &),
	     "current"_a,
	     "target"_a,
	     "scale"_a)
	CDOC(ScaleVirtualStateBlock);

	CLASS(ChainedVirtualStateBlock, VirtualStateBlock)
	CTOR(ChainedVirtualStateBlock,
	     PARAMS(vector<not_null<std::shared_ptr<VirtualStateBlock>>>),
	     "to_chain"_a)
	CDOC(ChainedVirtualStateBlock);

	CLASS(FirstOrderVirtualStateBlock, VirtualStateBlock)
	CTOR(FirstOrderVirtualStateBlock,
	     PARAMS(const std::string &,
	            const std::string &,
	            std::function<Vector(const Vector &)>,
	            std::function<Matrix(const Vector &)>),
	     "current"_a,
	     "target"_a,
	     "fx"_a,
	     "jx"_a = 0)
	CDOC(FirstOrderVirtualStateBlock);

	CLASS(ShiftVirtualStateBlock, VirtualStateBlock)
	CTOR(ShiftVirtualStateBlock,
	     PARAMS(const std::string &,
	            const std::string &,
	            const Vector3 &,
	            const Matrix3 &,
	            std::function<Vector(const Vector &, const Vector3 &, const Matrix3 &)>,
	            std::function<Matrix(const Vector &, const Vector3 &, const Matrix3 &)>),
	     "current"_a,
	     "target"_a,
	     "l_bs_b"_a,
	     "C_platform_to_sensor"_a,
	     "fx"_a,
	     "jx"_a)
	CDOC(ShiftVirtualStateBlock);

	CLASS(SensorToPlatformCartesianVirtualStateBlock, VirtualStateBlock)
	CTOR(SensorToPlatformCartesianVirtualStateBlock,
	     PARAMS(const std::string &,
	            const std::string &,
	            const Vector3 &,
	            const Matrix3 &,
	            const Matrix3 &),
	     "current"_a,
	     "target"_a,
	     "l_bs_b"_a,
	     "C_platform_to_sensor"_a,
	     "C_k_to_j"_a)
	CDOC(SensorToPlatformCartesianVirtualStateBlock);

	CLASS(PlatformToSensorCartesianVirtualStateBlock, VirtualStateBlock)
	CTOR(PlatformToSensorCartesianVirtualStateBlock,
	     PARAMS(const std::string &,
	            const std::string &,
	            const Vector3 &,
	            const Matrix3 &,
	            const Matrix3 &),
	     "current"_a,
	     "target"_a,
	     "l_bs_b"_a,
	     "C_platform_to_sensor"_a,
	     "C_k_to_j"_a)
	CDOC(PlatformToSensorCartesianVirtualStateBlock);

	CLASS(EcefToStandard, VirtualStateBlock)
	CTOR(EcefToStandard, PARAMS(const std::string &, const std::string &), "current"_a, "target"_a)
	CDOC(EcefToStandard);

	CLASS(EcefToStandardQuat, VirtualStateBlock)
	CTOR(EcefToStandardQuat,
	     PARAMS(const std::string &, const std::string &),
	     "current"_a,
	     "target"_a)
	CDOC(EcefToStandardQuat);

	CLASS(PinsonErrorToStandard, VirtualStateBlock)
	CTOR(PinsonErrorToStandard,
	     PARAMS(const std::string &,
	            const std::string &,
	            std::function<NavSolution(const aspn_xtensor::TypeTimestamp &)>),
	     "current"_a,
	     "target"_a,
	     "ref_fun"_a)
	CDOC(PinsonErrorToStandard);

	CLASS(PinsonErrorToStandardQuat, VirtualStateBlock)
	CTOR(PinsonErrorToStandardQuat,
	     PARAMS(const std::string &,
	            const std::string &,
	            std::function<NavSolution(const aspn_xtensor::TypeTimestamp &)>),
	     "current"_a,
	     "target"_a,
	     "ref_fun"_a)
	CDOC(PinsonErrorToStandardQuat);

	CLASS(PinsonToSensor, NumericalVirtualStateBlock)
	CTOR(PinsonToSensor,
	     PARAMS(const std::string &,
	            const std::string &,
	            std::function<NavSolution(const aspn_xtensor::TypeTimestamp &)>,
	            const TypeMounting &,
	            const TypeMounting &),
	     "current"_a,
	     "target"_a,
	     "ref_fun"_a,
	     "inertial_mount"_a,
	     "sensor_mount"_a)
	CDOC(PinsonToSensor);

	CLASS(PinsonToSensorLlh, NumericalVirtualStateBlock)
	CTOR(PinsonToSensorLlh,
	     PARAMS(const std::string &,
	            const std::string &,
	            std::function<NavSolution(const aspn_xtensor::TypeTimestamp &)>,
	            const TypeMounting &,
	            const TypeMounting &),
	     "current"_a,
	     "target"_a,
	     "ref_fun"_a,
	     "inertial_mount"_a,
	     "sensor_mount"_a)
	CDOC(PinsonToSensorLlh);

	CLASS(PlatformToSensorEcef, VirtualStateBlock)
	CTOR(PlatformToSensorEcef,
	     PARAMS(const std::string &, const std::string &, const TypeMounting &, double),
	     "current"_a,
	     "target"_a,
	     "sensor_mount"_a,
	     "scale_factor"_a = 1.0)
	CDOC(PlatformToSensorEcef);

	CLASS(PlatformToSensorEcefQuat, VirtualStateBlock)
	CTOR(PlatformToSensorEcefQuat,
	     PARAMS(const std::string &, const std::string &, const TypeMounting &, double),
	     "current"_a,
	     "target"_a,
	     "sensor_mount"_a,
	     "scale_factor"_a = 1.0)
	CDOC(PlatformToSensorEcefQuat);

	CLASS(QuatToRpyPva, VirtualStateBlock)
	CTOR(QuatToRpyPva, PARAMS(const std::string &, const std::string &), "current"_a, "target"_a)
	CDOC(QuatToRpyPva);

	CLASS(SensorToPlatformEcef, VirtualStateBlock)
	CTOR(SensorToPlatformEcef,
	     PARAMS(const std::string &, const std::string &, const TypeMounting &, double),
	     "current"_a,
	     "target"_a,
	     "sensor_mount"_a,
	     "scale_factor"_a = 1.0)
	CDOC(SensorToPlatformEcef);

	CLASS(SensorToPlatformEcefQuat, VirtualStateBlock)
	CTOR(SensorToPlatformEcefQuat,
	     PARAMS(const std::string &, const std::string &, const TypeMounting &, double),
	     "current"_a,
	     "target"_a,
	     "sensor_mount"_a,
	     "scale_factor"_a = 1.0)
	CDOC(SensorToPlatformEcefQuat);

	CLASS(StandardToEcef, VirtualStateBlock)
	CTOR(StandardToEcef, PARAMS(const std::string &, const std::string &), "current"_a, "target"_a)
	CDOC(StandardToEcef);

	CLASS(StandardToEcefQuat, VirtualStateBlock)
	CTOR(StandardToEcefQuat,
	     PARAMS(const std::string &, const std::string &),
	     "current"_a,
	     "target"_a)
	CDOC(StandardToEcefQuat);

	CLASS(StateExtractor, VirtualStateBlock)
	CTOR(StateExtractor,
	     PARAMS(std::string, std::string, Size, const std::vector<Size> &),
	     "current"_a,
	     "target"_a,
	     "incoming_state_size"_a,
	     "indices"_a)
	CDOC(StateExtractor);

	// Not exactly right; template arg can't be passed so behavior will need to
	// be differentiated by function names. However, 15 and 21 Pinson blocks are interpreted the
	// same so this is good enough for now
	FUNCTIONT(apply_error_states, PARAMS(Pinson15NedBlock, NavSolution), "pva"_a, "x"_a);
	FUNCTIONT(apply_error_states,
	          PARAMS(Pinson15NedBlock, aspn_xtensor::MeasurementPositionVelocityAttitude),
	          "pva"_a,
	          "x"_a);
	FUNCTIONT(
	    apply_error_states,
	    PARAMS(Pinson15NedBlock, not_null<std::shared_ptr<navtk::inertial::InertialPosVelAtt>>),
	    "pva"_a,
	    "x"_a);

	FUNCTION_CAST(
	    calc_numerical_jacobian,
	    Matrix(*)(const std::function<Vector(const Vector &)> &, const Vector &, const Vector &),
	    ,
	    "f"_a,
	    "x"_a,
	    "eps"_a)
	FUNCTION_CAST(calc_numerical_jacobian,
	              Matrix(*)(const std::function<Vector(const Vector &)> &, const Vector &, Scalar),
	              _2,
	              "f"_a,
	              "x"_a,
	              "eps"_a = 0.001)
	FUNCTION_CAST(
	    calc_numerical_hessians,
	    std::vector<Matrix>(*)(
	        const std::function<Vector(const Vector &)> &, const Vector &, const Vector &),
	    ,
	    "f"_a,
	    "x"_a,
	    "eps"_a)
	FUNCTION_CAST(calc_numerical_hessians,
	              std::vector<Matrix>(*)(
	                  const std::function<Vector(const Vector &)> &, const Vector &, Scalar),
	              _2,
	              "f"_a,
	              "x"_a,
	              "eps"_a = 0.001)
	NAMESPACE_FUNCTION(calc_mean_cov, navtk::filtering, "samples"_a)
	NAMESPACE_FUNCTION(monte_carlo_approx, navtk::filtering, "ec"_a, "fx"_a, "num_samples"_a = 100)
	NAMESPACE_FUNCTION(
	    monte_carlo_approx_rpy, navtk::filtering, "ec"_a, "fx"_a, "num_samples"_a = 100)
	NAMESPACE_FUNCTION(
	    second_order_approx, navtk::filtering, "ec"_a, "fx"_a, "jx"_a = 0, "hx"_a = 0)
	NAMESPACE_FUNCTION(first_order_approx, navtk::filtering, "ec"_a, "fx"_a, "jx"_a = 0)
	NAMESPACE_FUNCTION(first_order_approx_rpy, navtk::filtering, "ec"_a, "fx"_a)

	ADD_SUBNAMESPACE(m, experimental, filtering);
}
