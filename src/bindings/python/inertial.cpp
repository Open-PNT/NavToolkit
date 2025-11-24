#include <memory>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/inertial/AidingAltData.hpp>
#include <navtk/inertial/AlignBase.hpp>
#include <navtk/inertial/BasicInsAndFilter.hpp>
#include <navtk/inertial/BufferedImu.hpp>
#include <navtk/inertial/BufferedIns.hpp>
#include <navtk/inertial/BufferedPva.hpp>
#include <navtk/inertial/CoarseDynamicAlignment.hpp>
#include <navtk/inertial/DynData.hpp>
#include <navtk/inertial/ImuErrors.hpp>
#include <navtk/inertial/Inertial.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/ManualAlignment.hpp>
#include <navtk/inertial/ManualHeadingAlignment.hpp>
#include <navtk/inertial/Mechanization.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/MechanizationStandard.hpp>
#include <navtk/inertial/MovementDetector.hpp>
#include <navtk/inertial/MovementDetectorImu.hpp>
#include <navtk/inertial/MovementDetectorPlugin.hpp>
#include <navtk/inertial/MovementDetectorPos.hpp>
#include <navtk/inertial/MovementStatus.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/inertial/StaticAlignment.hpp>
#include <navtk/inertial/StaticWahbaAlignment.hpp>
#include <navtk/inertial/WanderPosVelAtt.hpp>
#include <navtk/inertial/inertial_functions.hpp>
#include <navtk/inertial/mechanization_standard.hpp>
#include <navtk/inertial/mechanization_wander.hpp>
#include <navtk/inertial/quaternion_static_alignment.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/not_null.hpp>
#include <navtk/utils/compiler.hpp>


#include "binding_helpers.hpp"

using namespace pybind11::literals;

using aspn_xtensor::MeasurementImu;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::TypeTimestamp;
using navtk::Matrix3;
using navtk::not_null;
using navtk::Vector3;
using navtk::filtering::ImuModel;
using navtk::filtering::NavSolution;
using navtk::inertial::AidingAltData;
using navtk::inertial::AlignBase;
using navtk::inertial::BasicInsAndFilter;
using navtk::inertial::BufferedImu;
using navtk::inertial::BufferedIns;
using navtk::inertial::BufferedPva;
using navtk::inertial::calc_force_ned;
using navtk::inertial::calc_rot_rate;
using navtk::inertial::CoarseDynamicAlignment;
using navtk::inertial::DcmIntegrationMethods;
using navtk::inertial::DynData;
using navtk::inertial::EarthModels;
using navtk::inertial::ImuErrors;
using navtk::inertial::Inertial;
using navtk::inertial::InertialPosVelAtt;
using navtk::inertial::IntegrationMethods;
using navtk::inertial::ManualAlignment;
using navtk::inertial::ManualHeadingAlignment;
using navtk::inertial::Mechanization;
using navtk::inertial::mechanization_standard;
using navtk::inertial::mechanization_wander;
using navtk::inertial::MechanizationFunction;
using navtk::inertial::MechanizationOptions;
using navtk::inertial::MechanizationStandard;
using navtk::inertial::MovementDetector;
using navtk::inertial::MovementDetectorImu;
using navtk::inertial::MovementDetectorPlugin;
using navtk::inertial::MovementDetectorPluginStat;
using navtk::inertial::MovementDetectorPos;
using navtk::inertial::MovementStatus;
using navtk::inertial::quaternion_static_alignment;
using navtk::inertial::StandardPosVelAtt;
using navtk::inertial::StaticAlignment;
using navtk::inertial::StaticWahbaAlignment;
using navtk::inertial::WanderPosVelAtt;
using navtk::navutils::GravModels;

using mat_double_pair = std::pair<Matrix3, double>;
using prnav           = std::pair<bool, NavSolution>;
using prmat           = std::pair<bool, navtk::Matrix>;
using primuerr        = std::pair<bool, ImuErrors>;

void add_inertial_functions(pybind11::module &m) {

	m.doc() = "Inertial navigation library.";

	ENUM(IntegrationMethods)
	CHOICE(IntegrationMethods, RECTANGULAR)
	CHOICE(IntegrationMethods, TRAPEZOIDAL)
	CHOICE(IntegrationMethods, SIMPSONS_RULE).finalize();

	ENUM(DcmIntegrationMethods)
	CHOICE(DcmIntegrationMethods, FIRST_ORDER)
	CHOICE(DcmIntegrationMethods, SIXTH_ORDER)
	CHOICE(DcmIntegrationMethods, EXPONENTIAL).finalize();

	ENUM(EarthModels)
	CHOICE(EarthModels, ELLIPTICAL)
	CHOICE(EarthModels, SPHERICAL).finalize();

	NAMESPACE_FUNCTION(quaternion_static_alignment, navtk::inertial, "dv_avg"_a, "dth_avg"_a)

	class PyAlignBase : public AlignBase, public py::trampoline_self_life_support {
	public:
		AlignmentStatus process(std::shared_ptr<aspn_xtensor::AspnBase> message) override {
			PYBIND11_OVERRIDE(AlignmentStatus, AlignBase, process, message);
		}
		prnav get_computed_alignment() const override {
			PYBIND11_OVERRIDE(prnav, AlignBase, get_computed_alignment, );
		}
		prmat get_computed_covariance(
		    const AlignBase::CovarianceFormat format =
		        AlignBase::CovarianceFormat::PINSON15NEDBLOCK) const override {
			PYBIND11_OVERRIDE(prmat, AlignBase, get_computed_covariance, format);
		}
		primuerr get_imu_errors() const override {
			PYBIND11_OVERRIDE(primuerr, AlignBase, get_imu_errors, );
		}
	};

	// clang-format off
	auto align_base = CLASS(AlignBase, PyAlignBase);
	ENUM_SCOPED(CovarianceFormat, AlignBase, align_base)
	CHOICE_SCOPED(CovarianceFormat, AlignBase, PINSON15NEDBLOCK)
	CHOICE_SCOPED(CovarianceFormat, AlignBase, PINSON21NEDBLOCK)
	.finalize();

	ENUM_SCOPED(AlignmentStatus, AlignBase, align_base)
	CHOICE_SCOPED(AlignmentStatus, AlignBase, ALIGNING_COARSE)
	CHOICE_SCOPED(AlignmentStatus, AlignBase, ALIGNING_FINE)
	CHOICE_SCOPED(AlignmentStatus, AlignBase, ALIGNED_GOOD)
	.finalize();

	align_base
		METHOD(AlignBase, process, "message"_a)
	    METHOD_VOID(AlignBase, requires_dynamic)
		METHOD_VOID(AlignBase, check_alignment_status)
		METHOD_VOID(AlignBase, get_computed_alignment)
		METHOD(AlignBase, get_computed_covariance, "format"_a=AlignBase::CovarianceFormat::PINSON15NEDBLOCK)
		METHOD_VOID(AlignBase, get_imu_errors)
		METHOD_VOID(AlignBase, motion_needed)
		CDOC(AlignBase);
	// clang-format on

	CLASS(StaticAlignment, AlignBase)
	CTOR(StaticAlignment,
	     PARAMS(const ImuModel &, const double, const Matrix3 &),
	     "model"_a      = navtk::filtering::stim300_model(),
	     "align_time"_a = 120.0,
	     "vel_cov"_a    = Matrix3{{1e-4, 0, 0}, {0, 1e-4, 0}, {0, 0, 1e-4}})
	CDOC(StaticAlignment);

	CLASS(StaticWahbaAlignment, StaticAlignment)
	CTOR(StaticWahbaAlignment,
	     PARAMS(const ImuModel &, const double, const Matrix3 &),
	     "model"_a      = navtk::filtering::stim300_model(),
	     "align_time"_a = 120.0,
	     "vel_cov"_a    = Matrix3{{1e-4, 0, 0}, {0, 1e-4, 0}, {0, 0, 1e-4}})
	CDOC(StaticWahbaAlignment);

	CLASS(ManualAlignment, AlignBase)
	CTOR(ManualAlignment,
	     PARAMS(const aspn_xtensor::MeasurementPositionVelocityAttitude &,
	            bool,
	            bool,
	            bool,
	            bool,
	            const ImuModel &),
	     "pva"_a,
	     "wait_for_time"_a = false,
	     "wait_for_pos"_a  = false,
	     "wait_for_vel"_a  = false,
	     "wait_for_att"_a  = false,
	     "model"_a         = navtk::filtering::stim300_model())
	CDOC(ManualAlignment);

	CLASS(ManualHeadingAlignment, StaticAlignment)
	CTOR(ManualHeadingAlignment,
	     PARAMS(const double, const double, const ImuModel &, const double, const Matrix3 &),
	     "heading"_a,
	     "heading_sigma"_a = 0.017453292519943295,
	     "model"_a         = navtk::filtering::stim300_model(),
	     "align_time"_a    = 120.0,
	     "vel_cov"_a       = Matrix3{{1e-4, 0, 0}, {0, 1e-4, 0}, {0, 0, 1e-4}})
	CDOC(ManualHeadingAlignment);

	CLASS(CoarseDynamicAlignment, AlignBase)
	CTOR(CoarseDynamicAlignment,
	     PARAMS(const ImuModel &, const double, const double, DcmIntegrationMethods),
	     "model"_a                  = navtk::filtering::stim300_model(),
	     "static_time"_a            = 30.0,
	     "reset_time"_a             = 300.0,
	     "dcm_integration_method"_a = DcmIntegrationMethods::FIRST_ORDER)
	CDOC(CoarseDynamicAlignment);

	// clang-format off
	auto dyn_data = CLASS(DynData);

	ENUM_SCOPED(RecentPositionsEnum, DynData, dyn_data)
	CHOICE_SCOPED(RecentPositionsEnum, DynData, MOST_RECENT)
	CHOICE_SCOPED(RecentPositionsEnum, DynData, SECOND_MOST_RECENT)
	CHOICE_SCOPED(RecentPositionsEnum, DynData, THIRD_MOST_RECENT)
	.finalize();

	BEGIN_SUPPRESS_WARNING("-Wdeprecated-declarations");
	dyn_data
	CTOR(DynData, const aspn_xtensor::MeasurementPosition &, "origin"_a)
	METHOD_VOID(DynData, enough_data)
	METHOD(DynData, update, "new_pos"_a, "align_buffer"_a)
	METHOD_VOID(DynData, get_force_from_imu)
	METHOD_VOID(DynData, get_force_from_pos)
	METHOD_VOID(DynData, get_vel_mid)
	METHOD(DynData, get_position, "recency"_a)
	METHOD_VOID(DynData, get_positions)
	METHOD_VOID(DynData, get_origin)
	METHOD_VOID(DynData, get_lat_lon_factors)
	CDOC(DynData);

	END_SUPPRESS_WARNING;
	// clang-format on


	CLASS(MechanizationOptions)
	CTOR_NODOC(PARAMS(GravModels, EarthModels, DcmIntegrationMethods, IntegrationMethods),
	           "grav_model"_a  = GravModels::SCHWARTZ,
	           "earth_model"_a = EarthModels::ELLIPTICAL,
	           "dcm_method"_a  = DcmIntegrationMethods::SIXTH_ORDER,
	           "int_method"_a  = IntegrationMethods::TRAPEZOIDAL)
	FIELD(MechanizationOptions, grav_model)
	FIELD(MechanizationOptions, earth_model)
	FIELD(MechanizationOptions, dcm_method)
	FIELD(MechanizationOptions, int_method)
	CDOC(MechanizationOptions);

	CLASS(AidingAltData)
	CTOR_NODOC(PARAMS(double, double, double),
	           "aiding_alt"_a           = 0.0,
	           "integrated_alt_error"_a = 0.0,
	           "time_constant"_a        = 0.01)
	FIELD(AidingAltData, aiding_alt)
	FIELD(AidingAltData, integrated_alt_error)
	FIELD(AidingAltData, time_constant) CDOC(AidingAltData);

	class PyPosVelAtt : public InertialPosVelAtt, public py::trampoline_self_life_support {
	public:
		bool is_wander_capable() const override {
			PYBIND11_OVERRIDE_PURE(bool, InertialPosVelAtt, is_wander_capable, );
		}
		Vector3 get_llh() const override {
			PYBIND11_OVERRIDE_PURE(Vector3, InertialPosVelAtt, get_llh, );
		}
		Vector3 get_vned() const override {
			PYBIND11_OVERRIDE_PURE(Vector3, InertialPosVelAtt, get_vned, );
		}
		Matrix3 get_C_s_to_ned() const override {
			PYBIND11_OVERRIDE_PURE(Matrix3, InertialPosVelAtt, get_C_s_to_ned, );
		}
		mat_double_pair get_C_n_to_e_h() const override {
			PYBIND11_OVERRIDE_PURE(mat_double_pair, InertialPosVelAtt, get_C_n_to_e_h, );
		}
		Vector3 get_vn() const override {
			PYBIND11_OVERRIDE_PURE(Vector3, InertialPosVelAtt, get_vn, );
		}
		Matrix3 get_C_s_to_l() const override {
			PYBIND11_OVERRIDE_PURE(Matrix3, InertialPosVelAtt, get_C_s_to_l, );
		}
		std::shared_ptr<InertialPosVelAtt> clone() const override {
			PYBIND11_OVERRIDE_PURE(std::shared_ptr<InertialPosVelAtt>, InertialPosVelAtt, clone, );
		}
		PyPosVelAtt(const aspn_xtensor::TypeTimestamp &time,
		            AspnMessageType message_type = ASPN_EXTENDED_BEGIN)
		    : InertialPosVelAtt::InertialPosVelAtt(time, message_type) {}
	};

	CLASS(InertialPosVelAtt, PyPosVelAtt, aspn_xtensor::AspnBase)
	CTOR(InertialPosVelAtt,
	     const aspn_xtensor::TypeTimestamp &,
	     "t"_a = aspn_xtensor::TypeTimestamp((int64_t)0))
	METHOD_VOID(InertialPosVelAtt, is_wander_capable)
	METHOD_VOID(InertialPosVelAtt, get_llh)
	METHOD_VOID(InertialPosVelAtt, get_vned)
	METHOD_VOID(InertialPosVelAtt, get_C_s_to_ned)
	METHOD_VOID(InertialPosVelAtt, get_C_n_to_e_h)
	METHOD_VOID(InertialPosVelAtt, get_vn)
	METHOD_VOID(InertialPosVelAtt, get_C_s_to_l)
	FIELD(InertialPosVelAtt, time_validity)
	METHOD_VOID(InertialPosVelAtt, clone)
	CDOC(InertialPosVelAtt);

	CLASS(WanderPosVelAtt, InertialPosVelAtt)
	CTOR(WanderPosVelAtt,
	     PARAMS(const aspn_xtensor::TypeTimestamp &,
	            Matrix3,
	            double,
	            Vector3,
	            Matrix3,
	            AspnMessageType),
	     "time"_a         = aspn_xtensor::TypeTimestamp((int64_t)0),
	     "C_n_to_e"_a     = navtk::eye(3),
	     "alt"_a          = 0.0,
	     "v_n"_a          = navtk::zeros(3),
	     "C_s_to_l"_a     = navtk::eye(3),
	     "message_type"_a = ASPN_EXTENDED_BEGIN)
	CTOR_OVERLOAD(WanderPosVelAtt,
	              PARAMS(const aspn_xtensor::TypeTimestamp &,
	                     const std::tuple<Matrix3, double, Vector3, Matrix3> &,
	                     AspnMessageType),
	              _2,
	              "time"_a,
	              "tup"_a,
	              "message_type"_a = ASPN_EXTENDED_BEGIN)
	CDOC(WanderPosVelAtt);

	CLASS(StandardPosVelAtt, InertialPosVelAtt)
	CTOR(StandardPosVelAtt,
	     PARAMS(const aspn_xtensor::TypeTimestamp &, Vector3, Vector3, Matrix3, AspnMessageType),
	     "time"_a         = aspn_xtensor::TypeTimestamp((int64_t)0),
	     "lla"_a          = navtk::zeros(3),
	     "v_ned"_a        = navtk::zeros(3),
	     "C_s_to_ned"_a   = navtk::eye(3),
	     "message_type"_a = ASPN_EXTENDED_BEGIN)
	CTOR_OVERLOAD(StandardPosVelAtt,
	              PARAMS(const aspn_xtensor::TypeTimestamp &,
	                     const std::tuple<Vector3, Vector3, Matrix3> &,
	                     AspnMessageType),
	              _2,
	              "time"_a,
	              "tup"_a,
	              "message_type"_a = ASPN_EXTENDED_BEGIN)
	METHOD_OVERLOAD_CONST(StandardPosVelAtt, get_C_n_to_e_h, double, _2, "wander"_a)
	METHOD_OVERLOAD_CONST(StandardPosVelAtt, get_vn, double, _2, "wander"_a)
	METHOD_OVERLOAD_CONST(StandardPosVelAtt, get_C_s_to_l, double, _2, "wander"_a)
	// The functions get_C_n_to_e_h, get_vn, and get_C_s_to_l already exist on the base class
	// bindings, and normally wouldn't need to be overridden in this class; however, because this
	// class declares overloads for these functions, python won't know about the overridden
	// functions unless we explicitly bind them here.
	METHOD_OVERLOAD_CONST_VOID(StandardPosVelAtt, get_C_n_to_e_h, )
	METHOD_OVERLOAD_CONST_VOID(StandardPosVelAtt, get_vn, )
	METHOD_OVERLOAD_CONST_VOID(StandardPosVelAtt, get_C_s_to_l, )
	CDOC(StandardPosVelAtt);

	class PyMechanization : public Mechanization, public py::trampoline_self_life_support {
	public:
		not_null<std::shared_ptr<InertialPosVelAtt>> mechanize(
		    const Vector3 &dv_s,
		    const Vector3 &dth_s,
		    const double dt,
		    const not_null<std::shared_ptr<InertialPosVelAtt>> pva,
		    const not_null<std::shared_ptr<InertialPosVelAtt>> pva_old,
		    const MechanizationOptions &mech_options,
		    AidingAltData *aiding) override {
			PYBIND11_OVERRIDE_PURE(std::shared_ptr<InertialPosVelAtt>,
			                       Mechanization,
			                       mechanize,
			                       dv_s,
			                       dth_s,
			                       dt,
			                       pva,
			                       pva_old,
			                       mech_options,
			                       aiding);
		}
	};

	CLASS(Mechanization, PyMechanization)
	CTOR_DEFAULT_NODOC(Mechanization)
	METHOD(Mechanization,
	       mechanize,
	       "dv_s"_a,
	       "dth_s"_a,
	       "dt"_a,
	       "pva"_a,
	       "pva_old"_a,
	       "mech_options"_a,
	       "aiding"_a)
	CDOC(Mechanization);

	CLASS(MechanizationStandard, Mechanization)
	CTOR_CLSDOC_DEFAULT(MechanizationStandard)
	CDOC(MechanizationStandard);

	MechanizationFunction mech_default =
	    static_cast<not_null<std::shared_ptr<InertialPosVelAtt>> (*)(
	        const Vector3 &,
	        const Vector3 &,
	        const double,
	        const not_null<std::shared_ptr<InertialPosVelAtt>>,
	        const not_null<std::shared_ptr<InertialPosVelAtt>>,
	        const MechanizationOptions &,
	        AidingAltData *)>(mechanization_standard);

	CLASS(Inertial)
	CTOR(Inertial,
	     PARAMS(const not_null<std::shared_ptr<InertialPosVelAtt>>,
	            const MechanizationOptions &,
	            MechanizationFunction),
	     "start_pva"_a    = StandardPosVelAtt(),
	     "mech_options"_a = MechanizationOptions{},
	     "mech_fun"_a     = mech_default)
	CTOR_OVERLOAD(Inertial,
	              PARAMS(not_null<std::shared_ptr<Mechanization>>,
	                     const not_null<std::shared_ptr<InertialPosVelAtt>>,
	                     const MechanizationOptions &),
	              _2,
	              "mech_class"_a,
	              "start_pva"_a    = StandardPosVelAtt(),
	              "mech_options"_a = MechanizationOptions{})
	CTOR_OVERLOAD(Inertial, PARAMS(const Inertial &), _3, "ins"_a)
	METHOD_OVERLOAD(Inertial,
	                reset,
	                PARAMS(const not_null<std::shared_ptr<InertialPosVelAtt>>,
	                       const std::shared_ptr<InertialPosVelAtt>),
	                ,
	                "new_pva"_a,
	                "old_pva"_a)
	METHOD_OVERLOAD(Inertial, reset, StandardPosVelAtt, _2, "new_pva"_a)
	METHOD_OVERLOAD(Inertial, reset, WanderPosVelAtt, _3, "new_pva"_a)
	METHOD_VOID(Inertial, get_solution)
	METHOD_VOID(Inertial, get_gyro_biases)
	METHOD(Inertial, set_gyro_biases, "gyro_biases"_a)
	METHOD_VOID(Inertial, get_gyro_scale_factors)
	METHOD(Inertial, set_gyro_scale_factors, "gyro_scale_factors"_a)
	METHOD_VOID(Inertial, get_accel_biases)
	METHOD(Inertial, set_accel_biases, "accel_biases"_a)
	METHOD_VOID(Inertial, get_accel_scale_factors)
	METHOD(Inertial, set_accel_scale_factors, "accel_scale_factors"_a)
	METHOD(Inertial, set_imu_errors, "errors"_a)
	METHOD(
	    Inertial, mechanize, "time"_a, "accel_meas"_a, "gyro_meas"_a, "aiding_alt_data"_a = nullptr)
	CDOC(Inertial);

	m.def("mechanization_wander",
	      py::overload_cast<const Vector3 &,
	                        const Vector3 &,
	                        double,
	                        const Matrix3 &,
	                        double,
	                        const Vector3 &,
	                        const Matrix3 &,
	                        const MechanizationOptions &,
	                        AidingAltData *>(&mechanization_wander),
	      PROCESS_DOC(mechanization_wander),
	      "dv_s"_a,
	      "dth_s"_a,
	      "dt"_a,
	      "C_s_to_e_0"_a,
	      "h0"_a,
	      "v_n_0"_a,
	      "C_s_to_l_0"_a,
	      "mech_options"_a    = MechanizationOptions{},
	      "aiding_alt_data"_a = nullptr);
	m.def("mechanization_wander",
	      py::overload_cast<const Vector3 &,
	                        const Vector3 &,
	                        double,
	                        const not_null<std::shared_ptr<InertialPosVelAtt>>,
	                        const not_null<std::shared_ptr<InertialPosVelAtt>>,
	                        const MechanizationOptions &,
	                        AidingAltData *>(&mechanization_wander),
	      PROCESS_DOC(mechanization_wander_2),
	      "dv_s"_a,
	      "dth_s"_a,
	      "dt"_a,
	      "pva"_a,
	      "old_pva"_a,
	      "mech_options"_a    = MechanizationOptions{},
	      "aiding_alt_data"_a = nullptr);

	m.def("mechanization_standard",
	      py::overload_cast<const Vector3 &,
	                        const Vector3 &,
	                        double,
	                        const Vector3 &,
	                        const Matrix3 &,
	                        const Vector3 &,
	                        const Vector3 &,
	                        const MechanizationOptions &,
	                        AidingAltData *>(&mechanization_standard),

	      "dv_s"_a,
	      "dth_s"_a,
	      "dt"_a,
	      "llh0"_a,
	      "C_s_to_n0"_a,
	      "v_ned0"_a,
	      "v_ned_prev"_a,
	      "mech_options"_a    = MechanizationOptions{},
	      "aiding_alt_data"_a = nullptr);
	m.def("mechanization_standard",
	      py::overload_cast<const Vector3 &,
	                        const Vector3 &,
	                        double,
	                        const not_null<std::shared_ptr<InertialPosVelAtt>>,
	                        const not_null<std::shared_ptr<InertialPosVelAtt>>,
	                        const MechanizationOptions &,
	                        AidingAltData *>(&mechanization_standard),
	      PROCESS_DOC(mechanization_standard_2),
	      "dv_s"_a,
	      "dth_s"_a,
	      "dt"_a,
	      "pva"_a,
	      "old_pva"_a,
	      "mech_options"_a    = MechanizationOptions{},
	      "aiding_alt_data"_a = nullptr);

	CLASS(ImuErrors, aspn_xtensor::AspnBase)
	CTOR(ImuErrors,
	     PARAMS(const Vector3 &,
	            const Vector3 &,
	            const Vector3 &,
	            const Vector3 &,
	            const aspn_xtensor::TypeTimestamp &,
	            AspnMessageType),
	     "accel_bias"_a          = navtk::zeros(3),
	     "gyro_biases"_a         = navtk::zeros(3),
	     "accel_scale_factors"_a = navtk::zeros(3),
	     "gyro_scale_factors"_a  = navtk::zeros(3),
	     "time"_a                = aspn_xtensor::TypeTimestamp((int64_t)0),
	     "message_type"_a        = ASPN_EXTENDED_BEGIN)
	FIELD(ImuErrors, accel_biases)
	FIELD(ImuErrors, gyro_biases)
	FIELD(ImuErrors, accel_scale_factors)
	FIELD(ImuErrors, gyro_scale_factors)
	FIELD(ImuErrors, time_validity)
	CDOC(ImuErrors);

	CLASS(BufferedPva)
	METHOD_OVERLOAD_CONST_VOID(BufferedPva, calc_pva, )
	METHOD(BufferedPva, add_data, "data"_a)
	METHOD_OVERLOAD_CONST(BufferedPva, calc_pva, const aspn_xtensor::TypeTimestamp &, _2, "time"_a)
	METHOD_VOID(BufferedPva, time_span)
	METHOD(BufferedPva, in_range, "t"_a)
	METHOD_OVERLOAD_CONST(
	    BufferedPva, calc_force_and_rate, const aspn_xtensor::TypeTimestamp &, , "time"_a)
	METHOD_OVERLOAD_CONST(
	    BufferedPva,
	    calc_force_and_rate,
	    PARAMS(const aspn_xtensor::TypeTimestamp &, const aspn_xtensor::TypeTimestamp &),
	    _2,
	    "time1"_a,
	    "time2"_a)
	CDOC(BufferedPva);

	CLASS(BufferedImu, BufferedPva)
	CTOR(BufferedImu, const BufferedImu &, "other"_a)
	CTOR(BufferedImu,
	     PARAMS(const aspn_xtensor::MeasurementPositionVelocityAttitude &,
	            std::shared_ptr<MeasurementImu>,
	            double,
	            const ImuErrors &,
	            const MechanizationOptions &,
	            double),
	     "pva"_a,
	     "initial_imu"_a   = nullptr,
	     "expected_dt"_a   = 0.01,
	     "imu_errs"_a      = ImuErrors{},
	     "mech_options"_a  = MechanizationOptions{},
	     "buffer_length"_a = 60.0)
	METHOD(BufferedImu, reset, "pva"_a = nullptr, "imu_errs"_a = nullptr, "previous"_a = nullptr)
	METHOD(BufferedImu, calc_pva_no_reset_since, "time"_a, "since"_a)
	METHOD_OVERLOAD(BufferedImu, mechanize, PARAMS(std::shared_ptr<MeasurementImu>), , "imu"_a)
	METHOD_OVERLOAD(BufferedImu,
	                mechanize,
	                PARAMS(const aspn_xtensor::TypeTimestamp &, const Vector3 &, const Vector3 &),
	                _2,
	                "time"_a,
	                "delta_v"_a,
	                "delta_theta"_a)
	METHOD(BufferedImu, get_imu_errors, "t"_a)
	CDOC(BufferedImu);

	CLASS(BufferedIns, BufferedPva)
	CTOR(BufferedIns,
	     PARAMS(std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>, double, double),
	     "pva"_a           = nullptr,
	     "expected_dt"_a   = 1.0,
	     "buffer_length"_a = 60.0)
	METHOD_OVERLOAD(
	    BufferedIns,
	    add_pva,
	    navtk::not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>>,
	    ,
	    "pva"_a)
	METHOD_OVERLOAD(
	    BufferedIns, add_pva, const aspn_xtensor::MeasurementPositionVelocityAttitude &, , "pva"_a)
	CDOC(BufferedIns);

	FUNCTION_CAST(calc_force_ned,
	              Vector3(*)(const Matrix3 &, double, const Vector3 &, const Vector3 &),
	              ,
	              "C_s_to_n"_a,
	              "dt"_a,
	              "dth"_a,
	              "dv"_a);

	FUNCTION_CAST(calc_force_ned,
	              Vector3(*)(const MeasurementPositionVelocityAttitude &,
	                         const MeasurementPositionVelocityAttitude &),
	              _2,
	              "pva1"_a,
	              "pva2"_a);

	FUNCTION_CAST(calc_rot_rate,
	              Vector3(*)(const Matrix3 &,
	                         double,
	                         double,
	                         double,
	                         double,
	                         double,
	                         const Vector3 &,
	                         double,
	                         double,
	                         const Vector3 &,
	                         double),
	              ,
	              "C_s_to_n0"_a,
	              "r_e"_a,
	              "r_n"_a,
	              "alt0"_a,
	              "cos_l"_a,
	              "dt"_a,
	              "dth"_a,
	              "sin_l"_a,
	              "tan_l"_a,
	              "v_ned0"_a,
	              "omega"_a = navtk::navutils::ROTATION_RATE);

	FUNCTION_CAST(
	    calc_rot_rate,
	    Vector3(*)(
	        const aspn_xtensor::MeasurementPositionVelocityAttitude &, double, const Vector3 &),
	    _2,
	    "pva"_a,
	    "dt"_a,
	    "dth"_a);

	FUNCTION_CAST(calc_rot_rate,
	              Vector3(*)(const MeasurementPositionVelocityAttitude &,
	                         const MeasurementPositionVelocityAttitude &),
	              _3,
	              "pva1"_a,
	              "pva2"_a);

	NAMESPACE_FUNCTION(apply_aiding_alt_accel,
	                   navtk::inertial,
	                   "r_zero"_a,
	                   "accel_vector"_a,
	                   "aiding_alt_data"_a,
	                   "alt0"_a,
	                   "dt"_a,
	                   "g"_a)

	CLASS(BasicInsAndFilter)
	CTOR(BasicInsAndFilter, const BasicInsAndFilter &, "other"_a)
	CTOR(BasicInsAndFilter,
	     PARAMS(const aspn_xtensor::MeasurementPositionVelocityAttitude &,
	            const navtk::filtering::ImuModel &,
	            std::shared_ptr<MeasurementImu>,
	            const ImuErrors &,
	            double,
	            const MechanizationOptions &),
	     "pva"_a,
	     "model"_a        = navtk::filtering::stim300_model(),
	     "initial_imu"_a  = nullptr,
	     "imu_errs"_a     = ImuErrors{},
	     "expected_dt"_a  = 0.01,
	     "mech_options"_a = MechanizationOptions{})
	METHOD(BasicInsAndFilter, mechanize, "imu"_a)
	METHOD(BasicInsAndFilter, update, "gp3d"_a)
	METHOD_OVERLOAD_CONST(BasicInsAndFilter, calc_pva, const aspn_xtensor::TypeTimestamp &, , "t"_a)
	METHOD_OVERLOAD_CONST_VOID(BasicInsAndFilter, calc_pva, _2)
	METHOD_VOID(BasicInsAndFilter, calc_imu_errors)
	METHOD_VOID(BasicInsAndFilter, get_pinson15_cov);

	ENUM(MovementStatus)
	CHOICE(MovementStatus, INVALID)
	CHOICE(MovementStatus, NOT_MOVING)
	CHOICE(MovementStatus, POSSIBLY_MOVING)
	CHOICE(MovementStatus, MOVING).finalize();

	class PyMovementDetectorPlugin : public MovementDetectorPlugin,
	                                 public py::trampoline_self_life_support {
	public:
		MovementStatus process(not_null<std::shared_ptr<aspn_xtensor::AspnBase>>) override {
			PYBIND11_OVERRIDE_PURE(MovementStatus, MovementDetectorPlugin, process, );
		}

		MovementStatus get_status() override {
			PYBIND11_OVERRIDE(MovementStatus, MovementDetectorPlugin, get_status, );
		}

		aspn_xtensor::TypeTimestamp get_time() override {
			PYBIND11_OVERRIDE_PURE(aspn_xtensor::TypeTimestamp, MovementDetectorPlugin, get_time, );
		}

		using MovementDetectorPlugin::last_status;
	};

	CLASS(MovementDetectorPlugin, PyMovementDetectorPlugin)
	METHOD(MovementDetectorPlugin, process, "data"_a)
	METHOD_VOID(MovementDetectorPlugin, get_status)
	METHOD_VOID(MovementDetectorPlugin, get_time)
	CDOC(MovementDetectorPlugin);

	CLASS(MovementDetectorPos, MovementDetectorPlugin)
	CTOR(MovementDetectorPos,
	     PARAMS(const double, const double),
	     "speed_cutoff"_a       = 0.2,
	     "zero_corr_distance"_a = 100.0)
	CDOC(MovementDetectorPos);

	CLASS(MovementDetectorImu, MovementDetectorPlugin)
	CTOR(MovementDetectorImu,
	     PARAMS(const navtk::Size, const double),
	     "window"_a     = 10,
	     "calib_time"_a = 30.0)
	CDOC(MovementDetectorImu);

	CLASS(MovementDetectorPluginStat)
	CTOR_CLSDOC(MovementDetectorPluginStat, PARAMS(double, double), "weight"_a, "stale_time"_a)
	FIELD(MovementDetectorPluginStat, weight)
	FIELD(MovementDetectorPluginStat, stale_time);

	CLASS(MovementDetector)
	CTOR_CLSDOC_DEFAULT(MovementDetector)
	METHOD(MovementDetector, add_plugin, "id"_a, "plugin"_a, "weight"_a = 1.0, "stale_time"_a = 1.0)
	METHOD(MovementDetector, remove_plugin, "id"_a)
	METHOD_OVERLOAD(MovementDetector,
	                process,
	                PARAMS(not_null<std::shared_ptr<aspn_xtensor::AspnBase>>),
	                ,
	                "data"_a)
	METHOD_OVERLOAD(
	    MovementDetector,
	    process,
	    PARAMS(const std::vector<std::string> &, not_null<std::shared_ptr<aspn_xtensor::AspnBase>>),
	    _2,
	    "ids"_a,
	    "data"_a)
	METHOD_VOID(MovementDetector, get_status)
	METHOD_VOID(MovementDetector, plugin_info);
}
