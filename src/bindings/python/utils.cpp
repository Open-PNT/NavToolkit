#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <navtk/aspn.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/CubicSplineModel.hpp>
#include <navtk/utils/DimensionValidator.hpp>
#include <navtk/utils/GriddedInterpolant.hpp>
#include <navtk/utils/InterpolationModel.hpp>
#include <navtk/utils/LinearModel.hpp>
#include <navtk/utils/OutlierDetection.hpp>
#include <navtk/utils/OutlierDetectionSigma.hpp>
#include <navtk/utils/OutlierDetectionThreshold.hpp>
#include <navtk/utils/QuadraticSplineModel.hpp>
#include <navtk/utils/ValidationContext.hpp>
#include <navtk/utils/ValidationResult.hpp>
#include <navtk/utils/algorithm.hpp>
#include <navtk/utils/conversions.hpp>
#include <navtk/utils/data.hpp>
#include <navtk/utils/human_readable.hpp>
#include <navtk/utils/interpolation.hpp>
#include <navtk/utils/sortable_vectors.hpp>

#include "binding_helpers.hpp"

namespace su = navtk::utils;

using namespace pybind11::literals;

using aspn_xtensor::TypeTimestamp;
using navtk::ErrorMode;
using navtk::Matrix;
using navtk::Size;
using navtk::Vector;
using navtk::Vector3;
using navtk::filtering::NavSolution;
using navtk::inertial::InertialPosVelAtt;
using navtk::inertial::StandardPosVelAtt;
using navtk::utils::condition_source_data;
using navtk::utils::cubic_spline_interpolate;
using navtk::utils::CubicSplineModel;
using navtk::utils::diff;
using navtk::utils::DimensionValidator;
using navtk::utils::find_duplicates;
using navtk::utils::find_outside;
using navtk::utils::GriddedInterpolant;
using navtk::utils::InterpolationModel;
using navtk::utils::linear_interp_pva;
using navtk::utils::linear_interp_rpy;
using navtk::utils::linear_interpolate;
using navtk::utils::LinearModel;
using navtk::utils::normalize;
using navtk::utils::OutlierDetection;
using navtk::utils::OutlierDetectionSigma;
using navtk::utils::OutlierDetectionThreshold;
using navtk::utils::pair_and_time_sort_data;
using navtk::utils::quadratic_spline_interpolate;
using navtk::utils::QuadraticSplineModel;
using navtk::utils::remove_at_indices;
using navtk::utils::repr;
using navtk::utils::split_vector_pairs;
using navtk::utils::to_inertial_aux;
using navtk::utils::to_navsolution;
using navtk::utils::to_positionvelocityattitude;
using navtk::utils::to_standardposvelatt;
using navtk::utils::to_vector_pva;
using navtk::utils::trapezoidal_area;
using navtk::utils::ValidationContext;
using navtk::utils::ValidationResult;
using navtk::utils::detail::visit_possible_file_paths;

using PVA   = aspn_xtensor::MeasurementPositionVelocityAttitude;
using vd    = std::vector<double>;
using prsd  = std::pair<std::vector<navtk::Size>, vd>;
using vprdd = std::vector<std::pair<double, double>>;

// The real "navtk::open_data_file" returns a `shared_ptr` to an istream, which is of limited value
// to python people. This version behaves identically but returns a python file object instead.
py::object py_open_data_file(navtk::ErrorMode error_mode,
                             const py::str &label,
                             const py::str &basename,
                             const py::str &mode) {
	py::object out = py::none();
	auto io        = py::module_::import("io");
	auto open      = io.attr("open");
	visit_possible_file_paths(error_mode, label, basename, [&](const std::string &path) -> bool {
		try {
			out = open(path, mode);
			return true;
		} catch (py::error_already_set &) {
			return false;
		}
	});
	return out;
}

void add_utils_functions(pybind11::module &m) {
	m.doc() = "NavToolkit Utils";

	m.attr("NANO_PER_SEC") = py::int_(navtk::utils::NANO_PER_SEC);

	FUNCTIONT(trapezoidal_area, PARAMS(double, double), "x0"_a, "y0"_a, "x1"_a, "y1"_a)

	// InRange and NearestNeighbors are templated classes, and require separate bindings for each
	// type implementation. Could implement common types, but isn't really worth it. If we want to
	// bind these later, we could pass a type name to a function that implements the class for that
	// type.

	FUNCTION_CAST(to_navsolution, NavSolution(*)(const PVA &), , "pva"_a)
	FUNCTION_CAST(to_navsolution, NavSolution(*)(const InertialPosVelAtt &), _2, "pva"_a)
	FUNCTION_CAST(to_navsolution, NavSolution(*)(const Vector &), _3, "pva"_a)
	FUNCTION_CAST(to_positionvelocityattitude, PVA(*)(const NavSolution &), , "pva"_a)
	FUNCTION_CAST(to_positionvelocityattitude, PVA(*)(const InertialPosVelAtt &), _2, "pva"_a)
	FUNCTION_CAST(to_positionvelocityattitude, PVA(*)(const Vector &), _3, "pva"_a)
	FUNCTION_CAST(to_standardposvelatt, StandardPosVelAtt(*)(const NavSolution &), , "pva"_a)
	FUNCTION_CAST(to_standardposvelatt, StandardPosVelAtt(*)(const PVA &), _2, "pva"_a)
	FUNCTION_CAST(to_standardposvelatt, StandardPosVelAtt(*)(const Vector &), _3, "pva"_a)
	FUNCTION_CAST(to_vector_pva, Vector(*)(const NavSolution &), , "pva"_a)
	FUNCTION_CAST(to_vector_pva, Vector(*)(const PVA &), _2, "pva"_a)
	FUNCTION_CAST(to_vector_pva, Vector(*)(const InertialPosVelAtt &), _3, "pva"_a)
	FUNCTION(to_inertial_aux, "nav_sol"_a, "forces"_a, "rates"_a = navtk::zeros(3))

	FUNCTION_CAST(repr, std::string(*)(const Matrix &, const std::string &), , "matrix"_a, "decl"_a)
	FUNCTION_CAST(repr, std::string(*)(const Matrix &), , "matrix"_a)
	// Don't need binding for template `std::string repr(const xt::xexpression<E>& expr)` function.
	// xtensor is mapped to numpy, which already has a repr implementation.

	FUNCTION_CAST(diff,
	              std::string(*)(const std::string &,
	                             const std::string &,
	                             const Matrix &,
	                             const Matrix &,
	                             double,
	                             double),
	              ,
	              "before_name"_a,
	              "after_name"_a,
	              "before"_a,
	              "after"_a,
	              "rtol"_a = 1e-05,
	              "atol"_a = 1e-08)
	FUNCTION_CAST(diff,
	              std::string(*)(const Matrix &, const Matrix &, double, double),
	              ,
	              "before"_a,
	              "after"_a,
	              "rtol"_a = 1e-05,
	              "atol"_a = 1e-08)

	// Don't need binding for `std::string identify_type()`. Python handles this with `type()`

	// Python side doesn't reflect modifications of pass-by-ref args with a condition_source_data
	// binding. Might work with
	// PYBIND11_MAKE_OPAQUE(std::vector<double>)
	// and then hand-rolling a vector<double> binding
	// https://pybind11.readthedocs.io/en/master/advanced/cast/stl.html
	// But since the index getting functions (outside, duplicate, pair, sort)
	// all work, it's probably easier to use built-ins/numpy to remove elements by
	// index if desired.
	// FUNCTION_CAST(condition_source_data,
	//               std::vector<navtk::Size>(*)(vd &, vd &, vd &),
	//               "time_source"_a,
	//               "data_source"_a,
	//               "time_interp"_a)

	FUNCTION_CAST(linear_interpolate,
	              prsd(*)(const vd &, const vd &, const vd &),
	              ,
	              "time_source"_a,
	              "data_source"_a,
	              "time_interp"_a)

	FUNCTIONT_CAST(linear_interpolate,
	               // Template type (Y)
	               double,
	               // All parameters, including resolved template types
	               PARAMS(double, const double &, double, const double &, double),
	               _2,
	               "x0"_a,
	               "x1"_a,
	               "y0"_a,
	               "y1"_a,
	               "x"_a)
	FUNCTIONT_CAST(linear_interpolate,
	               // Template type (Y)
	               double,
	               // All parameters, including resolved template types
	               PARAMS(const aspn_xtensor::TypeTimestamp &,
	                      const double &,
	                      const aspn_xtensor::TypeTimestamp &,
	                      const double &,
	                      const aspn_xtensor::TypeTimestamp &),
	               _2,
	               "x0"_a,
	               "x1"_a,
	               "y0"_a,
	               "y1"_a,
	               "x"_a)
	FUNCTIONT_CAST(linear_interpolate,
	               // Template type (Y)
	               Vector3,
	               // All parameters, including resolved template types
	               PARAMS(const aspn_xtensor::TypeTimestamp &,
	                      const Vector3 &,
	                      const aspn_xtensor::TypeTimestamp &,
	                      const Vector3 &,
	                      const aspn_xtensor::TypeTimestamp &),
	               _2,
	               "x0"_a,
	               "x1"_a,
	               "y0"_a,
	               "y1"_a,
	               "x"_a)
	FUNCTIONT_CAST(linear_interpolate,
	               // Template type (Y)
	               Vector3,
	               // All parameters, including resolved template types
	               PARAMS(double, const Vector3 &, double, const Vector3 &, double),
	               _2,
	               "x0"_a,
	               "x1"_a,
	               "y0"_a,
	               "y1"_a,
	               "x"_a)
	NAMESPACE_FUNCTION(
	    quadratic_spline_interpolate, su, "time_source"_a, "data_source"_a, "time_interp"_a)
	NAMESPACE_FUNCTION(
	    cubic_spline_interpolate, su, "time_source"_a, "data_source"_a, "time_interp"_a)
	FUNCTION_CAST(linear_interp_pva,
	              navtk::not_null<std::shared_ptr<PVA>>(*)(navtk::not_null<std::shared_ptr<PVA>>,
	                                                       navtk::not_null<std::shared_ptr<PVA>>,
	                                                       const aspn_xtensor::TypeTimestamp &),
	              ,
	              "pva1"_a,
	              "pva2"_a,
	              "t"_a)
	NAMESPACE_FUNCTION(linear_interp_rpy, su, "t1"_a, "rpy1"_a, "t2"_a, "rpy2"_a, "t"_a)

	FUNCTIONT(normalize, PARAMS(double), "orig"_a, "min_val"_a)
	FUNCTIONT(diff, PARAMS(double), "data"_a)
	FUNCTIONT(diff, PARAMS(aspn_xtensor::TypeTimestamp), "data"_a)
	FUNCTION_CAST(find_duplicates, std::vector<navtk::Size>(*)(const vprdd &), , "data"_a)
	FUNCTION_CAST(find_duplicates, std::vector<navtk::Size>(*)(const vd &), _2, "data"_a)
	FUNCTIONT(find_outside, double, "query_time"_a, "data"_a)
	FUNCTIONT(remove_at_indices, double, "data"_a, "to_remove"_a)
	FUNCTIONT(split_vector_pairs, PARAMS(double, double), "orig"_a)
	FUNCTIONT(pair_and_time_sort_data, double, "tags"_a, "data"_a)

	class PyInterpolationModel : public InterpolationModel {
	public:
		PyInterpolationModel(const vd &x, const vd &y) : InterpolationModel(x, y) {}

		double y_at(double x_interp) override {
			PYBIND11_OVERRIDE_PURE(double, InterpolationModel, y_at, x_interp);
		}
	};

	CLASS(InterpolationModel, PyInterpolationModel)
	CTOR(InterpolationModel, PARAMS(const vd &, const vd &), "x"_a, "y"_a)
	METHOD(InterpolationModel, y_at, "x_interp"_a)
	CDOC(InterpolationModel);

	CLASS(LinearModel, InterpolationModel)
	CTOR(LinearModel, PARAMS(const vd &, const vd &), "x"_a, "y"_a)
	CDOC(LinearModel);

	CLASS(QuadraticSplineModel, InterpolationModel)
	CTOR(QuadraticSplineModel, PARAMS(const vd &, const vd &), "x"_a, "y"_a)
	CDOC(QuadraticSplineModel);

	CLASS(CubicSplineModel, InterpolationModel)
	CTOR(CubicSplineModel, PARAMS(const vd &, const vd &), "x"_a, "y"_a)
	CDOC(CubicSplineModel);

	CLASS(GriddedInterpolant)
	CTOR(GriddedInterpolant, PARAMS(Vector, Vector, Matrix), "x_vector"_a, "y_vector"_a, "q_mat"_a)
	METHOD(GriddedInterpolant, interpolate, "x"_a, "y"_a)
	CDOC(GriddedInterpolant);

	class PyOutlierDetection : public OutlierDetection {
	public:
		PyOutlierDetection(size_t buffer_size) : OutlierDetection(buffer_size) {}

		bool is_last_item_an_outlier(navtk::Vector const &data) const override {
			PYBIND11_OVERRIDE_PURE(bool, OutlierDetection, is_last_item_an_outlier, data);
		}
	};

	CLASS(OutlierDetection, PyOutlierDetection)
	CTOR(OutlierDetection, PARAMS(size_t), "buffer_size"_a)
	METHOD(OutlierDetection, is_outlier, "value"_a)
	CDOC(OutlierDetection);

	CLASS(OutlierDetectionSigma)
	CTOR(OutlierDetectionSigma, PARAMS(size_t, double), "buffer_size"_a, "sigma"_a)
	METHOD(OutlierDetectionSigma, is_last_item_an_outlier, "value_history_vector"_a)
	CDOC(OutlierDetectionSigma);

	CLASS(OutlierDetectionThreshold)
	CTOR(OutlierDetectionThreshold, PARAMS(size_t, double), "buffer_size"_a, "threshold"_a)
	METHOD(OutlierDetectionThreshold, is_last_item_an_outlier, "value_history_vector"_a)
	CDOC(OutlierDetectionThreshold);

	// RingBuffer, RingBufferIterator, and IteratorAdapter are templated classes, and require
	// separate bindings for each type implementation. Could implement common types, but isn't
	// really worth it. If we want to bind these later, we could pass a type name to a function that
	// implements the class for that type.

	CLASS(DimensionValidator)
	CTOR_NODOC_DEFAULT
	METHOD_OVERLOAD(DimensionValidator,
	                dim,
	                PARAMS(const std::string &, const Matrix &, Size, Size),
	                ,
	                "name"_a,
	                "matrix"_a,
	                "rows"_a,
	                "cols"_a)
	METHOD_OVERLOAD(DimensionValidator,
	                dim,
	                PARAMS(const std::string &, const Matrix &, Size, char),
	                _2,
	                "name"_a,
	                "matrix"_a,
	                "rows"_a,
	                "cols"_a)
	METHOD_OVERLOAD(DimensionValidator,
	                dim,
	                PARAMS(const std::string &, const Matrix &, char, Size),
	                _3,
	                "name"_a,
	                "matrix"_a,
	                "rows"_a,
	                "cols"_a)
	METHOD_OVERLOAD(DimensionValidator,
	                dim,
	                PARAMS(const std::string &, const Matrix &, char, char),
	                _4,
	                "name"_a,
	                "matrix"_a,
	                "rows"_a,
	                "cols"_a)
	METHOD(DimensionValidator, perform_validation, "mode"_a, "result_out"_a)
	CDOC(DimensionValidator);

	CLASS(ValidationContext)
	CTOR_DEFAULT(ValidationContext)
	CTOR(ValidationContext, ErrorMode, "mode"_a)
	METHOD_OVERLOAD(ValidationContext, add_matrix, const Matrix &, , "matrix"_a)
	METHOD_OVERLOAD(ValidationContext,
	                add_matrix,
	                PARAMS(const Matrix &, const std::string &),
	                _2,
	                "matrix"_a,
	                "name"_a)
	METHOD_OVERLOAD(ValidationContext, add_matrix, PARAMS(const Vector &), _3, "matrix"_a)
	METHOD_OVERLOAD(ValidationContext,
	                add_matrix,
	                PARAMS(const Vector &, const std::string &),
	                _2,
	                "matrix"_a,
	                "name"_a)
	METHOD(ValidationContext, symmetric, "rtol"_a = 1e-5, "atol"_a = 1e-8)
	METHOD(ValidationContext, max, "limit"_a)
	METHOD(ValidationContext, min, "limit"_a)
	METHOD_OVERLOAD(ValidationContext, dim, PARAMS(Size, Size), , "rows"_a, "cols"_a)
	METHOD_OVERLOAD(ValidationContext, dim, PARAMS(Size, int), _2, "rows"_a, "cols"_a)
	METHOD_OVERLOAD(ValidationContext, dim, PARAMS(int, Size), _3, "rows"_a, "cols"_a)
	METHOD_OVERLOAD(ValidationContext, dim, PARAMS(int, int), _4, "rows"_a, "cols"_a)
	METHOD_OVERLOAD(ValidationContext, dim, PARAMS(int, char), _5, "rows"_a, "cols"_a)
	METHOD_OVERLOAD(ValidationContext, dim, PARAMS(Size, char), _6, "rows"_a, "cols"_a)
	METHOD_OVERLOAD(ValidationContext, dim, PARAMS(char, int), _7, "rows"_a, "cols"_a)
	METHOD_OVERLOAD(ValidationContext, dim, PARAMS(char, Size), _8, "rows"_a, "cols"_a)
	METHOD_OVERLOAD(ValidationContext, dim, PARAMS(char, char), _9, "rows"_a, "cols"_a)
	// METHOD_VOID(ValidationContext, transposable) // TODO #587: Not implemented
	METHOD_VOID(ValidationContext, validate)
	METHOD_VOID(ValidationContext, get_mode)
	METHOD_VOID(ValidationContext, is_enabled)
	CDOC(ValidationContext);

	ENUM(ValidationResult)
	CHOICE(ValidationResult, NOT_CHECKED)
	CHOICE(ValidationResult, GOOD)
	CHOICE(ValidationResult, BAD)
	ENUM_REPR

	// Because istream isn't useful from Python, rather than a straight binding for
	// open_data_file, we provide a version that behaves like Python's `open()` builtin,
	// returning a python file object.
	m.def(
	    "open_data_file",
	    [](const py::str &label, const py::str &basename) -> py::object {
		    return py_open_data_file(::navtk::get_global_error_mode(), label, basename, "r");
	    },
	    "label"_a,
	    "basename"_a);
	m.def(
	    "open_data_file",
	    [](navtk::ErrorMode error_mode, const py::str &label, const py::str &basename)
	        -> py::object { return py_open_data_file(error_mode, label, basename, "r"); },
	    "error_mode"_a,
	    "label"_a,
	    "basename"_a);
	m.def(
	    "open_data_file",
	    [](const py::str &label, const py::str &basename, const py::str &mode) -> py::object {
		    return py_open_data_file(::navtk::get_global_error_mode(), label, basename, mode);
	    },
	    "label"_a,
	    "basename"_a,
	    "mode"_a);
	m.def(
	    "open_data_file",
	    [](navtk::ErrorMode error_mode,
	       const py::str &label,
	       const py::str &basename,
	       const py::str &mode) -> py::object {
		    return py_open_data_file(error_mode, label, basename, mode);
	    },
	    DOCFMT(__doc_open_data_file).c_str(),
	    "error_mode"_a,
	    "label"_a,
	    "basename"_a,
	    "mode"_a);
}
