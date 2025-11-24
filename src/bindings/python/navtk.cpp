#include <functional>
#include <string>

#define FORCE_IMPORT_ARRAY

#include <pybind11/eval.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <xtensor-python/pyarray.hpp>
#include <xtensor-python/pytensor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/experimental/random.hpp>
#include <navtk/get_time.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/version.hpp>

#include "binding_helpers.hpp"

using aspn_xtensor::TypeTimestamp;
using navtk::ErrorMode;
using navtk::ErrorModeLock;
using navtk::get_global_error_mode;
using navtk::set_global_error_mode;
using navtk::solve_wahba_davenport;
using navtk::solve_wahba_svd;
using navtk::experimental::s_rand;
using spdlog::level::level_enum;
using std::function;
using std::string;
using namespace pybind11::literals;
namespace py = pybind11;

// Because log_or_throw uses template parameters that python can't, it needs to be re-implemented in
// terms of python runtime types.
void py_log_or_throw_(function<void(const string &)> throw_action,
                      level_enum level,
                      ErrorMode mode,
                      const py::str &format_string,
                      py::args format_args) {
	std::string formatted{format_string.format(*format_args)};
	if (mode != ErrorMode::OFF) spdlog::log(level, "{}", formatted);
	if (mode == ErrorMode::DIE) throw_action(formatted);
}

// Pybind gets confused if we `throw` a Python exception type directly from C++
void py_raise_(py::object &exception_factory, const std::string &msg) {
	auto locals = py::dict("exc"_a = exception_factory(msg));
	py::exec("raise exc", py::globals(), locals);
}

// Pybind harvests the name of each parameter type for its docstring, but their name for callable is
// py::function which is extremely misleading in a context where we're really expecting an exception
// type. However, py::type is too restrictive in its type-checking to use as an error-factory
// parameter, so to get both a sane behavior and a sane docstring, we define "callable" ourselves.
// Annoyingly, we have to do this inside the pybind namespace to avoid getting visibility errors
// when compiling under gcc.
PYBIND11_NAMESPACE_BEGIN(PYBIND11_NAMESPACE)
struct type_or_callable : py::function {
	using function::function;
};
PYBIND11_NAMESPACE_END(PYBIND11_NAMESPACE)


// We need a cartesian product of overloads to handle all the different possible combinations of
// optional arguments. The language gods are punishing me for designing an API that looks pretty in
// C++.
//
// These macros work using token pasting to select either the parameterized (1) version of a part of
// the binding, or the default version (0) of the binding. BIND_LOG_OR_THROW_OVERLOAD(1, ...) for
// example selects EX_PARAM_1, EX_EXPR_1 and EX_A_1 to accept an exception_factory parameter, throw
// the result of that exception factory, and reference the Exc in log_or_throw's docstring,
// respectively. This results in each of the first three BIND_LOG_OR_THROW_OVERLOAD parameters being
// a binary 0-1 flag for whether to include that parameter in the binding. The last parameter is
// injected to the m.def call where it expects a docstring, and is used to only supply a docstring
// for the last of the overloads (since otherwise pybind will repeat the docstring eight times)
//
// Inside the PYBIND11_MODULE below, BIND_LOG_OR_THROW_OVERLOAD is called with every possible
// combination of 0s and 1s.
#define EX_PARAM_0
#define EX_PARAM_1 py::type_or_callable &exception_factory,
#define EX_EXPR_0 [](const std::string &msg) { throw navtk::DefaultLogOrThrowException(msg); }
#define EX_EXPR_1 [&](const std::string &msg) { py_raise_(exception_factory, msg); }
#define EX_A_0
#define EX_A_1 "Exc"_a,
#define LEVEL_PARAM_0
#define LEVEL_PARAM_1 int level,
#define LEVEL_EXPR_0 navtk::DEFAULT_LOG_OR_THROW_LEVEL
#define LEVEL_EXPR_1 static_cast<level_enum>(level)
#define LEVEL_A_0
#define LEVEL_A_1 "Level"_a,
#define MODE_PARAM_0
#define MODE_PARAM_1 ErrorMode mode,
#define MODE_EXPR_0 get_global_error_mode()
#define MODE_EXPR_1 mode
#define MODE_A_0
#define MODE_A_1 "mode"_a,
#define RESCAN(X) X
#define COMMA ,
#define BIND_LOG_OR_THROW_OVERLOAD(ex_flag, level_flag, mode_flag, docs)    \
	m.def(                                                                  \
	    "log_or_throw",                                                     \
	    [](RESCAN(EX_PARAM_##ex_flag) RESCAN(LEVEL_PARAM_##level_flag)      \
	           RESCAN(MODE_PARAM_##mode_flag) const py::str &format_string, \
	       py::args format_args) {                                          \
		    py_log_or_throw_(RESCAN(EX_EXPR_##ex_flag),                     \
		                     RESCAN(LEVEL_EXPR_##level_flag),               \
		                     RESCAN(MODE_EXPR_##mode_flag),                 \
		                     format_string,                                 \
		                     std::move(format_args));                       \
	    },                                                                  \
	    docs RESCAN(EX_A_##ex_flag) RESCAN(LEVEL_A_##level_flag)            \
	        RESCAN(MODE_A_##mode_flag) "message"_a)


void add_experimental_functions(pybind11::module &m) {
	m.doc() = "Bindings to the NavToolkit Experimental Work.";
	FUNCTION(s_rand, "seed"_a);
}

PYBIND11_MODULE(navtk, m) {
	xt::import_numpy();

	py::module_::import("aspn23_xtensor");

	ADD_NAMESPACE(m, filtering);
	ADD_NAMESPACE(m, navutils);
	ADD_NAMESPACE(m, geospatial);
	ADD_NAMESPACE(m, exampleutils);
	ADD_NAMESPACE(m, inertial);
	ADD_NAMESPACE(m, magnetic);
	ADD_NAMESPACE(m, utils);
	ADD_NAMESPACE(m, experimental);
	m.doc() = "NavToolkit";

	NAMESPACE_FUNCTION_OVERLOAD(
	    to_seconds, navtk, PARAMS(const std::vector<TypeTimestamp> &), , "times"_a)

	ENUM(ErrorMode)
	CHOICE(ErrorMode, OFF)
	CHOICE(ErrorMode, LOG)
	CHOICE(ErrorMode, DIE).finalize();

	FUNCTION_VOID(get_global_error_mode);
	m.def(
	    "set_global_error_mode",
	    [](ErrorMode mode) {
		    // Since set_global_error_mode checks for ErrorModeLocks, it's possible to deadlock on
		    // the python GIL unless we explicitly release it before trying to
		    // set_global_error_mode.
		    py::gil_scoped_release unlock_gil;
		    set_global_error_mode(mode);
	    },
	    DOCFMT(__doc_set_global_error_mode).c_str(),
	    "mode"_a);

	struct PyErrorModeLock : ErrorModeLock, public py::trampoline_self_life_support {
		using ErrorModeLock::ErrorModeLock;
		using ErrorModeLock::relock;
		using ErrorModeLock::unlock;
	};

	CLASS(ErrorModeLock, PyErrorModeLock)
	CTOR(ErrorModeLock, PARAMS(ErrorMode, bool), "target_mode"_a, "restore_on_destruct"_a)
	CTOR(ErrorModeLock, PARAMS(ErrorMode), "target_mode"_a)
	    .def("__enter__",
	         [](PyErrorModeLock &lock) {
		         py::gil_scoped_release unlock_gil;
		         lock.relock();
	         })
	    .def("__exit__",
	         [](PyErrorModeLock &lock, pybind11::object &, pybind11::object &, pybind11::object &) {
		         lock.unlock();
	         });

	BIND_LOG_OR_THROW_OVERLOAD(0, 0, 0, );
	BIND_LOG_OR_THROW_OVERLOAD(0, 0, 1, );
	BIND_LOG_OR_THROW_OVERLOAD(0, 1, 0, );
	BIND_LOG_OR_THROW_OVERLOAD(0, 1, 1, );
	BIND_LOG_OR_THROW_OVERLOAD(1, 0, 0, );
	BIND_LOG_OR_THROW_OVERLOAD(1, 0, 1, );
	BIND_LOG_OR_THROW_OVERLOAD(1, 1, 0, );
	BIND_LOG_OR_THROW_OVERLOAD(1, 1, 1, DOCFMT(__doc_log_or_throw).c_str() COMMA);

	m.def("version", []() -> py::tuple {
		return py::make_tuple(
		    NAVTK_VERSION_MAJOR, NAVTK_VERSION_MINOR, NAVTK_VERSION_PATCH, NAVTK_VERSION_TOKEN);
	});

	FUNCTION_CAST(solve_wahba_svd, navtk::Matrix3(*)(const navtk::Matrix3 &), , "outer"_a);
	FUNCTION_CAST(
	    solve_wahba_svd,
	    navtk::Matrix3(*)(const std::vector<navtk::Vector3> &, const std::vector<navtk::Vector3> &),
	    _2,
	    "p"_a,
	    "r"_a);
	FUNCTION_CAST(solve_wahba_davenport,
	              std::vector<navtk::Matrix3>(*)(const std::vector<navtk::Vector3> &,
	                                             const std::vector<navtk::Vector3> &),
	              ,
	              "p"_a,
	              "r"_a);
	FUNCTION_CAST(solve_wahba_davenport,
	              std::vector<navtk::Matrix3>(*)(const navtk::Matrix3 &, const navtk::Vector3 &),
	              _2,
	              "outer"_a,
	              "cr"_a);

	// Bindings not needed for not_null, as this is a C++ pointer feature.  The NOT_NONE macro is
	// sufficient to implement not_null functionality in python.

	// Bindings not needed for functions in linear_algebra.hpp, factory.hpp, inspect.hpp,
	// random.hpp, tensors.hpp, transform.hpp, as these functions all have numpy alternatives.
	// Furthermore, RandomNumberGenerator classes are expected to be replaced with librandom, so not
	// worth binding these at this time.
}
