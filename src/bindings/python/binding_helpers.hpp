#pragma once

#include <iostream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>

#include <pybind11/pybind11.h>

#include <navtk/not_null.hpp>

// navtk_generated.hpp is the result of extracting all of the docstrings from the navtk header
// files so they can be reused as the bindings docs.
#ifdef NAVTK_PYTHON_DOCSTRINGS
#	include "navtk_generated.hpp"
#	define DOCFMT(NAME) docfmt(NAME)
#else
#	define DOCFMT(NAME) docfmt("")
#endif

// TODO PNTOS-262 Better version of this (still has issues recovering whitespace sometimes).
std::string docfmt(const std::string& docstring);

namespace pybind11 {
namespace detail {

template <typename T>
struct type_caster<navtk::not_null<T>> {
	typedef type_caster<T> TCaster;
	TCaster subcaster;
	static handle cast(navtk::not_null<T> src, return_value_policy p, handle h) {
		return TCaster::cast(std::forward<T>(src.get()), p, h);
	}

	bool load(handle src, bool convert) { return subcaster.load(src, convert); }

	template <typename _>
	using cast_op_type         = navtk::not_null<T>;
	static constexpr auto name = TCaster::name;
	operator navtk::not_null<T>() { return static_cast<T>(subcaster); }
};

}  // namespace detail
}  // namespace pybind11

namespace py = pybind11;

// Used to add a namespace inside the top navtk namespace
#define ADD_NAMESPACE(M, NAME)                            \
	{                                                     \
		auto submod = M.def_submodule(#NAME);             \
		void add_##NAME##_functions(pybind11::module& m); \
		add_##NAME##_functions(submod);                   \
	}
// Same as ADD_NAMESPACE but for a namespace nested inside a child of navtk namespace.
// The only difference between this and ADD_NAMESPACE is the name of the function containing
// bindings for this subnamespace.
#define ADD_SUBNAMESPACE(M, SUBNAMESPACE, NAMESPACE)                            \
	{                                                                           \
		auto submod = M.def_submodule(#SUBNAMESPACE);                           \
		void add_##NAMESPACE##_##SUBNAMESPACE##_functions(pybind11::module& m); \
		add_##NAMESPACE##_##SUBNAMESPACE##_functions(submod);                   \
	}

// Used to pass parameter types by grouping a comma-separated collection of arguments to be passed
// in as a single macro argument. Can be passed to constructor macros or overloaded function macros
#define PARAMS(...) __VA_ARGS__
// Used to mark parameters as navtk::not_null.
#define NOT_NONE(NAME) py::arg(NAME).none(false)
// Macro helper returning the first argument in a __VA_ARGS__
#define FIRST(X, ...) X
// Macro trick forcing the ... to be rescanned
#define CALL_MACRO(macro, ...) macro(__VA_ARGS__)
#define PY_CLASS_IMPL(NAME, ...) py::class_<__VA_ARGS__, std::shared_ptr<NAME>>(m, #NAME)
// Binds a class. First parameter is the class name. Optional additional params are base classes.
#define CLASS(...) CALL_MACRO(PY_CLASS_IMPL, FIRST(__VA_ARGS__, MISSING CLASS NAME), __VA_ARGS__)
// Binds a templated class with the default template parameters.
#define CLASST(NAME, ...) py::class_<NAME<>, __VA_ARGS__, std::shared_ptr<NAME<>>>(m, #NAME)
// Finds and adds the relevant docstring from navtk_generated.hpp.
#define CDOC(NAME) .doc() = PROCESS_DOC(NAME)
// Accepts a string argument, which it matches against the docstrings in navtk_generated.hpp,
// returning the matched string.
#define PROCESS_DOC(DOCNAME) DOCFMT(__doc_##DOCNAME).c_str()
// Binds a constructor
#define CTOR(CLASSNAME, TYPES, ...) \
	.def(py::init<TYPES>(), PROCESS_DOC(CLASSNAME##_##CLASSNAME), __VA_ARGS__)
// Binds a default constructor (some compilers give warnings when not all macro arguments are used).
#define CTOR_DEFAULT(CLASSNAME) .def(py::init<>(), PROCESS_DOC(CLASSNAME##_##CLASSNAME))
// Same as CTOR, but uses top-level docstring rather than constructor docstring
#define CTOR_CLSDOC(CLASSNAME, TYPES, ...) \
	.def(py::init<TYPES>(), PROCESS_DOC(CLASSNAME), __VA_ARGS__)
// Same as CTOR_CLSDOC, but for a default constructor (some compilers give warnings when not all
// macro arguments are used).
#define CTOR_CLSDOC_DEFAULT(CLASSNAME) .def(py::init<>(), PROCESS_DOC(CLASSNAME))
// Same as CTOR, but without docstring
#define CTOR_NODOC(TYPES, ...) .def(py::init<TYPES>(), __VA_ARGS__)
// Same as CTOR_NODOC, but for a default constructor (some compilers give warnings when not all
// macro arguments are used).
#define CTOR_NODOC_DEFAULT .def(py::init<>())
// Bind an overloaded constructor. DOC_SUFFIX is used to bind it with the correct docstring.
#define CTOR_OVERLOAD(CLASSNAME, TYPES, DOC_SUFFIX, ...) \
	.def(py::init<TYPES>(), PROCESS_DOC(CLASSNAME##_##CLASSNAME##DOC_SUFFIX), __VA_ARGS__)
// Bind a class's field.
#define FIELD(CLASSNAME, FIELDNAME) \
	.def_readwrite(#FIELDNAME, &CLASSNAME::FIELDNAME, PROCESS_DOC(CLASSNAME##_##FIELDNAME))
// Bind a templated class's field.
#define FIELDT(CLASSNAME, FIELDNAME) \
	.def_readwrite(#FIELDNAME, &CLASSNAME<>::FIELDNAME, PROCESS_DOC(CLASSNAME##_##FIELDNAME))
// Bind a class's method.
#define METHOD(CLASSNAME, METHODNAME, ...) \
	.def(#METHODNAME, &CLASSNAME::METHODNAME, PROCESS_DOC(CLASSNAME##_##METHODNAME), __VA_ARGS__)
// Bind a method with no parameters (some compilers give warnings when not all macro arguments are
// used).
#define METHOD_VOID(CLASSNAME, METHODNAME) \
	.def(#METHODNAME, &CLASSNAME::METHODNAME, PROCESS_DOC(CLASSNAME##_##METHODNAME))
// Bind a protected member, given a trampoline class which exposes the protected member as public.
#define METHOD_PROTECTED(CLASSNAME, METHODNAME, TRAMPOLINE_CLASS_NAME, ...) \
	.def(#METHODNAME,                                                       \
	     &TRAMPOLINE_CLASS_NAME::METHODNAME,                                \
	     PROCESS_DOC(CLASSNAME##_##METHODNAME),                             \
	     __VA_ARGS__)
// Same as METHOD_PROTECTED but without extra parameters for methods with no parameters (some
// compilers give warnings when not all macro arguments are used).
#define METHOD_PROTECTED_VOID(CLASSNAME, METHODNAME, TRAMPOLINE_CLASS_NAME) \
	.def(#METHODNAME, &TRAMPOLINE_CLASS_NAME::METHODNAME, PROCESS_DOC(CLASSNAME##_##METHODNAME))
// Bind a templated class's method.
#define METHODT(CLASSNAME, METHODNAME, ...) \
	.def(#METHODNAME, &CLASSNAME<>::METHODNAME, PROCESS_DOC(CLASSNAME##_##METHODNAME), __VA_ARGS__)
// Same as METHODT but without extra parameters for methods with no parameters (some compilers give
// warnings when not all macro arguments are used).
#define METHODT_VOID(CLASSNAME, METHODNAME) \
	.def(#METHODNAME, &CLASSNAME<>::METHODNAME, PROCESS_DOC(CLASSNAME##_##METHODNAME))
// Same as METHOD_OVERLOAD but for a method marked `const`.
#define METHOD_OVERLOAD_CONST(CLASSNAME, METHODNAME, CASTTYPE, DOC_SUFFIX, ...) \
	.def(#METHODNAME,                                                           \
	     py::overload_cast<CASTTYPE>(&CLASSNAME::METHODNAME, py::const_),       \
	     PROCESS_DOC(CLASSNAME##_##METHODNAME##DOC_SUFFIX),                     \
	     __VA_ARGS__)
// Same as METHOD_OVERLOAD_CONST but for a method that accepts no parameters.
#define METHOD_OVERLOAD_CONST_VOID(CLASSNAME, METHODNAME, DOC_SUFFIX) \
	.def(#METHODNAME,                                                 \
	     py::overload_cast<>(&CLASSNAME::METHODNAME, py::const_),     \
	     PROCESS_DOC(CLASSNAME##_##METHODNAME##DOC_SUFFIX))
// Bind an overloaded method. DOC_SUFFIX is used to select the correct docstring from
// navtk_generated.hpp to bind.
#define METHOD_OVERLOAD(CLASSNAME, METHODNAME, CASTTYPE, DOC_SUFFIX, ...) \
	.def(#METHODNAME,                                                     \
	     py::overload_cast<CASTTYPE>(&CLASSNAME::METHODNAME),             \
	     PROCESS_DOC(CLASSNAME##_##METHODNAME##DOC_SUFFIX),               \
	     __VA_ARGS__)
// Same as METHOD_OVERLOAD but for a method that accepts no parameters.
#define METHOD_OVERLOAD_VOID(CLASSNAME, METHODNAME, DOC_SUFFIX) \
	.def(#METHODNAME,                                           \
	     py::overload_cast<>(&CLASSNAME::METHODNAME),           \
	     PROCESS_DOC(CLASSNAME##_##METHODNAME##DOC_SUFFIX))
// Used to bind a property.
#define PROPERTY(CLASSNAME, PROPNAME) \
	.def_property(#PROPNAME, &CLASSNAME::get_##PROPNAME, &CLASSNAME::set_##PROPNAME)

// Used to bind a free-function.
#define FUNCTION(NAME, ...) m.def(#NAME, &NAME, PROCESS_DOC(NAME), __VA_ARGS__);
// Same as FUNCTION but for functions that don't accept parameters (some compilers give warnings
// when not all macro arguments are used).
#define FUNCTION_VOID(NAME) m.def(#NAME, &NAME, PROCESS_DOC(NAME));
// Same as FUNCTION, but for a templated function.
#define FUNCTIONT(NAME, TYPES, ...) m.def(#NAME, &NAME<TYPES>, PROCESS_DOC(NAME), __VA_ARGS__);
// Same as FUNCTION but used for overloaded functions.
#define FUNCTION_CAST(NAME, CAST, DOC_SUFFIX, ...) \
	m.def(#NAME, (CAST) & NAME, PROCESS_DOC(NAME##DOC_SUFFIX), __VA_ARGS__);
// Same as FUNCTION_CAST, but for a templated function.
#define FUNCTIONT_CAST(NAME, TEMPLATE_TYPE, TYPES, DOC_SUFFIX, ...) \
	m.def(#NAME,                                                    \
	      py::overload_cast<TYPES>(&NAME<TEMPLATE_TYPE>),           \
	      PROCESS_DOC(NAME##DOC_SUFFIX),                            \
	      __VA_ARGS__);
// Same as FUNCTION, but allows the namespace qualification to be passed as an argument. Normally
// each function would need a `using NS::function;` statement, but with this macro a spacespace
// alias can be defined and passed as an argument instead.
#define NAMESPACE_FUNCTION(NAME, NS, ...) m.def(#NAME, &NS::NAME, PROCESS_DOC(NAME), __VA_ARGS__);
// Same as NAMESPACE_FUNCTION but for functions that are multiply defined.
#define NAMESPACE_FUNCTION_OVERLOAD(NAME, NS, CASTTYPE, DOC_SUFFIX, ...) \
	m.def(#NAME,                                                         \
	      py::overload_cast<CASTTYPE>(&NS::NAME),                        \
	      PROCESS_DOC(NAME##DOC_SUFFIX),                                 \
	      __VA_ARGS__);
// Same as NAMESPACE_FUNCTION but for functions that don't accept parameters (some compilers give
// warnings when not all macro arguments are used).
#define NAMESPACE_FUNCTION_VOID(NAME, NS) m.def(#NAME, &NS::NAME, PROCESS_DOC(NAME));

// Used to bind an enum.
#define ENUM(NAME) py::enum_<NAME>(m, #NAME, PROCESS_DOC(NAME))
// Bind an enum that is defined on a class
#define ENUM_SCOPED(NAME, SCOPE, INSTANCE) \
	py::enum_<SCOPE::NAME>(INSTANCE, #NAME, PROCESS_DOC(SCOPE##_##NAME))
// Used to bind an enum value.
#define CHOICE(ENUMNAME, CHOICENAME) \
	.value(#CHOICENAME, ENUMNAME::CHOICENAME, PROCESS_DOC(ENUMNAME##_##CHOICENAME))
// Used to bind an enum value defined on a class.
#define CHOICE_SCOPED(ENUMNAME, SCOPE, CHOICENAME) \
	.value(#CHOICENAME, SCOPE::ENUMNAME::CHOICENAME, PROCESS_DOC(SCOPE##_##ENUMNAME##_##CHOICENAME))

// Used to bind a constexpr attribute.
#define ATTR(NAME) m.attr(#NAME) = py::cast(NAME);
// Used to bind a constexpr attribute defined on a class.
#define ATTR_SCOPED(NAME, SCOPE) m.attr(#NAME) = py::cast(SCOPE::NAME);

// Used to overload python __repr__ function, simulates binding of C++ operator<<
#define REPR(NAME)                         \
	.def("__repr__", [](const NAME& obj) { \
		std::ostringstream ss;             \
		ss << obj;                         \
		return ss.str();                   \
	})
#define REPR_NON_CONST(NAME)        \
	.def("__repr__", [](NAME obj) { \
		std::ostringstream ss;      \
		ss << obj;                  \
		return ss.str();            \
	})
// Same as REPR but for an enum. Requires overriding of both __str__ and __repr__ because pybind has
// default implementations of these functions for enums.
#define ENUM_REPR                                                                    \
	.def(                                                                            \
	    "__str__",                                                                   \
	    [](const py::object& obj) -> py::str { return py::detail::enum_name(obj); }, \
	    py::name("__str__"),                                                         \
	    py::is_method(m))                                                            \
	    .def(                                                                        \
	        "__repr__",                                                              \
	        [](const py::object& obj) -> py::str { return obj.attr("__str__")(); },  \
	        py::name("__repr__"),                                                    \
	        py::is_method(m));

// NOTE: No reasonable way to bind C++ assignment operator. Use constructor bindings instead
