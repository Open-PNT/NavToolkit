#pragma once

#include <string>
#include <type_traits>
#include <typeinfo>

#include <navtk/tensors.hpp>

// To support GCC's oddball name mangling
#ifdef __GNUG__
#	include <cxxabi.h>
#	include <cstdlib>
#endif

namespace navtk {
namespace utils {

/**
 * Returns a `std::string` representation of the given Matrix, optionally
 * prefixed by a given type declaration.
 *
 * Returned string is both human-readable, and copypasteable as source.
 *
 * @param matrix The Matrix to inspect.
 * @param decl Prefix string of output.
 *
 * @return A string declaration for the Matrix.
 */
std::string repr(const Matrix& matrix, const std::string& decl);

/**
 * Returns a `std::string` representation of the given Matrix, prefixed by
 * one of Matrix or Vector, based on its shape.
 *
 * Returned string is both human-readable, and copypasteable as source.
 *
 * @param matrix The Matrix to inspect.
 *
 * @return A string declaration for the Matrix.
 */
std::string repr(const Matrix& matrix);

/**
 * Returns a `std::string` representation of the given xtensor expression.
 *
 * Unlike the overloads for concrete types, this version deliberately
 * returns output that is not valid source code. This is to help us
 * notice when values we expect to be concrete are actually lazy-eval.
 *
 * @param expr The xtensor expression to inspect.
 *
 * @return A string representation of the matrix, prefixed by "(expr)".
 */
template <class E>
std::string repr(const xt::xexpression<E>& expr) {
	return repr(expr, "(expr)");
}

/**
 * Returns a human-readable description of the difference between two
 * matrices. Returns an empty string if `xt::allclose(before, after, rtol, atol)`.
 *
 * In general, most code should use `xt::allclose` or `xt::isclose` instead.
 * This function exists primarily for use from within gdb and unit tests.
 *
 * When possible, the output will also be valid c++ code, assigning the
 * value of \p before to \p after , and then making changes to match the value
 * of \p after . For example,
 * `diff("in", "out", xt::eye(3), xt::eye(3) * 2)` will yield:
 *
 * ```
 * out = in;
 * out(0, 0) = 2;  // in(0, 0) == 1
 * out(1, 1) = 2;  // in(1, 1) == 1
 * out(2, 2) = 2;  // in(2, 2) == 1
 * ```
 *
 * Special cases exist for transposes and mismatched dimensionality.
 *
 * @param before_name A name to use in describing the `before` matrix.
 * @param after_name A name to use in describing the `after` matrix.
 * @param before Matrix supplying 'expected' data.
 * @param after Matrix supplying 'actual' data.
 * @param rtol `xt::isclose`'s relative tolerance parameter (default 1e-05).
 * @param atol `xt::isclose`'s absolute tolerance parameter (default 1e-08).
 *
 * @return Human-readable description of the difference between two matrices.
 */
std::string diff(const std::string& before_name,
                 const std::string& after_name,
                 const Matrix& before,
                 const Matrix& after,
                 double rtol = 1e-05,
                 double atol = 1e-08);

/**
 * Returns a human-readable description of the difference between two
 * matrices.
 *
 * This is a convenience overload which supplies hardcoded names "before"
 * and "after" to the \p before and \p after matrices.
 * `diff(x, y, r, a) == diff("before", "after", x, y, r, a)`.
 *
 * See the other overload for a more detailed description.
 *
 * @param before The 'before' matrix to be compared.
 * @param after The 'after' matrix to be compared.
 * @param rtol `xt::isclose`'s relative tolerance parameter (default 1e-05).
 * @param atol `xt::isclose`'s absolute tolerance parameter (default 1e-08).
 *
 * @return Human-readable description of the difference between two matrices.
 */
std::string diff(const Matrix& before,
                 const Matrix& after,
                 double rtol = 1e-05,
                 double atol = 1e-08);


/**
 * Return a human-readable string describing the compile-time type name of the given template
 * parameter. Useful for constructing error messages about template parameters.
 *
 * @tparam T Type to identify
 * @return The name of the type (exact format is compiler-specific).
 */
template <typename T>
std::string identify_type() {
#ifdef __GNUG__
	int err;
	typedef std::unique_ptr<char[], decltype(&std::free)> CStrPtr;
	CStrPtr buffer{__cxxabiv1::__cxa_demangle(typeid(T).name(), nullptr, 0, &err), &std::free};
	std::string out{err ? "<UNKNOWN TYPE>" : buffer.get()};
#else
	std::string out{typeid(T).name()};
#endif
	if (std::is_rvalue_reference<T>::value)
		return out + "&&";
	else if (std::is_reference<T>::value)
		return out + "&";
	return out;
}


}  // namespace utils
}  // namespace navtk
