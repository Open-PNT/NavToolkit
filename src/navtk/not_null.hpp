#pragma once

#include <algorithm>
#include <iosfwd>
#include <memory>
#include <stdexcept>
#include <type_traits>

#include <navtk/errors.hpp>

// Both forward declarations necessary here to allow a template class in one namespace to have a
// friend function in a separate namespace.
namespace navtk {
template <typename T>
class not_null;
}
namespace std {
template <typename T, typename U>
std::shared_ptr<T> dynamic_pointer_cast(const navtk::not_null<U>&) noexcept;
}

namespace navtk {

/**
 * A pointer enforced to never be equivalent to `nullptr`.
 */
template <typename T>
class not_null {
public:
	static_assert(std::is_convertible<decltype(std::declval<T>() == nullptr), bool>::value,
	              "T cannot be compared to nullptr");

	/**
	 * Lvalue constructor. Disabled if type `T` is `std::nullptr_t`.
	 *
	 * @param p A non-null pointer.
	 *
	 * @throw std::invalid_argument if \p p is equivalent to `nullptr` and the error mode is
	 * ErrorMode::DIE.
	 */
	template <typename = std::enable_if_t<!std::is_null_pointer<T>::value>>
	constexpr not_null(T p) : ptr(std::move(p)) {
		if (ptr == nullptr) log_or_throw<std::invalid_argument>("Pointer must be non-null.");
	}

	/**
	 * Rvalue constructor. Disabled if type `U` is not implicitly convertible to type `T`.
	 *
	 * @param p Reference to a non-null pointer.
	 *
	 * @throw std::invalid_argument if \p p references `nullptr` and the error mode is
	 * ErrorMode::DIE.
	 */
	template <typename U, typename = std::enable_if_t<std::is_convertible<U, T>::value>>
	constexpr not_null(U&& p) : ptr(std::forward<U>(p)) {
		if (ptr == nullptr) log_or_throw<std::invalid_argument>("Pointer must be non-null.");
	}

	/**
	 * Custom copy constructor defined to use get() . Disabled if type `U` of \p other is not
	 * implicitly convertible to type `T`.
	 *
	 * @param other Object to copy.
	 */
	template <typename U, typename = std::enable_if_t<std::is_convertible<U, T>::value>>
	constexpr not_null(const not_null<U>& other) : not_null(other.get()) {}

	/**
	 * Default copy constructor.
	 *
	 * @param other Object to copy.
	 */
	not_null(const not_null& other) = default;

	/**
	 * Default move constructor.
	 *
	 * @param other Object to move.
	 */
	not_null(not_null&& other) = default;

	/**
	 * Default copy assignment operator.
	 *
	 * @param other Object to copy.
	 *
	 * @return Reference to the newly copied object.
	 */
	not_null& operator=(const not_null& other) = default;

	/**
	 * Default move assignment operator.
	 *
	 * @param other Object to move.
	 *
	 * @return Reference to the newly moved object.
	 */
	not_null& operator=(not_null&& other) = default;

	/**
	 * Check and return the underlying pointer.
	 *
	 * @return The underlying pointer if it is still non-null. Returns a const reference if type `T`
	 * is not copy-constructible.
	 *
	 * @throw std::runtime_error if underlying pointer is equivalent to `nullptr` and the error mode
	 * is ErrorMode::DIE.
	 */
	constexpr std::conditional_t<std::is_copy_constructible<T>::value, T, const T&> get() const {
		if (ptr == nullptr) log_or_throw<std::runtime_error>("Held pointer is no longer non-null.");
		return ptr;
	}

	/**
	 * Define access operator to use get() .
	 *
	 * @return Underlying pointer after null check.
	 */
	constexpr operator T() const { return get(); }
	/**
	 * Define member dereference operator to use get() .
	 *
	 * @return Underlying pointer after null check.
	 */
	constexpr decltype(auto) operator->() const { return get(); }
	/**
	 * Define dereference operator to use get() .
	 *
	 * @return Dereferenced pointer after null check.
	 */
	constexpr decltype(auto) operator*() const { return *get(); }

	/**
	 * Deleted.
	 */
	void operator[](std::ptrdiff_t) const = delete;

	/**
	 * Deleted to prevent `nullptr` construction.
	 */
	not_null(std::nullptr_t) = delete;
	/**
	 * Deleted to prevent `nullptr` assignment.
	 */
	not_null& operator=(std::nullptr_t) = delete;

	/**
	 * Deleted to prevent invalidation.
	 */
	not_null& operator++() = delete;
	/**
	 * Deleted to prevent invalidation.
	 */
	not_null& operator--() = delete;
	/**
	 * Deleted to prevent invalidation.
	 */
	not_null operator++(int) = delete;
	/**
	 * Deleted to prevent invalidation.
	 */
	not_null operator--(int) = delete;
	/**
	 * Deleted to prevent invalidation.
	 */
	not_null& operator+=(std::ptrdiff_t) = delete;
	/**
	 * Deleted to prevent invalidation.
	 */
	not_null& operator-=(std::ptrdiff_t) = delete;

	/** Declare friend overload for `std::dynamic_pointer_cast` for ease of use with `not_null`. */
	template <typename U, typename V>
	friend std::shared_ptr<U> std::dynamic_pointer_cast(const navtk::not_null<V>&) noexcept;

private:
	T ptr;
};

/**
 * Helper function for creating not_null pointers.
 *
 * @param p Reference to a non-null pointer.
 *
 * @return An object of type `not_null<U>` which holds the pointer referenced by \p p . If the type
 * `T` is a reference type, type `U` is the type referred to by `T` with the topmost `const` and/or
 * `volatile` removed. Otherwise type `U` is type `T` with the topmost `const` and/or `volatile`
 * removed.
 */
template <class T>
auto make_not_null(T&& p) noexcept {
	return not_null<std::remove_cv_t<std::remove_reference_t<T>>>{std::forward<T>(p)};
}

/**
 * Define `std::ostream` stream operator to use not_null::get().
 * @param os A `std::ostream` reference.
 * @param val The not_null value.
 * @return A `std::ostream` reference.
 */
template <class T>
std::ostream& operator<<(std::ostream& os, const not_null<T>& val) {
	os << val.get();
	return os;
}

/**
 * Define not_null comparison operators to use not_null::get().
 * @param lhs The is-equal comparison left hand side.
 * @param rhs The is-equal comparison right hand side.
 * @return The result of the is-equal comparison.
 */
template <class T, class U>
auto operator==(const not_null<T>& lhs, const not_null<U>& rhs) noexcept(
    noexcept(lhs.get() == rhs.get())) -> decltype(lhs.get() == rhs.get()) {
	return lhs.get() == rhs.get();
}
/**
 * Define not_null comparison operators to use not_null::get().
 * @param lhs The not-equal comparison left hand side.
 * @param rhs The not-equal comparison right hand side.
 * @return The result of not-equal comparison.
 */
template <class T, class U>
auto operator!=(const not_null<T>& lhs, const not_null<U>& rhs) noexcept(
    noexcept(lhs.get() != rhs.get())) -> decltype(lhs.get() != rhs.get()) {
	return lhs.get() != rhs.get();
}
/**
 * Define not_null comparison operators to use not_null::get().
 * @param lhs The less-than comparison left hand side.
 * @param rhs The less-than comparison right hand side.
 * @return The result of the less-than comparison.
 */
template <class T, class U>
auto operator<(const not_null<T>& lhs, const not_null<U>& rhs) noexcept(
    noexcept(lhs.get() < rhs.get())) -> decltype(lhs.get() < rhs.get()) {
	return lhs.get() < rhs.get();
}
/**
 * Define not_null comparison operators to use not_null::get().
 * @param lhs The less-than-or-equal comparison left hand side.
 * @param rhs The less-than-or-equal comparison right hand side.
 * @return The result of the less-than-or-equal comparison.
 */
template <class T, class U>
auto operator<=(const not_null<T>& lhs, const not_null<U>& rhs) noexcept(
    noexcept(lhs.get() <= rhs.get())) -> decltype(lhs.get() <= rhs.get()) {
	return lhs.get() <= rhs.get();
}
/**
 * Define not_null comparison operators to use not_null::get().
 * @param lhs The greater-than comparison left hand side.
 * @param rhs The greater-than comparison right hand side.
 * @return The result of the greater-than comparison.
 */
template <class T, class U>
auto operator>(const not_null<T>& lhs, const not_null<U>& rhs) noexcept(
    noexcept(lhs.get() > rhs.get())) -> decltype(lhs.get() > rhs.get()) {
	return lhs.get() > rhs.get();
}
/**
 * Define not_null comparison operators to use not_null::get().
 * @param lhs The greater-than-or-equal comparison left hand side.
 * @param rhs The greater-than-or-equal comparison right hand side.
 * @return The result of the greater-than-or-equal comparison.
 */
template <class T, class U>
auto operator>=(const not_null<T>& lhs, const not_null<U>& rhs) noexcept(
    noexcept(lhs.get() >= rhs.get())) -> decltype(lhs.get() >= rhs.get()) {
	return lhs.get() >= rhs.get();
}

#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND
// Disable not_null pointer arithmetic addition/subtraction.
template <class T, class U>
std::ptrdiff_t operator-(const not_null<T>&, const not_null<U>&) = delete;
template <class T>
not_null<T> operator-(const not_null<T>&, std::ptrdiff_t) = delete;
template <class T>
not_null<T> operator+(const not_null<T>&, std::ptrdiff_t) = delete;
template <class T>
not_null<T> operator+(std::ptrdiff_t, const not_null<T>&) = delete;
#endif

}  // namespace navtk

namespace std {
// Implementation of overload for `std::dynamic_pointer_cast` declared in `navtk::not_null`.
template <typename T, typename U>
std::shared_ptr<T> dynamic_pointer_cast(const navtk::not_null<U>& p) noexcept {
	return dynamic_pointer_cast<T>(p.ptr);
}
}  // namespace std
