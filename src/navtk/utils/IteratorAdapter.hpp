#pragma once

#include <cstddef>
#include <iterator>
#include <type_traits>

namespace navtk {
namespace utils {

/**
 * IteratorAdapter wraps an Iterator but returns the type specified
 * by T (using the function get_value) whenever the IteratorAdapter
 * is dereferenced.
 */
template <typename Iterator, typename T, T (*get_value)(Iterator)>
class IteratorAdapter {
private:
	Iterator it;

public:
	/**
	 * Type when taking the difference between two iterators
	 */
	using difference_type = std::ptrdiff_t;
	/**
	 * Type of elements pointed to by the iterator
	 */
	using value_type = std::remove_const_t<T>;
	/**
	 * Type to represent a pointer to an element pointed by the iterator
	 */
	using pointer = value_type*;
	/**
	 * Type to represent a reference to an element pointed by the iterator
	 */
	using reference = value_type&;
	/**
	 * Category to which the iterator belongs
	 */
	using iterator_category = std::random_access_iterator_tag;

	/**
	 * Explicit constructor from an Iterator
	 * @param it iterator to wrap
	 */
	explicit IteratorAdapter(Iterator it) : it(it) {}

	/**
	 * Implicit conversion to Iterator
	 * @return Wrapped iterator
	 */
	operator Iterator() { return it; }

	/**
	 * Equal operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterators are equal
	 */
	bool operator==(const IteratorAdapter& rhs) { return it == rhs.it; }

	/**
	 * Not equal operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterators are not equal
	 */
	bool operator!=(const IteratorAdapter& rhs) { return !(it == rhs.it); }

	/**
	 * Prefix increment operator
	 * @return `*this`
	 */
	IteratorAdapter& operator++() {
		++it;
		return *this;
	}

	/**
	 * Postfix increment operator
	 * @return Iterator (by value)
	 */
	IteratorAdapter operator++(int) {
		auto t = *this;
		++it;
		return t;
	}

	/**
	 * Addition operator
	 * @param amount to add
	 * @return Iterator (by value)
	 */
	IteratorAdapter operator+(int amount) const { return IteratorAdapter(it + amount); }

	/**
	 * Subtraction operator
	 * @param amount to subtract
	 * @return Iterator (by value)
	 */
	IteratorAdapter operator-(int amount) const { return IteratorAdapter(it - amount); }

	/**
	 * Prefix decrement operator
	 * @return `*this`
	 */
	IteratorAdapter& operator--() {
		--it;
		return *this;
	}

	/**
	 * Postfix decrement operator
	 * @return Iterator (by value)
	 */
	IteratorAdapter operator--(int) {
		auto t = *this;
		--it;
		return t;
	}

	/**
	 * Difference operator
	 * @param rhs iterator to compare to
	 * @return Difference between iterator and rhs (iterator - rhs)
	 */
	std::ptrdiff_t operator-(IteratorAdapter const& rhs) const { return it - rhs.it; }

	/**
	 * Addition assignment operator
	 * @param amount to add to iterator
	 * @return Modified iterator (by reference)
	 */
	IteratorAdapter& operator+=(int amount) {
		it += amount;
		return *this;
	}

	/**
	 * Subtraction assignment operator
	 * @param amount to subtract from iterator
	 * @return Modified iterator (by reference)
	 */
	IteratorAdapter& operator-=(int amount) {
		it -= amount;
		return *this;
	}

	/**
	 * Less than operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterator is less than rhs (iterator < rhs)
	 */
	bool operator<(IteratorAdapter const& rhs) const { return it < rhs.it; }

	/**
	 * Less than or equal to operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterator is less than or equal to rhs (iterator <= rhs)
	 */
	bool operator<=(IteratorAdapter const& rhs) const { return it <= rhs.it; }

	/**
	 * Greater than operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterator is greater than rhs (iterator > rhs)
	 */
	bool operator>(IteratorAdapter const& rhs) const { return it > rhs.it; }

	/**
	 * Greater than or equal to operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterator is greater than or equal to rhs (iterator >= rhs)
	 */
	bool operator>=(IteratorAdapter const& rhs) const { return it >= rhs.it; }

	/**
	 * @param index of element from iterator
	 * @return Type T from element at index (using get_value())
	 */
	auto operator[](int index) const { return get_value(it + index); }

	/**
	 * @return Type T from element pointed to (using get_value())
	 */
	auto operator*() const { return get_value(it); }

	/**
	 * @return Address of type T from element pointed to (using get_value())
	 */
	auto operator->() const { return &get_value(it); }
};

}  // namespace utils
}  // namespace navtk
