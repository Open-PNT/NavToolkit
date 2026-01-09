#pragma once

#include <deque>

#include <navtk/utils/RingBuffer.hpp>
#include <navtk/utils/algorithm.hpp>

namespace navtk {
namespace utils {

/**
 * Ordered is a type of sequence container adaptor designed to keep elements in order.
 * Elements are ordered in the container on insertion.
 * Ordered is optimized for insertion at the back of the container (similar to a queue).
 * Ordered can be instantiated with different underlying containers.
 * The underlying container may be any of the standard container class templates or some other
 * specifically designed container class. The container shall be accessible through random access
 * iterators and support the following operations:
 *   empty()
 *   size()
 *   front()
 *   back()
 *   pop_back()
 *   erase()
 */
template <typename Data,
          typename Container,
          typename Compare,
          typename SortIterator,
          typename SortCompare>
class Ordered {
public:
	/**
	 * Type of elements stored in the container
	 */
	using value_type = Data;
	/**
	 * Type of container `iterator`
	 */
	using iterator = typename Container::iterator;
	/**
	 * Type of container `const_iterator`
	 */
	using const_iterator = typename Container::const_iterator;
	/**
	 * Type of container `reverse_iterator`
	 */
	using reverse_iterator = std::reverse_iterator<iterator>;
	/**
	 * Type of container `const_reverse_iterator`
	 */
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;
	/**
	 * Type of pair of container `const_iterator`
	 */
	using const_iterator_pair = std::pair<const_iterator, const_iterator>;
	/**
	 * Type of pair of container `const_reverse_iterator`
	 */
	using const_reverse_iterator_pair = std::pair<const_reverse_iterator, const_reverse_iterator>;

	/**
	 * @return Number of elements in the container
	 */
	std::size_t size() const { return buffer.size(); }

	/**
	 * @return `true` if the container is empty
	 */
	bool empty() const { return buffer.empty(); }

	/**
	 * @return `true` if the container is full
	 */
	bool full() const { return buffer.full(); }

	/**
	 * Pops an element from the front of the container
	 */
	void pop_front() { buffer.pop_front(); }

	/**
	 * Pops an element from the back of the container
	 */
	void pop_back() { buffer.pop_back(); }

	/**
	 * @return The element at the front of the container (by const reference)
	 */
	const Data& front() const { return buffer.front(); }

	/**
	 * @return The element at the back of the container (by const reference)
	 */
	const Data& back() const { return buffer.back(); }

	/**
	 * Erases elements in the range [first, last)
	 * @param first the first element to erase
	 * @param last one past the last element to erase
	 * @return The element after the last element erased
	 */
	iterator erase(const_iterator first, const_iterator last) { return buffer.erase(first, last); }

	/**
	 * @return Iterator to the element at the beginning of the container
	 */
	iterator begin() { return buffer.begin(); }

	/**
	 * @return Iterator to the element at the end of the container
	 */
	iterator end() { return buffer.end(); }

	/**
	 * @return Reverse iterator to the element at the reverse beginning of the container
	 */
	reverse_iterator rbegin() { return buffer.rbegin(); }

	/**
	 * @return Reverse iterator to the element at the reverse end of the container
	 */
	reverse_iterator rend() { return buffer.rend(); }

	/**
	 * @return Constant iterator to the element at the beginning of the container
	 */
	const_iterator begin() const { return buffer.begin(); }

	/**
	 * @return Constant iterator to the element at the end of the container
	 */
	const_iterator end() const { return buffer.end(); }

	/**
	 * @return Constant reverse iterator to the element at the reverse beginning of the container
	 */
	const_reverse_iterator rbegin() const { return buffer.rbegin(); }

	/**
	 * @return Constant reverse iterator to the element at the reverse end of the container
	 */
	const_reverse_iterator rend() const { return buffer.rend(); }

	/**
	 * @return Constant iterator to the element at the beginning of the container
	 */
	const_iterator cbegin() const { return buffer.cbegin(); }

	/**
	 * @return Constant iterator to the element at the end of the container
	 */
	const_iterator cend() const { return buffer.cend(); }

	/**
	 * @return Constant reverse iterator to the element at the reverse beginning of the container
	 */
	const_reverse_iterator crbegin() const { return buffer.crbegin(); }

	/**
	 * @return Constant reverse iterator to the element at the reverse end of the container
	 */
	const_reverse_iterator crend() const { return buffer.crend(); }

	/**
	 * Inserts data in the container.
	 * @param data to insert
	 */
	void insert(const Data& data) {
		Compare compare;
		if (is_insert_ok(data, buffer, capacity)) {
			pre_insert(data, buffer, capacity);
			bool sort_it = (!buffer.empty()) && (compare(data, buffer.back()));
			// push data on the back of the buffer
			buffer.push_back(data);
			if (sort_it) {
				// minor efficiency if inserted data is very near the end
				auto begin_sort = std::upper_bound(buffer.begin(), buffer.end() - 1, data, compare);
				std::stable_sort(begin_sort, buffer.end(), compare);
			}
			post_insert(data, buffer, capacity);
		}
	}

	/**
	 * Gets elements in the range [t0, t1].
	 * @param t0 beginning of range
	 * @param t1 end of range
	 * @return Pair of constant iterators representing the first element in the specified range
	 * and one past the last element in the range, [first,last).
	 */
	const_iterator_pair get_in_range(typename SortIterator::value_type t0,
	                                 typename SortIterator::value_type t1) const {
		auto range =
		    in_range.get(SortIterator{buffer.cbegin()}, SortIterator{buffer.cend()}, t0, t1);
		return std::make_pair(const_iterator{range.first}, const_iterator{range.second});
	}

	/**
	 * Gets the nearest elements to `t`.
	 * @param t element
	 * @return Pair of iterators representing the nearest elements to `t`. If `t` is before
	 * all elements then return will be {end, begin}; if `t` is after all elements return will
	 * be {end - 1, end}. If t is exactly matched, both iterators will be to the last such element
	 * that matched `t`. Finally, if `t` is between two elements then the return will be the
	 * elements before and after `t`.
	 */
	const_iterator_pair get_nearest_neighbors(const typename SortIterator::value_type& t) const {
		auto neighbors =
		    nearest_neighbors.get(SortIterator{buffer.cbegin()}, SortIterator{buffer.cend()}, t);
		return std::make_pair(const_iterator{neighbors.first}, const_iterator{neighbors.second});
	}

protected:
	/**
	 * Protected constructor can only be called from derived classes
	 * @param capacity size (number of elements) limit of underlying storage
	 * @param initial_capacity size (number of elements) to pre-allocate
	 */
	Ordered(std::size_t capacity, std::size_t initial_capacity)
	    : capacity(capacity), buffer(Container(initial_capacity)) {}
	virtual ~Ordered() = default;

	/**
	 * Copy constructor. Defaulted.
	 */
	Ordered(const Ordered&) = default;

	/**
	 * Copy assignment. Defaulted.
	 * @return `*this`
	 */
	Ordered& operator=(const Ordered&) = default;

	/**
	 * Move constructor. Defaulted.
	 */
	Ordered(Ordered&&) = default;

	/**
	 * Move assignment. Defaulted.
	 * @return `*this`
	 */
	Ordered& operator=(Ordered&&) = default;

private:
	std::size_t capacity;
	Container buffer;
	virtual bool is_insert_ok(const Data&, const Container&, std::size_t) const { return true; }
	virtual void pre_insert(const Data&, Container&, std::size_t) {}
	virtual void post_insert(const Data&, Container&, std::size_t) {}

	InRange<SortIterator, SortCompare> in_range;
	NearestNeighbors<SortIterator, SortCompare> nearest_neighbors;
};

/**
 * OrderedRing is an Ordered container that uses a RingBuffer as its underlying container.
 */
template <typename Data,
          typename Compare      = std::less<Data>,
          typename SortIterator = typename RingBuffer<Data>::const_iterator,
          typename SortCompare  = std::less<Data>>
class OrderedRing final : public Ordered<Data,
                                         typename navtk::utils::RingBuffer<Data>,
                                         Compare,
                                         SortIterator,
                                         SortCompare> {
public:
	/**
	 * Constructor
	 * @param capacity size (number of elements) in RingBuffer
	 */
	explicit OrderedRing(std::size_t capacity)
	    : Ordered<Data, RingBuffer<Data>, Compare, SortIterator, SortCompare>(capacity, capacity) {}

private:
	bool is_insert_ok(const Data& data,
	                  const RingBuffer<Data>& buffer,
	                  std::size_t capacity) const override {
		// if the buffer is full and this element won't fit in order, return false
		return !(buffer.size() == capacity && Compare()(data, buffer.front()));
	}
};

/**
 * OrderedDeque is an Ordered container that uses a std::deque as its underlying container.
 * While a deque does not typically have a capacity (beside `max_size`), OrderedDeque does
 * limit the deque capacity. The storage is not reserved up front.
 */
template <typename Data,
          typename Compare      = std::less<Data>,
          typename SortIterator = typename std::deque<Data>::const_iterator,
          typename SortCompare  = std::less<Data>>
class OrderedDeque final
    : public Ordered<Data, typename std::deque<Data>, Compare, SortIterator, SortCompare> {
public:
	/**
	 * Constructor
	 * @param capacity size (number of elements) to limit deque
	 */
	explicit OrderedDeque(std::size_t capacity)
	    : Ordered<Data, std::deque<Data>, Compare, SortIterator, SortCompare>(capacity, 0) {}

private:
	void post_insert(const Data&, std::deque<Data>& buffer, std::size_t capacity) override {
		// if the buffer is over capacity remove an element from the front
		if (buffer.size() > capacity) {
			buffer.pop_front();
		}
	}
};

}  // namespace utils
}  // namespace navtk
