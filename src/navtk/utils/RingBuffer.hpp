#pragma once

#include <iterator>
#include <stdexcept>

#include <navtk/errors.hpp>

namespace navtk {
namespace utils {

/**
 * Forward declaration
 */
template <typename T>
class RingBufferIterator;

/**
 * Ring (or circular) buffers are sequence containers with static sizes.
 * They allow for the individual elements to be accessed directly through random access iterators.
 * They allow for insertion/deletion at either end of the buffer. Inserting to a full buffer will
 * pop an element off the other end.
 * Elements are not guaranteed to be stored in contiguous memory (due to the wrap around nature of
 * the underlying buffer).
 */
template <typename T>
class RingBuffer {
public:
	/**
	 * Type of elements stored in the ring buffer
	 */
	using value_type = T;
	/**
	 * Type of `iterator` to ring buffer element
	 */
	using iterator = RingBufferIterator<T>;
	/**
	 * Type of `const_iterator` to ring buffer element
	 */
	using const_iterator = RingBufferIterator<const T>;
	/**
	 * Type of `reverse_iterator` to ring buffer element
	 */
	using reverse_iterator = std::reverse_iterator<iterator>;
	/**
	 * Type of `const_reverse_iterator` to ring buffer element
	 */
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	/**
	 * Default constructor
	 * @param capacity number of elements the ring buffer can hold
	 */
	explicit RingBuffer(std::size_t capacity) : capacity(capacity) {
		if (capacity == 0) {
			throw std::invalid_argument("Buffer size cannot be 0.");
		}
		// Workaround for GCC bug https://gcc.gnu.org/bugzilla/show_bug.cgi?id=87544
		// See issue navtk#895 for details.
		if (capacity >= std::size_t((std::numeric_limits<std::ptrdiff_t>::max)())) {
			throw std::invalid_argument(
			    "Buffer size cannot be greater than or equal to "
			    "std::size_t((std::numeric_limits<std::ptrdiff_t>::max)())");
		}
		buffer = new T[capacity];
	}

	/**
	 * Default destructor
	 */
	~RingBuffer() { delete[] buffer; }


	/**
	 * Copy constructor
	 * @param other buffer to copy
	 */
	RingBuffer(const RingBuffer& other)
	    : capacity(other.capacity), buffer(new T[other.capacity]()) {
		buffer_size = other.buffer_size;
		std::copy(other.begin(), other.end(), buffer);
	}

	/**
	 * Copy assignment operator
	 * @param other buffer to copy
	 * @return Buffer (by reference)
	 */
	RingBuffer& operator=(const RingBuffer& other) {
		if (this != &other) {
			// Free the existing resource.
			delete[] buffer;

			capacity    = other.capacity;
			buffer      = new T[capacity];
			buffer_size = other.buffer_size;
			std::copy(other.begin(), other.end(), buffer);
		}
		return *this;
	}

	/**
	 * Move constructor
	 * @param other buffer to move from
	 */
	RingBuffer(RingBuffer&& other) : capacity(0), buffer(nullptr) { *this = std::move(other); }

	/**
	 * Move assignment operator
	 * @param other buffer to move from
	 * @return Buffer (by reference)
	 */
	RingBuffer& operator=(RingBuffer&& other) {
		if (this != &other) {
			// Free the existing resource.
			delete[] buffer;

			// Copy the data pointer and its length from the
			// source object.
			capacity    = other.capacity;
			buffer      = other.buffer;
			buffer_size = other.buffer_size;
			begin_index = other.begin_index;

			// Release the data pointer from the source object so that
			// the destructor does not free the memory multiple times.
			other.capacity    = 0;
			other.buffer      = nullptr;
			other.buffer_size = 0;
			other.begin_index = 0;
		}
		return *this;
	}

	/**
	 * @return The number of elements in the container.
	 */
	std::size_t size() const { return buffer_size; }

	/**
	 * @return `true` if the container size is 0, `false` otherwise.
	 */
	bool empty() const { return buffer_size == 0; }

	/**
	 * @return `true` if the container is at capacity.
	 */
	bool full() const { return buffer_size == capacity; }

	/**
	 * @return Reference to the first element in the container.
	 */
	T& front() { return first_value(); }

	/**
	 * @return Constant reference to the first element in the container.
	 */
	const T& front() const { return first_value(); }

	/**
	 * @return Reference to the last element in the container.
	 */
	T& back() { return last_value(); }

	/**
	 * @return Constant reference to the last element in the container.
	 */
	const T& back() const { return last_value(); }

	/**
	 * Removes the first element in the container, effectively reducing its size by one.
	 * This destroys the removed element.
	 */
	void pop_front() {
		if (buffer_size == 0) return;
		T t = std::move(first_value());
		(void)t;
		++begin_index %= capacity;
		--buffer_size;
	}

	/**
	 * Removes the last element in the container, effectively reducing its size by one.
	 * This destroys the removed element.
	 */
	void pop_back() {
		if (buffer_size == 0) return;
		T t = std::move(last_value());
		(void)t;
		--buffer_size;
	}

	/**
	 * Inserts a new element at the beginning of the container, right before its current first
	 * element. The content of value is copied to the inserted element.
	 * This effectively increases the container size by one unless the container is full in
	 * which case the element at the back of the container is removed and destroyed.
	 * @param value to push
	 */
	void push_front(const T& value) {
		if (buffer_size < capacity) {
			++buffer_size;
		}
		begin_index   = (begin_index + capacity - 1) % capacity;
		first_value() = value;
	}

	/**
	 * Inserts a new element at the beginning of the container, right before its current first
	 * element. The content of value is moved to the inserted element.
	 * This effectively increases the container size by one unless the container is full in
	 * which case the element at the back of the container is removed and destroyed.
	 * @param value to push
	 */
	void push_front(T&& value) {
		if (buffer_size < capacity) {
			++buffer_size;
		}
		begin_index   = (begin_index + capacity - 1) % capacity;
		first_value() = std::move(value);
	}

	/**
	 * Inserts a new element at the end of the container, right after its current last
	 * element. The content of value is copied to the inserted element.
	 * This effectively increases the container size by one unless the container is full in
	 * which case the element at the beginning of the container is removed and destroyed.
	 * @param value to push
	 */
	void push_back(const T& value) {
		if (buffer_size < capacity) {
			++buffer_size;
		} else {
			++begin_index %= capacity;
		}
		last_value() = value;
	}

	/**
	 * Inserts a new element at the end of the container, right after its current last
	 * element. The content of value is moved to the inserted element.
	 * This effectively increases the container size by one unless the container is full in
	 * which case the element at the beginning of the container is removed and destroyed.
	 * @param value to push
	 */
	void push_back(T&& value) {
		if (buffer_size < capacity) {
			++buffer_size;
		} else {
			++begin_index %= capacity;
		}
		last_value() = std::move(value);
	}

	/**
	 * Removes from the container the range of elements in [first, last).
	 * The range includes all the elements between first and last, including the
	 * element pointed by first but not the one pointed by last.
	 * This effectively reduces the container size by the number of elements removed,
	 * which are destroyed.
	 * Return Value: An iterator pointing to the location of the element that followed
	 * the last element erased by the function call. This is the container end if the
	 * operation erased the last element in the container.
	 * Exception: Only supports erasing elements from the beginning or end of the
	 * container. If the specified range is in the middle of the container an
	 * exception will be thrown.
	 * @param first the first element to remove
	 * @param last one past the last element to remove
	 * @return Element after the last element removed
	 */
	iterator erase(const_iterator first, const_iterator last) {
		auto number_of_elements_to_pop = last - first;
		iterator return_val            = end();
		if (first == cbegin()) {
			for (auto i = 0; i < number_of_elements_to_pop; i++) {
				pop_front();
				return_val = begin();
			}
		} else if (last == cend()) {
			for (auto i = 0; i < number_of_elements_to_pop; i++) {
				pop_back();
				return_val = end();
			}
		} else {
			log_or_throw<std::runtime_error>(
			    "unimplemented operation erase from middle of RingBuffer");
		}
		return return_val;
	}

	/**
	 * @return An iterator pointing to the first element in the container.
	 *
	 * If the container is empty, the returned iterator value shall not be dereferenced.
	 */
	iterator begin() { return iterator(buffer, begin_index, capacity); }

	/**
	 * @return A constant iterator pointing to the first element in the container.
	 *
	 * If the container is empty, the returned iterator value shall not be dereferenced.
	 */
	const_iterator begin() const { return const_iterator(buffer, begin_index, capacity); }

	/**
	 * @return A constant iterator pointing to the first element in the container.
	 *
	 * If the container is empty, the returned iterator value shall not be dereferenced.
	 */
	const_iterator cbegin() const { return begin(); }

	/**
	 * @return An iterator pointing to the past-the-end element in the container.
	 *
	 * The past-the-end element is the theoretical element that would follow the last element in
	 * the container. It does not point to any element, and thus shall not be dereferenced.
	 */
	iterator end() { return iterator(buffer, begin_index + buffer_size, capacity); }

	/**
	 * @return A constant iterator pointing to the past-the-end element in the container.
	 *
	 * The past-the-end element is the theoretical element that would follow the last element in
	 * the container. It does not point to any element, and thus shall not be dereferenced.
	 */
	const_iterator end() const {
		return const_iterator(buffer, begin_index + buffer_size, capacity);
	}

	/**
	 * @return A constant iterator pointing to the past-the-end element in the container.
	 *
	 * The past-the-end element is the theoretical element that would follow the last element in
	 * the container. It does not point to any element, and thus shall not be dereferenced.
	 */
	const_iterator cend() const { return end(); }

	/**
	 * @return A reverse iterator pointing to the last element in the container.
	 *
	 * Reverse iterators iterate backwards: increasing them moves them towards the beginning of
	 * the container.
	 * rbegin points to the element right before the one that would be pointed to by member end.
	 * If the container is empty, the returned iterator value shall not be dereferenced.
	 */
	reverse_iterator rbegin() { return reverse_iterator(end()); }

	/**
	 * @return A constant reverse iterator pointing to the last element in the container.
	 *
	 * Reverse iterators iterate backwards: increasing them moves them towards the beginning of
	 * the container.
	 * rbegin points to the element right before the one that would be pointed to by member end.
	 * If the container is empty, the returned iterator value shall not be dereferenced.
	 */
	const_reverse_iterator rbegin() const { return const_reverse_iterator(end()); }

	/**
	 * @return A constant reverse iterator pointing to the last element in the container.
	 *
	 * Reverse iterators iterate backwards: increasing them moves them towards the beginning of
	 * the container.
	 * rbegin points to the element right before the one that would be pointed to by member end.
	 * If the container is empty, the returned iterator value shall not be dereferenced.
	 */
	const_reverse_iterator crbegin() const { return rbegin(); }

	/**
	 * @return A reverse iterator pointing to the before-the-beginning element in the container.
	 *
	 * The before-the-beginning element is the theoretical element that would preceed the first
	 * element in the container. It does not point to any element, and thus shall not be
	 * dereferenced.
	 */
	reverse_iterator rend() { return reverse_iterator(begin()); }

	/**
	 * @return A constant reverse iterator pointing to the before-the-beginning element in the
	 * container.
	 *
	 * The before-the-beginning element is the theoretical element that would preceed the first
	 * element in the container. It does not point to any element, and thus shall not be
	 * dereferenced.
	 */
	const_reverse_iterator rend() const { return const_reverse_iterator(begin()); }

	/**
	 * @return A constant reverse iterator pointing to the before-the-beginning element in the
	 * container.
	 *
	 * The before-the-beginning element is the theoretical element that would preceed the first
	 * element in the container. It does not point to any element, and thus shall not be
	 * dereferenced.
	 */
	const_reverse_iterator crend() const { return rend(); }

private:
	std::size_t capacity;
	T* buffer;
	std::size_t buffer_size = 0;
	std::size_t begin_index = 0;

	T& first_value() { return buffer[begin_index]; }
	const T& first_value() const { return buffer[begin_index]; }
	T& last_value() { return buffer[(begin_index + buffer_size - 1) % capacity]; }
	const T& last_value() const { return buffer[(begin_index + buffer_size - 1) % capacity]; }
};

/**
 * RingBufferIterator is the random access iterator class used with the RingBuffer container.
 */
template <typename T>
class RingBufferIterator {
public:
	/**
	 * Type when taking the difference between two iterators
	 */
	using difference_type = ptrdiff_t;
	/**
	 * Type of elements pointed to by the iterator
	 */
	using value_type = T;
	/**
	 * Type to represent a pointer to an element pointed by the iterator
	 */
	using pointer = T*;
	/**
	 * Type to represent a reference to an element pointed by the iterator
	 */
	using reference = T&;
	/**
	 * Category to which the iterator belongs
	 */
	using iterator_category = std::random_access_iterator_tag;

	/**
	 * Constructor
	 * @param buffer the storage for the ring buffer
	 * @param offset iterator value used to index the buffer
	 * @param capacity size (number of elements) of buffer
	 */
	RingBufferIterator(T* buffer = new T[1](), std::size_t offset = 0, std::size_t capacity = 1)
	    : capacity(capacity), buffer(buffer), offset(offset) {}

	/**
	 * Equal operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterators are equal
	 */
	bool operator==(const RingBufferIterator& rhs) {
		return rhs.buffer == buffer && rhs.offset == offset;
	}

	/**
	 * Not equal operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterators are not equal
	 */
	bool operator!=(const RingBufferIterator& rhs) { return !(*this == rhs); }

	/**
	 * Prefix increment operator
	 * @return `*this`
	 */
	RingBufferIterator& operator++() {
		++offset;
		return *this;
	}

	/**
	 * Postfix increment operator
	 * @return Iterator (by value)
	 */
	RingBufferIterator operator++(int) {
		auto t = *this;
		++offset;
		return t;
	}

	/**
	 * Addition operator
	 * @param amount to add
	 * @return Iterator (by value)
	 */
	RingBufferIterator operator+(int amount) const {
		return RingBufferIterator(buffer, offset + amount, capacity);
	}

	/**
	 * Subtraction operator
	 * @param amount to subtract
	 * @return Iterator (by value)
	 */
	RingBufferIterator operator-(int amount) const {
		return RingBufferIterator(buffer, offset - amount, capacity);
	}

	/**
	 * Prefix decrement operator
	 * @return `*this`
	 */
	RingBufferIterator& operator--() {
		--offset;
		return *this;
	}

	/**
	 * Postfix decrement operator
	 * @return Iterator (by value)
	 */
	RingBufferIterator operator--(int) {
		auto t = *this;
		--offset;
		return t;
	}

	/**
	 * Difference operator
	 * @param rhs iterator to compare to
	 * @return Difference between iterator and rhs (iterator - rhs)
	 */
	std::ptrdiff_t operator-(RingBufferIterator const& rhs) const { return offset - rhs.offset; }

	/**
	 * Addition assignment operator
	 * @param amount to add to iterator
	 * @return Modified iterator (by reference)
	 */
	RingBufferIterator& operator+=(int amount) {
		offset += amount;
		return *this;
	}

	/**
	 * Subtraction assignment operator
	 * @param amount to subtract from iterator
	 * @return Modified iterator (by reference)
	 */
	RingBufferIterator& operator-=(int amount) {
		offset -= amount;
		return *this;
	}

	/**
	 * Less than operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterator is less than rhs (iterator < rhs)
	 */
	bool operator<(RingBufferIterator const& rhs) const { return offset < rhs.offset; }

	/**
	 * Less than or equal to operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterator is less than or equal to rhs (iterator <= rhs)
	 */
	bool operator<=(RingBufferIterator const& rhs) const { return offset <= rhs.offset; }

	/**
	 * Greater than operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterator is greater than rhs (iterator > rhs)
	 */
	bool operator>(RingBufferIterator const& rhs) const { return offset > rhs.offset; }

	/**
	 * Greater than or equal to operator
	 * @param rhs iterator to compare to
	 * @return `true` if iterator is greater than or equal to rhs (iterator >= rhs)
	 */
	bool operator>=(RingBufferIterator const& rhs) const { return offset >= rhs.offset; }

	/**
	 * @param index of element from iterator
	 * @return Element at index (by reference)
	 */
	T& operator[](int index) const { return *(*this + index); }

	/**
	 * @return Element pointed to by iterator (by reference)
	 */
	T& operator*() const { return buffer[offset % capacity]; }

	/**
	 * @return Address of element pointed to by iterator
	 */
	T* operator->() const { return &buffer[offset % capacity]; }

private:
	std::size_t capacity;
	T* buffer;
	std::size_t offset;
};

}  // namespace utils
}  // namespace navtk
