#include <gtest/gtest.h>
#include <error_mode_assert.hpp>

#include <navtk/errors.hpp>
#include <navtk/utils/RingBuffer.hpp>

// Classes for testing the Rule of Five methods
class AResource {
public:
	int x = 0;
};
class A {
public:
	static int n_xtors;
	A() : p{new AResource{}} {
		n_xtors++;
		unique = n_xtors;
	}
	A(int val) : p{new AResource{}}, val(val) {
		n_xtors++;
		unique = n_xtors;
	}
	A(const A& other) : p{new AResource{*(other.p)}}, val(other.val) {
		n_xtors++;
		unique = n_xtors;
	}
	A(A&& other) : p{other.p}, val(other.val) {
		n_xtors++;
		unique    = n_xtors;
		other.p   = nullptr;
		other.val = 0;
	}
	A& operator=(const A& other) {
		if (&other != this) {
			delete p;
			p   = nullptr;
			p   = new AResource{*(other.p)};
			val = other.val;
		}
		return *this;
	}
	A& operator=(A&& other) {
		if (&other != this) {
			delete p;
			p         = other.p;
			val       = other.val;
			other.p   = nullptr;
			other.val = 0;
		}
		return *this;
	}
	~A() {
		delete p;
		n_xtors--;
		n_dtors++;
		assert(n_dtors == 1);
	}
	AResource* p;
	unsigned val = 0;
	int n_dtors  = 0;
	int unique;
};
int A::n_xtors = 0;

TEST(RingBuffer, RuleOfFiveTests) {
	{
		// make a ring buffer, b, and fill to capacity plus one
		std::size_t capacity = 3;
		navtk::utils::RingBuffer<A> b(capacity);
		for (std::size_t i = 1; i <= capacity + 1; i++) {
			b.push_back(A(i));
			// check b size
			EXPECT_EQ(b.size(), std::min(i, capacity));
			// check last value pushed
			EXPECT_EQ(b.back().val, i);
		}

		// make a copy of b for later
		auto a(b);

		// invoke b's copy constructor
		auto b1(b);

		// invoke b's assignment operator
		navtk::utils::RingBuffer<A> b2(1);
		b2 = b;

		// invoke b's move constructor
		auto b3 = std::move(b);

		// invoke a's move assignment operator (since the use of b is now undefined)
		navtk::utils::RingBuffer<A> b4(1);
		b4 = std::move(a);

		// do some sanity checking that the copying happened correctly
		EXPECT_EQ(b1.size(), capacity);
		EXPECT_EQ(b1.size(), b2.size());
		EXPECT_EQ(b1.size(), b3.size());
		EXPECT_EQ(b1.size(), b4.size());

		EXPECT_EQ(b1.front().val, 2);
		EXPECT_EQ(b1.front().val, b2.front().val);
		EXPECT_EQ(b1.front().val, b3.front().val);
		EXPECT_EQ(b1.front().val, b4.front().val);

		EXPECT_EQ(b1.back().val, 4);
		EXPECT_EQ(b1.back().val, b2.back().val);
		EXPECT_EQ(b1.back().val, b3.back().val);
		EXPECT_EQ(b1.back().val, b4.back().val);

		// make a ring buffer, b11, fill it to one over capacity with push_back(&)
		navtk::utils::RingBuffer<A> b11(capacity);
		for (std::size_t i = 1; i <= capacity + 1; i++) {
			A a(i);
			b11.push_back(a);
		}
		b11.pop_back();

		// make a ring buffer, b22, fill it to one over capacity with push_front(&)
		navtk::utils::RingBuffer<A> b22(capacity);
		for (std::size_t i = 1; i <= capacity + 1; i++) {
			A a(i);
			b22.push_front(a);
		}

		// make a ring buffer, b33, fill it to one over capacity with push_front(&&)
		navtk::utils::RingBuffer<A> b33(capacity);
		for (std::size_t i = 1; i <= capacity + 1; i++) {
			b33.push_front(A(i));
		}
		b33.pop_front();

		// test iterator -> operator
		EXPECT_EQ(b1.begin()->val, b1.front().val);
	}

	// make sure all the A's in all the buffers got destructed
	EXPECT_EQ(A::n_xtors, 0);
}

TEST(RingBuffer, PushBackTests) {
	// make a ring buffer, b, and fill to capacity plus one
	int capacity = 3;
	navtk::utils::RingBuffer<int> b(capacity);
	for (int i = 1; i <= capacity + 1; i++) {
		b.push_back(i);
		// check b size
		EXPECT_EQ(b.size(), std::min(i, capacity));
		// check last value
		EXPECT_EQ(b.back(), i);
		// check first value
		EXPECT_EQ(b.front(), std::max(1, i - capacity + 1));
	}
}

TEST(RingBuffer, PushBackMoveTests) {
	// make a ring buffer, b, and fill to capacity plus one
	int capacity = 3;
	navtk::utils::RingBuffer<int> b(capacity);
	for (int i = 1; i <= capacity + 1; i++) {
		b.push_back(std::move(i));
		// check b size
		EXPECT_EQ(b.size(), std::min(i, capacity));
		// check last value
		EXPECT_EQ(b.back(), i);
		// check first value
		EXPECT_EQ(b.front(), std::max(1, i - capacity + 1));
	}
}

TEST(RingBuffer, PushFrontTests) {
	// make a ring buffer, b, and fill to capacity plus one
	int capacity = 3;
	navtk::utils::RingBuffer<int> b(capacity);
	for (int i = 1; i <= capacity + 1; i++) {
		b.push_front(i);
		// check b size
		EXPECT_EQ(b.size(), std::min(i, capacity));
		// check first value
		EXPECT_EQ(b.front(), i);
		// check first value
		EXPECT_EQ(b.back(), std::max(1, i - capacity + 1));
	}
}

TEST(RingBuffer, PushFrontMoveTests) {
	// make a ring buffer, b, and fill to capacity plus one
	int capacity = 3;
	navtk::utils::RingBuffer<int> b(capacity);
	for (int i = 1; i <= capacity + 1; i++) {
		b.push_front(std::move(i));
		// check b size
		EXPECT_EQ(b.size(), std::min(i, capacity));
		// check first value
		EXPECT_EQ(b.front(), i);
		// check first value
		EXPECT_EQ(b.back(), std::max(1, i - capacity + 1));
	}
}

TEST(RingBuffer, EmptySizeTests) {
	int capacity = 3;
	navtk::utils::RingBuffer<int> b(capacity);
	EXPECT_TRUE(b.empty());
	EXPECT_EQ(b.size(), 0);
	for (int i = 1; i <= capacity * 2; i++) {
		b.push_back(i);
		EXPECT_FALSE(b.empty());
		EXPECT_EQ(b.size(), std::min(i, capacity));
	}

	for (int i = capacity - 1; i >= 0; i--) {
		b.pop_front();
		EXPECT_EQ(b.size(), i);
	}

	EXPECT_TRUE(b.empty());

	b.push_back(500);

	EXPECT_FALSE(b.empty());
}

TEST(RingBuffer, FrontTests) {
	navtk::utils::RingBuffer<int> b(3);
	b.push_back(1);
	EXPECT_EQ(b.front(), 1);
	// use front accessor to modify value in front
	b.front() = 10;
	b.push_back(2);
	b.push_back(3);
	EXPECT_EQ(b.front(), 10);
	// wrapping
	b.push_back(4);
	EXPECT_EQ(b.front(), 2);
}

TEST(RingBuffer, BackTests) {
	navtk::utils::RingBuffer<int> b(3);
	b.push_front(1);
	EXPECT_EQ(b.back(), 1);
	// use back accessor to modify value in back
	b.back() = 10;
	b.push_front(2);
	b.push_front(3);
	EXPECT_EQ(b.back(), 10);
	// wrapping
	b.push_front(4);
	EXPECT_EQ(b.back(), 2);
}

TEST(RingBuffer, EraseTests) {
	// make a ring buffer, b, and fill to capacity plus one
	int capacity = 5;
	navtk::utils::RingBuffer<int> b(capacity);
	for (int i = 1; i <= capacity + 1; i++) {
		b.push_back(i);
	}

	EXPECT_UB_OR_DIE(b.erase(b.cbegin() + 1, b.cend() - 1),
	                 "unimplemented operation erase from middle of RingBuffer",
	                 std::runtime_error);

	// b: [2] [3] [4] [5] [6]
	EXPECT_EQ(b.size(), 5);
	EXPECT_EQ(b.front(), 2);
	EXPECT_EQ(b.back(), 6);
	b.erase(b.cbegin(), b.cbegin() + 2);
	// b: [4] [5] [6]
	EXPECT_EQ(b.size(), 3);
	EXPECT_EQ(b.front(), 4);
	EXPECT_EQ(b.back(), 6);
	b.erase(b.cend() - 2, b.cend());
	// b: [4]
	EXPECT_EQ(b.size(), 1);
	EXPECT_EQ(b.front(), 4);
	EXPECT_EQ(b.back(), 4);
}



TEST(RingBuffer, IteratorOperatorTests) {
	// make a ring buffer, b, and fill to capacity
	int capacity = 3;
	navtk::utils::RingBuffer<int> b(capacity);
	for (int i = 1; i <= capacity; i++) {
		b.push_back(i);
	}
	for (auto itr = b.begin(); itr < b.end(); itr++) {
		EXPECT_EQ(*itr, itr - b.begin() + 1);
	}
	for (auto itr = b.end() - 1; itr > b.begin(); itr--) {
		EXPECT_EQ(*itr, itr - b.begin() + 1);
	}
	auto itr = b.end();
	itr -= capacity;
	EXPECT_EQ(*itr, 1);
	EXPECT_TRUE(itr <= b.begin());
	EXPECT_TRUE(itr >= b.begin());
	EXPECT_EQ(itr[0], 1);
}

TEST(RingBuffer, IteratorsTests) {
	// make a ring buffer, b, and fill to capacity
	int capacity = 3;
	navtk::utils::RingBuffer<int> b(capacity);
	for (int i = 1; i <= capacity; i++) {
		b.push_back(i);
	}
	for (auto i = b.begin(); i < b.end(); i++) {
		EXPECT_EQ(*i, i - b.begin() + 1);
	}
}

TEST(RingBuffer, ConstIteratorsTests) {
	// make a ring buffer, b, and fill to capacity
	int capacity = 3;
	navtk::utils::RingBuffer<int> b(capacity);
	for (int i = 1; i <= capacity; i++) {
		b.push_back(i);
	}
	for (auto i = b.cbegin(); i < b.cend(); i++) {
		EXPECT_EQ(*i, i - b.cbegin() + 1);
	}
}

TEST(RingBuffer, ReverseIteratorsTests) {
	// make a ring buffer, b, and fill to capacity
	int capacity = 3;
	navtk::utils::RingBuffer<int> b(capacity);
	for (int i = 1; i <= capacity; i++) {
		b.push_back(i);
	}
	for (auto i = b.rbegin(); i < b.rend(); i++) {
		EXPECT_EQ(capacity - *i, i - b.rbegin());
	}
}

TEST(RingBuffer, ConstReverseIteratorsTests) {
	// make a ring buffer, b, and fill to capacity
	int capacity = 3;
	navtk::utils::RingBuffer<int> b(capacity);
	for (int i = 1; i <= capacity; i++) {
		b.push_back(i);
	}
	for (auto i = b.crbegin(); i < b.crend(); i++) {
		EXPECT_EQ(capacity - *i, i - b.crbegin());
	}
}


TEST(RingBuffer, testBadInputBufferSize) {
	auto guard = navtk::ErrorModeLock(navtk::ErrorMode::DIE);
	EXPECT_THROW(navtk::utils::RingBuffer<A> b(0), std::invalid_argument);
}

TEST(RingBuffer, InputBufferSizeTooBig) {
	auto guard           = navtk::ErrorModeLock(navtk::ErrorMode::DIE);
	std::size_t capacity = std::size_t((std::numeric_limits<std::ptrdiff_t>::max)());
	EXPECT_THROW(navtk::utils::RingBuffer<A> b(capacity), std::invalid_argument);
}
