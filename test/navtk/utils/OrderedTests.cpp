#include <gtest/gtest.h>

#include <navtk/utils/Ordered.hpp>

// These performance tests don't "test" anything but they do log test times in the gtest output when
// run which is handy for comparing the performance of ring and deque (and whatever may be used in
// the future)
template <class T>
void optimal_performance(T& b) {
	for (auto i = 0; i < 1000000; i++) {
		b.insert(i);
	}
}

TEST(Ordered, DISABLED_OptimalPerformanceDeque) {
	navtk::utils::OrderedDeque<double> b(10);
	optimal_performance(b);
}

TEST(Ordered, DISABLED_OptimalPerformanceRing) {
	navtk::utils::OrderedRing<double> b(10);
	optimal_performance(b);
}

template <class T>
void sub_optimal_performance(T& b) {
	b.insert(1.0);
	for (auto i = 0; i < 1000000; i++) {
		b.insert(0.9);
	}
}

TEST(Ordered, DISABLED_SubOptimalPerformanceDeque) {
	navtk::utils::OrderedDeque<double> b(10);
	sub_optimal_performance(b);
}

TEST(Ordered, DISABLED_SubOptimalPerformanceRing) {
	navtk::utils::OrderedRing<double> b(10);
	sub_optimal_performance(b);
}

template <class T>
void worst_case_performance(T& b) {
	b.insert(1000001.0);
	b.insert(1000002.0);
	b.insert(1000003.0);
	b.insert(1000004.0);
	b.insert(1000005.0);
	b.insert(1000006.0);
	b.insert(1000007.0);
	b.insert(1000008.0);
	b.insert(1000009.0);
	for (auto i = 0; i < 1000000; i++) {
		b.insert(i);
	}
}

TEST(Ordered, DISABLED_WorstCasePerformanceDeque) {
	navtk::utils::OrderedDeque<double> b(10);
	worst_case_performance(b);
}

TEST(Ordered, DISABLED_WorstCasePerformanceRing) {
	navtk::utils::OrderedRing<double> b(10);
	worst_case_performance(b);
}

template <class T>
void in_order_insertion(T& b) {
	b.insert(0.1);
	EXPECT_EQ(b.cend() - b.cbegin(), 1);
	EXPECT_EQ(*(b.cbegin() + 0), 0.1);

	b.insert(0.2);
	EXPECT_EQ(b.cend() - b.cbegin(), 2);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 0), 0.1);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 1), 0.2);

	b.insert(0.3);
	EXPECT_EQ(b.cend() - b.cbegin(), 3);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 0), 0.1);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 1), 0.2);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 2), 0.3);

	b.insert(0.4);
	EXPECT_EQ(b.cend() - b.cbegin(), 3);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 0), 0.2);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 1), 0.3);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 2), 0.4);
}

TEST(Ordered, InOrderInsertionDeque) {
	// create buffer capable of holding 3 entries
	navtk::utils::OrderedDeque<double> b(3);
	in_order_insertion(b);
}

TEST(Ordered, InOrderInsertionRing) {
	// create buffer capable of holding 3 entries
	navtk::utils::OrderedRing<double> b(3);
	in_order_insertion(b);
}

template <class T>
void out_of_order_insertion(T& b) {

	EXPECT_TRUE(b.begin() == b.end());
	EXPECT_TRUE(b.cbegin() == b.cend());
	EXPECT_TRUE(b.rbegin() == b.rend());
	EXPECT_TRUE(b.crbegin() == b.crend());

	b.insert(0.5);
	EXPECT_EQ(b.cend() - b.cbegin(), 1);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 0), 0.5);

	EXPECT_FALSE(b.begin() == b.end());
	EXPECT_FALSE(b.cbegin() == b.cend());
	EXPECT_FALSE(b.rbegin() == b.rend());
	EXPECT_FALSE(b.crbegin() == b.crend());

	b.insert(0.2);
	EXPECT_EQ(b.cend() - b.cbegin(), 2);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 0), 0.2);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 1), 0.5);

	b.insert(0.4);
	EXPECT_EQ(b.cend() - b.cbegin(), 3);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 0), 0.2);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 1), 0.4);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 2), 0.5);

	// Wrapping

	b.insert(0.3);
	EXPECT_EQ(b.cend() - b.cbegin(), 3);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 0), 0.3);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 1), 0.4);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 2), 0.5);

	b.insert(0.2);
	EXPECT_EQ(b.cend() - b.cbegin(), 3);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 0), 0.3);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 1), 0.4);
	EXPECT_DOUBLE_EQ(*(b.cbegin() + 2), 0.5);
}

TEST(Ordered, OutOfOrderInsertionDeque) {
	// create buffer capable of holding 3 entries
	navtk::utils::OrderedDeque<double> b(3);
	out_of_order_insertion(b);
}

TEST(Ordered, OutOfOrderInsertionRing) {
	// create buffer capable of holding 3 entries
	navtk::utils::OrderedRing<double> b(3);
	out_of_order_insertion(b);
}

template <class T>
void get_in_range(T& b) {

	auto range = b.get_in_range(0.0, 1000.0);
	EXPECT_TRUE(range.first == b.cend());
	EXPECT_TRUE(range.second == b.cend());

	b.insert(100.0);  // Should get dropped when wrapping
	b.insert(200.0);  // Should get dropped when wrapping

	b.insert(222.1);
	b.insert(222.2);
	b.insert(222.3);
	b.insert(222.4);
	b.insert(222.5);
	b.insert(222.6);
	b.insert(222.7);
	b.insert(222.8);
	b.insert(222.9);

	// out of range before
	range = b.get_in_range(0.0, 222.0);
	EXPECT_TRUE(range.first == b.cend());
	EXPECT_TRUE(range.second == b.cend());

	// out of range after
	range = b.get_in_range(223.0, 1000.0);
	EXPECT_TRUE(range.first == b.cend());
	EXPECT_TRUE(range.second == b.cend());

	// beginning of range
	range = b.get_in_range(0.0, 222.1);
	EXPECT_DOUBLE_EQ(*range.first, 222.1);
	EXPECT_DOUBLE_EQ(*range.second, 222.2);

	// end of range
	range = b.get_in_range(222.9, 1000.0);
	EXPECT_DOUBLE_EQ(*range.first, 222.9);
	EXPECT_TRUE(range.second == b.cend());

	// out of range inside
	range = b.get_in_range(222.21, 222.29);
	EXPECT_TRUE(range.first == b.cend());
	EXPECT_TRUE(range.second == b.cend());

	// beginning of range plus some
	range = b.get_in_range(0.0, 222.55);
	EXPECT_DOUBLE_EQ(*range.first, 222.1);
	EXPECT_DOUBLE_EQ(*range.second, 222.6);

	// end of range plus some
	range = b.get_in_range(222.55, 1000.0);
	EXPECT_DOUBLE_EQ(*range.first, 222.6);
	EXPECT_TRUE(range.second == b.cend());

	// full range
	range = b.get_in_range(0.0, 1000.0);
	EXPECT_DOUBLE_EQ(*range.first, 222.1);
	EXPECT_TRUE(range.second == b.cend());

	// full range exact
	range = b.get_in_range(222.1, 222.9);
	EXPECT_DOUBLE_EQ(*range.first, 222.1);
	EXPECT_TRUE(range.second == b.cend());
}

TEST(Ordered, GetInRangeDeque) {
	// create buffer capable of holding 9 entries
	navtk::utils::OrderedDeque<double> b(9);
	get_in_range(b);
}

TEST(Ordered, GetInRangeRing) {
	// create buffer capable of holding 9 entries
	navtk::utils::OrderedRing<double> b(9);
	get_in_range(b);
}

template <class T>
void get_in_range_duplicates(T& b) {

	b.insert(0.7);  // Should get dropped when wrapping
	b.insert(0.8);  // Should get dropped when wrapping
	b.insert(0.9);  // Should get dropped when wrapping

	b.insert(1.1);
	b.insert(1.2);
	b.insert(1.2);
	b.insert(1.3);
	b.insert(1.3);
	b.insert(1.3);
	b.insert(1.4);

	// out of range
	auto range = b.get_in_range(1.21, 1.22);
	EXPECT_TRUE(range.first == b.cend());
	EXPECT_TRUE(range.second == b.cend());

	// range ends with duplicates
	range = b.get_in_range(0.0, 1.2);
	EXPECT_DOUBLE_EQ(*range.first, 1.1);
	EXPECT_DOUBLE_EQ(*range.second, 1.3);
	EXPECT_EQ(range.second - range.first, 3);

	// range starts with duplicates
	range = b.get_in_range(1.2, 100.0);
	EXPECT_DOUBLE_EQ(*range.first, 1.2);
	EXPECT_TRUE(range.second == b.cend());
	EXPECT_EQ(range.second - range.first, 6);

	// range is only duplicates
	range = b.get_in_range(1.2, 1.2);
	EXPECT_DOUBLE_EQ(*range.first, 1.2);
	EXPECT_DOUBLE_EQ(*range.second, 1.3);
	EXPECT_EQ(range.second - range.first, 2);

	// range is sequential duplicates
	range = b.get_in_range(1.2, 1.3);
	EXPECT_DOUBLE_EQ(*range.first, 1.2);
	EXPECT_DOUBLE_EQ(*range.second, 1.4);
	EXPECT_EQ(range.second - range.first, 5);
}

TEST(Ordered, GetInRangeDuplicatesDeque) {
	// create buffer capable of holding 7 entries
	navtk::utils::OrderedDeque<double> b(7);
	get_in_range_duplicates(b);
}

TEST(Ordered, GetInRangeDuplicatesRing) {
	// create buffer capable of holding 7 entries
	navtk::utils::OrderedRing<double> b(7);
	get_in_range_duplicates(b);
}

template <class T>
void get_nearest_neighbor_empty(T& b) {

	auto neighbors = b.get_nearest_neighbors(1.0);
	EXPECT_TRUE(neighbors.first == b.cend());
	EXPECT_TRUE(neighbors.second == b.cend());
	EXPECT_TRUE(b.cbegin() == b.cend());
}

template <class T>
void get_nearest_neighbor_1_entry(T& b) {
	b.insert(0.9);

	// Interesting areas: before, equal, after entry

	auto neighbors = b.get_nearest_neighbors(0.8);
	EXPECT_TRUE(neighbors.first == b.cend());
	EXPECT_TRUE(neighbors.second == b.cbegin());
	EXPECT_EQ(*(neighbors.second), 0.9);
	EXPECT_FALSE(b.cbegin() == b.cend());

	neighbors = b.get_nearest_neighbors(0.9);
	EXPECT_TRUE(neighbors.first == b.cbegin());
	EXPECT_EQ(*(neighbors.first), 0.9);
	EXPECT_TRUE(neighbors.second == b.cbegin());
	EXPECT_EQ(*(neighbors.second), 0.9);
	EXPECT_FALSE(b.cbegin() == b.cend());

	neighbors = b.get_nearest_neighbors(1.0);
	EXPECT_TRUE(neighbors.first == (b.cend() - 1));
	EXPECT_EQ(*(neighbors.first), 0.9);
	EXPECT_TRUE(neighbors.second == b.cend());
	EXPECT_FALSE(b.cbegin() == b.cend());
}

template <class T>
void get_nearest_neighbor_2_entries(T& b) {
	b.insert(0.9);
	b.insert(1.1);

	// Interesting areas: before first, first, between, second, after second

	auto neighbors = b.get_nearest_neighbors(0.8);
	EXPECT_TRUE(neighbors.first == b.cend());
	EXPECT_TRUE(neighbors.second == b.cbegin());
	EXPECT_EQ(*(neighbors.second), 0.9);
	EXPECT_FALSE(b.cbegin() == b.cend());

	neighbors = b.get_nearest_neighbors(0.9);
	EXPECT_TRUE(neighbors.first == b.cbegin());
	EXPECT_EQ(*(neighbors.first), 0.9);
	EXPECT_TRUE(neighbors.second == b.cbegin());
	EXPECT_EQ(*(neighbors.second), 0.9);
	EXPECT_FALSE(b.cbegin() == b.cend());

	neighbors = b.get_nearest_neighbors(1.0);
	EXPECT_TRUE(neighbors.first == b.cbegin());
	EXPECT_EQ(*(neighbors.first), 0.9);
	EXPECT_TRUE(neighbors.second == (b.cend() - 1));
	EXPECT_EQ(*(neighbors.second), 1.1);
	EXPECT_FALSE(b.cbegin() == b.cend());

	neighbors = b.get_nearest_neighbors(1.1);
	EXPECT_TRUE(neighbors.first == (b.cend() - 1));
	EXPECT_EQ(*(neighbors.first), 1.1);
	EXPECT_TRUE(neighbors.second == (b.cend() - 1));
	EXPECT_EQ(*(neighbors.second), 1.1);
	EXPECT_FALSE(b.cbegin() == b.cend());

	neighbors = b.get_nearest_neighbors(1.2);
	EXPECT_TRUE(neighbors.first == (b.cend() - 1));
	EXPECT_EQ(*(neighbors.first), 1.1);
	EXPECT_TRUE(neighbors.second == b.cend());
	EXPECT_FALSE(b.cbegin() == b.cend());
}

template <class T>
void get_nearest_neighbor_at_capacity(T& b) {
	b.insert(0.9);
	b.insert(1.1);
	b.insert(1.3);

	// Interesting areas: before first, middle, after last

	auto neighbors = b.get_nearest_neighbors(0.8);
	EXPECT_TRUE(neighbors.first == b.cend());
	EXPECT_TRUE(neighbors.second == b.cbegin());
	EXPECT_EQ(*(neighbors.second), 0.9);
	EXPECT_FALSE(b.cbegin() == b.cend());

	neighbors = b.get_nearest_neighbors(1.1);
	EXPECT_TRUE(neighbors.first == (b.cbegin() + 1));
	EXPECT_EQ(*(neighbors.first), 1.1);
	EXPECT_TRUE(neighbors.second == (b.cbegin() + 1));
	EXPECT_EQ(*(neighbors.second), 1.1);
	EXPECT_FALSE(b.cbegin() == b.cend());

	neighbors = b.get_nearest_neighbors(1.4);
	EXPECT_TRUE(neighbors.first == (b.cend() - 1));
	EXPECT_EQ(*(neighbors.first), 1.3);
	EXPECT_TRUE(neighbors.second == b.cend());
	EXPECT_FALSE(b.cbegin() == b.cend());
}

template <class T>
void get_nearest_neighbor_over_capacity(T& b) {
	b.insert(0.9);  // Should be discarded
	b.insert(1.1);
	b.insert(1.3);
	b.insert(1.5);

	// Interesting areas: before discarded, before first, middle, after last

	auto neighbors = b.get_nearest_neighbors(0.8);
	EXPECT_TRUE(neighbors.first == b.cend());
	EXPECT_TRUE(neighbors.second == b.cbegin());
	EXPECT_EQ(*(neighbors.second), 1.1);
	EXPECT_FALSE(b.cbegin() == b.cend());

	neighbors = b.get_nearest_neighbors(1.0);
	EXPECT_TRUE(neighbors.first == b.cend());
	EXPECT_TRUE(neighbors.second == b.cbegin());
	EXPECT_EQ(*(neighbors.second), 1.1);
	EXPECT_FALSE(b.cbegin() == b.cend());

	neighbors = b.get_nearest_neighbors(1.3);
	EXPECT_TRUE(neighbors.first == (b.cbegin() + 1));
	EXPECT_EQ(*(neighbors.first), 1.3);
	EXPECT_TRUE(neighbors.second == (b.cbegin() + 1));
	EXPECT_EQ(*(neighbors.second), 1.3);
	EXPECT_FALSE(b.cbegin() == b.cend());

	neighbors = b.get_nearest_neighbors(1.6);
	EXPECT_TRUE(neighbors.first == (b.cend() - 1));
	EXPECT_EQ(*(neighbors.first), 1.5);
	EXPECT_TRUE(neighbors.second == b.cend());
	EXPECT_FALSE(b.cbegin() == b.cend());
}


TEST(Ordered, GetNearestNeighborDeque) {
	// create buffer capable of holding 3 entries
	navtk::utils::OrderedDeque<double> b(3);
	get_nearest_neighbor_empty(b);
	get_nearest_neighbor_1_entry(b);
	b.erase(b.cbegin(), b.cend());
	get_nearest_neighbor_2_entries(b);
	b.erase(b.cbegin(), b.cend());
	get_nearest_neighbor_at_capacity(b);
	b.erase(b.cbegin(), b.cend());
	get_nearest_neighbor_over_capacity(b);
}

TEST(Ordered, GetNearestNeighborRing) {
	// create buffer capable of holding 3 entries
	navtk::utils::OrderedRing<double> b(3);
	get_nearest_neighbor_empty(b);
	get_nearest_neighbor_1_entry(b);
	b.erase(b.cbegin(), b.cend());
	get_nearest_neighbor_2_entries(b);
	b.erase(b.cbegin(), b.cend());
	get_nearest_neighbor_at_capacity(b);
	b.erase(b.cbegin(), b.cend());
	get_nearest_neighbor_over_capacity(b);
}
