#include <vector>

#include <gtest/gtest.h>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/containers/TimestampedDataSeries.hpp>
#include <navtk/get_time.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::to_type_timestamp;

namespace {
std::shared_ptr<aspn_xtensor::MeasurementImu> val(double v) {
	auto t = to_type_timestamp(v);
	return std::make_shared<aspn_xtensor::MeasurementImu>(
	    aspn_xtensor::TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0),
	    t,
	    ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED,
	    navtk::ones(3) * v,
	    navtk::ones(3) * v,
	    std::vector<aspn_xtensor::TypeIntegrity>{});
}

template <typename T, typename U>
void wipe(navtk::filtering::TimestampedDataSeries<T, U>& buf) {
	buf.erase(buf.cbegin(), buf.cend());
}

template <typename T, typename U>
void almost_fill(navtk::filtering::TimestampedDataSeries<T, U>& buf) {
	buf.insert(val(2.0));
	buf.insert(val(1.0));
	buf.insert(val(2.5));
	buf.insert(val(1.0));
}

template <typename T, typename U>
void fill(navtk::filtering::TimestampedDataSeries<T, U>& buf) {
	almost_fill(buf);
	buf.insert(val(3.0));
}

}  // namespace

struct TimestampedDataSeriesTests : public ::testing::Test {
	navtk::filtering::TimestampedDataSeries<aspn_xtensor::MeasurementImu> buf;

	TimestampedDataSeriesTests()
	    : ::testing::Test(),
	      buf(navtk::filtering::TimestampedDataSeries<aspn_xtensor::MeasurementImu>(5)) {}

	virtual void SetUp() override { wipe(buf); }
};

TEST_F(TimestampedDataSeriesTests, empty) {
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(1.0));
	ASSERT_EQ(nbrs.first - buf.cend(), 0);
	ASSERT_EQ(nbrs.second - buf.cend(), 0);
}

TEST_F(TimestampedDataSeriesTests, one_element_exact) {
	buf.insert(val(1.0));
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(1.0));
	ASSERT_EQ(*nbrs.first, *nbrs.second);
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
}

TEST_F(TimestampedDataSeriesTests, one_element_early) {
	buf.insert(val(1.0));
	auto nbrs = buf.get_nearest_neighbors(aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_EQ(*nbrs.first, nullptr);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 1.0);
}

TEST_F(TimestampedDataSeriesTests, one_element_late) {
	buf.insert(val(1.0));
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(2.0));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(*nbrs.second, nullptr);
}

TEST_F(TimestampedDataSeriesTests, two_match_element_exact) {
	buf.insert(val(1.0));
	buf.insert(val(1.0));
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(1.0));
	ASSERT_EQ(*nbrs.first, *nbrs.second);
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(nbrs.first - buf.cbegin(), 1);
}

TEST_F(TimestampedDataSeriesTests, two_match_element_early) {
	buf.insert(val(1.0));
	buf.insert(val(1.0));
	auto nbrs = buf.get_nearest_neighbors(aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_EQ(*nbrs.first, nullptr);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 1.0);
	ASSERT_EQ(nbrs.second - buf.cbegin(), 0);
}

TEST_F(TimestampedDataSeriesTests, two_match_element_late) {
	buf.insert(val(1.0));
	buf.insert(val(1.0));
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(2.0));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(*nbrs.second, nullptr);
	ASSERT_EQ(nbrs.first - buf.cbegin(), 1);
}

TEST_F(TimestampedDataSeriesTests, mults_before_all_full) {
	fill(buf);
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(0.1));
	ASSERT_EQ(*nbrs.first, *nbrs.second);
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 1.0);
	ASSERT_EQ(nbrs.second - buf.cbegin(), 0);  // Two 'ones'; make sure we have the right one
}

TEST_F(TimestampedDataSeriesTests, mults_before_all_not_full) {
	almost_fill(buf);
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(0.1));
	ASSERT_EQ(*nbrs.first, nullptr);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 1.0);
	ASSERT_EQ(nbrs.second - buf.cbegin(), 0);  // Two 'ones'; make sure we have the right one
}

TEST_F(TimestampedDataSeriesTests, mults_after_all_full) {
	fill(buf);
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(4.0));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 3.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 1.0);
}

TEST_F(TimestampedDataSeriesTests, mults_after_all_not_full) {
	almost_fill(buf);
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(4.0));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 2.5);
	ASSERT_EQ(*nbrs.second, nullptr);
}

TEST_F(TimestampedDataSeriesTests, mults_exact_full) {
	fill(buf);
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(2.0));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 2.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 2.0);

	auto nbrs2 = buf.get_nearest_neighbors(to_type_timestamp(2.5));
	ASSERT_EQ(navtk::get_time(*nbrs2.first).second, 2.5);
	ASSERT_EQ(navtk::get_time(*nbrs2.second).second, 2.5);

	auto nbrs3 = buf.get_nearest_neighbors(to_type_timestamp(3.0));
	ASSERT_EQ(navtk::get_time(*nbrs3.first).second, 3.0);
	ASSERT_EQ(navtk::get_time(*nbrs3.second).second, 3.0);

	// Push off first element and get first exactly
	buf.insert(val(10.0));
	auto nbrs4 = buf.get_nearest_neighbors(to_type_timestamp(1.0));

	// Make sure the buffer no longer contains two 1.0 values
	std::vector<double> expected{1, 2, 2.5, 3, 10};
	auto buf_itr = buf.begin();
	for (auto value : expected) {
		EXPECT_TRUE(buf_itr != buf.end()) << "Reached end of buffer early.";
		EXPECT_EQ(value, navtk::get_time(*buf_itr++).second);
	}

	ASSERT_EQ(navtk::get_time(*nbrs4.first).second, 1.0);
	ASSERT_EQ(navtk::get_time(*nbrs4.second).second, 1.0);
}

TEST_F(TimestampedDataSeriesTests, mults_exact_not_full) {
	almost_fill(buf);

	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(1.0));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 1.0);

	auto nbrs2 = buf.get_nearest_neighbors(to_type_timestamp(2.0));
	ASSERT_EQ(navtk::get_time(*nbrs2.first).second, 2.0);
	ASSERT_EQ(navtk::get_time(*nbrs2.second).second, 2.0);

	auto nbrs3 = buf.get_nearest_neighbors(to_type_timestamp(2.5));
	ASSERT_EQ(navtk::get_time(*nbrs3.first).second, 2.5);
	ASSERT_EQ(navtk::get_time(*nbrs3.second).second, 2.5);
}

TEST_F(TimestampedDataSeriesTests, mults_exact_repeats) {
	fill(buf);
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(1.0));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 1.0);
	ASSERT_EQ(nbrs.second - buf.cbegin(), 1);  // Two 'ones'; make sure we have the right one
}

TEST_F(TimestampedDataSeriesTests, mults_between_full) {
	fill(buf);
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(0.5));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 1.0);
	ASSERT_EQ(nbrs.second - buf.cbegin(), 0);  // Two 'ones'; make sure we have the right one

	nbrs = buf.get_nearest_neighbors(to_type_timestamp(1.5));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 2.0);

	nbrs = buf.get_nearest_neighbors(to_type_timestamp(2.2));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 2.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 2.5);

	nbrs = buf.get_nearest_neighbors(to_type_timestamp(1.0 + 1e-9));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 2.0);

	nbrs = buf.get_nearest_neighbors(to_type_timestamp(2.0 - 1e-9));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 2.0);
}

TEST_F(TimestampedDataSeriesTests, mults_between_not_full) {
	almost_fill(buf);
	auto nbrs = buf.get_nearest_neighbors(to_type_timestamp(0.5));
	ASSERT_EQ(*nbrs.first, nullptr);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 1.0);
	ASSERT_EQ(nbrs.second - buf.cbegin(), 0);  // Two 'ones'; make sure we have the right one

	nbrs = buf.get_nearest_neighbors(to_type_timestamp(1.5));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 2.0);

	nbrs = buf.get_nearest_neighbors(to_type_timestamp(2.2));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 2.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 2.5);

	nbrs = buf.get_nearest_neighbors(to_type_timestamp(1.0 + 1e-9));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 2.0);

	// Was 2 - 1e-10, but this is small enough to cause an error (fmod() * 1e9 leaves 0.1, which is
	// reduced to 0 when cast as int32 for second arg of Time)
	nbrs = buf.get_nearest_neighbors(to_type_timestamp(2.0 - 1e-9));
	ASSERT_EQ(navtk::get_time(*nbrs.first).second, 1.0);
	ASSERT_EQ(navtk::get_time(*nbrs.second).second, 2.0);
}
