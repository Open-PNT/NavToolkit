#include <gtest/gtest.h>
#include <spdlog_assert.hpp>
#include <tensor_assert.hpp>

#include <navtk/filtering/containers/MeasurementBuffer.hpp>
#include <navtk/filtering/containers/MeasurementBuffer3d.hpp>
#include <navtk/get_time.hpp>

using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeTimestamp;
using navtk::to_seconds;
using navtk::Vector;
using navtk::filtering::MeasurementBuffer;
using navtk::filtering::MeasurementBuffer3d;
using navtk::filtering::MeasurementBufferBase;


struct MeasurementBufferUniqueTests : ::testing::Test {};

struct MeasurementBuffer3dUniqueTests : ::testing::Test {};

class TestableMeasurementBuffer : public MeasurementBuffer {
public:
	TestableMeasurementBuffer() : base_measurement(1.0), base_covariance(0.1) {}
	double base_measurement;
	double base_covariance;
};

class TestableMeasurementBuffer3d : public MeasurementBuffer3d {
public:
	TestableMeasurementBuffer3d()
	    : base_measurement({10.0, 1.0, 0.1}),
	      base_covariance({{10.0, 0.1, 0.0}, {0.1, 1.0, 0.0}, {0.0, 0.0, 0.01}}) {}
	navtk::Vector3 base_measurement;
	navtk::Matrix3 base_covariance;
};

template <class T>
struct MeasurementBufferTests : ::testing::Test {
	static std::unique_ptr<T> make_buffer() { return std::make_unique<T>(); }
};

TYPED_TEST_SUITE_P(MeasurementBufferTests);

TYPED_TEST_P(MeasurementBufferTests, BufferEmptyTest) {
	auto buffer = *this->make_buffer();
	ASSERT_TRUE(buffer.is_empty());
	buffer.add_measurement(
	    TypeTimestamp((int64_t)0), 0.0 * buffer.base_measurement, 0.0 * buffer.base_covariance);
	ASSERT_FALSE(buffer.is_empty());
	buffer.clear();
	ASSERT_TRUE(buffer.is_empty());
}

TYPED_TEST_P(MeasurementBufferTests, CoversTimeTest) {
	auto buffer = *this->make_buffer();
	ASSERT_FALSE(buffer.covers_time(TypeTimestamp((int64_t)0)));
	buffer.add_measurement(
	    TypeTimestamp((int64_t)0), 0.0 * buffer.base_measurement, 0.0 * buffer.base_covariance);
	ASSERT_TRUE(buffer.covers_time(TypeTimestamp((int64_t)0)));
	for (int ii = 1; ii < 10; ++ii) {
		buffer.add_measurement(
		    to_type_timestamp(ii), ii * buffer.base_measurement, ii * buffer.base_covariance);
	}
	ASSERT_TRUE(buffer.covers_time(to_type_timestamp(9.0)));
	ASSERT_TRUE(buffer.covers_time(to_type_timestamp(3.278)));
	ASSERT_FALSE(buffer.covers_time(to_type_timestamp(9.1)));
}

TYPED_TEST_P(MeasurementBufferTests, RemoveOldMeasurementsTest) {
	auto buffer = *this->make_buffer();
	for (int ii = 0; ii < 7; ++ii) {
		buffer.add_measurement(
		    to_type_timestamp(ii), ii * buffer.base_measurement, ii * buffer.base_covariance);
	}
	buffer.remove_old_measurements(to_type_timestamp(0.5));
	ASSERT_TRUE(buffer.covers_time(to_type_timestamp(0.5)));
	buffer.remove_old_measurements(to_type_timestamp(3.0));
	ASSERT_FALSE(buffer.covers_time(to_type_timestamp(2.0)));
	ASSERT_TRUE(buffer.covers_time(to_type_timestamp(3.0)));
	buffer.remove_old_measurements(to_type_timestamp(5.5));
	ASSERT_FALSE(buffer.covers_time(to_type_timestamp(4.0)));
	ASSERT_TRUE(buffer.covers_time(to_type_timestamp(5.5)));
}

TYPED_TEST_P(MeasurementBufferTests, LastTimeTest) {
	auto buffer    = *this->make_buffer();
	auto last_time = buffer.get_last_time();
	ASSERT_FALSE(last_time.first);
	for (int ii = 0; ii < 3; ++ii) {
		buffer.add_measurement(
		    to_type_timestamp(ii), ii * buffer.base_measurement, ii * buffer.base_covariance);
		last_time = buffer.get_last_time();
		ASSERT_TRUE(last_time.first);
		ASSERT_EQ(last_time.second, to_type_timestamp(ii, 0));
	}
}

TYPED_TEST_P(MeasurementBufferTests, AddMeasurementAndGetTimesTest) {
	auto buffer = *this->make_buffer();
	buffer.add_measurement(to_type_timestamp(1.0), buffer.base_measurement, buffer.base_covariance);
	ASSERT_INFO(buffer.add_measurement(
	                to_type_timestamp(1.0), buffer.base_measurement, buffer.base_covariance),
	            "Received measurement at exact time already in buffer");
	buffer.add_measurement(to_type_timestamp(2.0), buffer.base_measurement, buffer.base_covariance);
	ASSERT_INFO(buffer.add_measurement(
	                to_type_timestamp(1.5), buffer.base_measurement, buffer.base_covariance),
	            "Measurement out of order, sorting.");
	auto precise_times                        = buffer.get_times();
	std::vector<TypeTimestamp> expected_times = {
	    to_type_timestamp(1.0), to_type_timestamp(1.5), to_type_timestamp(2.0)};
	ASSERT_EQ(expected_times, precise_times);
}

REGISTER_TYPED_TEST_SUITE_P(MeasurementBufferTests,
                            BufferEmptyTest,
                            CoversTimeTest,
                            RemoveOldMeasurementsTest,
                            LastTimeTest,
                            AddMeasurementAndGetTimesTest);

typedef ::testing::Types<TestableMeasurementBuffer, TestableMeasurementBuffer3d> BufferTestTypes;

INSTANTIATE_TYPED_TEST_SUITE_P(BufferTests, MeasurementBufferTests, BufferTestTypes, );

TEST(MeasurementBuffer3dUniqueTests, GetMeasurementTest) {
	auto buffer           = TestableMeasurementBuffer3d();
	auto base_measurement = buffer.base_measurement;
	auto base_covariance  = buffer.base_covariance;
	for (int ii = 0; ii < 5; ++ii) {
		buffer.add_measurement(to_type_timestamp(ii), ii * base_measurement, ii * base_covariance);
	}
	auto measurement = buffer.get_measurement(TypeTimestamp((int64_t)0));
	auto covariance  = buffer.get_covariance(TypeTimestamp((int64_t)0));
	ASSERT_TRUE(measurement.first);
	ASSERT_TRUE(covariance.first);
	ASSERT_ALLCLOSE(0.0 * base_measurement, measurement.second);
	ASSERT_ALLCLOSE(0.0 * base_covariance, covariance.second);

	std::vector<double> test_times = {0.5, 1.0, 3.5, 4.0};
	for (double t : test_times) {
		measurement = buffer.get_measurement(to_type_timestamp(t));
		covariance  = buffer.get_covariance(to_type_timestamp(t));
		ASSERT_TRUE(measurement.first);
		ASSERT_TRUE(covariance.first);
		ASSERT_ALLCLOSE(t * base_measurement, measurement.second);
		ASSERT_ALLCLOSE(t * base_covariance, covariance.second);
	}
	ASSERT_INFO(measurement = buffer.get_measurement(to_type_timestamp(5.0)),
	            "Returning invalid measurement");
	ASSERT_INFO(covariance = buffer.get_covariance(to_type_timestamp(5.0)),
	            "Returning invalid covariance");
	ASSERT_FALSE(measurement.first);
	ASSERT_FALSE(covariance.first);
}

TEST(MeasurementBufferUniqueTests, GetMeasurementTest) {
	auto buffer           = TestableMeasurementBuffer();
	auto base_measurement = buffer.base_measurement;
	auto base_covariance  = buffer.base_covariance;
	for (int ii = 0; ii < 5; ++ii) {
		buffer.add_measurement(to_type_timestamp(ii), ii * base_measurement, ii * base_covariance);
	}
	auto measurement = buffer.get_measurement(TypeTimestamp((int64_t)0));
	auto covariance  = buffer.get_covariance(TypeTimestamp((int64_t)0));
	ASSERT_TRUE(measurement.first);
	ASSERT_TRUE(covariance.first);
	ASSERT_NEAR(0.0 * base_measurement, measurement.second, 1e-12);
	ASSERT_NEAR(0.0 * base_covariance, covariance.second, 1e-12);

	std::vector<double> test_times = {0.5, 1.0, 3.5, 4.0};
	for (double t : test_times) {
		measurement = buffer.get_measurement(to_type_timestamp(t));
		covariance  = buffer.get_covariance(to_type_timestamp(t));
		ASSERT_TRUE(measurement.first);
		ASSERT_TRUE(covariance.first);
		ASSERT_NEAR(t * base_measurement, measurement.second, 1e-12);
		ASSERT_NEAR(t * base_covariance, covariance.second, 1e-12);
	}

	ASSERT_INFO(measurement = buffer.get_measurement(to_type_timestamp(5.0)),
	            "Returning invalid measurement");
	ASSERT_INFO(covariance = buffer.get_covariance(to_type_timestamp(5.0)),
	            "Returning invalid covariance");
	ASSERT_FALSE(measurement.first);
	ASSERT_FALSE(covariance.first);
}

TEST(MeasurementBufferUniqueTests, IteratorTest) {
	int ii      = 0;
	auto buffer = MeasurementBuffer();
	for (int i = 0; i < 10; ++i) {
		buffer.add_measurement(to_type_timestamp(i), i, i + 1);
	}
	// using a range-based for loop here demonstrates that the
	// MeasurementBuffer class is iterable
	for (const auto& entry : buffer) {
		ASSERT_NEAR(ii, entry.second.first, 1e-10);
		ASSERT_NEAR(ii + 1, entry.second.second, 1e-10);
		ii++;
	}
}

const MeasurementBuffer& get_measurement_buffer_from_elsewhere() {
	static MeasurementBuffer buf;
	for (int i = 0; i < 10; i++) {
		buf.add_measurement(to_type_timestamp(i), i, i + 1);
	}
	return buf;
}

TEST(MeasurementBufferUniqueTests, ConstIteratorTest) {
	int ii          = 0;
	const auto& buf = get_measurement_buffer_from_elsewhere();
	for (auto const& entry : buf) {
		ASSERT_NEAR(ii, entry.second.first, 1e-10);
		ASSERT_NEAR(++ii, entry.second.second, 1e-10);
	}
}

TEST(MeasurementBufferUniqueTests, MutableIteratorTest) {
	TypeTimestamp when = to_type_timestamp(1, 0);
	auto buffer        = MeasurementBuffer();
	buffer.add_measurement(when, 2.0, 10.0);
	for (auto& entry : buffer) entry.second.first++;
	ASSERT_NEAR(buffer.get_measurement(when).second, 3.0, 1e-10);
}

TEST(MeasurementBufferUniqueTests, MeasurementBufferGetMeasurementsAroundTest) {
	auto buffer = MeasurementBuffer();
	for (int ii = 0; ii < 10; ++ii) {
		buffer.add_measurement(to_type_timestamp(ii), 2.0 * ii, 0.1 * ii);
	}

	auto measurements =
	    buffer.get_measurements_around(TypeTimestamp((int64_t)0), to_type_timestamp(1.5));
	Vector expected_times        = {0.0, 1.0, 2.0};
	Vector expected_measurements = {0.0, 2.0, 4.0};
	ASSERT_TRUE(measurements.first);
	ASSERT_ALLCLOSE(expected_times, to_seconds(measurements.second.first));
	ASSERT_ALLCLOSE(expected_measurements, measurements.second.second);

	measurements   = buffer.get_measurements_around(to_type_timestamp(2.2), to_type_timestamp(2.3));
	expected_times = {2.0, 3.0};
	expected_measurements = {4.0, 6.0};
	ASSERT_TRUE(measurements.first);
	ASSERT_ALLCLOSE(expected_times, to_seconds(measurements.second.first));
	ASSERT_ALLCLOSE(expected_measurements, measurements.second.second);

	measurements   = buffer.get_measurements_around(to_type_timestamp(7.7), to_type_timestamp(9.0));
	expected_times = {7.0, 8.0, 9.0};
	expected_measurements = {14.0, 16.0, 18.0};
	ASSERT_TRUE(measurements.first);
	ASSERT_ALLCLOSE(expected_times, to_seconds(measurements.second.first));
	ASSERT_ALLCLOSE(expected_measurements, measurements.second.second);

	ASSERT_INFO(measurements = buffer.get_measurements_around(TypeTimestamp((int64_t)0),
	                                                          to_type_timestamp(10.0)),
	            "No data is available");
	ASSERT_FALSE(measurements.first);

	ASSERT_INFO(measurements = buffer.get_measurements_around(TypeTimestamp((int64_t)0),
	                                                          TypeTimestamp((int64_t)0)),
	            "t_0 == t_1");
	ASSERT_TRUE(measurements.first);
	ASSERT_EQ(measurements.second.second(0), 0.0);
	ASSERT_EQ(measurements.second.second(1), 0.0);
}

TEST(MeasurementBufferUniqueTests, MeasurementBufferGetAverageVarianceTest) {
	auto buffer = MeasurementBuffer();
	for (int ii = 0; ii < 10; ++ii) {
		buffer.add_measurement(to_type_timestamp(ii), 2.0 * ii, 0.1 * ii);
	}

	auto average_variance =
	    buffer.get_average_variance(TypeTimestamp((int64_t)0), to_type_timestamp(1.5));
	double expected_average_variance = 0.05;
	ASSERT_TRUE(average_variance.first);
	ASSERT_EQ(expected_average_variance, average_variance.second);

	average_variance = buffer.get_average_variance(to_type_timestamp(2.2), to_type_timestamp(2.3));
	expected_average_variance = 0.225;
	ASSERT_TRUE(average_variance.first);
	ASSERT_NEAR(expected_average_variance, average_variance.second, 1e-12);

	average_variance = buffer.get_average_variance(to_type_timestamp(7.7), to_type_timestamp(9.0));
	expected_average_variance = 0.85;
	ASSERT_TRUE(average_variance.first);
	ASSERT_NEAR(expected_average_variance, average_variance.second, 1e-12);

	ASSERT_INFO(average_variance =
	                buffer.get_average_variance(TypeTimestamp((int64_t)0), to_type_timestamp(10.0)),
	            "No data is available");
	ASSERT_FALSE(average_variance.first);

	average_variance =
	    buffer.get_average_variance(TypeTimestamp((int64_t)0), TypeTimestamp((int64_t)0));
	expected_average_variance = 0.0;
	ASSERT_TRUE(average_variance.first);
	ASSERT_NEAR(expected_average_variance, average_variance.second, 1e-12);
}
