#include <memory>

#include <gtest/gtest.h>
#include <tensor_assert.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/experimental/virtualstateblocks/BaronavMotionToLatitudeLongitude.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::dot;
using navtk::Vector;
using navtk::Vector3;
using navtk::filtering::experimental::BaronavMotionToLatitudeLongitude;

class BaronavMotionToLatitudeLongitudeTests : public ::testing::Test {
public:
	BaronavMotionToLatitudeLongitudeTests() {
		ecef_reference = {497435.818180, -4885950.925448, 4056197.688132};
		vsb =
		    std::make_shared<BaronavMotionToLatitudeLongitude>("current", "target", ecef_reference);
	}
	Vector3 ecef_reference;
	std::shared_ptr<BaronavMotionToLatitudeLongitude> vsb;
};

TEST_F(BaronavMotionToLatitudeLongitudeTests, test_convert) {
	Vector x       = {0.0, 0.0, 0.0};
	auto converted = vsb->convert_estimate(x, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(converted, Vector({0.69363901, -1.46933649, 0.0}));
}

TEST_F(BaronavMotionToLatitudeLongitudeTests, test_set_ecef_reference) {
	Vector x       = {0.0, 0.0, 0.0};
	auto converted = vsb->convert_estimate(x, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(converted, Vector({0.69363901, -1.46933649, 0.0}));

	Vector3 test_llh = {0.6941972, -1.4675769, 0.0};
	auto ecef        = navtk::navutils::llh_to_ecef(test_llh);
	vsb->set_ecef_reference(ecef);
	converted = vsb->convert_estimate(x, aspn_xtensor::TypeTimestamp((int64_t)0));
	ASSERT_ALLCLOSE(converted, test_llh);
}
