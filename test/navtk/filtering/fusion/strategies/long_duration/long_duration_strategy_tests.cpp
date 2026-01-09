#include <gtest/gtest.h>
#include "DemoStandardEkf.hpp"
#include "DemoStandardRbpf.hpp"
#include "DemoStandardUkf.hpp"

#include <navtk/filtering/experimental/fusion/strategies/RbpfStrategy.hpp>
#include <navtk/filtering/fusion/strategies/EkfStrategy.hpp>
#include <navtk/filtering/fusion/strategies/UkfStrategy.hpp>
#include <navtk/tensors.hpp>

using namespace detail;
using namespace navtk::filtering;
using xt::allclose;

TEST(StandardEKF, CompareToKnownSolution_SLOW) {
	DemoStandardEkf demo(1);
	EXPECT_NEAR(demo.out[80], 9.009297650529405, 1e-15);
	EXPECT_NEAR(demo.outP[80], 1.708203932499337, 1e-15);
}

TEST(StandardRBPF, CompareToKnownSolution_SLOW) {
	DemoStandardRbpf demo(1);
	// TODO: change tolerances back to 1e-8, 1e-13 and fix test to be more robust (PNTOS-625)
	EXPECT_NEAR(demo.out[80], 9.009297650529405, 1e-7);
	EXPECT_NEAR(demo.outP[80], 1.708203932499337, 1e-12);
}

TEST(StandardEKF, samePDifferentMeasurements_SLOW) {
	DemoStandardEkf demo(1);
	DemoStandardEkf demo2(2);
	ASSERT_TRUE(allclose(demo.outP, demo2.outP, 1e-15));
	EXPECT_TRUE(demo.out[80] != demo2.out[80]);
}

TEST(StandardEKF, samePDifferentMeasurementsEKF_SLOW) {
	DemoStandardEkf demo(1);
	DemoStandardEkf demo2(2);

	ASSERT_TRUE(allclose(demo.outP, demo2.outP, 1e-15));
	EXPECT_TRUE(demo.out[80] != demo2.out[80]);
}


TEST(StandardUKF, CompareToKnownSolution_SLOW) {
	DemoStandardUkf demo(1);
	EXPECT_NEAR(demo.out[80], 9.009297650529405, 1e-15);
	EXPECT_NEAR(demo.outP[80], 1.708203932499337, 1e-14);
}


TEST(StandardUKF, samePDifferentMeasurements_SLOW) {
	DemoStandardUkf demo(1);
	DemoStandardUkf demo2(2);
	ASSERT_TRUE(allclose(demo.outP, demo2.outP, 1e-15));
	EXPECT_TRUE(demo.out[80] != demo2.out[80]);
}
