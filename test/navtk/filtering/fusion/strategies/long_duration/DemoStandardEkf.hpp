#pragma once

#include "TestSet.hpp"

#include <navtk/tensors.hpp>

namespace detail {

using navtk::Matrix;


/**
 * Declaration class for StandardEKF testing
 */
class DemoStandardEkf {

public:
	/**
	 * A constructor with a seed number parameter for which TestSet values to use
	 */
	DemoStandardEkf(int);
	TestSet ekf_test; /**< TestSet object to use TestSet values as DemoStandardEkf variables */
	Matrix out;
	Matrix outP;
	Matrix truth;
	Matrix measurements;
};

}  // namespace detail
