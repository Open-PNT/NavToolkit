#pragma once

#include "TestSet.hpp"

#include <navtk/tensors.hpp>

namespace detail {

using navtk::Matrix;


/**
 * Declaration class for StandardUKF testing
 */
class DemoStandardUkf {

public:
	/**
	 * A constructor with a seed number parameter for which TestSet values to use
	 */
	DemoStandardUkf(int);
	TestSet ukf_test; /**< TestSet object to use TestSet values as DemoStandardUkf variables */
	Matrix out;
	Matrix outP;
	Matrix truth;
	Matrix measurements;
};

}  // namespace detail
