#pragma once

#include "TestSet.hpp"

#include <navtk/tensors.hpp>

namespace detail {

using navtk::Matrix;


/**
 * Declaration class for StandardRBPF testing
 */
class DemoStandardRbpf {

public:
	/**
	 * A constructor with a seed number parameter for which TestSet values to use
	 */
	DemoStandardRbpf(int);
	TestSet rbpf_test; /**< TestSet object to use TestSet values as DemoStandardRbpf variables */
	Matrix out;
	Matrix outP;
	Matrix truth;
	Matrix measurements;
};

}  // namespace detail
