#pragma once

#include <vector>

namespace detail {
/**
 * Reference file to TestSet data structure
 */
class TestSet {
public:
	std::vector<double> truth(
	    int); /**< Function to set DemoStandardKF/EKF/UKF objects truth variable */
	std::vector<double> measurements(
	    int); /**< Function to set DemoStandardKF/EKF/UKF objects truth variable */
	std::size_t test_size(
	    int); /**< Function to return DemoStandardKF/EKF/UKF objects truth dataset size */
};

}  // namespace detail
