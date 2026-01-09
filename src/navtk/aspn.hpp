#pragma once

#include <vector>

#ifdef NAVTK_PYTHON_TENSOR
#	include <aspn23/xtensor_py/aspn_xtensor.hpp>
#else
#	include <aspn23/xtensor/aspn_xtensor.hpp>
#endif

#include <navtk/tensors.hpp>

typedef std::vector<std::shared_ptr<aspn_xtensor::AspnBase>> AspnBaseVector;
