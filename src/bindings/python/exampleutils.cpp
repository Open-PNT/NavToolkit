#include <utils/exampleutils.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "binding_helpers.hpp"

using navtk::zeros;
using namespace pybind11::literals;
namespace ex = navtk::exampleutils;


void add_exampleutils_functions(pybind11::module &m) {
	m.doc() = "Bindings to the Example Utilities";

	NAMESPACE_FUNCTION(constant_vel_pva, ex, "start_pva"_a, "dt"_a, "stop_time"_a);
	NAMESPACE_FUNCTION(noisy_pos_meas, ex, "truth"_a, "sigma"_a = zeros(3), "err"_a = zeros(0, 0));
	NAMESPACE_FUNCTION(noisy_alt_meas, ex, "truth"_a, "sigma"_a = 0.0, "err"_a = zeros(0));
	NAMESPACE_FUNCTION(noisy_vel_meas, ex, "truth"_a, "sigma"_a = zeros(3), "err"_a = zeros(0, 0));
	NAMESPACE_FUNCTION(
	    noisy_att_meas, ex, "truth"_a, "tilt_sigma"_a = zeros(3), "tilts"_a = zeros(0, 0));
}
