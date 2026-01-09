#include <memory>

#include <navtoolkit.hpp>

int main() {

	auto filter = navtk::filtering::StandardFusionEngine();
	// CREATE_VSB
	std::function<navtk::Vector(const navtk::Vector&)> fx = [](const navtk::Vector& x) {
		return 3.28084 * x;
	};

	std::function<navtk::Matrix(const navtk::Vector&)> jx = [&](const navtk::Vector& x) {
		return navtk::Matrix{{3.28084 * x[0]}};
	};

	auto vsb = std::make_shared<navtk::filtering::FirstOrderVirtualStateBlock>(
	    "speed_meters", "speed_feet", fx, jx);
	// END

	// ADD_TO_FILTER
	filter.add_virtual_state_block(vsb);
	// END

	// CREATE_MP
	auto proc =
	    navtk::filtering::DirectMeasurementProcessor("proc", "speed_feet", navtk::Matrix{{1.0}});
	// END
}

// CREATE_VSB2
auto vsb = navtk::filtering::FirstOrderVirtualStateBlock(
    "llh_pos", "ecef_pos", navtk::navutils::llh_to_ecef);
// END
