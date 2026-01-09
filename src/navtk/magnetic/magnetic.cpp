#include <navtk/magnetic/magnetic.hpp>

#include <navtk/navutils/math.hpp>

#include <cmath>

namespace navtk {
namespace magnetic {

double mag_to_heading(double mag_x, double mag_y, double magnetic_declination) {
	double true_heading = -atan2(mag_y, mag_x);
	true_heading -= magnetic_declination;
	return navutils::wrap_to_pi(true_heading);
}

}  // namespace magnetic
}  // namespace navtk
