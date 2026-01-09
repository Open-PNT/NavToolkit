#include <navtk/navutils/wgs84.hpp>

#include <cmath>

namespace navtk {
namespace navutils {

const double ECCENTRICITY_SQUARED = 0.00669437999013;
// 9.7803253359 in  table 3.6 of https://nga-rescue.is4s.us/NGA.STND.0036_1.0.0_WGS84.pdf
const double EQUATORIAL_GRAVITY = 9.7803267714;
const double POLAR_GRAVITY      = 9.8321849379;
const double ECCENTRICITY       = 0.081819190842622;
const double FLATTENING         = 1 / 298.257223563;
const double SEMI_MAJOR_RADIUS  = 6378137;
const double SEMI_MINOR_RADIUS  = 6356752.3142;
const double MU                 = 3.986004418e14;  // Gravitational parameter (m^3/s^2)

// Two slightly different values for earth rate in 'official' docs.
// 7.292115e-5 is given in the WGS84 standards doc (and Titterton and Weston)
// https://nga-rescue.is4s.us/NGA.STND.0036_1.0.0_WGS84.pdf
// Table 3.1, most recent revision in 2014, and matches GRS80
// While this one is from the GPS interface specification document
// Earth rate from https://www.gps.gov/technical/icwg/IS-GPS-200K.pdf
// table 20-IV, most recent revision 2019
const double ROTATION_RATE = 7.2921151467e-5;  // Earth spin rate (rad/s)

// Commonly used derivatives from above
const double OMF2 = pow((1.0 - FLATTENING), 2);
const double OMF4 = OMF2 * OMF2;

}  // namespace navutils
}  // namespace navtk
