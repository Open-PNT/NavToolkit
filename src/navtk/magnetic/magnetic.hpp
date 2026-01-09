namespace navtk {
/**
 * NavToolkit namespace for magnetic utilities.
 */
namespace magnetic {

/**
 * Compute heading given a magnetometer measurement. Assumes the measurements are
 * already calibrated.
 *
 * @param mag_x magnetic field value on x-axis (any units). Positive x direction is assumed to be to
 * the front.
 * @param mag_y magnetic field value on y-axis (any units). Positive y direction is assumed to be to
 * the right.
 *  @param magnetic_declination magnetic declination (the angle on the horizontal plane between
 * magnetic north and true north). This value varies depending on position on the Earth's surface.
 * Defaults to 0.0.
 *
 * @return the heading in range (-PI, PI] in radians.
 */
double mag_to_heading(double mag_x, double mag_y, double magnetic_declination = 0.0);

}  // namespace magnetic
}  // namespace navtk
