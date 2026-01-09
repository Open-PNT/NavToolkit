#pragma once

/**
 * @file wgs84.hpp Contains definitions of WGS-84 constants.
 *
 * Contains definitions of WGS-84 constants according to the National Geospatial Agency standards
 * document number NGA.STND.0036, available through the NGA website here:
 * https://nga-rescue.is4s.us/NGA.STND.0036_1.0.0_WGS84.pdf.
 * The values in this file are explained in much greater detail in that source document.
 */

namespace navtk {
namespace navutils {

/**
 * Earth's eccentricity squared, as defined by WGS-84.
 */
extern const double ECCENTRICITY_SQUARED;

/**
 * Ellipticity of Earth, as defined by WGS-84.
 */
extern const double ECCENTRICITY;

/**
 * Gravitational acceleration at the equator in meters/(second)^2 as defined by WGS-84.
 */
extern const double EQUATORIAL_GRAVITY;

/**
 * Gravitational acceleration at the poles in meters/(second)^2 as defined by WGS-84.
 */
extern const double POLAR_GRAVITY;

/**
 * Ellipse flattening of the shape of Earth, as defined by WGS-84.
 */
extern const double FLATTENING;

/**
 * Equatorial radius of Earth in meters as defined by WGS-84.
 */
extern const double SEMI_MAJOR_RADIUS;

/**
 * Polar radius of Earth in meters as defined by WGS-84.
 */
extern const double SEMI_MINOR_RADIUS;

/**
 * Gravitational parameter (m^3/s^2).
 */
extern const double MU;

/**
 * Earth spin rate (rad/s).
 */
extern const double ROTATION_RATE;

/**
 * `(1-FLATTENING)^2`, convenience parameter (OMF = One Minus Flattening).
 */
extern const double OMF2;

/**
 * `(1-FLATTENING)^4`, convenience parameter (OMF = One Minus Flattening).
 */
extern const double OMF4;

}  // namespace navutils
}  // namespace navtk
