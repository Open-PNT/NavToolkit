#pragma once

#include <navtk/tensors.hpp>

namespace navtk {
namespace navutils {
/**
 * Constant definition of PI.
 */
extern const double PI;
/**
 * Ratio to convert degrees to radians.
 */
extern const double DEG2RAD;
/**
 * Ratio to convert radians to degrees.
 */
extern const double RAD2DEG;

/**
 * Forms a skew symmetric matrix from a 3-length vector of values.
 * Typically used to form a matrix to correct a Direction Cosine Matrix
 * from the platform frame or sensor frame to the navigation frame with
 * estimated tilt angles in the navigation frame.
 *
 * If the input vector is \f$[x, y, z]\f$, then the resulting skew-symmetric
 * matrix is
 *
 * \f$    \begin{bmatrix}
        0 & -z & y \\
        z & 0 & -x \\
        -y & x & 0
    \end{bmatrix}\f$
 *
 *
 * @param angles 3-length vector of angular values
 *
 * @return Equivalent skew-symmetric matrix
 */
Matrix3 skew(const Vector3& angles);

/**
 * Performs orthonormalization of a Direction Cosine Matrix. Uses an
 * iterative gradient projection method from Bar-Itzhack, as referenced
 * by Mao.
 *
 * Reference: Optimal Orthonormalization of the Strapdown Matrix by
 * Using Singular Value Decomposition, Jianqin Mao, sec 2.
 *
 * @param dcm Original DCM (3x3)
 *
 * @return Orthonormalized matrix (3x3)
 */
Matrix3 ortho_dcm(const Matrix3& dcm);

/**
 * Adjust an angle value so that it lies within (-PI, PI].
 *
 * @param orig Original angle measure, radians.
 *
 * @return Adjusted angle, radians.
 */
double wrap_to_pi(double orig);

/**
 * Adjust an angle value so that it lies within [0, 2PI).
 *
 * @param orig Original angle measure, radians.
 *
 * @return Adjusted angle, radians.
 */
double wrap_to_2_pi(double orig);

}  // namespace navutils
}  // namespace navtk
