#pragma once

#include <functional>

#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
/**
 * NavToolkit namespace for filtering capabilities.
 */
namespace filtering {

/**
 * Calculate the numerical Jacobian of a multivariate function.
 *
 * Let $dx_i$ with `0 <= i < N` be the unit vector whose $i$-th element is 1 and
 * all others are zero.  The $(j,i)$-th element of the Jacobian $J$ is computed
 * as follows:
 *
 * \f$J_{j,i}=\frac{f_j(x+\epsilon{}dx_i) - f_j(x - \epsilon{}dx_i)}{2\epsilon{}dx_i}\f$
 *
 * @param f A function mapping a column vector of N inputs into a column
 *     vector of M outputs. If the function returns a matrix, then the
 *     matrix is converted to a column vector, and the function is treated
 *     as though this is the format of its output.
 * @param x A N-element column vector at which to compute the Jacobian of `f`.
 * @param eps A column vector of steps the same size as `x` with which to perturb the function `f`.
 *
 * @return A MxN matrix of the Jacobian of `f` evaluated at `x`.
 *
 * @throw std::range_error If `x` or `eps` are not Nx1 and the error mode is ErrorMode::DIE.
 */
Matrix calc_numerical_jacobian(const std::function<Vector(const Vector&)>& f,
                               const Vector& x,
                               const Vector& eps);

#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND
/**
 * Calculate the numerical Jacobian of a multivariate function.
 *
 * Let $dx_i$ with `0 <= i < N` be the unit vector whose $i$-th element is 1 and
 * all others are zero.  The $(j,i)$-th element of the Jacobian $J$ is computed
 * as follows:
 *
 * \f$J_{j,i}=\frac{f_j(x+\epsilon{}dx_i) - f_j(x - \epsilon{}dx_i)}{2\epsilon{}dx_i}\f$
 *
 * @param f A function mapping a column vector of N inputs into a column
 *     vector of M outputs. If the function returns a matrix, then the
 *     matrix is converted to a column vector, and the function is treated
 *     as though this is the format of its output.
 * @param x A N-element column vector at which to compute the Jacobian of `f`.
 * @param eps The step size used to compute the Jacobian. Applied as a scale factor
 * to each element of `x` as the Jacobian is computed. If 0 then a default value of 0.001 is used.
 *
 * @return A MxN matrix of the Jacobian of `f` evaluated at `x`.
 */
Matrix calc_numerical_jacobian(const std::function<Vector(const Vector&)>& f,
                               const Vector& x,
                               Scalar eps = 0.001);
#endif

/**
 * Calculate the numerical Hessians of a multivariate function.
 * Given a function g(x) that maps the N elements of vector `x` to
 * a vector of M elements (`v`), we treat the function g(x) as a composite
 * of M functions, each of which maps `x` to a scalar value (an element of `v`):
 * \f$g(x) = {f_0(x), f_1(x)... f_{m-1}(x)}\f$.
 * Then the second derivative of g(x) can be viewed as a collection of the
 * NxN Hessians of each f(x).
 *
 * The Hessian has the form:
 *
 * \f$\begin{bmatrix}
 * \frac{\delta f}{\delta^2x_0} & \frac{\delta f}{\delta x_0 \delta x_1} & \frac{\delta f}{\delta
 * x_0 \delta x_2} & \dots & \frac{\delta f}{\delta x_0 \delta x_{n-1}} \\
 * \frac{\delta f}{\delta x_1 \delta x_0} & \frac{\delta f}{\delta^2 x_1} & \frac{\delta f}{\delta
 * x_1 \delta x_2} & \dots & \frac{\delta f}{\delta x_1 \delta x_{n-1}} \\
 * \vdots & \vdots & \vdots & \ddots & \vdots \\
 * \frac{\delta f}{\delta x_{n-1} \delta x_0} & \frac{\delta f}{\delta x_{n-1}\delta x_1} &
 * \frac{\delta f}{\delta x_{n-1} \delta x_2} & \dots & \frac{\delta f}{\delta^2 x_{n-1}}
 * \end{bmatrix}\f$
 *
 * @param f A function mapping a column vector of N inputs into a column
 *     vector of M outputs.
 * @param x A N-element column vector at which to compute the Hessians of `f`.
 * @param eps A column vector of steps the same size as `x` with which to perturb the function `f`.
 *
 * @return An M-sized collection of NxN matrices of the Hessians of `f` evaluated at `x`.
 */
std::vector<Matrix> calc_numerical_hessians(const std::function<Vector(const Vector&)>& f,
                                            const Vector& x,
                                            const Vector& eps);

#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND
/**
 * Calculate the numerical Hessians of a multivariate function.
 * Given a function g(x) that maps the N elements of vector `x` to
 * a vector of M elements (`v`), we treat the function g(x) as a composite
 * of M functions, each of which maps `x` to a scalar value (an element of `v`):
 * \f$g(x) = {f_0(x), f_1(x)... f_{m-1}(x)}\f$.
 * Then the second derivative of g(x) can be viewed as a collection of the
 * NxN Hessians of each f(x).
 *
 * The Hessian has the form:
 *
 * \f$\begin{bmatrix}
 * \frac{\delta f}{\delta^2x_0} & \frac{\delta f}{\delta x_0 \delta x_1} & \frac{\delta f}{\delta
 * x_0 \delta x_2} & \dots & \frac{\delta f}{\delta x_0 \delta x_{n-1}} \\
 * \frac{\delta f}{\delta x_1 \delta x_0} & \frac{\delta f}{\delta^2 x_1} & \frac{\delta f}{\delta
 * x_1 \delta x_2} & \dots & \frac{\delta f}{\delta x_1 \delta x_{n-1}} \\
 * \vdots & \vdots & \vdots & \ddots & \vdots \\
 * \frac{\delta f}{\delta x_{n-1} \delta x_0} & \frac{\delta f}{\delta x_{n-1}\delta x_1} &
 * \frac{\delta f}{\delta x_{n-1} \delta x_2} & \dots & \frac{\delta f}{\delta^2 x_{n-1}}
 * \end{bmatrix}\f$
 *
 * @param f A function mapping a column vector of N inputs into a column
 *     vector of M outputs.
 * @param x A N-element column vector at which to compute the Hessians of `f`.
 * @param eps Common scale factor to use in perturbing each state (i.e. the k'th step size is `n =
 * x(k) * eps`).
 *
 * @return An M sized collection of NxN matrices of the Hessians of `f` evaluated at `x.
 */
std::vector<Matrix> calc_numerical_hessians(const std::function<Vector(const Vector&)>& f,
                                            const Vector& x,
                                            Scalar eps = 0.001);
#endif

/**
 * Calculate the mean and covariance of a set of observations.
 *
 * @param samples NxM matrix, a collection of M independent observations
 * of N random variables.
 *
 * @return A pair containing an N-length Vector of sample mean as first value,
 * and NxN covariance matrix for the second.
 */
EstimateWithCovariance calc_mean_cov(const Matrix& samples);

/**
 * Pass samples of a joint Gaussian through an arbitrary function and calculate
 * the ensemble mean and covariance of the result.
 *
 * @param ec First and second moments of an N-state starting distribution.
 * @param fx A function that takes an N-length observation (`x`) and maps it to
 * an M-length result.
 * @param num_samples Number of samples to pass through fun to generate the statistics.
 *
 * @return A pair containing an M-length Vector of transformed observations and an
 * MxM Matrix of corresponding covariances.
 */
EstimateWithCovariance monte_carlo_approx(const EstimateWithCovariance& ec,
                                          const std::function<Vector(const Vector&)>& fx,
                                          Size num_samples = 100);

/**
 * Pass samples of a joint Gaussian through a function that generates Euler
 * angle attitude results and calculate the ensemble mean and covariance of the result.
 *
 * @param ec First and second moments of an N-state starting distribution.
 * @param fx A function that takes an N-length observation (`x`) and maps it to
 * a 3-length Euler angle result (RPY).
 * @param num_samples Number of samples to pass through fun to generate the statistics.
 *
 * @return A pair containing 3 Vector roll, pitch, and yaw attitude and the associated
 * 3x3 Matrix of tilt covariances.
 */
EstimateWithCovariance monte_carlo_approx_rpy(const EstimateWithCovariance& ec,
                                              const std::function<Vector(const Vector&)>& fx,
                                              Size num_samples = 100);

/**
 * Transform a joint Gaussian using a second order approximation of an
 * arbitrary mapping function.
 *
 * @param ec First and second moments of an N-state starting distribution.
 * @param fx A function that takes an N-length observation and maps it to
 * an M-length result.
 * @param jx Function that accepts `x` and returns an MxN Jacobian valid at `x`.
 * When null, defaults to the calc_numerical_jacobian() function using its default values.
 * @param hx Function that accepts `x` and returns an N-length vector of
 * MxN Hessian matrices valid at `x`. When null, defaults to the calc_numerical_hessians()
 * function using its default values.
 *
 * @return A pair containing an M-length Vector of transformed observations and an
 * MxM Matrix of corresponding covariances.
 */
EstimateWithCovariance second_order_approx(
    const EstimateWithCovariance& ec,
    std::function<Vector(const Vector&)>& fx,
    std::function<Matrix(const Vector&)> jx              = 0,
    std::function<std::vector<Matrix>(const Vector&)> hx = 0);

/**
 * Transform a joint Gaussian using a first-order approximation of an
 * arbitrary mapping function.
 *
 * @param ec First and second moments of an N-state starting distribution.
 * @param fx A function that takes an N-length observation and maps it to
 * an M-length result.
 * @param jx Function that accepts `x` and returns an MxN Jacobian valid at `x`.
 * When null, defaults to the calc_numerical_jacobian() function using its default values.
 *
 * @return A pair containing an M-length Vector of transformed observations and an
 * MxM Matrix of corresponding covariances.
 */
EstimateWithCovariance first_order_approx(const EstimateWithCovariance& ec,
                                          std::function<Vector(const Vector&)>& fx,
                                          std::function<Matrix(const Vector&)> jx = 0);

/**
 * Transform a joint Gaussian using a numerically derived first-order
 * approximation of a mapping function that produces an Euler angle attitude.
 * Uses a modified version of the central difference equation to use tilts
 * rather than direct arithmetic of Euler angles. Accuracy will suffer if tilts
 * aren't small/approximately linear over the perturbation, which is +- 1% of
 * nominal value \p ec.
 *
 * @param ec First and second moments of an N-state starting distribution.
 * @param fx A function that takes an N-length observation and maps it to
 * a 3 vector roll, pitch and heading in radians.
 *
 * @return A pair containing estimated Euler angles (typically roll, pitch and heading) and a
 * 3x3 Matrix of reference frame (usually NED) tilt angle covariances.
 */
EstimateWithCovariance first_order_approx_rpy(const EstimateWithCovariance& ec,
                                              std::function<Vector(const Vector&)>& fx);

}  // namespace filtering
}  // namespace navtk
