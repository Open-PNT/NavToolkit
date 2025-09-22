#pragma once

#include <memory>
#include <type_traits>
#include <typeinfo>

#include <navtk/aspn.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/Ordered.hpp>

namespace navtk {
namespace utils {

/**
 * Modifies vector inputs so that they can be used to generate a valid interpolation. Inputs
 * are sorted in time; elements with identical time tags are removed, keeping only the first
 * occurrence; and `time_interp` elements that do not overlap with `time_source` elements are
 * removed.
 *
 * @param time_source The base times, size N.
 * @param data_source The interpolation base data, size N.
 * @param time_interp The times to interpolate to.
 *
 * @return A vector of indices of elements that were removed from the sorted `time_interp` vector
 * due to being duplicates or not overlapping with `time_source`.
 *
 * @throw std::runtime_error if `time_source.size() != data_source.size()` or `data_source.size < 2`
 * and the error mode is ErrorMode::DIE for either case.
 */
template <typename T>
std::vector<Size> condition_source_data(std::vector<double> &time_source,
                                        std::vector<T> &data_source,
                                        std::vector<double> &time_interp);

/**
 * Interpolates the source data and source time using the
 * vector of interpolation times (as doubles). If source data/time
 * is not available to interpolate for a `time_interp` time tag, then,
 * that time tag is unused. The unused index is reported to the user.
 * @param time_source the source time stamps to be interpolated
 * @param data_source the source data to be interpolated
 * @param time_interp the time stamps to interpolate the data
 *
 * @return A pair, where the first element is vector of unused indexes from `time_interp`, and the
 * second element is interpolated source data.
 **/
std::pair<std::vector<Size>, std::vector<double>> linear_interpolate(
    const std::vector<double> &time_source,
    const std::vector<double> &data_source,
    const std::vector<double> &time_interp);

/**
 * Interpolates using quadratic spline interpolation. If source data/time
 * is not available to interpolate for a `time_interp` time tag, then,
 * that time tag is unused. The unused index is reported to the user.
 * @param time_source the source time stamps to be interpolated
 * @param data_source the source data to be interpolated
 * @param time_interp the time stamps to interpolate the data
 *
 * @return A pair, where the first element is vector of unused indexes from `time_interp`, and the
 * second element is interpolated source data.
 */
std::pair<std::vector<Size>, std::vector<double>> quadratic_spline_interpolate(
    const std::vector<double> &time_source,
    const std::vector<double> &data_source,
    const std::vector<double> &time_interp);

/** Interpolates using cubic spline interpolation. If source data/time
 * is not available to interpolate for a `time_interp` time tag, then,
 * that time tag is unused. The unused index is reported to the user.
 * @param orig_time_source the source time stamps to be interpolated
 * @param data_source the source data to be interpolated
 * @param orig_time_interp the time stamps to interpolate the data
 *
 * @return A pair, where the first element is vector of unused indexes from `time_interp`, and the
 * second element is interpolated source data.
 */
std::pair<std::vector<Size>, std::vector<double>> cubic_spline_interpolate(
    const std::vector<double> &orig_time_source,
    const std::vector<double> &data_source,
    const std::vector<double> &orig_time_interp);

/**
 * Performs a linear interpolation.
 * This is an approximation of y at x, given two reference points (`x0`, `y0`) and (`x1`, `y1`).
 *
 * @param x0 First evaluation point of `f()`
 * @param y0 `f(x0)`
 * @param x1 Second evaluation point of `f()`
 * @param y1 `f(x1)`
 * @param x Point between `x0` and `x1` at which to calculate approximation.
 *
 * @return An approximation for `y = f(x)`. If `x0 == x1`, `y1` is returned.
 */
template <typename Y>
Y linear_interpolate(double x0, const Y &y0, double x1, const Y &y1, double x) {
	if (x0 == x1) return y1;
	double xpart = (x - x0) / (x1 - x0);
	Y ypart      = (y1 - y0) * xpart;
	return y0 + ypart;
}

/**
 * Performs a linear interpolation.
 * This is an approximation of y at x, given two reference points (`x0`, `y0`) and (`x1`, `y1`).
 *
 * @param x0 First evaluation point of `f()`
 * @param y0 `f(x0)`
 * @param x1 Second evaluation point of `f()`
 * @param y1 `f(x1)`
 * @param x Point between `x0` and `x1` at which to calculate approximation.
 *
 * @return An approximation for `y = f(x)`. If `x0 == x1`, `y1` is returned.
 */
template <typename Y>
Y linear_interpolate(const aspn_xtensor::TypeTimestamp &x0,
                     const Y &y0,
                     const aspn_xtensor::TypeTimestamp &x1,
                     const Y &y1,
                     const aspn_xtensor::TypeTimestamp &x) {
	if (x0 == x1) return y1;
	double xpart = (double)(x.get_elapsed_nsec() - x0.get_elapsed_nsec()) /
	               (x1.get_elapsed_nsec() - x0.get_elapsed_nsec());
	Y ypart = (y1 - y0) * xpart;
	return y0 + ypart;
}

/**
 * Performs linear interpolation between two MeasurementPositionVelocityAttitude records.
 *
 * @param pva1 First record.
 * @param pva2 Second record.
 * @param t Time between pva1 and pva2 to interpolate to.
 *
 * @return Approximate pva at time `t`. If `pva1` and `pva2` have the same `time_validity`, `pva2`
 * is returned. If `t` is outside of the range between `pva1` and `pva2` the return value will be
 * the nearest of the inputs (constant endpoint extrapolation) with a warning. When interpolation is
 * performed the covariance matrix is not interpolated, but copied directly from the pva input with
 * the latest time.
 */
aspn_xtensor::MeasurementPositionVelocityAttitude linear_interp_pva(
    const aspn_xtensor::MeasurementPositionVelocityAttitude &pva1,
    const aspn_xtensor::MeasurementPositionVelocityAttitude &pva2,
    const aspn_xtensor::TypeTimestamp &t);

/**
 * Performs linear interpolation between two MeasurementPositionVelocityAttitude records.
 *
 * @param pva1 First record.
 * @param pva2 Second record.
 * @param t Time between pva1 and pva2 to interpolate to.
 *
 * @return Approximate pva at time `t`. If `pva1` and `pva2` have the same `time_validity`, `pva2`
 * is returned. If `t` is outside of the range between `pva1` and `pva2` the return value will be
 * the nearest of the inputs (constant endpoint extrapolation) with a warning. When interpolation is
 * performed the covariance matrix is not interpolated, but copied directly from the pva input with
 * the latest time.
 */
not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> linear_interp_pva(
    navtk::not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> pva1,
    navtk::not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> pva2,
    const aspn_xtensor::TypeTimestamp &t);

/**
 * Performs linear interpolation between or extrapolation beyond two
 * MeasurementPositionVelocityAttitude records.
 *
 * @param pva1 First record.
 * @param pva2 Second record.
 * @param t Time to interpolate or extrapolate to.
 *
 * @return Approximate pva at time `t`. If `pva1` and `pva2` have the same `time_validity`, `pva2`
 * is returned. When interpolation is performed the covariance matrix is not interpolated, but
 * copied directly from the pva input with the latest time.
 */
aspn_xtensor::MeasurementPositionVelocityAttitude linear_extrapolate_pva(
    const aspn_xtensor::MeasurementPositionVelocityAttitude &pva1,
    const aspn_xtensor::MeasurementPositionVelocityAttitude &pva2,
    const aspn_xtensor::TypeTimestamp &t);

/**
 * Performs linear interpolation between or extrapolation beyond two
 * MeasurementPositionVelocityAttitude records.
 *
 * @param pva1 First record.
 * @param pva2 Second record.
 * @param t Time to interpolate or extrapolate to.
 *
 * @return Approximate pva at time `t`. If `pva1` and `pva2` have the same `time_validity`, `pva2`
 * is returned. When interpolation is performed the covariance matrix is not interpolated, but
 * copied directly from the pva input with the latest time.
 */
not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> linear_extrapolate_pva(
    navtk::not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> pva1,
    navtk::not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> pva2,
    const aspn_xtensor::TypeTimestamp &t);

/**
 * Linearly interpolate between two RPY (roll, pitch, yaw) attitude representations.
 *
 * @param t1 Time of first attitude value.
 * @param rpy1 First attitude, in radians.
 * @param t2 Time of second attitude value.
 * @param rpy2 Second attitude, in radians.
 * @param t Time to interpolate to. Should be between `t1` and `t2`.
 *
 * @return RPY attitude representation at `t`, in radians. If `t` is outside of the range between
 * `t1` and `t2` the return value will be the nearest of the inputs (constant endpoint
 * extrapolation) with a warning.
 */
Vector3 linear_interp_rpy(const aspn_xtensor::TypeTimestamp &t1,
                          const Vector3 &rpy1,
                          const aspn_xtensor::TypeTimestamp &t2,
                          const Vector3 &rpy2,
                          const aspn_xtensor::TypeTimestamp &t);

/**
 * Linearly interpolate between or extraplate beyond two RPY (roll, pitch, yaw) attitude
 * representations.
 *
 * @param t1 Time of first attitude value.
 * @param rpy1 First attitude, in radians.
 * @param t2 Time of second attitude value.
 * @param rpy2 Second attitude, in radians.
 * @param t Time to interpolate to.
 *
 * @return RPY attitude representation at `t`, in radians. If `t` is outside of the range between
 * `t1` and `t2` the return value will be the nearest of the inputs (constant endpoint
 * extrapolation) with a warning.
 */
Vector3 linear_extrapolate_rpy(const aspn_xtensor::TypeTimestamp &t1,
                               const Vector3 &rpy1,
                               const aspn_xtensor::TypeTimestamp &t2,
                               const Vector3 &rpy2,
                               const aspn_xtensor::TypeTimestamp &t);

}  // namespace utils
}  // namespace navtk
