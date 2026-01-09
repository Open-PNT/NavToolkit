#pragma once

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <gps_ins/utils/results.hpp>
#include <xtensor-python/pytensor.hpp>

namespace navtk {
namespace exampleutils {
/**
 * Plots the accelerometer (m/s^2) and gyroscope (radians/s) bias states.
 * The function makes the following assumptions:
 * 1) For filter_results, the accelerometer x, y, z bias correspond to row 18, 19, 20 respectively.
 * 2) For filter_results, the gyroscope x, y, z bias correspond to row 21, 22, 23 respectively.
 * 3) For sigma_results, the accelerometer x, y, z standard deviation of the bias corresponds to row
 * 9, 10, 11 respectively.
 * 4) For sigma_results, the gyroscope x, y, z standard deviation of the bias corresponds to row
 * 12, 13, 14.
 *
 * @param filter_results filtered data results generated in generate_output()
 * @param sigma_results stores the state of the filter sigma results generated in generate_output()
 * @param time_tags filter time tags valid for INS, filter, and sigma results
 */
void plot_acc_gyro_err(const Matrix &filter_results,
                       const Matrix &sigma_results,
                       const Vector &time_tags);

/**
 * Plots the position and velocity profile of the truth and filtered results.
 * The function makes the following assumptions:
 * 1) For filter_results, the velocity north, east, and down correspond to rows
 * 3, 4, and 5 respectively.
 * @param filter_results filtered data results generated in generate_output()
 * @param d_ned_true delta NED position and velocity from starting point of truth data
 * @param d_ned_filt delta NED position and velocity from starting point of filtered data
 * @param truth_interp truth data interpolated to the filtered results time tags
 * @param time_tags filter time tags valid for INS, filter, and sigma results
 */
void plot_pos_vel_profile(const Matrix &filter_results,
                          const Matrix &d_ned_true,
                          const Matrix &d_ned_filt,
                          const TruthPlotResults &truth_interp,
                          const Vector &time_tags);

/**
 * Plots the trajectory profile.
 * @param d_ned_true delta NED position and velocity from starting position of truth data
 * @param d_ned_filt delta NED position and velocity from starting position of filtered data
 */
void plot_trajectory(const Matrix &d_ned_true, const Matrix &d_ned_filt);

/**
 * Plot the difference between the filtered data and the truth data of
 * position, velocity, and tilt.
 * @param filter_results filtered data results generated in generate_output()
 * @param sigma_results stores the state of the filter sigma results generated in generate_output()
 * @param truth_interp truth data interpolated to the resultant filter time tags, see
 * TruthPlotResults definition
 * @param delta_tilts difference in NED tilt between the truth and filtered data
 * @param time_tags filter time tags valid for INS, filter, and sigma results
 * The function makes the following assumptions:
 * 1) For filter_results, the latitude, longitude, altitude correspond to rows
 * 0, 1, 2 respectively.
 * 2) For filter_results, the velocity north, east, down correspond to rows 3,
 * 3, 5 respectively.
 * 3) For sigma results, the standard deviation of latitude, longitude, altitude
 * correspond to rows 0, 1, 2.
 * 4) For sigma results, the standard deviation of velocity north, east, down
 * correspond to rows 3, 4, 5.
 * 5) For sigma results the standard deviation of NED tilt correspond to rows
 * 6, 7, 8.
 */
void plot_delta_results(const Matrix &filter_results,
                        const Matrix &sigma_results,
                        const TruthPlotResults &truth_interp,
                        const Matrix &delta_tilts,
                        const Vector &time_tags);

/**
 * Plots the truth and filter NED tilt results.
 * @param filter_results filtered data results generated in generate_output()
 * @param true_tilts stores the tilt of the truth data
 * @param sigma_results stores the state of the sigma filter results
 * @param time_tags filter time tags valid for INS, filter, and sigma results
 * For filter_results, the tilt NED correspond to rows 15, 16, 17.
 * For sigma_results, the standard deviation of the NED tilt correspond to rows
 * 6, 7, 8.
 */
void plot_tilt(const Matrix &filter_results,
               const Matrix &true_tilts,
               const Matrix &sigma_results,
               const Vector &time_tags);

/**
 * Plots the truth RPY and filter RPY.
 * @param truth struct storing the truth message data of time valid, LLH, NED velocity, and attitude
 * (RPY)
 * @param filter_results stores the state of the filter results
 * @param time_tags filter time tags valid for INS, filter, and sigma results
 */
void plot_rpy(const TruthResults &truth, const Matrix &filter_results, const Vector &time_tags);

/**
 * Calculates and plots the clock bias in meters and the clock drift in meters/second
 * error. For use with tightly coupled example.
 * @param filter_results stores the state of the filter results from generate_output()
 * @param time_tags filter time tags valid for INS, filter, and sigma results
 */
void plot_clock_error(const Matrix &filter_results, const Vector &time_tags);

/**
 * Plots the pseudorange bias in meters. For use with tightly coupled example.
 * @param pr_bias_results stores a map of the pseudorange bias data. The prns are keys,
 * the data stored is a vector of time tags with a vector of corresponding pseudorange bias
 * values.
 */
void plot_pr_bias(
    const std::map<int, std::pair<std::vector<double>, std::vector<double>>> &pr_bias_results);

}  // namespace exampleutils
}  // namespace navtk
