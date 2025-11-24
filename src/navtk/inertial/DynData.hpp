#pragma once

#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * A container that calculates a number of derived values (velocities, forces) covering a short
 * time history of position and IMU data.
 */
class DynData {
public:
	/**
	 * DynData contains the three most recently received positions.  This enum allows selection
	 * of these positions.
	 */
	enum class RecentPositionsEnum {
		/**
		The position corresponding to the current position in time
		*/
		MOST_RECENT,
		/**
		The position one time step in the past
		*/
		SECOND_MOST_RECENT,
		/**
		The position two time steps in the past
		*/
		THIRD_MOST_RECENT,
	};

	/**
	 * Constructor.
	 *
	 * @param origin Position of the origin of the NED frame in which all local-level values will
	 *  be referenced.
	 */
	DynData(const aspn_xtensor::MeasurementPosition& origin);

	/**
	 * Indicates if the user has provided enough data that forces can theoretically be calculated.
	 *
	 *
	 * @return `true` if update() has been called at least twice; `false` otherwise.
	 */
	bool enough_data();

	/**
	 * Update the time history of measurements and derived values.
	 *
	 * @param new_pos The latest position measurement. Assumed to have occurred after the position
	 * provided to the last call of this function (or class instantiation).
	 * @param align_buffer A time sorted collection of all IMU measurements that occurred between
	 * the last call to this function and the time of validity of \p new_pos. If a significant
	 * number of IMU measurements are missing (a more than 5% deviation between the average delta
	 * time between measurements and the expected dt based on the number of measurements received)
	 * or the number of IMU measurements is less than 2, then this measurement period will be
	 * considered invalid. It is assumed that all imu measurements in \p align_buffer are of type
	 * ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED.
	 */
	void update(const aspn_xtensor::MeasurementPosition& new_pos,
	            const std::vector<aspn_xtensor::MeasurementImu>& align_buffer);

	/**
	 * Get the estimated average specific force from IMU data.
	 *
	 * @return A pair where `.first` is the validity of the force, and `.second` is the 3-D force,
	 *  valid at the 'mid time'; that is the same time of validity returned by get_vel_mid().
	 *  Validity will be false if not enough_data().
	 */
	std::pair<bool, Vector3> get_force_from_imu();

	/**
	 * Get the estimated average specific force from position data.
	 *
	 * @return The 3-D force, valid at the 'mid time'; that is the same time of validity returned by
	 *  get_vel_mid().
	 */
	Vector3 get_force_from_pos();

	/**
	 * Get the estimated velocity at the 'midpoint' position time.
	 *
	 * @return A pair containing the time of validity and the NED velocity in m/s.
	 * The velocity is calculated from double differencing of the positions returned by
	 *  get_positions(), and corresponds to the middle element of that vector. Will not be valid if
	 * enough_data() returns false.
	 */
	std::pair<aspn_xtensor::TypeTimestamp, Vector3> get_vel_mid();

	/**
	 * Get the last 3 positions received.
	 *
	 * @return A 3-length vector that contains the last 3 positions received, oldest first. This
	 *  vector is initially filled with copies of the origin (as returned by get_origin()); each
	 * call to update() will 'push down' the stack, removing the previous oldest element.
	 */
	[[deprecated(
	    "Deprecated due to extra copies. Use get_position instead which returns one "
	    "position ")]] std::vector<aspn_xtensor::MeasurementPosition>
	get_positions();


	/**
	 * Get one of the most recent positions.
	 * @param recency RecentPositionsEnum.
	 * @return The chosen position.
	 */
	const aspn_xtensor::MeasurementPosition& get_position(const RecentPositionsEnum& recency) const;

	/**
	 * The origin of the local frame (as passed to the constructor).
	 *
	 * @return The position of the frame origin.
	 */
	const aspn_xtensor::MeasurementPosition& get_origin() const;

	/**
	 * Get the radians to meters conversion factors for the local frame defined by the position
	 * returned by get_origin().
	 *
	 * @return A pair containing a) the multiplier that converts a delta latitude (in radians)
	 * to meters in local north, and b) the multiplier that converts a delta longitude (in radians)
	 * to meters in local east.
	 */
	std::pair<double, double> get_lat_lon_factors();

private:
	/* First position supplied, forms the origin of the local NED frame */
	aspn_xtensor::MeasurementPosition origin;
	/* Multiply by delta latitude in radians to get meters north in local level frame at origin */
	double lat_factor = 0.0;
	/* Multiply by delta longitude in radians to get meters east in local level frame at origin */
	double lon_factor = 0.0;
	/* The most recent supplied position, time t(k)*/
	aspn_xtensor::MeasurementPosition current;
	/* The last position passed in, time t(k-1) */
	aspn_xtensor::MeasurementPosition prior;
	/* The position supplied 2 calls ago, time t(k-2) */
	aspn_xtensor::MeasurementPosition prior2;
	/* The avg NED velocity between times t(k-1) and t(k) (m/s) */
	Vector3 vel_current_prior = zeros(3);
	/* The avg NED velocity between times t(k-2) and t(k-1) (m/s) */
	Vector3 vel_prior_prior2 = zeros(3);
	/* The avg NED velocity at time t(k-1) (avg of vel_current_prior and vel_prior_prior2) (m/s) */
	Vector3 vel_prior = zeros(3);
	/* Time halfway between t(k-1) and t(k) (i.e. time of validity for vel_current_prior) */
	aspn_xtensor::TypeTimestamp t_current_prior;
	/* Time halfway between t(k-2) and t(k-1) (i.e. time of validity for vel_prior_prior2) */
	aspn_xtensor::TypeTimestamp t_prior_prior2;
	/* The respective averages of the first and second halves of the IMU data measured over t(k-2)
	 * (exclusive) to t(k-1) (inclusive) */
	std::pair<Vector3, Vector3> split_forces_prior_prior2;
	/* True if split_forces_prior_prior2 is suitable for use in force calculations */
	bool prior_prior2_imu_valid = false;
	/* The respective averages of the first and second halves of the IMU data measured over t(k-1)
	 * (exclusive) to t(k) (inclusive) */
	std::pair<Vector3, Vector3> split_forces_current_prior;
	/* True if split_forces_current_prior is suitable for use in force calculations */
	bool current_prior_imu_valid = false;
	/* Average specific force at t(k-1), from IMU data over t(k-1.5) to t(k-0.5) */
	Vector3 avg_force_prior_imu = zeros(3);
	/* Estimated specific force at t(k-1), from double differencing pos data over t(k-2) to t(k) */
	Vector3 avg_force_prior_pos = zeros(3);
	/* Tracks number of times update called to determine when sufficient data is available to begin
	 * calculating forces. */
	Size update_counts = 0;

	/*
	 * Calculate the NED delta position from origin.
	 *
	 * @param p Position to place in local level frame.
	 * @return NED position in local frame, meters NED.
	 */
	Vector3 delta_pos(const aspn_xtensor::MeasurementPosition& p) const;

	/*
	 * Calculate estimated specific force from velocity measurements.
	 *
	 * @param pos Position at which to calculate the forces (pos halfway between t_v0 and t_v1).
	 * @param v0 NED velocity at t_v0 (m/s).
	 * @param v1 NED velocity at t_v1 (m/s).
	 * @param t_v0 Time of validity for v0.
	 * @param t_v1 Time of validity for v1.
	 *
	 * @return Estimated NED specific force in m/s^2 at time halfway between t_v0 and t_v1.
	 */
	Vector3 force_from_velocity(const aspn_xtensor::MeasurementPosition& pos,
	                            const Vector3& v0,
	                            const Vector3& v1,
	                            const aspn_xtensor::TypeTimestamp& t_v0,
	                            const aspn_xtensor::TypeTimestamp& t_v1);
};
}  // namespace inertial
}  // namespace navtk
