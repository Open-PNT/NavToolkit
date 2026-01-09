#pragma once

#include <navtk/inertial/AlignBase.hpp>

namespace navtk {
namespace inertial {

/**
 * Class implementing a gyrocompassed static alignment. In general, the goal of this class is to
 * produce an inertial alignment that it transmits to the inertial instance, returning what
 * the status of the alignment is to the message sender.
 *
 * To enable an alignment calculation, all provided measurements should be from a stationary
 * vehicle, IMU measurements must span a user-specified length of time, and at least one
 * MeasurementPosition measurement must be provided. Imu data (specifically gyro measurements) must
 * generally be navigation-grade or higher; low sensitivity or highly-biased gyros will produce a
 * poor result.
 */
class StaticAlignment : public AlignBase {

public:
	/**
	 * Constructor.
	 *
	 * @param model The model of the inertial being aligned. Used in approximating the uncertainty
	 * of the orientation in the alignment solution.
	 * @param align_time The amount of IMU data that must be collected before calculating an
	 * alignment, in seconds. Once this threshold has been reached an alignment and covariance
	 * are calculated and status will change to AlignmentStatus::ALIGNED_GOOD.
	 * @param vel_cov Initial covariance to attach to the stationary NED velocity.
	 */
	StaticAlignment(const filtering::ImuModel& model = filtering::stim300_model(),
	                const double align_time          = 120.0,
	                const Matrix3& vel_cov = Matrix3{{1e-4, 0, 0}, {0, 1e-4, 0}, {0, 0, 1e-4}});

	/**
	 * Add a measurement to be used in the alignment.
	 *
	 * @param message Measurement to use. Currently, only aspn_xtensor::MeasurementPosition
	 * and aspn_xtensor::MeasurementImu are recognized. All others will be ignored.
	 * Measurements provided after status has changed to AlignmentStatus::ALIGNED_GOOD are discarded
	 * and a warning logged; the alignment will not incorporate new information.
	 *
	 * @return Status of alignment after incorporating the input.
	 */
	AlignmentStatus process(std::shared_ptr<aspn_xtensor::AspnBase> message) override;

	/**
	 * Get the covariance associated with the computed alignment.
	 *
	 * @param format Format for the covariance block.
	 *
	 * @return A pair consisting of a boolean indicating usability of covariance, and the
	 * covariance itself. If no alignment has been computed (status !=
	 * AlignmentStatus::ALIGNED_GOOD), then the bool is false and the covariance invalid. Otherwise
	 * the bool is true and the covariance is a block diagonal 9x9 Matrix of position covariance
	 * (m^2 NED), velocity covariance (m^2/s^2, NED), and tilt covariance (rad^2, NED). The position
	 * covariance is the covariance of all provided MeasurementPosition measurements, or in the case
	 * of only a single measurement being available, a copy of that measurements' covariance field.
	 * Velocity covariance is fixed at 1e-4. The tilt covariance is a first-order approximation
	 * assuming only zero-mean constant biases on the IMU measurements based on the supplied
	 * ImuModel. Cross-terms between blocks are all 0.
	 */
	std::pair<bool, Matrix> get_computed_covariance(
	    const CovarianceFormat format = CovarianceFormat::PINSON15NEDBLOCK) const override;

	MotionNeeded motion_needed() const override;

protected:
	/**
	 * Do the actual alignment algorithm once sufficient data is available.
	 *
	 * @param imu_time Latest received IMU time, to be affixed to the generated solution.
	 */
	virtual void calc_alignment(const aspn_xtensor::TypeTimestamp& imu_time);

	/**
	 * Calculate the mean and cov of all entries in gps_buffer.
	 *
	 * @return Mean pos (llh in rad, rad, m) and cov (in m^2 NED).
	 */
	std::pair<Vector3, Matrix3> pos_stats() const;

	/**
	 * Averages measurements in align_buffer and returns avg dv and dth. It does not account for
	 * dt and therefore assumes constant rate of IMU data.
	 *
	 * @return Average delta velocity (m/s) and delta theta (rad) from buffered measurements.
	 */
	std::pair<Vector3, Vector3> calc_avg() const;

	/**
	 * @return The average delta time between measurements in the alignment buffer, in seconds.
	 */
	double calc_average_delta_time() const;

	/**
	 * Minimum time in seconds for recording of sensor inputs before alignment is attempted.
	 */
	double align_time;

private:
	/* We've yet to get a good alignment, so keep trying to align using the latest message */
	void aligning(const aspn_xtensor::MeasurementImu& message);

	/* Check if enough IMU/GPS data has been buffered to do a static align */
	bool sufficient_data();

	/* Storage for tilt cov from sensor messages for use in solution cov output */
	Matrix3 tilt_cov;

	/* Storage for tilt cov from sensor messages for use in solution cov output */
	Matrix3 vel_cov;
};

}  // namespace inertial
}  // namespace navtk
