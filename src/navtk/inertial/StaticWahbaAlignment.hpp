#pragma once

#include <navtk/inertial/StaticAlignment.hpp>

namespace navtk {
namespace inertial {

/**
 * A variation of StaticAlignment that compares average delta velocities and delta rotations
 * measured on a stationary platform against simulated NED values of the same and determines the
 * rotation between them via solve_wahba_svd().
 */
class StaticWahbaAlignment : public StaticAlignment {

public:
	/**
	 * Constructor.
	 *
	 * @param model The model of the inertial being aligned. Used in approximating the uncertainty
	 * of the orientation in the alignment solution.
	 * @param align_time The amount of IMU data that must be collected before calculating an
	 * alignment, in seconds. Once this threshold has been reached an alignment and covariance
	 * are calculated and status will change to AlignmentStatus::ALIGNED_GOOD. The type of imu
	 * available (ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED or
	 * ASPN_MEASUREMENT_IMU_IMU_TYPE_SAMPLED) is assumed constant for all supplied imu data.
	 * @param vel_cov Initial covariance to attach to the stationary NED velocity.
	 */
	StaticWahbaAlignment(const filtering::ImuModel& model = filtering::stim300_model(),
	                     const double align_time          = 120.0,
	                     const Matrix3& vel_cov           = Matrix3{
	                                   {1e-4, 0, 0}, {0, 1e-4, 0}, {0, 0, 1e-4}});

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

protected:
	/* Do the actual alignment algorithm once sufficient data is available. */
	void calc_alignment(const aspn_xtensor::TypeTimestamp& imu_time) override;

private:
	/* Storage for tilt cov from sensor messages for use in solution cov output */
	Matrix3 tilt_cov;
};

}  // namespace inertial
}  // namespace navtk
