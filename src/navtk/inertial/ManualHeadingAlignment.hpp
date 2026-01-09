#pragma once

#include <memory>

#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/inertial/StaticAlignment.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Static alignment, but override the computed heading with a user-provided one for use when
 * gyrocompassing isn't possible. Should only be used with approximately NED
 * (z axis down) aligned IMUs to ensure heading is translated accurately.
 */
class ManualHeadingAlignment : public StaticAlignment {
public:
	/**
	 * Initialize with user-provided heading, using a leveling procedure to determine a roll and
	 * pitch that minimizes horizontal acceleration.
	 *
	 * @param heading Heading to assign to the alignment, radians (right handed rotation from north,
	 * about down axis).
	 * @param heading_sigma One sigma uncertainty in heading, radians. Defaults to the equivalent
	 * of 1 deg.
	 * @param model The model of the inertial being aligned. Used in approximating the uncertainty
	 * of the orientation in the alignment solution.
	 * @param align_time The amount of stationary IMU data that must be collected before calculating
	 * an alignment, in seconds. Once this threshold has been reached an alignment and covariance
	 * are calculated and status will change to AlignmentStatus::ALIGNED_GOOD.
	 * @param vel_cov Initial covariance to attach to the stationary NED velocity.
	 */
	ManualHeadingAlignment(const double heading,
	                       const double heading_sigma       = 0.017453292519943295,
	                       const filtering::ImuModel& model = filtering::stim300_model(),
	                       const double align_time          = 120.0,
	                       const Matrix3& vel_cov           = Matrix3{
	                                     {1e-4, 0, 0}, {0, 1e-4, 0}, {0, 0, 1e-4}});

	/**
	 * Get the covariance associated with the computed alignment.
	 *
	 *
	 * @return A pair consisting of a boolean indicating usability of solution, and the solution
	 * itself. The covariance is the same as that provided by
	 * StaticAlignment::get_computed_covariance, aside from the variance of the down tilt set
	 * according to the heading sigma provided by the user, and NE tilt covariances being calculated
	 * as a function of the filtering::ImuModel#accel_bias_initial_sigma values provided by the IMU
	 * model. Cross-terms between the accel bias and tilts are also included, as follows. \n
	 * With tilt errors means defined as \n
	 * \f$ \underline{\epsilon} = \begin{bmatrix} \epsilon_x \\ \epsilon_y
	 * \\ \epsilon_z\end{bmatrix}\f$ \n
	 * and the measured, NED frame specific force vector as \n
	 * \f$ f^n = \begin{bmatrix} f_x \\ f_y \\ f_z \end{bmatrix}\f$ \n
	 * The relation between the tilts and measured horizontal specific force components is \n
	 * \f$ f_e = g\sin{\epsilon_x} \f$ \n
	 * \f$ \epsilon_x \approx \frac{1}{g}f_e \f$ \n
	 * \f$ -f_n = g\sin{\epsilon_y} \f$ \n
	 * \f$ \epsilon_y \approx -\frac{1}{g}f_n \f$ \n
	 * Defining \n
	 * \f$ W = \begin{bmatrix} 0 & \frac{1}{g} & 0 \\ -\frac{1}{g} & 0 & 0 \\ 0 & 0 & 0
	 * \end{bmatrix} \f$ \n
	 * we have \n
	 * \f$ \underline{\epsilon} \approx W\begin{bmatrix} f_n \\ f_e \\f_d \end{bmatrix} \f$ \n
	 * the covariance of these is \n
	 * \f$ P_{\epsilon\epsilon} = E[\delta\underline{\epsilon}\delta\underline{\epsilon}^T] \f$ \n
	 * \f$ = WP_{f^nf^n}W^T \f$ \n
	 * \f$ = WC_s^nP_{f^sf^s}C^s_nW^T \f$ \n
	 * This, along with the user-provided heading/down tilt uncertainty is used to initialize the
	 * tilt covariance. \n
	 * As for the cross terms, given the provided variance of the accel bias errors (from the sensor
	 * model) \n
	 * \f$ E[\delta f_e^2] = \sigma^2_e; E[\delta f_n^2] = \sigma^2_n \f$ \n
	 * then \n
	 * \f$ E[\delta f_e\delta\epsilon_x] = E[\delta f_e\frac{1}{g}f_e] = \frac{1}{g}\sigma^2_e \f$
	 * \n \f$ E[\delta f_n\delta\epsilon_y] = E[\delta f_e\frac{1}{g}f_e] = -\frac{1}{g}\sigma^2_n
	 * \f$ \n and \n \f$ P_{f^n}\epsilon = E[\delta f^n\delta \epsilon] = \begin{bmatrix}0 &
	 * -\frac{1}{g}\sigma^2_n & 0 \\ \frac{1}{g}\sigma^2_e & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix} \f$ \n
	 *
	 * @param format Format for the covariance block.
	 */
	std::pair<bool, Matrix> get_computed_covariance(
	    const CovarianceFormat format = CovarianceFormat::PINSON15NEDBLOCK) const override;

	/**
	 * Get IMU errors
	 *
	 * @return A bool representing the validity of the attached errors, and the estimated accel
	 * (m/s^2) and gyro bias (rad/s) parameters; no other ImuErrors fields are currently estimated.
	 */
	std::pair<bool, ImuErrors> get_imu_errors() const override;

	/**
	 * Add a measurement to be used in the alignment.
	 *
	 * @param message Measurement to use. Currently, only aspn_xtensor::MeasurementPosition
	 * and aspn_xtensor::MeasurementImu from the are recognized. All others will be ignored. For
	 * imu measurements only ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED are supported.
	 * Measurements provided after status has changed to AlignmentStatus::ALIGNED_GOOD are discarded
	 * and a warning logged; the alignment will not incorporate new information.
	 *
	 * @return Status of alignment after incorporating the input.
	 *
	 * @throw std::invalid_argument if error mode is DIE and an imu measurement of type other
	 * than ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED is received.
	 */
	AlignmentStatus process(std::shared_ptr<aspn_xtensor::AspnBase> message) override;

private:
	/* Storage for user-provided heading */
	double heading;

	/* Storage for user-provided heading sigma */
	double heading_sigma;

	/* Sets `computed_alignment` and `alignment_status` if requirements met */
	void calc_align();
};
}  // namespace inertial
}  // namespace navtk
