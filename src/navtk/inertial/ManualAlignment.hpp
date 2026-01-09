#pragma once

#include <navtk/aspn.hpp>
#include <navtk/inertial/AlignBase.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Generates an alignment using some mix of upfront user-provided information and 'live' sensor
 * data. The solution is initialized with the user-provided PVA solution. If additional sensor data
 * is supplied it will be used to update appropriate fields.
 * Solution times are tagged with the latest IMU `time_validity` if IMU data have been supplied,
 * allowing for a seamless redirection of the IMU data stream from the alignment to the aligned
 * inertial class. If only other data types are supplied for the alignment, the time from these
 * measurements are used to tag the generated solution. This may cause an inaccurate mechanization
 * period/dt when the first IMU measurement is supplied to the mechanizing class; to avoid errors
 * the user should adjust the solution time to accurately reflect the IMU measurement period.
 *
 * No interpolation between or sorting of sensor measurements is performed.
 */
class ManualAlignment : public AlignBase {
public:
	/**
	 * Provide an initial alignment solution and choose AlignBase::AlignmentStatus::ALIGNED_GOOD
	 * trigger conditions.
	 *
	 * @param pva Default PVA and covariance. Must be in
	 * ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC reference frame.
	 * @param wait_for_time When true, require that at least one IMU measurement be received (thus
	 * tagging the alignment solution with IMU time) before switching to
	 * AlignBase::AlignmentStatus::ALIGNED_GOOD.
	 * @param wait_for_pos When true, require that at least one position measurement be received
	 * (thus updating the position of \p pva) before switching to
	 * AlignBase::AlignmentStatus::ALIGNED_GOOD.
	 * @param wait_for_vel When true, require that at least one velocity measurement be received
	 * (thus updating the velocity of \p pva) before switching to
	 * AlignBase::AlignmentStatus::ALIGNED_GOOD.
	 * @param wait_for_att When true, require that at least one attitude measurement be received
	 * (thus updating the attitude of \p pva) before switching to
	 * AlignBase::AlignmentStatus::ALIGNED_GOOD.
	 * @param model Model for IMU measurements used in alignment, typically used in calculating
	 * covariance of alignment solution.
	 *
	 * If all of the above bool params are `false` (the default), this class is immediately placed
	 * in an AlignBase::AlignmentStatus::ALIGNED_GOOD state with \p pva as the available solution.
	 * Otherwise status remains AlignBase::AlignmentStatus::ALIGNING_COARSE until selected
	 * conditions are met.
	 *
	 * @throw std::runtime_error If ErrorMode::DIE, an invalid \p pva field is supplied and the
	 * corresponding 'wait_for' flag is set to false.
	 * @throw std::invalid_argument If ErrorMode::DIE and covariance shape of \p pva does not match
	 * the number of valid data elements in the message.
	 * @throw std::invalid_argument If the \p pva reference frame is not
	 * ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC.
	 */
	ManualAlignment(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva,
	                bool wait_for_time               = false,
	                bool wait_for_pos                = false,
	                bool wait_for_vel                = false,
	                bool wait_for_att                = false,
	                const filtering::ImuModel& model = filtering::stim300_model());

	/**
	 * Update the alignment solution with various measurements by copying the relevant fields into
	 * the PVA solution.
	 * @return Current alignment status.
	 * @param message Data to align with. Supported data types and effects are: \n
	 * aspn_xtensor::MeasurementImu : Update the solution time and stops any other sensor time from
	 * being used. \n aspn_xtensor::MeasurementPosition : Update position and position covariance.
	 * Updates time only if no IMU data has been received. Only
	 * ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC reference frame supported. \n
	 * aspn_xtensor::MeasurementVelocity : Update velocity and velocity covariance. Updates time
	 * only if no IMU data has been received. Only ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED
	 * and ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_ECEF frames are supported.
	 * \n aspn_xtensor::MeasurementAttitude3D : Update attitude and attitude covariance. Updates
	 * time only if no IMU data has been received. Only
	 * ASPN_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_NED and
	 * ASPN_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_ECEF are supported.
	 * \n aspn_xtensor::MeasurementPositionVelocityAttitude : Replaces the entire solution,
	 * including covariance, depending on what data elements are considered valid. Updates time
	 * only if no IMU data has been received. Only
	 * ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC reference frame is
	 * supported.
	 *
	 * Generally speaking, the following holds for all inputs:
	 * 1. Measurements are only considered valid if all values representing a 3 dimensional
	 * measurement are valid. For example, if a aspn_xtensor::MeasurementPosition returns NAN for
	 * get_p2(), the entire measurement is rejected.
	 * 2. It is acceptable for 'grouped' messages such as
	 * aspn_xtensor::MeasurementPositionVelocityAttitude to have 'partial validity', for example an
	 * invalid position but valid velocity and attitude. In such a case the velocity and attitude
	 * will be used for alignment and the position ignored.
	 * 3. Measurements are considered invalid if their covariance matrix cannot be interpreted to
	 * support the available data.
	 * 4. A valid position of any supported type must be provided before non-position ECEF-frame
	 * data can be interpreted for alignment. ECEF frame data delivered before a valid position has
	 * been provided is ignored.
	 * 5. For all messages, warnings are generated if an unsupported reference frame is
	 * supplied and the data is ignored.
	 *
	 * @throw std::runtime_error If ErrorMode::DIE and message is an unsupported data type.
	 * @throw std::invalid_argument If ErrorMode::DIE and covariance shape does not match the
	 * number of valid data elements in the message.
	 */
	AlignBase::AlignmentStatus process(std::shared_ptr<aspn_xtensor::AspnBase> message) override;

	/**
	 * Covariance of solution. Baseline is what the user provided at construction on the
	 * MeasurementPositionVelocityAttitude parameter; individual diagonal blocks (position,
	 * velocity, attitude) may be updated depending on the rest of the settings and what data has
	 * been received. See the constructor and process(std::shared_ptr<aspn_xtensor::AspnBase>)
	 * function documentation for details.
	 *
	 * @param format Format for the covariance block.
	 *
	 * @return A pair containing a bool describing the validity of the covariance, and the current
	 * covariance matrix.
	 */
	std::pair<bool, Matrix> get_computed_covariance(
	    const CovarianceFormat format = CovarianceFormat::PINSON15NEDBLOCK) const override;

	MotionNeeded motion_needed() const override;

private:
	/* The timestamp of the most recent IMU message received */
	aspn_xtensor::TypeTimestamp last_imu_time = aspn_xtensor::to_type_timestamp();

	/* Flag indicating if last_imu_time can be used to time-tag generated NavSolutions */
	bool use_imu_time;

	/* Copy of PVA covariance from sensor messages for use in solution covariance output */
	Matrix cov;

	/* Storage for user-provided constructor parameters */
	std::vector<bool> wait_for;

	/* Log/throw if desired, otherwise update solution time and status */
	AlignBase::AlignmentStatus post_meas_received(
	    bool should_throw,
	    const aspn_xtensor::TypeTimestamp& meas_time = aspn_xtensor::to_type_timestamp());

	/*
	 * Figure out what data is valid and assign into alignment pva/covariance.
	 *
	 * @param pva Candidate PositionVelocityAttitude.
	 * @param set_flags Whether or not to allow the function to change the wait_for flags based on
	 * supplied_data.
	 * @return bool indicating that all valid data was assigned, effectively only false when the
	 * covariance matrix size does not match the number of valid data elements.
	 */
	bool assign_from_pva(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva,
	                     bool set_flags = true);
};
}  // namespace inertial
}  // namespace navtk
