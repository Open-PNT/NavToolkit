#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/inertial/ImuErrors.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

/**
 * Enumerates the various types of motion needed by the alignment strategy.
 */
enum class MotionNeeded {
	/**
	 * Stationary data is needed.
	 */
	NO_MOTION,
	/**
	 * Dynamic data is needed.
	 */
	MOTION_NEEDED,
	/**
	 * No particular type of motion is required.
	 */
	ANY_MOTION
};

/** Base class for IMU alignment algorithms. */
class AlignBase {
public:
	/** The possible states of the return value of get_computed_alignment(). */
	enum class AlignmentStatus {
		/**
		 * We are attempting to align the inertial. The requires_dynamic() function should be
		 * checked to determine if the aligning algorithm requires movement to align.
		 */
		ALIGNING_COARSE,
		/**
		 * A coarse alignment has been calculated, and
		 * the alignment is being tested and/or adjusted before setting the inertial.
		 */
		ALIGNING_FINE,
		/**
		 * We have a good INS alignment which can be used to initialize an inertial class and INS
		 * error state block.
		 */
		ALIGNED_GOOD
	};

	/** Supported filtering::StateBlock types for returned covariance size/units */
	enum class CovarianceFormat {
		/**
		 * Position, velocity, attitude and sensor biases. See navtk::filtering::Pinson15NedBlock.
		 */
		PINSON15NEDBLOCK,
		/**
		 * Position, velocity, attitude, sensor biases and scale factors. See
		 * navtk::filtering::Pinson21NedBlock.
		 */
		PINSON21NEDBLOCK,
	};

public:
	virtual ~AlignBase() = default;

	/**
	 * Base constructor which sets `supports_static` and `supports_dynamic`
	 *
	 * @param supports_static Indicates the alignment algorithm supports static data
	 * @param supports_dynamic Indicates the alignment algorithm supports dynamic data
	 * @param model Model for IMU measurements used in alignment, typically used in calculating
	 * covariance of alignment solution.
	 */
	AlignBase(bool supports_static,
	          bool supports_dynamic,
	          const filtering::ImuModel& model = filtering::stim300_model());

	/**
	 * Copy constructor for AlignBase. Performs a shallow copy of \p other except for
	 * `computed_alignment` which is deep copied.
	 *
	 * @param other AlignBase object to copy.
	 */
	AlignBase(const AlignBase& other) = default;

	/**
	 * Default move constructor for AlignBase.
	 *
	 * @param other AlignBase object to move.
	 */
	AlignBase(AlignBase&& other) = default;

	/**
	 * Copy assignment operator for AlignBase. Performs a shallow copy of \p other except for
	 * `computed_alignment` which is deep copied.
	 *
	 * @param other AlignBase object to copy.
	 * @return Copy of \p other .
	 */
	AlignBase& operator=(const AlignBase& other) = default;

	/**
	 * Default move assignment operator for AlignBase.
	 *
	 * @param other AlignBase object to move.
	 * @return Reference to the moved \p other .
	 */
	AlignBase& operator=(AlignBase&& other) = default;

	/**
	 * Supply new data to alignment estimation algorithm. Behavior may vary based on current
	 * `alignment_status`.
	 *
	 * @param message Pointer to aspn_xtensor::AspnBase.
	 *
	 * @return Current alignment status following message processing.
	 */
	virtual AlignmentStatus process(std::shared_ptr<aspn_xtensor::AspnBase> message) = 0;

	/**
	 * Allows caller to determine if alignment requires movement.
	 *
	 * @return `true` if movement required for alignment.
	 *
	 * @throw std::runtime_error if class has not been flagged for supporting static or dynamic and
	 * the error mode is ErrorMode::DIE.
	 */
	bool requires_dynamic();

	/**
	 * Determine the current alignment status.
	 *
	 * @return Current alignment status.
	 */
	AlignmentStatus check_alignment_status();

	/**
	 * Get the computed alignment if one has been calculated.
	 *
	 * @return A pair with a bool representing validity of the attached solution, along with a copy
	 * of the computed alignment if one has been calculated.
	 */
	virtual std::pair<bool, filtering::NavSolution> get_computed_alignment() const;

	/**
	 * Get the computed position, velocity and attitude covariance matrix if one has been
	 * calculated.
	 *
	 * @param format Format for the covariance block. Covariance, if able to be generated, will be
	 * of the correct size and units for the requested block type.
	 * @return A pair with a bool representing validity of the attached matrix, and an NxN
	 * covariance matrix where N is the number of states requested in \p format.
	 */
	virtual std::pair<bool, Matrix> get_computed_covariance(
	    const CovarianceFormat format = CovarianceFormat::PINSON15NEDBLOCK) const;

	/**
	 * Get any estimated inertial sensor errors.
	 *
	 * @return A pair with a bool representing validity of the attached ImuErrors, and the current
	 * estimates of the IMU errors.
	 */
	virtual std::pair<bool, ImuErrors> get_imu_errors() const;

	/**
	 * @return Indicates whether the alignment strategy requires stationary data, motion, or doesn't
	 * care.
	 */
	virtual MotionNeeded motion_needed() const = 0;

protected:
	/**
	 * Extract the initial bias covariance etc.from model and generate a covariance block. Does not
	 * include PVA terms (cov block will just be sensor errors, for example
	 * CovarianceFormat::PINSON15NEDBLOCK format will be 6x6)
	 *
	 * @param format Partial covariance type to return.
	 *
	 * @return Covariance, initialized from model.
	 */
	Matrix bias_stats_from_model(
	    const CovarianceFormat format = CovarianceFormat::PINSON15NEDBLOCK) const;

	/** Current alignment status */
	AlignmentStatus alignment_status = AlignmentStatus::ALIGNING_COARSE;

	/** Current available alignment */
	std::pair<bool, filtering::NavSolution> computed_alignment;

	/** Buffer for IMU data */
	std::vector<aspn_xtensor::MeasurementImu> align_buffer;

	/** Buffer for position data */
	std::vector<aspn_xtensor::MeasurementPosition> gps_buffer;

	/** Flag indicating the alignment algorithm supports static data */
	bool supports_static;

	/** Flag indicating the alignment algorithm supports dynamic data */
	bool supports_dynamic;

	/** IMU model for incoming measurements */
	filtering::ImuModel model;
};

}  // namespace inertial
}  // namespace navtk
