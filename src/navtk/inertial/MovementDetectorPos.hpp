#pragma once

#include <memory>
#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/inertial/MovementDetectorPlugin.hpp>
#include <navtk/inertial/MovementStatus.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace inertial {

/**
 * Accepts position data and attempts to detect movement based on delta positions.
 */
class MovementDetectorPos : public MovementDetectorPlugin {
public:
	/**
	 * Constructor.
	 *
	 * @param speed_cutoff Nominal speed threshold for platform to be considered moving, in m/s.
	 * Speed in all axes must be below this value to be classified as stationary over the period
	 * between 2 position inputs.
	 *
	 * @param zero_corr_distance Threshold distance between two sequential position measurements
	 * at which they are considered uncorrelated, in m. Correlation between the the two measurements
	 * is linearly scaled based on the norm distance between the two points; @see process().
	 * Minimum value is 1, and anything less will be silently replaced.
	 */
	MovementDetectorPos(const double speed_cutoff = 0.2, const double zero_corr_distance = 100.0);

	/**
	 * Add another position measurement and possibly update status.
	 * When a new measurement is received, the NED speed is calculated and if under `speed_cutoff`
	 * in all axes status is set to MovementStatus::NOT_MOVING. Otherwise, the delta position is
	 * compared to the covariance of the difference measurements. If the change in position in any
	 * single axis is greater than the 3 sigma bounds, status is set to MovementStatus::MOVING. For
	 * all other cases status is set to MovementStatus::POSSIBLY_MOVING. The covariance of the
	 * differenced measurements is calculated as
	 *
	 * \f$ \sigma_{diff}^2 = \sigma^2_{k-1} + \sigma^2_{k} - 2 \rho \sigma_{k - 1} \sigma_{k} \f$
	 *
	 * where \f$ \rho \f$, the correlation coefficient, is
	 *
	 * \f$ 1 - \frac{\left|\Delta p_{k-1, k}\right|}{z} \f$
	 *
	 * and \f$ z \f$ is the value of the zero_corr_distance constructor parameter.
	 *
	 * @param data Position measurement to process. Must dynamic cast to a MeasurementPosition or
	 * will be ignored.
	 *
	 * @return Best estimate of movement status based on the last 2 measurements received. If
	 *  \p data is tagged with a time earlier than the last measurement received (defined as a dt <
	 * 1e-20), a warning will be generated, or an error thrown, depending on the value of
	 * navtk::ErrorMode.
	 */
	MovementStatus process(not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) override;

	/**
	 * Time of last measurement received.
	 *
	 * @return Last time; `to_type_timestamp(0, 0)` if no measurements yet received.
	 */
	aspn_xtensor::TypeTimestamp get_time() override;

private:
	/* At least one position has been received */
	bool ready_to_test = false;

	/* Last position received */
	std::shared_ptr<aspn_xtensor::MeasurementPosition> last = nullptr;

	/* Stores ctor params */
	double speed_cutoff       = 0.0;
	double zero_corr_distance = 100.0;

	/*
	 * Calculate the 1 sigma NED uncertainty between pos and whatever data is stored in last.
	 *
	 * @param pos Most recent position measurement
	 * @param est_spd Norm of NED delta position between pos and last, divided by dt (m/s)
	 * @param dt Delta time between pos and last, sec.
	 */
	Vector calc_sig(not_null<std::shared_ptr<aspn_xtensor::MeasurementPosition>> pos,
	                double est_spd,
	                double dt);
};
}  // namespace inertial
}  // namespace navtk
