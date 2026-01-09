#pragma once

#include <memory>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/inertial/BufferedImu.hpp>
#include <navtk/inertial/ImuErrors.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>

namespace navtk {
namespace inertial {

/**
 * BasicInsAndFilter Implements a BufferedImu and EKF with Pinson15 error state block and
 * filtering::PinsonPositionMeasurementProcessor.
 */
class BasicInsAndFilter {

public:
	/**
	 * Constructor
	 *
	 * @param pva Time-tagged initial position, velocity and attitude. Attitude in `pva` should be
	 * w.r.t. frame of IMU measurements to be mechanized.
	 * @param model IMU model
	 * @param initial_imu Imu measurement measured over the `expected_dt` period prior to `pva`
	 * measurement time; `nullptr` if unknown. Failure to provide an appropriate value will result
	 * in possibly inaccurate IMU-based values (specific forces, rotation rates) being returned from
	 * their respective functions over the first mechanization period.
	 * @param imu_errs Initial values of IMU error parameters. Time of validity is assumed to be
	 * the same as that of `pva`.
	 * @param expected_dt The typical delta time over which an IMU measurement is measured, in
	 * seconds.
	 * @param mech_options Gravity model, DCM integration method, etc. to use when mechanizing
	 * inertial data.
	 */
	BasicInsAndFilter(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva,
	                  const filtering::ImuModel& model = filtering::stim300_model(),
	                  std::shared_ptr<aspn_xtensor::MeasurementImu> initial_imu = nullptr,
	                  const ImuErrors& imu_errs                                 = ImuErrors{},
	                  const double expected_dt                                  = 0.01,
	                  const MechanizationOptions& mech_options = MechanizationOptions{});


	/**
	 * Propagate the current inertial solution by integrating the supplied IMU measurement.
	 *
	 * @param imu Imu measurement to mechanize. IMU data with a delta time between measurements
	 * that varies by 50% or more from the average delta will generate a warning. To account for
	 * the possibility that this class may have been initialized 'between' IMU measurements,
	 * the input to the first call of this function may be scaled accordingly.
	 */
	void mechanize(const aspn_xtensor::MeasurementImu& imu);

	/**
	 * Update the filter with a position measurement and feed back state estimates into the
	 * inertial.
	 * @param gp3d Position measurement valid at or later than current filter time.
	 */
	void update(const aspn_xtensor::MeasurementPosition& gp3d);

	/**
	 * Get the PVA solution at time \p t. Attached covariance is most current and is not guaranteed
	 * to valid at \p t (see calc_pva()).
	 *
	 * @param t Time of the requested solution.
	 *
	 * @return The solution at time `t`. If no solution is available at \p t, a `nullptr`.
	 */
	std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> calc_pva(
	    const aspn_xtensor::TypeTimestamp& t) const;

	/**
	 * Get most recent solution. Attached covariance is most current available and can only be
	 * guaranteed to be valid at solution time if a) mechanize() and update() are called in 'sorted'
	 * order (i.e. user does not mechanize ahead of the filter) and b) this function is called
	 * immediately after update() is called.
	 *
	 * @return Most recent solution.
	 */
	std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> calc_pva() const;

	/**
	 * Calculate IMU errors
	 *
	 * @return Estimated IMU errors.
	 */
	ImuErrors calc_imu_errors() const;

	/**
	 * Get the current covariance from Pinson state block.
	 *
	 * @return 15x15 covariance block, units as in navtk::filtering::Pinson15NedBlock.
	 */
	Matrix get_pinson15_cov() const;

private:
	const std::string p_tag = "ins_states";
	const std::string g_tag = "pos";
	filtering::ImuModel model;
	BufferedImu ins;
	filtering::StandardFusionEngine engine;
};
}  // namespace inertial
}  // namespace navtk
