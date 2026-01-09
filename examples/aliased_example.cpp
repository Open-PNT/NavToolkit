#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <memory>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/processors/GeodeticPos3dMeasurementProcessor.hpp>
#include <navtk/filtering/processors/PinsonPositionMeasurementProcessor.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/virtualstateblocks/EcefToStandardQuat.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonErrorToStandardQuat.hpp>
#include <navtk/filtering/virtualstateblocks/PinsonToSensorLlh.hpp>
#include <navtk/filtering/virtualstateblocks/PlatformToSensorEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/ScaleVirtualStateBlock.hpp>
#include <navtk/filtering/virtualstateblocks/SensorToPlatformEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/StandardToEcefQuat.hpp>
#include <navtk/filtering/virtualstateblocks/StateExtractor.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::TypeMounting;
using aspn_xtensor::TypeTimestamp;
using navtk::Matrix;
using navtk::Vector;
using navtk::Vector3;
using navtk::filtering::GeodeticPos3dMeasurementProcessor;
using navtk::filtering::NavSolution;
using navtk::filtering::PairedPva;
using navtk::filtering::Pinson15NedBlock;
using navtk::filtering::PinsonPositionMeasurementProcessor;
using navtk::filtering::StandardFusionEngine;
using navtk::filtering::StateExtractor;
using navtk::navutils::dcm_to_quat;
using navtk::navutils::rpy_to_dcm;

/*
 * Helper function. Simulates an inertial by generating a reference trajectory point to linearize
 * about. In this case, that point is stationary for simplicity.
 *
 * @param time Time to get solution at.
 * @return A NavSolution valid at the requested time.
 */
NavSolution reference_generator(aspn_xtensor::TypeTimestamp time) {
	return NavSolution(
	    {1.0, 2.0, 3.0}, navtk::zeros(3), xt::transpose(rpy_to_dcm({0.7, 0.8, -1.9})), time);
}

/*
 * Help function for creating a MeasurementPosition measurement at the requested time. In this
 * case, only the time varies.
 *
 * @param s Number of seconds since reference epoch to generate solution at.
 * @param ns Number of nanoseconds that when combined with s gives the complete time of measurement.
 * Defaults to 0.
 * @return MeasurementPosition with a stationary position and static covariance at the requested
 * time.
 */
std::shared_ptr<MeasurementPosition> create_meas(int s, long ns = 0) {
	Vector3 nom_pos{1.0, 2.0, 3.0};

	auto meas_llh = nom_pos + Vector3{1e-7, 3e-7, 22.0};
	auto meas_cov = xt::diag(Vector3{2.5, 6.4, 12.0});

	// Create position measurement
	auto timestamp = aspn_xtensor::TypeTimestamp(s * navtk::utils::NANO_PER_SEC + ns);
	auto header    = aspn_xtensor::TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0);
	return std::make_shared<MeasurementPosition>(header,
	                                             timestamp,
	                                             ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
	                                             meas_llh(0),
	                                             meas_llh(1),
	                                             meas_llh(2),
	                                             meas_cov,
	                                             ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
	                                             Vector{},
	                                             std::vector<aspn_xtensor::TypeIntegrity>{});
}

/*
 * Filter initialization that is common between all test cases.
 *
 * @param pinson_name Label for the Pinson state block
 * @param pinson_state 15-element initial state vector for Pinson block
 * @param pinson_cov 15x15 Pinson block initial covariance
 * @param nav_sol Initial NavSolution to supply to Pinson block as initial linearization point.
 * @return A StandardFusionEngine with a Pinson15 state block added with the supplied parameters.
 */
StandardFusionEngine setup_engine(const std::string& pinson_name,
                                  const Vector& pinson_state,
                                  const Matrix& pinson_cov,
                                  const NavSolution& nav_sol) {
	auto engine    = StandardFusionEngine();
	auto imu_model = navtk::filtering::hg1700_model();
	engine.add_state_block(std::make_shared<Pinson15NedBlock>(pinson_name, imu_model));
	engine.set_state_block_covariance(pinson_name, pinson_cov);
	engine.set_state_block_estimate(pinson_name, pinson_state);
	auto aux = navtk::utils::to_inertial_aux(nav_sol, Vector{0, 0, -9.8});
	engine.give_state_block_aux_data(pinson_name, aux);
	return engine;
}

/*
 * Comparison of a standard error state filter approach vs. two aliasing methods.
 *
 * This example generates 3 filters, all which contain Pinson15 state blocks to estimate errors
 * in an inertial process, and accept position measurement updates in the form of latitude,
 * longitude and altitude measured at the sensor location.
 *
 * The first filter has a PinsonPositionMeasurementProcessor that represents what a user would have
 * to code up to bring in a position measurement the standard way- that is accepting a measurement
 * as-is and formulating an update model that matches the StateBlock. As the block is an error
 * model, the measurement coming in must be paired with a reference trajectory point that is the
 * 'raw' inertial PVA at the time the measurement was collected (PairedPVA). The processor must also
 * 'move' either the incoming position measurement or the reference PVA by adjusting for lever arms
 * so that they are in the same frame and may be differenced. Finally, the measurement covariance
 * must undergo a change in units from meters NED to latitude in radians, longitude in radians and
 * altitude in meters to match the state units.
 *
 * The second filter takes advantage of aliasing by adding a VirtualStateBlock that transforms the
 * Pinson15 StateBlock into a virtual representation of the estimated position at the position
 * sensor. That is, rather than getting a 15-element error state to formulate a measurement against,
 * the processor will get a 3 state lat, lon, alt whole state Vector and associated covariance in
 * rad, rad, meters that has already been corrected for lever arms. This allows the
 * GeodeticPos3dMeasurementProcessor to be used instead, whose generate_model function is greatly
 * simplified as compared to PinsonPositionMeasurementProcessor.
 *
 * The third filter does the same as the second, but rather than having a single VirtualStateBlock
 * that does the full conversion, it uses a chain of 8 VirtualStateBlocks to achieve the same
 * effect. The MeasurementProcessor used is the same as in the second filter. This shows that the
 * user can build the VirtualStateBlock they need from a library of more atomic operations.
 *
 * The end result of the example is to display the relative error between the 'standard' filter and
 * the aliased versions Pinson15 state estimates after num_updates (nominally 1000).
 */
int main(int argc, char* argv[]) {
	/**** Common, shared data and other front matter *****/
	int num_updates = 1000;

	if (argc > 1) {
		num_updates = atoi(argv[1]);
	}

	// Labels for StateBlocks and MeasurementProcessors
	std::string pinson_name     = "pinson";
	std::string sensor_pos_name = "sensor_pos";
	std::string mp_name         = "processor";

	// State block initial conditions
	Vector pinson_state{
	    3.5, -1.1, 2.5, 0.0, 0.0, 0.0, 1e-9, 1e-9, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	Vector pinson_cov_diag{
	    1.0, 1.0, 1.0, 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6, 1e-9, 1e-9, 1e-9, 1e-12, 1e-12, 1e-12};
	auto pinson_cov = xt::diag(pinson_cov_diag);

	// Inertial-to-platform and platform-to-inertial sensor mountings
	auto mount_i_to_p = TypeMounting({3.0, 2.0, -4.0},
	                                 navtk::zeros(3),
	                                 dcm_to_quat(xt::transpose(rpy_to_dcm({1.2, -.7, -0.9}))),
	                                 navtk::zeros(3, 3));
	auto mount_p_to_s = TypeMounting({-1.5, 4.0, 8.0},
	                                 navtk::zeros(3),
	                                 dcm_to_quat(xt::transpose(rpy_to_dcm({0.45, -1.2, 1.3}))),
	                                 navtk::zeros(3, 3));

	auto nav_sol = reference_generator(aspn_xtensor::TypeTimestamp((int64_t)0));

	/****** Set up 'normal' filter *****/
	auto engine = setup_engine(pinson_name, pinson_state, pinson_cov, nav_sol);

	engine.add_measurement_processor(std::make_shared<PinsonPositionMeasurementProcessor>(
	    mp_name, std::vector<std::string>{pinson_name}, mount_i_to_p, mount_p_to_s));

	/******* Single Aliased filter *******/
	auto aliased = setup_engine(pinson_name, pinson_state, pinson_cov, nav_sol);

	// For this filter a single VirtualStateBlock performs the full conversion from Pinson15Ned
	// error states in the inertial sensor frame to whole state latitude, longitude and altitude
	// states at the measurement sensor. Unfortunately packing all of that complexity into a single
	// VirtualStateBlock makes the Jacobian (required for transforming the state covariance)
	// difficult to derive 'by hand'. This VirtualStateBlock therefore relies on numerical
	// differentiation, which has a number of accuracy and efficiency downsides.
	aliased.add_virtual_state_block(std::make_shared<navtk::filtering::PinsonToSensorLlh>(
	    pinson_name, sensor_pos_name, &reference_generator, mount_i_to_p, mount_p_to_s));

	aliased.add_measurement_processor(std::make_shared<GeodeticPos3dMeasurementProcessor>(
	    mp_name, sensor_pos_name, navtk::eye(3)));

	/******* Aliased filter, but using chained aliases *******/

	// In contrast with the 'Single Aliased' filter, here the filter is provided with multiple
	// VirtualStateBlocks, each of which performs a single step in the aliasing of Pinson15Ned
	// error states to the sensor frame latitude, longitude and altitude required by the
	// MeasurementProcessor. This allows for a modular aliasing approach where each and every
	// 'intermediate' alias is available, and none of the VirtualStateBlocks are so complicated
	// that they must rely on numerical differentiation.

	auto chained = setup_engine(pinson_name, pinson_state, pinson_cov, nav_sol);

	// Add the VirtualStateBlocks. Note how the 'current' and 'target' tags (the first 2 arguments
	// to each constructor) form a 'chain' that determines the order in which the transforms are
	// applied.
	// First alias- transforms from error state to whole state by combining the error states
	// with the nominal(inertial) PVA from reference_generator. Attitude is represented as a
	// quaternion.
	chained.add_virtual_state_block(std::make_shared<navtk::filtering::PinsonErrorToStandardQuat>(
	    pinson_name, "inertial_whole", &reference_generator));

	// Second alias- converts from lat, lon, alt position, NED velocity, and inertial to platform
	// quaternion to an ECEF referenced representation, to facilitate lever arm corrections.
	chained.add_virtual_state_block(
	    std::make_shared<navtk::filtering::StandardToEcefQuat>("inertial_whole", "inertial_ecef"));

	// Third alias (optional)- scales the ECEF position states from m to km, to help avoid loss of
	// precision errors during lever arm corrections.
	double sc                               = 0.001;
	auto scale_vector                       = navtk::ones(16);
	xt::view(scale_vector, xt::range(0, 3)) = sc;

	chained.add_virtual_state_block(std::make_shared<navtk::filtering::ScaleVirtualStateBlock>(
	    "inertial_ecef", "inertial_ecef_scaled", scale_vector));

	// Fourth alias- shifts from inertial sensor frame to the platform frame. Note that both this
	// block and the next one need to know how to relate the lever arm (in meters) to the position
	// states (which are now scaled to km).
	chained.add_virtual_state_block(std::make_shared<navtk::filtering::SensorToPlatformEcefQuat>(
	    "inertial_ecef_scaled", "platform_ecef", mount_i_to_p, 1.0 / sc));

	// Fifth alias- shifts from platform frame to the measurement sensor frame.
	chained.add_virtual_state_block(std::make_shared<navtk::filtering::PlatformToSensorEcefQuat>(
	    "platform_ecef", "sensor_ecef", mount_p_to_s, 1.0 / sc));

	// Sixth alias- unscales position fom km back to meters.
	chained.add_virtual_state_block(std::make_shared<navtk::filtering::ScaleVirtualStateBlock>(
	    "sensor_ecef", "sensor_ecef_unscaled", 1.0 / scale_vector));

	// Seventh alias- converts from the ECEF representation back to lat, lon, altitude position,
	// NED referenced velocity, and sensor to NED frame quaternion.
	chained.add_virtual_state_block(std::make_shared<navtk::filtering::EcefToStandardQuat>(
	    "sensor_ecef_unscaled", "sensor_whole"));

	// Final alias- allows for a select group of states (here, the nominal latitude, longitude and
	// altitude of the measurement sensor, the first 3 states of the EcefToStandardQuat transform)
	// to be treated as an individual StateBlock.
	chained.add_virtual_state_block(std::make_shared<StateExtractor>(
	    "sensor_whole", sensor_pos_name, 16, std::vector<navtk::Size>{0, 1, 2}));

	// Measurement processor, exactly the same as the second 'Single Alias' filter.
	chained.add_measurement_processor(std::make_shared<GeodeticPos3dMeasurementProcessor>(
	    mp_name, sensor_pos_name, navtk::eye(3)));

	/****** Main filter loop *******/
	for (int k = 0; k < num_updates; k++) {
		auto wrap_meas_geo_straight = create_meas(k);

		// Each filter is provided with the same measurement data; however the 'normal' filter,
		// which is using the PinsonPositionMeasurementProcessor, requires the nominal inertial
		// sensor PVA to be stapled to the measurement so that the measured and nominal positions
		// can be differenced and related to the position error states.
		auto paired        = PairedPva(wrap_meas_geo_straight, nav_sol);
		auto wrap_meas_geo = std::make_shared<PairedPva>(paired);

		engine.update(mp_name, wrap_meas_geo);
		aliased.update(mp_name, wrap_meas_geo_straight);
		chained.update(mp_name, wrap_meas_geo_straight);
	}

	// Output comparison
	auto standard_estimate = engine.get_state_block_estimate(pinson_name);
	auto aliased_estimate  = aliased.get_state_block_estimate(pinson_name);
	auto chained_estimate  = chained.get_state_block_estimate(pinson_name);

	auto aliased_error = aliased_estimate - standard_estimate;
	auto chained_error = chained_estimate - standard_estimate;

	spdlog::info("Full scale (estimate):\n{}\n", standard_estimate);
	spdlog::info("Full aliased (estimate):\n{}\n", aliased_estimate);
	spdlog::info("Full aliased (chained):\n{}\n", chained_estimate);
	spdlog::info("Full alias - standard (error):\n{}\n", aliased_error);
	spdlog::info("Chained alias - standard (error):\n{}\n", chained_error);

	return EXIT_SUCCESS;
}
