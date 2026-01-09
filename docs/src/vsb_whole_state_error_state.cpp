#include <memory>

#include <navtoolkit.hpp>

int main() {

	// Create a dummy PVA
	auto timestamp = aspn_xtensor::TypeTimestamp(int64_t(0));
	auto header = aspn_xtensor::TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0);
	auto pva    = aspn_xtensor::MeasurementPositionVelocityAttitude(
        header,
        timestamp,
        ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
        0,
        0,
        0,
        0,
        0,
        0,
        {},
        {},
        ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
        {},
        {});

	// BEGIN
	// Assume we have a starting MeasurementPositionVelocityAttitude 'pva' previously calculated

	// Create the filter and add the basic StateBlock
	auto engine       = navtk::filtering::StandardFusionEngine(pva.get_time_of_validity());
	auto imu_model    = navtk::filtering::sagem_primus200_model();
	auto pinson_block = std::make_shared<navtk::filtering::Pinson15NedBlock>("pinson", imu_model);
	engine.add_state_block(pinson_block);

	// Create the BufferedImu that we will use to process IMU data and provide
	// a reference PVA
	auto ins = navtk::inertial::BufferedImu(pva);

	// The reference PVA source is provided to the VSB as a generic function, so
	// the user is not beholden to the BufferedImu class. As such, we capture
	// the reference to the BufferedImu and convert its output to proper format
	auto nav_fun = [&](const aspn_xtensor::TypeTimestamp& time) {
		return navtk::utils::to_navsolution(*ins.calc_pva(time));
	};

	// We create the VSB using the label to the Pinson15NedBlock, the label we want
	// to associate with the VSB output, and the function that provides the reference PVA
	auto vsb1 = std::make_shared<navtk::filtering::PinsonErrorToStandard>(
	    "pinson", "standard_at_imu", nav_fun);

	// Then we add it to the filter
	engine.add_virtual_state_block(vsb1);
	// END
}
