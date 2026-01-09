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

	// Create a dummy inertial
	auto ins = navtk::inertial::BufferedImu(pva);

	// Create a dummy fusion engine
	auto engine = navtk::filtering::StandardFusionEngine();

	// BEGIN
	// Assume we have previously created an instance of BufferedImu called "ins" and
	// StandardFusionEngine called "engine".

	std::function<navtk::filtering::NavSolution(const aspn_xtensor::TypeTimestamp& time)> nav_fun =
	    [&](const aspn_xtensor::TypeTimestamp& time) {
		    return navtk::utils::to_navsolution(*ins.calc_pva(time));
	    };

	// Define the lever arms/sensor rotations between the platform frame and the two sensors
	// Each is tagged with the target label of the VSB it is associated with
	navtk::Vector quat{1, 0, 0, 0};
	auto mount1 = aspn_xtensor::TypeMounting(
	    navtk::Vector3{1.0, -2.0, 0.0}, navtk::zeros(3), quat, navtk::zeros(3, 3));
	auto mount2 = aspn_xtensor::TypeMounting(
	    navtk::Vector3{0.0, 0.0, 4.0}, navtk::zeros(3), quat, navtk::zeros(3, 3));

	// Create a StateBlock and add it to the fusion engine.
	auto imu_model    = navtk::filtering::sagem_primus200_model();
	auto pinson_block = std::make_shared<navtk::filtering::Pinson15NedBlock>("pinson", imu_model);
	engine.add_state_block(pinson_block);

	// Create all of the VSBs. All of the less-important conversions are given single-letter 'dummy'
	// tags; using more meaningful signifiers is a good idea

	// This block is similar to PinsonErrorToStandard, but with a quaternion attitude representation
	auto vsb1 =
	    std::make_shared<navtk::filtering::PinsonErrorToStandardQuat>("pinson", "A", nav_fun);
	// Converts from LLH 'Standard' representation to ECEF units
	auto vsb2 = std::make_shared<navtk::filtering::StandardToEcefQuat>("A", "B");
	// Shifts the PVA to the platform frame
	auto vsb3 = std::make_shared<navtk::filtering::SensorToPlatformEcefQuat>("B", "C", mount1);
	// Shifts the platform frame PVA to the GPS sensor frame
	auto vsb4 = std::make_shared<navtk::filtering::PlatformToSensorEcefQuat>("C", "D", mount2);
	// Converts ECEF units back to LLH
	auto vsb5 = std::make_shared<navtk::filtering::EcefToStandardQuat>("D", "E");
	// Pulls out just the states of interest--the first 3. Must know the number of input states
	// (16- 3 pos, 3 vel, 4 quaternion, and 6 IMU sensor error)
	auto vsb6 = std::make_shared<navtk::filtering::StateExtractor>(
	    "E", "pos_at_gps", 16, std::vector<navtk::Size>{0, 1, 2});

	// Add all the VSBs to the filter
	engine.add_virtual_state_block(vsb1);
	engine.add_virtual_state_block(vsb2);
	engine.add_virtual_state_block(vsb3);
	engine.add_virtual_state_block(vsb4);
	engine.add_virtual_state_block(vsb5);
	engine.add_virtual_state_block(vsb6);

	// Get the ECEF PVA at the platform frame
	engine.get_state_block_estimate("C");

	// Get the position covariance at the gps frame
	engine.get_state_block_covariance("pos_at_gps");
	// END
}
