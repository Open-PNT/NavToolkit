#include <navtk/filtering/processors/PinsonPositionMeasurementProcessor.hpp>

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/processors/MeasurementProcessor.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/navutils/leverarms.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::TypeMounting;
using navtk::navutils::dcm_to_rpy;
using navtk::navutils::rpy_to_dcm;

namespace navtk {
namespace filtering {

PinsonPositionMeasurementProcessor::PinsonPositionMeasurementProcessor(
    const std::string& label,
    std::vector<std::string> state_block_labels,
    TypeMounting inertial_mount,
    TypeMounting sensor_mount)
    : MeasurementProcessor<>(label, std::move(state_block_labels)),
      inertial_mount(std::move(inertial_mount)),
      sensor_mount(std::move(sensor_mount)) {}

PinsonPositionMeasurementProcessor::PinsonPositionMeasurementProcessor(
    const PinsonPositionMeasurementProcessor& processor)
    : MeasurementProcessor(processor),
      inertial_mount(processor.inertial_mount),
      sensor_mount(processor.sensor_mount) {}

std::shared_ptr<StandardMeasurementModel> PinsonPositionMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	// Extract raw data containers. PairedPva has 2 elements: the actual measurement from the
	// sensor and the inertial solution valid at the same time as the measurement.
	auto paired = std::dynamic_pointer_cast<PairedPva>(measurement);
	if (paired == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (PairedPva). Unable to perform update.");
		return nullptr;
	}
	auto data = std::dynamic_pointer_cast<MeasurementPosition>(paired->meas_data);
	if (data == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "PairedPva measurement data is not of correct type (MeasurementPosition). Unable to "
		    "perform update.");
		return nullptr;
	}

	auto nav_sol = paired->ref_pva;

	if (data->get_reference_frame() != ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC)
		spdlog::warn("Expected a MeasurementPosition with a geodetic reference frame.");

	auto latitude  = data->get_term1();
	auto longitude = data->get_term2();
	auto altitude  = data->get_term3();

	if (isnan(latitude) || isnan(longitude) || isnan(altitude))
		log_or_throw(
		    "Expected MeasurementPosition to be a 3D measurement but received NaN's for one "
		    "or more terms.");

	// Format data into frames that are required for lever arm corrections; provided functions
	// require a Cartesian position vector and the rotation between the position reference frame
	// (here ECEF) and the attitude reference frame (NED)
	Vector3 inertial_llh = nav_sol.pos;
	auto inertial_ecef   = navutils::llh_to_ecef(inertial_llh);
	auto C_nav_to_sensor = nav_sol.rot_mat;
	auto C_nav_to_ecef   = navutils::llh_to_cen(inertial_llh);

	// Correct the raw measurement for the lever arm between the sensor and platform, producing an
	// ECEF position vector
	auto platform_ecef = navutils::sensor_to_platform(
	    std::pair<Vector3, Matrix3>(inertial_ecef, C_nav_to_sensor),
	    inertial_mount.get_lever_arm(),
	    navtk::navutils::quat_to_dcm(inertial_mount.get_orientation_quaternion()),
	    C_nav_to_ecef);

	// Second correction, from platform to inertial sensor frame
	auto sensor_ecef = navutils::platform_to_sensor(
	    platform_ecef,
	    sensor_mount.get_lever_arm(),
	    navtk::navutils::quat_to_dcm(sensor_mount.get_orientation_quaternion()),
	    C_nav_to_ecef);

	// Convert the corrected ECEF postion back to LLH, difference with the inertial position,
	// and convert to delta NED values
	auto sensor_llh = navutils::ecef_to_llh(sensor_ecef.first);
	auto lat_fac    = navutils::delta_lat_to_north(1, sensor_llh[0], sensor_llh[2]);
	auto lon_fac    = navutils::delta_lon_to_east(1, sensor_llh[0], sensor_llh[2]);
	auto delta_n    = lat_fac * (latitude - sensor_llh[0]);
	auto delta_e    = lon_fac * (longitude - sensor_llh[1]);
	auto delta_d    = -(altitude - sensor_llh[2]);

	// Construct the measurement vector for the Kalman update, and generate the measurement matrix
	// relating the 15 Pinson states to the measurement. Since the measurement and state units now
	// match, the relations between the 2 is identity
	auto xhat_p = gen_x_and_p_func(get_state_block_labels());
	if (xhat_p == nullptr) {
		return nullptr;
	}
	Vector meas{delta_n, delta_e, delta_d};
	Matrix meas_matrix = zeros(3, num_rows(xhat_p->estimate));
	meas_matrix(0, 0)  = 1.0;
	meas_matrix(1, 1)  = 1.0;
	meas_matrix(2, 2)  = 1.0;

	// Return the MeasurementModel; the non-linear update function z = h(x) is equivalent to H*x.
	return std::make_shared<StandardMeasurementModel>(
	    StandardMeasurementModel(meas, meas_matrix, data->get_covariance()));
}

void PinsonPositionMeasurementProcessor::receive_aux_data(const AspnBaseVector& aux_data) {
	auto metadata = std::dynamic_pointer_cast<aspn_xtensor::MetadataGeneric>(aux_data[0]);
	if (metadata != nullptr) {
		std::string label = metadata->get_info().get_sensor_description();
		if (label == get_label()) sensor_mount = metadata->get_mounting();
		if (label == get_state_block_labels()[0]) inertial_mount = metadata->get_mounting();
	} else {
		MeasurementProcessor::receive_aux_data(aux_data);
	}
}

not_null<std::shared_ptr<MeasurementProcessor<>>> PinsonPositionMeasurementProcessor::clone() {
	return std::make_shared<PinsonPositionMeasurementProcessor>(*this);
}

}  // namespace filtering
}  // namespace navtk
