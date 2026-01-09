#include <navtk/filtering/processors/DirectionToPoints3dMeasurementProcessor.hpp>

#include <vector>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>

using aspn_xtensor::MeasurementDirection3DToPoints;
using aspn_xtensor::MetadataGeneric;
using navtk::navutils::quat_to_dcm;

namespace navtk {
namespace filtering {

namespace {
Matrix project_world_points(const Matrix& ccn,
                            const Matrix& xn,
                            const Vector3& tn,
                            const Vector3& tilts,
                            const Vector3& delta_pos) {

	Matrix xn_t = xt::transpose(xn);
	Matrix map  = zeros(num_rows(xn_t), num_cols(xn_t));
	for (std::size_t col = 0; col < num_cols(xn_t); col++) {
		xt::view(map, xt::all(), col) = xt::view(xn_t, xt::all(), col) - delta_pos - tn;
	}
	return xt::transpose(dot(dot(eye(3) - navutils::skew(tilts), ccn), map));
}
}  // namespace

DirectionToPoints3dMeasurementProcessor::DirectionToPoints3dMeasurementProcessor(
    std::string label, const std::string& state_block_label)
    : MeasurementProcessor(std::move(label), std::vector<std::string>(1, state_block_label)) {}

std::shared_ptr<StandardMeasurementModel> DirectionToPoints3dMeasurementProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	std::shared_ptr<PairedPva> input_meas = std::dynamic_pointer_cast<PairedPva>(measurement);
	if (input_meas == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (PairedPva). Unable to perform update.");
		return nullptr;
	}
	auto data = std::dynamic_pointer_cast<MeasurementDirection3DToPoints>(input_meas->meas_data);
	if (data == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "PairedPva measurement data is not of correct type (MeasurementDirection3DToPoints). "
		    "Unable to perform update.");
		return nullptr;
	}

	auto ins_pva = input_meas->ref_pva;
	auto xhat_p  = gen_x_and_p_func(get_state_block_labels());
	if (xhat_p == nullptr) {
		return nullptr;
	}

	// Set up measurements
	auto obs                      = data->get_obs();
	auto num_obs                  = obs.size();
	Matrix world_points           = zeros(num_obs, 3);
	Matrix projected_measurements = zeros(num_obs, 2);
	std::vector<Matrix> pos_cov(num_obs);
	for (size_t idx = 0; idx < num_obs; idx++) {
		auto observation                                 = obs[idx];
		xt::view(projected_measurements, idx, xt::all()) = observation.get_obs();
		auto remote_point                                = observation.get_remote_point();
		if (remote_point.get_position_reference_frame() !=
		    ASPN_TYPE_REMOTE_POINT_POSITION_REFERENCE_FRAME_GEODETIC)
			log_or_throw(
			    "Received a MeasurementDirection3DToPoints with a remote point whose reference "
			    "frame is not geodetic.");
		auto latitude  = remote_point.get_position1();
		auto longitude = remote_point.get_position2();
		auto altitude  = remote_point.get_position3();
		if (isnan(latitude) || isnan(longitude) || isnan(altitude))
			log_or_throw(
			    "Received a MeasurementDirection3DToPoints with a remote point whose position "
			    "contains NaN's.");
		xt::view(world_points, idx, xt::all()) = Vector{latitude, longitude, altitude};
		pos_cov[idx]                           = remote_point.get_position_covariance();
	}

	Matrix ccn       = dot(ccb_mat, ins_pva.rot_mat);
	Vector3 ref_ecef = navutils::llh_to_ecef(xt::view(world_points, 0, xt::all()));
	Vector3 tn       = navutils::ecef_to_local_level(ref_ecef, navutils::llh_to_ecef(ins_pva.pos));
	Matrix xn        = zeros(num_rows(world_points), num_cols(world_points));
	for (std::size_t row = 0; row < num_rows(world_points); row++) {
		xt::view(xn, row, xt::all()) = navutils::ecef_to_local_level(
		    ref_ecef, navutils::llh_to_ecef(xt::view(world_points, row, xt::all())));
	}
	Matrix xi = project_world_points(ccn, xn, tn, zeros(3), zeros(3));

	// Build residual Vector
	auto size = 2 * num_rows(projected_measurements);
	Vector z  = zeros(size);
	for (std::size_t row = 0; row < num_rows(projected_measurements); row++) {
		xt::view(z, xt::range(2 * row, 2 * row + 2)) =
		    xt::view(projected_measurements, row, xt::all());
	}

	// Build Measurement Function
	auto h = [size = size, ccn = ccn, xn = xn, tn = tn](const Vector& xhat) {
		Vector z_est      = zeros(size);
		Matrix projection = project_world_points(
		    ccn, xn, tn, xt::view(xhat, xt::range(6, 9)), xt::view(xhat, xt::range(0, 3)));
		for (std::size_t row = 0; row < num_rows(projection); row++) {
			z_est[2 * row]     = projection(row, 0) / projection(row, 2);
			z_est[2 * row + 1] = projection(row, 1) / projection(row, 2);
		}
		return z_est;
	};

	// Build Jacobian and Covariance
	Matrix J          = zeros(size, 15);
	Matrix R          = eye(size);
	Matrix h_w        = -ccn;
	Matrix meas_noise = 0.01 * eye(2);

	// Declare common ranges ahead of time rather than ten times per iteration
	auto range_0_2 = xt::range(0, 2);
	auto range_0_3 = xt::range(0, 3);
	for (std::size_t row = 0; row < num_rows(projected_measurements); row++) {
		auto range_2i     = xt::range(2 * row, 2 * row + 2);
		Matrix xi_block_t = xt::transpose(xt::view(xi, xt::keep(row), range_0_2));
		Matrix h_psi      = navutils::skew(dot(ccn, to_vec(xt::view(xn, row, range_0_3) - tn)));

		xt::view(J, range_2i, range_0_3) =
		    ((xi(row, 2) * xt::view(h_w, range_0_2, range_0_3)) -
		     dot(xi_block_t, xt::view(h_w, xt::keep(2), range_0_3))) /
		    std::pow(xi(row, 2), 2);

		xt::view(J, range_2i, xt::range(6, 9)) =
		    ((xi(row, 2) * xt::view(h_psi, range_0_2, range_0_3)) -
		     dot(xi_block_t, xt::view(h_psi, xt::keep(2), range_0_3))) /
		    std::pow(xi(row, 2), 2);

		Matrix hzy     = xt::view(J, range_2i, range_0_3);
		Matrix j_block = xt::view(J, range_2i, xt::all());

		xt::view(R, range_2i, range_2i) =
		    dot(dot(j_block, xhat_p->covariance), xt::transpose(j_block)) +
		    dot(dot(hzy, pos_cov[row]), xt::transpose(hzy)) + meas_noise;
	}

	// Make R symmetric
	R = R + xt::transpose(R);
	return std::make_shared<StandardMeasurementModel>(z, h, J, R);
}

void DirectionToPoints3dMeasurementProcessor::receive_aux_data(const AspnBaseVector& aux_data) {
	auto metadata = std::dynamic_pointer_cast<aspn_xtensor::MetadataGeneric>(aux_data[0]);
	if (metadata != nullptr) {
		ccb_mat = quat_to_dcm(metadata->get_mounting().get_orientation_quaternion());
	} else {
		MeasurementProcessor::receive_aux_data(aux_data);
	}
}

not_null<std::shared_ptr<MeasurementProcessor<>>> DirectionToPoints3dMeasurementProcessor::clone() {
	return std::make_shared<DirectionToPoints3dMeasurementProcessor>(*this);
}

}  // namespace filtering
}  // namespace navtk
