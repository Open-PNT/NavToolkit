#include <gps_ins/utils/results.hpp>

#include <memory>

#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::filtering::NavSolution;
using navtk::filtering::Pose;
using navtk::filtering::StandardFusionEngine;
using navtk::inertial::Inertial;
using navtk::inertial::StandardPosVelAtt;
using navtk::navutils::rpy_to_dcm;
using navtk::utils::cubic_spline_interpolate;
using std::get;
using std::pair;
using std::size_t;
using std::string;
using std::vector;
using xt::all;
using xt::range;
using xt::transpose;
using xt::view;

namespace navtk {
namespace exampleutils {

void generate_output_base(CouplingType type,
                          StandardFusionEngine &engine,
                          not_null<std::shared_ptr<inertial::InertialPosVelAtt>> ins_pva,
                          const vector<string> &block_label,
                          vector<Vector> &filter_data,
                          vector<Vector> &sigma_data,
                          vector<Vector> &ins_data,
                          vector<double> &time_tags,
                          const Matrix3 C_s_to_b) {

	Vector temp_filter, temp_sigma;
	int max_filter_size = 0;
	int max_sigma_size  = 0;
	if (type == CouplingType::LOOSE) {
		// loosely coupled
		max_filter_size = 24;
		max_sigma_size  = 15;
		temp_filter     = zeros(max_filter_size);
		temp_sigma      = zeros(max_sigma_size);
	} else if (type == CouplingType::TIGHT) {
		// tightly coupled
		max_filter_size = 26;
		max_sigma_size  = 17;
		temp_filter     = zeros(max_filter_size);
		temp_sigma      = zeros(max_sigma_size);
	}

	// pull out the current solution state
	Vector x_pinson  = (engine).get_state_block_estimate(block_label[0]);
	Matrix p_pinson  = (engine).get_state_block_covariance(block_label[0]);
	Matrix3 C_b_to_n = dot(ins_pva->get_C_s_to_ned(), transpose(C_s_to_b));
	auto spva        = StandardPosVelAtt(
        ins_pva->time_validity, ins_pva->get_llh(), ins_pva->get_vned(), C_b_to_n);
	auto corr_pva = filtering::apply_error_states<filtering::Pinson15NedBlock>(spva, x_pinson);

	view(temp_filter, range(0, 9)) = view(utils::to_vector_pva(corr_pva), range(1, 10));

	if (type == CouplingType::LOOSE) {
		// loosely coupled
		view(temp_filter, range(9, max_filter_size)) = x_pinson;
		view(temp_sigma, range(0, max_sigma_size))   = sqrt(diagonal(p_pinson));
	} else if (type == CouplingType::TIGHT) {
		// tightly coupled
		// clock bias and clock drift
		view(temp_filter, range(9, max_filter_size - 2)) = x_pinson;
		view(temp_filter, range(max_filter_size - 2, max_filter_size)) =
		    engine.get_state_block_estimate(block_label[1]);
		view(temp_sigma, range(0, max_sigma_size - 2)) = sqrt(diagonal(p_pinson));
		view(temp_sigma, range(max_sigma_size - 2, max_sigma_size)) =
		    sqrt(diagonal(engine.get_state_block_covariance(block_label[1])));
	}
	// processed INS errors
	Vector temp_ins = view(utils::to_vector_pva(spva), range(1, 10));

	filter_data.push_back(temp_filter);
	sigma_data.push_back(temp_sigma);
	ins_data.push_back(temp_ins);
	time_tags.push_back(to_seconds(engine.get_time()));
}

void generate_output(CouplingType type,
                     StandardFusionEngine &engine,
                     const Inertial &ins,
                     const vector<string> &block_label,
                     vector<Vector> &filter_data,
                     vector<Vector> &sigma_data,
                     vector<Vector> &ins_data,
                     vector<double> &time_tags,
                     const Matrix3 C_s_to_b) {


	auto ins_pva = ins.get_solution();
	generate_output_base(
	    type, engine, ins_pva, block_label, filter_data, sigma_data, ins_data, time_tags, C_s_to_b);
}

void generate_output(CouplingType type,
                     StandardFusionEngine &engine,
                     const inertial::BufferedImu &ins,
                     const vector<string> &block_label,
                     vector<Vector> &filter_data,
                     vector<Vector> &sigma_data,
                     vector<Vector> &ins_data,
                     vector<double> &time_tags,
                     const Matrix3 C_s_to_b) {

	auto aux = get_inertial_aux(ins, engine.get_time(), true);

	auto ins_pva = std::make_shared<StandardPosVelAtt>(utils::to_standardposvelatt(*aux.first));
	generate_output_base(
	    type, engine, ins_pva, block_label, filter_data, sigma_data, ins_data, time_tags, C_s_to_b);
}

void generate_pr_bias_output(std::map<int, pair<vector<double>, vector<double>>> &pr_bias_output,
                             vector<string> block_labels,
                             StandardFusionEngine &engine) {
	// parse the pseudorange bias state block labels for prns
	for (auto const &block_label : block_labels) {
		Vector pr_bias_sb_est;
		vector<double> time_tags;
		vector<double> prn_biases;
		pair<vector<double>, vector<double>> time_tag_pr_bias;
		auto last_underscore_position = block_label.find_last_of('_');
		// if underscore not found or if it is the last character
		if (last_underscore_position == string::npos ||
		    last_underscore_position == block_label.length() - 1) {
			return;
		}
		auto prn_num_string = block_label.substr(last_underscore_position + 1);
		int prn             = 0;
		try {
			prn = std::stoi(prn_num_string);
		} catch (const std::exception &) {
			return;
		}
		// pull out the current PRN bias state block estimate state
		pr_bias_sb_est = (engine).get_state_block_estimate(block_label);
		auto map_it    = pr_bias_output.find(prn);
		if (map_it != pr_bias_output.end()) {
			// pull out current state
			time_tag_pr_bias = map_it->second;
			pr_bias_output.erase(map_it);
			time_tags  = time_tag_pr_bias.first;
			prn_biases = time_tag_pr_bias.second;
		}
		time_tags.push_back(to_seconds(engine.get_time()));
		prn_biases.push_back(pr_bias_sb_est[0]);
		time_tag_pr_bias = std::make_pair(time_tags, prn_biases);
		pr_bias_output.insert({prn, time_tag_pr_bias});
	}
}

void propagate_filter(const Inertial &ins,
                      StandardFusionEngine &engine,
                      const aspn_xtensor::TypeTimestamp &measurement_time,
                      const Vector3 f_ned,
                      const string pinson15_label) {
	// Maximum Filter Propagation Time
	NavSolution ins_nav_solution_update(utils::to_navsolution(*ins.get_solution()));

	aspn_xtensor::MeasurementPositionVelocityAttitude pva =
	    navtk::utils::to_positionvelocityattitude(ins_nav_solution_update);
	// Send the aux data to the Pinson block
	engine.give_state_block_aux_data(
	    pinson15_label, navtk::utils::to_inertial_aux(ins_nav_solution_update, f_ned, zeros(3)));
	// Perform the propagation
	engine.propagate(measurement_time);
}

void propagate_filter(const inertial::BufferedImu &ins,
                      StandardFusionEngine &engine,
                      const aspn_xtensor::TypeTimestamp &measurement_time,
                      const string pinson15_label,
                      const bool ignore_imu_lag) {

	auto aux = get_inertial_aux(ins, measurement_time, ignore_imu_lag);

	NavSolution ins_nav_solution_update(utils::to_navsolution(*aux.first));
	// Send the aux data to the Pinson block
	engine.give_state_block_aux_data(
	    pinson15_label,
	    navtk::utils::to_inertial_aux(
	        ins_nav_solution_update, aux.second->get_meas_accel(), zeros(3)));
	// Perform the propagation
	engine.propagate(measurement_time);
}

void apply_feedback_pinson15(Inertial &ins,
                             const vector<Vector> &out_var,
                             const aspn_xtensor::TypeTimestamp &feedback_time) {
	Vector v = xt::concatenate(xt::xtuple(Vector{to_seconds(feedback_time)}, out_var.back()));
	ins.reset(std::make_shared<StandardPosVelAtt>(utils::to_standardposvelatt(v)));
}

Matrix std_to_navtk(const vector<Vector> &data) {
	int row_size        = data[0].size();
	int col_size        = data.size();
	Matrix output_navtk = zeros(row_size, col_size);
	for (decltype(col_size) k = 0; k < col_size; k++) {
		view(output_navtk, all(), k) = data[k];
	}
	return output_navtk;
}

Vector std_to_navtk(const vector<double> &data) {
	Vector output_data;
	if (!data.empty()) {
		int vector_size = data.size();
		output_data     = zeros(vector_size);
		for (decltype(vector_size) k = 0; k < vector_size; k++) {
			view(output_data, k) = data[k];
		}
	}
	return output_data;
}

pair<Matrix, Matrix> calculate_truth_tilts(const TruthResults &truth,
                                           TruthPlotResults &truth_interp,
                                           const Matrix &filter_results,
                                           const Matrix &ins_results,
                                           const vector<double> &time_tags) {
	// Calculate the truth tilts
	// Interpolate attitudes, convert to a quaternion and interpolate.
	// This avoids the discontinuities that occur in heading Euler angles.
	Vector3 rpy_temp;
	vector<double> quat_0, quat_1, quat_2, quat_3;
	Matrix quat_temp = zeros(4, truth.time.size());
	int truth_size   = truth.att_r.size();
	for (decltype(truth_size) j = 0; j < truth_size; j++) {
		rpy_temp[0]               = truth.att_r[j];
		rpy_temp[1]               = truth.att_p[j];
		rpy_temp[2]               = truth.att_y[j];
		view(quat_temp, all(), j) = navutils::rpy_to_quat(rpy_temp);
		quat_0.push_back(quat_temp(0, j));
		quat_1.push_back(quat_temp(1, j));
		quat_2.push_back(quat_temp(2, j));
		quat_3.push_back(quat_temp(3, j));
	}
	// interpolate quaternion, returns pair(unused index of interpolated data time tags,
	// interpolated data)
	auto quat_interp         = cubic_spline_interpolate(truth.time, quat_0, time_tags);
	size_t truth_interp_len  = (quat_interp.second).size();
	truth_interp.quat_matrix = zeros(4, truth_interp_len);
	view(truth_interp.quat_matrix, 0, all()) = std_to_navtk(quat_interp.second);
	quat_interp = cubic_spline_interpolate(truth.time, quat_1, time_tags);
	view(truth_interp.quat_matrix, 1, all()) = std_to_navtk(quat_interp.second);
	quat_interp = cubic_spline_interpolate(truth.time, quat_2, time_tags);
	view(truth_interp.quat_matrix, 2, all()) = std_to_navtk(quat_interp.second);
	quat_interp = cubic_spline_interpolate(truth.time, quat_3, time_tags);
	view(truth_interp.quat_matrix, 3, all()) = std_to_navtk(quat_interp.second);

	// The attitudes estimated by the filter are TILT errors, not errors in
	// Euler angles.  So, first, we must calculate what the true tilt errors
	// are--that is, what small rotations about the NED axes need to be applied
	// to make the ins-computed C_b_to_n matrix (DCM rotating from the ins body frame
	// to the NED navigation frame) be the same as the true C_platform_to_nav matrix.
	Matrix tilt_mat;
	// RPY index in ins_results
	int ins_rpy_index = 6;
	// Tilt index in filter_results
	int tilt_index = 15;
	// delta in tilt value between the truth and the filter estimate
	Matrix delta_tilts = zeros(3, truth_interp_len);
	// tilt value for the truth
	Matrix true_tilts = zeros(3, truth_interp_len);
	for (size_t k = 0; k < truth_interp_len; k++) {
		Matrix c_b_to_n_true = navutils::quat_to_dcm(view(truth_interp.quat_matrix, all(), k));
		Matrix c_n_to_b_ins  = xt::transpose(
            rpy_to_dcm(view(ins_results, range(ins_rpy_index, ins_rpy_index + 3), k)));
		Matrix tilt_mat = dot(c_b_to_n_true, c_n_to_b_ins);
		// tilt_mat is of the form I + tilt_x (where tilt_x is a skew symmetric
		// matrix of the tilt errors).  So, we can just pick out the tilts from
		// the off-diagonal terms)
		true_tilts(0, k) = tilt_mat(1, 2);
		true_tilts(1, k) = tilt_mat(2, 0);
		true_tilts(2, k) = tilt_mat(0, 1);
		view(delta_tilts, range(0, 3), k) =
		    view(filter_results, range(tilt_index, tilt_index + 3), k) -
		    view(true_tilts, range(0, 3), k);
	}
	return std::make_pair(true_tilts, delta_tilts);
}

pair<Matrix, Matrix> calculate_position_profile(const Matrix &filter_results,
                                                const TruthPlotResults &truth_interp) {
	// position profile calculations
	int filter_data_size = view(filter_results, 0, all()).size();
	Matrix d_ned_true    = zeros(3, filter_data_size);
	Matrix d_ned_filt    = zeros(3, filter_data_size);
	int lat_index        = 0;
	for (decltype(filter_data_size) j = 0; j < filter_data_size; j++) {
		d_ned_true(0, j) = navutils::delta_lat_to_north(
		    truth_interp.lat[j] - truth_interp.lat[0], truth_interp.lat[0], truth_interp.alt[0]);
		d_ned_true(1, j) = navutils::delta_lon_to_east(
		    truth_interp.lon[j] - truth_interp.lon[0], truth_interp.lat[0], truth_interp.alt[0]);
		d_ned_true(2, j) = truth_interp.alt[j] - truth_interp.alt[0];
		d_ned_filt(0, j) = navutils::delta_lat_to_north(
		    filter_results(lat_index, j) - filter_results(lat_index, 0),
		    truth_interp.lat[0],
		    truth_interp.alt[0]);
		d_ned_filt(1, j) = navutils::delta_lon_to_east(
		    filter_results((lat_index + 1), j) - filter_results((lat_index + 1), 0),
		    truth_interp.lat[0],
		    truth_interp.alt[0]);
		d_ned_filt(2, j) = filter_results((lat_index + 2), j) - filter_results((lat_index + 2), 0);
	}
	return std::make_pair(d_ned_true, d_ned_filt);
}

pair<vector<size_t>, TruthPlotResults> convert_truth(const TruthResults &truth,
                                                     const vector<double> interp_time) {
	TruthPlotResults truth_interp;
	pair<vector<size_t>, vector<double>> lat, lon, alt, vel_n, vel_e, vel_d;
	// interpolate the truth data
	lat   = cubic_spline_interpolate(truth.time, truth.lat, interp_time);
	lon   = cubic_spline_interpolate(truth.time, truth.lon, interp_time);
	alt   = cubic_spline_interpolate(truth.time, truth.alt, interp_time);
	vel_n = cubic_spline_interpolate(truth.time, truth.vel_n, interp_time);
	vel_e = cubic_spline_interpolate(truth.time, truth.vel_e, interp_time);
	vel_d = cubic_spline_interpolate(truth.time, truth.vel_d, interp_time);
	// store time tags for interpolated data
	vector<double> truth_interp_time = interp_time;
	vector<size_t> unused_index      = lat.first;
	if (!unused_index.empty()) {
		vector<size_t>::reverse_iterator rit = unused_index.rbegin();
		for (; rit != unused_index.rend(); ++rit) {
			size_t rem_index = *rit;
			truth_interp_time.erase(truth_interp_time.begin() + rem_index);
		}
	}
	truth_interp.time = std_to_navtk(truth_interp_time);
	// convert to Vector form for plotting
	truth_interp.lat   = std_to_navtk(lat.second);
	truth_interp.lon   = std_to_navtk(lon.second);
	truth_interp.alt   = std_to_navtk(alt.second);
	truth_interp.vel_n = std_to_navtk(vel_n.second);
	truth_interp.vel_e = std_to_navtk(vel_e.second);
	truth_interp.vel_d = std_to_navtk(vel_d.second);

	return std::make_pair(unused_index, truth_interp);
}

void store_truth(TruthResults &truth, const Vector &data) {
	truth.time.push_back(data[0] + data[1] * 1e-9);
	truth.lat.push_back(data[4]);
	truth.lon.push_back(data[5]);
	truth.alt.push_back(data[6]);
	truth.vel_n.push_back(data[7]);
	truth.vel_e.push_back(data[8]);
	truth.vel_d.push_back(data[9]);
	truth.att_r.push_back(data[10]);
	truth.att_p.push_back(data[11]);
	truth.att_y.push_back(data[12]);
}

std::pair<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>,
          std::shared_ptr<aspn_xtensor::MeasurementImu>>
get_inertial_aux(const inertial::BufferedImu &ins,
                 const aspn_xtensor::TypeTimestamp &t,
                 const bool ignore_imu_lag) {
	aspn_xtensor::TypeTimestamp time_to_use =
	    (ins.in_range(t) || !ignore_imu_lag) ? t : ins.time_span().second;
	return {ins.calc_pva(time_to_use), ins.calc_force_and_rate(time_to_use)};
}

}  // namespace exampleutils
}  // namespace navtk
