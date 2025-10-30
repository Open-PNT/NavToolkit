#include <navtk/inertial/CoarseDynamicAlignment.hpp>

#include <vector>

#include <spdlog/spdlog.h>
#include <xtensor-blas/xlinalg.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/inertial/AlignBase.hpp>
#include <navtk/inertial/Inertial.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/MovementStatus.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/wgs84.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>
#include <navtk/utils/human_readable.hpp>

using aspn_xtensor::MeasurementImu;
using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::to_seconds;
using aspn_xtensor::TypeTimestamp;
using navtk::filtering::NULL_GEN_XHAT_AND_P_FUNCTION;
using navtk::utils::extract_pos;
using navtk::utils::to_position;

namespace {
aspn_xtensor::MeasurementImu stationary_imu(
    const navtk::not_null<std::shared_ptr<MeasurementPositionVelocityAttitude>> pva,
    const double dt) {
	// As we are assuming stationary, l_dot and lambda_dot are both 0
	auto Csn = xt::transpose(navtk::navutils::quat_to_dcm(pva->get_quaternion()));
	auto wnie =
	    navtk::Vector3{cos(pva->get_p1()), 0, -sin(pva->get_p1())} * navtk::navutils::ROTATION_RATE;
	auto dth = navtk::dot(Csn, wnie * dt);
	auto g   = navtk::navutils::calculate_gravity_schwartz(pva->get_p3(), pva->get_p1());
	// Inverse of the rotation correction from the calc_force_ned function
	auto inv_corr = navtk::inverse(navtk::eye(3) + 0.5 * navtk::navutils::skew(dth));
	auto dv       = navtk::dot(inv_corr, navtk::dot(Csn, -g * dt));
	auto new_time = pva->get_time_of_validity() + dt;
	auto header   = aspn_xtensor::TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0);
	return aspn_xtensor::MeasurementImu(
	    header, new_time, ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED, dv, dth, {});
}

navtk::Vector3 calc_mean(const std::vector<navtk::Vector3>& v) {
	auto mn =
	    std::accumulate(v.cbegin(),
	                    v.cend(),
	                    navtk::Vector{0.0, 0.0, 0.0},
	                    [](const navtk::Vector& a, const navtk::Vector& b) { return a + b; }) /
	    (v.size());
	return mn;
}

navtk::Vector3 delta_pos(const navtk::Vector3& origin,
                         const navtk::Vector3& p,
                         const double lat_fac,
                         const double lon_fac) {
	navtk::Vector3 scaler = {lat_fac, lon_fac, -1};
	return (p - origin) * scaler;
}

// r > 1
int ratio_discriminator(const std::vector<double>& v, const double r) {

	for (navtk::Size i0 = 0; i0 < v.size(); i0++) {
		std::vector<double> ratios;
		for (navtk::Size i1 = 0; i1 < v.size(); i1++) {
			if (i0 != i1) {
				ratios.push_back(v[i1] / v[i0]);
			}
		}
		if (std::all_of(ratios.cbegin(), ratios.cend(), [r](double d) { return d > r; })) {
			return i0;
		}
	}
	return -1;
}
}  // namespace

namespace navtk {
namespace inertial {

CoarseDynamicAlignment::CoarseDynamicAlignment(const filtering::ImuModel& model,
                                               double static_time,
                                               double reset_time,
                                               DcmIntegrationMethods dcm_integration_method)
    : AlignBase(false, true, model),
      give_up_time(reset_time),
      detector(MovementDetectorImu(10, static_time)),
      static_time(static_time) {
	max_imu_count = std::numeric_limits<Size>::max();

	switch (dcm_integration_method) {
	case DcmIntegrationMethods::FIRST_ORDER:
		dtheta_integrator = [](const Vector3& meas) { return eye(3) + navutils::skew(meas); };
		break;
	case DcmIntegrationMethods::SIXTH_ORDER:
		dtheta_integrator = navutils::rot_vec_to_dcm;
		break;
	case DcmIntegrationMethods::EXPONENTIAL:
		dtheta_integrator = [](const Vector3& meas) { return expm(navutils::skew(meas)); };
		break;
	default:
		log_or_throw<std::runtime_error>(
		    "Unknown dcm_integration option; falling back to first order");
		dtheta_integrator = [](const Vector3& meas) { return eye(3) + navutils::skew(meas); };
	}
}

void CoarseDynamicAlignment::select_prospect(const Size prospect_index) {
	auto sol           = prospects[prospect_index].calc_pva();
	imu_bs             = prospects[prospect_index].calc_imu_errors();
	computed_alignment = {true, utils::to_navsolution(*sol)};
	alignment_status   = AlignmentStatus::ALIGNED_GOOD;
	full_cov           = prospects[prospect_index].get_pinson15_cov();
	spdlog::info("Dynamic alignment complete.");
}

void CoarseDynamicAlignment::check_prospects(const aspn_xtensor::MeasurementPosition& pos) {
	// Only one solution available, so we are done
	if (prospects.size() == 1) {
		select_prospect(0);
		return;
	}

	std::vector<double> err_norms;
	for (Size k = 0; k < prospects.size(); k++) {
		auto sol = prospects[k].calc_pva(pos.get_time_of_validity());
		if (sol != nullptr) {
			auto factors = iteration_data->get_lat_lon_factors();
			// Technically factors would change with pos, but since we just need a relative
			// comparison rather than an absolute right now there's no need to bother
			Vector err =
			    delta_pos(extract_pos(pos), extract_pos(*sol), factors.first, factors.second);
			err_norms.push_back(navtk::norm(err));
			spdlog::debug("Err {}", err);
		}
	}

	if (prospects.size() > 0 && err_norms.size() == 0) {
		spdlog::warn(
		    "All prospective solution testers returned null solutions. Data may be out of order "
		    "(position lagging imu).");
	}

	if (err_norms.size() > 1) {

		// The below is checking if any error norms are outside the largest position sigma-
		// an approximation because actual norm sigma would differ
		// But the idea is to see if any solutions are relatively 'bad' before moving on to other
		// calculations, because if they all look sort of good it is hard to choose one
		auto thresh =
		    std::max(
		        std::sqrt(xt::amax(xt::diagonal(
		            iteration_data->get_position(DynData::RecentPositionsEnum::SECOND_MOST_RECENT)
		                .get_covariance()))[0]),
		        1.0) *
		    position_sigma_multiplier;

		if (std::any_of(
		        err_norms.cbegin(), err_norms.cend(), [thresh](double d) { return d > thresh; })) {

			auto disc = ratio_discriminator(err_norms, error_ratio);
			if (disc != -1) {
				select_prospect(disc);
			}
		}
	}
}

std::vector<aspn_xtensor::MeasurementImu> CoarseDynamicAlignment::separate_imu_after_time(
    const aspn_xtensor::TypeTimestamp& t) {
	std::vector<aspn_xtensor::MeasurementImu> trim;
	auto first_late =
	    std::find_if(align_buffer.begin(), align_buffer.end(), [&t](const MeasurementImu& imu) {
		    return imu.get_time_of_validity() > t;
	    });
	std::copy(first_late, align_buffer.end(), std::back_inserter(trim));
	align_buffer.erase(first_late, align_buffer.end());
	return trim;
}

void CoarseDynamicAlignment::initialize_with_position(
    const not_null<std::shared_ptr<aspn_xtensor::MeasurementPosition>> pos) {
	iteration_data = std::make_shared<DynData>(*pos);
}

void CoarseDynamicAlignment::update_imu_times(
    const not_null<std::shared_ptr<aspn_xtensor::MeasurementImu>> imu) {
	if (first_imu_time.get_elapsed_nsec() == 0) {
		first_imu_time    = imu->get_time_of_validity();
		calib_last_notify = imu->get_time_of_validity();
	}
	latest_imu_time = imu->get_time_of_validity();

	if (num_imu_received < max_imu_count) {
		num_imu_received++;
	}
}

void CoarseDynamicAlignment::update_move_status(
    const not_null<std::shared_ptr<aspn_xtensor::MeasurementImu>> imu) {
	auto move_status = detector.process(imu);

	if (move_status == MovementStatus::MOVING) {
		if (last_status != MovementStatus::MOVING) {
			moving_start_time = imu->get_time_of_validity();
		}
		if ((imu->get_aspn_c()->time_of_validity.elapsed_nsec -
		     moving_start_time.get_elapsed_nsec()) *
		            1e-9 >=
		        required_moving_period &&
		    !movement_detected) {
			spdlog::info("Movement triggered");
			movement_detected = true;
			stat_dv_mean      = calc_mean(stationary_dv);
			stat_dth_mean     = calc_mean(stationary_dth);
			stationary_dv.clear();
			stationary_dth.clear();
			current_stage         = Stage::WAHBA_SOLVE;
			time_since_last_reset = imu->get_time_of_validity();
		}
	} else {
		stationary_dv.push_back(imu->get_meas_accel());
		stationary_dth.push_back(imu->get_meas_gyro());
	}
	if (move_status != last_status) {
		spdlog::debug(move_status);
		last_status = move_status;
	}
}

void CoarseDynamicAlignment::mechanize_or_warn(
    const not_null<std::shared_ptr<aspn_xtensor::MeasurementImu>> imu) {
	if (alignment_status == AlignmentStatus::ALIGNED_GOOD) {
		log_or_throw<std::runtime_error>(
		    "CoarseDynamicAlignment receiving imu measurements after aligned.");
	} else if (iteration_data != nullptr &&
	           imu->get_aspn_c()->time_of_validity.elapsed_nsec <
	               iteration_data->get_position(DynData::RecentPositionsEnum::SECOND_MOST_RECENT)
	                   .get_aspn_c()
	                   ->time_of_validity.elapsed_nsec) {
		spdlog::error("CoarseDynamicAlignment receiving very old imu measurements.");
	} else if (movement_detected) {
		align_buffer.push_back(*imu);

		for (auto k = prospects.begin(); k < prospects.end(); k++) {
			k->mechanize(*imu);
		}
	}
}

void CoarseDynamicAlignment::update_calibration_notifications() {
	if (last_status == MovementStatus::INVALID) {
		if ((latest_imu_time.get_elapsed_nsec() - calib_last_notify.get_elapsed_nsec()) * 1e-9 >=
		    calib_notify_period) {
			calib_last_notify         = latest_imu_time;
			auto calib_time_remaining = std::round(
			    static_time -
			    (latest_imu_time.get_elapsed_nsec() - first_imu_time.get_elapsed_nsec()) * 1e-9);
			spdlog::info("Calibrating imu biases, remain stationary. Approx time remaining: {} s",
			             calib_time_remaining);
			if (calib_time_remaining <= calib_notify_period && iteration_data == nullptr) {
				spdlog::warn(
				    "No position data received during stationary calibration period. If "
				    "this continues alignment will not complete.");
			}
		}
	} else if (!sustained_notified) {
		spdlog::info("Stationary calibration ended. You may begin moving.");
		sustained_notified = true;
	}
}


CoarseDynamicAlignment::AlignmentStatus CoarseDynamicAlignment::process(
    std::shared_ptr<aspn_xtensor::AspnBase> message) {

	if (alignment_status == AlignmentStatus::ALIGNED_GOOD) {
		log_or_throw<std::runtime_error>("CoarseDynamicAlignment: Aligned. Stop sending data.");
		return alignment_status;
	}

	auto posdata = std::dynamic_pointer_cast<aspn_xtensor::MeasurementPosition>(message);

	if (posdata == nullptr) {
		auto pvadata = std::dynamic_pointer_cast<MeasurementPositionVelocityAttitude>(message);
		if (pvadata != nullptr) {
			posdata = std::make_shared<aspn_xtensor::MeasurementPosition>(to_position(*pvadata));
		}
	}

	if (posdata != nullptr) {

		for (auto k = prospects.begin(); k < prospects.end(); k++) {
			k->update(*posdata);
		}

		check_prospects(*posdata);

		if (iteration_data == nullptr) {
			initialize_with_position(posdata);
			time_since_last_reset = posdata->get_time_of_validity();
			return alignment_status;
		}

		if ((posdata->get_aspn_c()->time_of_validity.elapsed_nsec -
		     time_since_last_reset.get_elapsed_nsec()) *
		        1e-9 >
		    give_up_time) {
			reset(posdata->get_time_of_validity());
			return alignment_status;
		}

		// Trim off and store late data
		auto trim = separate_imu_after_time(posdata->get_time_of_validity());

		add_new_meas(*posdata, trim);
		align_buffer = trim;
		return alignment_status;
	}

	auto imudata = std::dynamic_pointer_cast<aspn_xtensor::MeasurementImu>(message);
	if (imudata != nullptr) {
		if (imudata->get_imu_type() != ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED) {
			log_or_throw<std::invalid_argument>(
			    "Only ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED currently supported in "
			    "CoarseDynamicAlignment.");
			return alignment_status;
		}
		update_imu_times(imudata);
		update_move_status(imudata);
		mechanize_or_warn(imudata);

		if (!movement_detected) {
			update_calibration_notifications();
		}
		return alignment_status;
	}

	log_or_throw<std::runtime_error>("CoarseDynamicAlignment unused data type.");

	return alignment_status;
}

Matrix3 CoarseDynamicAlignment::integrate_dth(Matrix3 C_s_to_n) {
	for (auto k = align_buffer.cbegin(); k < align_buffer.cend(); ++k) {
		C_s_to_n = dot(C_s_to_n, dtheta_integrator(k->get_meas_gyro() - stat_dth_mean));
	}
	return C_s_to_n;
}

std::vector<filtering::NavSolution> CoarseDynamicAlignment::generate_prospective_solutions(
    const aspn_xtensor::MeasurementPosition& pos,
    const Vector3& vel,
    const Matrix3& C_k_to_start,
    const std::vector<Matrix3>& C_start_to_n_vec,
    const std::vector<aspn_xtensor::MeasurementImu>& trim) {
	std::vector<filtering::NavSolution> out;
	for (auto k = C_start_to_n_vec.cbegin(); k < C_start_to_n_vec.cend(); ++k) {
		auto C_s_to_n = dot(*k, C_k_to_start);
		out.push_back(mech_align(pos, vel, C_s_to_n, trim));
	}
	current_stage         = Stage::SOLUTION_COMPARE;
	time_since_last_reset = pos.get_time_of_validity();
	return out;
}

std::vector<Matrix> CoarseDynamicAlignment::calc_initial_solution_cov(
    const std::vector<Matrix3>& cnps,
    const std::vector<filtering::NavSolution>& sol,
    const Matrix3& tilt_cov) {

	// Since only attitude is propagated from inertial, we don't really need the pos/vel
	// terms now; these are set from pos sensor data and so aren't functions of imu drift.
	auto init_cov                                        = zeros(9, 9);
	xt::view(init_cov, xt::range(0, 3), xt::range(0, 3)) = tilt_cov;

	// As delta_thetas have been 'zeroed-out' using the stationary average (which includes measured
	// earth rate effects) the initial bias uncertainty should be based on these values. As we do
	// not know the initial orientation we have to apply to all axes. Also as the bias is capped
	// at rotation rate (it's not normally distributed), do a rough conversion equating rotation
	// rate to 3 sigma.
	xt::view(init_cov, xt::range(3, 9), xt::range(3, 9)) =
	    xt::diag(xt::pow(Vector{model.accel_bias_initial_sigma[0],
	                            model.accel_bias_initial_sigma[1],
	                            model.accel_bias_initial_sigma[2],
	                            navutils::ROTATION_RATE / 3.0,
	                            navutils::ROTATION_RATE / 3.0,
	                            navutils::ROTATION_RATE / 3.0},
	                     2));

	std::vector<Matrix> out;
	auto block         = filtering::Pinson15NedBlock("", model);
	auto later_vel_cov = update_vel_cov();
	for (Size k = 0; k < sol.size(); k++) {
		auto pins_cov = init_cov;
		auto t0       = iteration_data->get_origin().get_time_of_validity();

		for (auto ln = lin_points.cbegin(); ln != lin_points.cend(); ln++) {
			auto lin_pva_pt =
			    std::make_shared<aspn_xtensor::MeasurementPositionVelocityAttitude>(ln->first);
			auto lin_f_and_r_pt = std::make_shared<aspn_xtensor::MeasurementImu>(ln->second);
			// Linearization points recorded are just k to start, need to adjust to assumed nav
			// frame
			lin_pva_pt->set_quaternion(navtk::navutils::dcm_to_quat(
			    dot(cnps[k], navtk::navutils::quat_to_dcm(lin_pva_pt->get_quaternion()))));
			block.receive_aux_data({lin_pva_pt, lin_f_and_r_pt});
			// Pinson block doesn't need estimate or covariance.
			auto dyn = block.generate_dynamics(
			    NULL_GEN_XHAT_AND_P_FUNCTION, t0, lin_pva_pt->get_time_of_validity());
			auto phi = xt::view(dyn.Phi, xt::range(6, 15), xt::range(6, 15));
			auto qd  = xt::view(dyn.Qd, xt::range(6, 15), xt::range(6, 15));
			pins_cov = dot(dot(phi, pins_cov), xt::transpose(phi)) + qd;
			t0       = lin_pva_pt->get_time_of_validity();
		}
		// At this point we should be propagated to the time of 'prior' position. We can set pos and
		// vel covariance now and propagate the remainder over the last bit of time
		auto full_cov = zeros(15, 15);
		xt::view(full_cov, xt::range(0, 3), xt::range(0, 3)) =
		    iteration_data->get_position(DynData::RecentPositionsEnum::SECOND_MOST_RECENT)
		        .get_covariance();
		xt::view(full_cov, xt::range(3, 6), xt::range(3, 6))   = later_vel_cov;
		xt::view(full_cov, xt::range(6, 15), xt::range(6, 15)) = pins_cov;

		if (!lin_points.empty()) {
			auto aux =
			    navtk::utils::to_inertial_aux(sol[k], lin_points.back().second.get_meas_accel());
			block.receive_aux_data(aux);
		}
		// Pinson block doesn't need estimate or covariance.
		auto dyn = block.generate_dynamics(NULL_GEN_XHAT_AND_P_FUNCTION, t0, sol[k].time);
		full_cov = dot(dot(dyn.Phi, full_cov), xt::transpose(dyn.Phi)) + dyn.Qd;
		out.push_back(full_cov);
	}

	return out;
}

void CoarseDynamicAlignment::update_wahba_inputs() {

	// Get 3-D force from position data. Note that this isn't actually a raw accelerometer
	// measurement, we are re-using the aspn_xtensor::MeasurementImu type to hold the calculated
	// forces.
	auto f_km1 = iteration_data->get_force_from_pos();

	// Rotate imu forces into initial frame
	auto f_meas0 = dot(C_km1_to_start, iteration_data->get_force_from_imu().second);

	// Calculate B for SVD based solution to Wahba's problem
	auto f_norm  = f_km1 / xt::linalg::norm(f_km1, 2);
	auto f_0norm = f_meas0 / xt::linalg::norm(f_meas0, 2);
	B += xt::linalg::outer(f_norm, f_0norm);
	cross_terms += cross(f_norm, f_0norm);
	// Add in additional rotations to extend from t(k - 1) to t(k)
	C_k_to_start = integrate_dth(C_km1_to_start);
	// Davenport returns ref to platform dcm. f_0norm (imu in body0 frame) is our ref here,
	// and the navigation frame (f_km1) is the 'platform' (so kinda backwards)
	// In other words, each entry of est_cnps is Cn_p0
	est_cnps = solve_wahba_davenport(B, cross_terms);
	std::transform(est_cnps.begin(), est_cnps.end(), est_cnps.begin(), [](const Matrix& m) {
		return navutils::ortho_dcm(m);
	});
}

void CoarseDynamicAlignment::add_new_meas(const aspn_xtensor::MeasurementPosition& pos,
                                          const std::vector<aspn_xtensor::MeasurementImu>& trim) {

	if (movement_detected) {
		if (iteration_data != nullptr) {
			iteration_data->update(pos, align_buffer);
		} else {
			log_or_throw<std::runtime_error>("iteration_data null and shouldn't be.");
		}
		auto C_s_to_n = eye(3);

		call_num += 1;

		// At 2 measurements we can calculate velocity/accels to store off till next iteration
		if (call_num > 2) {

			auto frc = iteration_data->get_force_from_imu();

			if (!frc.first) {
				spdlog::info(
				    "Not enough imu measurements during 1 or more of the last 2 update periods. "
				    "Waiting for additional imu data.");
				return;
			}

			update_wahba_inputs();

			const auto& mid_pos =
			    iteration_data->get_position(DynData::RecentPositionsEnum::SECOND_MOST_RECENT);
			auto mid_vel = iteration_data->get_vel_mid().second;
			auto mid_dcm = C_k_to_start;
			auto pva     = navtk::utils::to_positionvelocityattitude(filtering::NavSolution(
                extract_pos(mid_pos), mid_vel, mid_dcm, mid_pos.get_time_of_validity()));
			auto f_and_r = navtk::utils::to_imu(pva.get_time_of_validity(), frc.second, zeros(3));
			lin_points.push_back(std::make_pair(pva, f_and_r));

			auto test_cov_res = update_last_n(est_cnps[0]);

			if (call_num > min_meas_before_solution_gen) {

				if (test_cov_res.first &&
				    sqrt(test_cov_res.second(2, 2)) < down_tilt_sig_thresh * navutils::PI / 180.0 &&
				    prospects.empty()) {
					// If not for velocity we wouldn't need to reintegrate all of this, just any
					// data after the pos meas
					auto sols = generate_prospective_solutions(
					    iteration_data->get_position(
					        DynData::RecentPositionsEnum::SECOND_MOST_RECENT),
					    iteration_data->get_vel_mid().second,
					    C_km1_to_start,
					    est_cnps,
					    trim);
					auto init_covs = calc_initial_solution_cov(est_cnps, sols, test_cov_res.second);

					for (Size k = 0; k < sols.size(); k++) {
						add_prospect(sols[k], init_covs[k], est_cnps[k]);
					}
					spdlog::info("Candidate alignments generated, undergoing validation.");
				}
			}
		}

		// Store variables for next iteration
		C_km1_to_start = C_k_to_start;
	}
}

void CoarseDynamicAlignment::add_prospect(const filtering::NavSolution& ns,
                                          const Matrix& init_cov,
                                          const Matrix3& initial_cns) {
	const auto& start = iteration_data->get_origin();
	auto pva          = utils::to_positionvelocityattitude(ns);

	pva.set_covariance(xt::view(init_cov, xt::range(0, 9), xt::range(0, 9)));

	auto est_dt = approx_dt();

	// Estimate accel biases using means and stationary data
	auto header = start.get_header();
	header.set_message_type(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE);
	auto feeder = std::make_shared<aspn_xtensor::MeasurementPositionVelocityAttitude>(
	    header,
	    start.get_time_of_validity(),
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
	    start.get_term1(),
	    start.get_term2(),
	    start.get_term3(),
	    0,
	    0,
	    0,
	    navutils::dcm_to_quat(initial_cns),
	    zeros(9, 9),
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
	    Vector{},
	    std::vector<aspn_xtensor::TypeIntegrity>{});
	auto perf       = stationary_imu(feeder, est_dt);
	auto dv_biases  = stat_dv_mean - perf.get_meas_accel();
	auto dth_biases = stat_dth_mean - perf.get_meas_gyro();

	auto imu_errs         = ImuErrors{};
	imu_errs.accel_biases = dv_biases / est_dt;
	imu_errs.gyro_biases  = dth_biases / est_dt;
	auto last_imu         = std::make_shared<aspn_xtensor::MeasurementImu>(align_buffer.back());
	auto buf              = BasicInsAndFilter(pva, model, last_imu, imu_errs, est_dt);
	prospects.push_back(buf);
}

std::pair<bool, Matrix> CoarseDynamicAlignment::get_computed_covariance(
    const CovarianceFormat format) const {
	auto model_terms = bias_stats_from_model(format);
	switch (format) {
	case CovarianceFormat::PINSON15NEDBLOCK: {
		return {alignment_status == AlignmentStatus::ALIGNED_GOOD, full_cov};
	}
	case CovarianceFormat::PINSON21NEDBLOCK: {
		auto out_cov                                          = zeros(21, 21);
		xt::view(out_cov, xt::range(0, 15), xt::range(0, 15)) = full_cov;
		xt::view(out_cov, xt::range(15, 21), xt::range(15, 21)) =
		    xt::view(model_terms, xt::range(6, 12), xt::range(6, 12));
		return {alignment_status == AlignmentStatus::ALIGNED_GOOD, out_cov};
	}
	default:
		return {false, zeros(1, 1)};
	}
}

std::pair<bool, Matrix3> CoarseDynamicAlignment::update_last_n(const Matrix3& c_b0_to_n) {
	if (last_n_solutions.size() < num_solutions_in_cov) {
		last_n_solutions.push_back(c_b0_to_n);
		return {false, zeros(3, 3)};
	} else {
		// Double check all this
		last_n_solutions[sol_ind] = c_b0_to_n;
		sol_ind++;
		if (sol_ind >= num_solutions_in_cov) {
			sol_ind = 0;
		}
		Matrix3 avg_dcm =
		    std::accumulate(last_n_solutions.cbegin(), last_n_solutions.cend(), zeros(3, 3)) /
		    num_solutions_in_cov;
		std::vector<Matrix3> tilt_outer;
		std::transform(last_n_solutions.cbegin(),
		               last_n_solutions.cend(),
		               std::back_inserter(tilt_outer),
		               [&avg_dcm](const Matrix3& c) {
			               auto tilt =
			                   navutils::dcm_to_rpy(xt::transpose(dot(avg_dcm, xt::transpose(c))));
			               return xt::linalg::outer(tilt, tilt);
		               });
		auto cov = std::accumulate(tilt_outer.cbegin(), tilt_outer.cend(), zeros(3, 3)) /
		           num_solutions_in_cov;
		return {true, cov};
	}
}

Matrix CoarseDynamicAlignment::update_vel_cov() const {
	if (iteration_data != nullptr && iteration_data->enough_data()) {
		const auto& last_pos =
		    iteration_data->get_position(DynData::RecentPositionsEnum::SECOND_MOST_RECENT);
		const auto& prior_last_pos =
		    iteration_data->get_position(DynData::RecentPositionsEnum::THIRD_MOST_RECENT);
		const auto& origin = iteration_data->get_origin();
		auto r             = navutils::SEMI_MAJOR_RADIUS;
		auto e2            = navutils::ECCENTRICITY_SQUARED;
		auto sl            = sin(origin.get_term1());
		auto cl            = cos(origin.get_term1());
		auto trm           = 1 - e2 * sl * sl;
		auto a             = r * (1 - e2) / std::pow(trm, 1.5);
		auto b             = r / std::pow(trm, 0.5);

		auto hand_jac = zeros(3, 9);

		auto pos_dt = (last_pos.get_aspn_c()->time_of_validity.elapsed_nsec -
		               prior_last_pos.get_aspn_c()->time_of_validity.elapsed_nsec) *
		              1e-9;
		hand_jac(0, 0) = (last_pos.get_term1() - prior_last_pos.get_term1()) / pos_dt * r *
		                 (1 - e2) * 3 * e2 * sl * cl / std::pow(trm, 2.5);
		hand_jac(0, 2) = (last_pos.get_term1() - prior_last_pos.get_term1()) / pos_dt;
		hand_jac(0, 3) = -(a + origin.get_term3()) / pos_dt;
		hand_jac(0, 6) = -hand_jac(0, 3);

		hand_jac(1, 0) = (last_pos.get_term2() - prior_last_pos.get_term2()) / pos_dt *
		                 (-r * sl * std::pow(trm, -0.5) +
		                  cl * r * std::pow(trm, -1.5) * e2 * sl * cl - sl * origin.get_term3());
		hand_jac(1, 2) = cl * (last_pos.get_term2() - prior_last_pos.get_term2()) / pos_dt;
		hand_jac(1, 4) = -cl * b / pos_dt;
		hand_jac(1, 7) = -hand_jac(1, 4);

		hand_jac(2, 5) = 1 / pos_dt;
		hand_jac(2, 8) = -hand_jac(2, 5);

		auto lat_lon_fac = iteration_data->get_lat_lon_factors();
		auto conv        = xt::diag(Vector{1 / lat_lon_fac.first, 1 / lat_lon_fac.second, -1});
		Matrix pos_cov   = zeros(9, 9);
		xt::view(pos_cov, xt::range(0, 3), xt::range(0, 3)) =
		    dot(conv, dot(Matrix{origin.get_covariance()}, conv));
		xt::view(pos_cov, xt::range(3, 6), xt::range(3, 6)) =
		    dot(conv, dot(Matrix{prior_last_pos.get_covariance()}, conv));
		xt::view(pos_cov, xt::range(6, 9), xt::range(6, 9)) =
		    dot(conv, dot(Matrix{last_pos.get_covariance()}, conv));

		return dot(hand_jac, dot(pos_cov, xt::transpose(hand_jac)));
	}
	return zeros(3, 3);
}

filtering::NavSolution CoarseDynamicAlignment::mech_align(
    const aspn_xtensor::MeasurementPosition& pos,
    const Vector3& vel,
    const Matrix3& C_s_to_n,
    const std::vector<aspn_xtensor::MeasurementImu>& trim) {

	auto approx_dt = (align_buffer[1].get_aspn_c()->time_of_validity.elapsed_nsec -
	                  align_buffer[0].get_aspn_c()->time_of_validity.elapsed_nsec);
	auto first_dt  = (align_buffer[0].get_aspn_c()->time_of_validity.elapsed_nsec -
                     pos.get_aspn_c()->time_of_validity.elapsed_nsec);

	auto spva = std::make_shared<StandardPosVelAtt>(
	    pos.get_time_of_validity(), extract_pos(pos), vel, C_s_to_n);

	// Mechanization is slow, so use first order DCM
	auto mops = MechanizationOptions{navutils::GravModels::SCHWARTZ,
	                                 EarthModels::ELLIPTICAL,
	                                 DcmIntegrationMethods::FIRST_ORDER,
	                                 IntegrationMethods::TRAPEZOIDAL};

	auto inertial = Inertial(spva, mops);

	for (Size r = 0; r < align_buffer.size(); ++r) {
		if (r == 0) {
			inertial.mechanize(
			    align_buffer[r].get_time_of_validity(),
			    align_buffer[r].get_meas_accel() / approx_dt * first_dt,
			    align_buffer[r].get_meas_gyro() / approx_dt * first_dt - stat_dth_mean);
		} else {
			inertial.mechanize(align_buffer[r].get_time_of_validity(),
			                   align_buffer[r].get_meas_accel(),
			                   align_buffer[r].get_meas_gyro() - stat_dth_mean);
		}
	}

	for (Size r = 0; r < trim.size(); ++r) {
		inertial.mechanize(trim[r].get_time_of_validity(),
		                   trim[r].get_meas_accel(),
		                   trim[r].get_meas_gyro() - stat_dth_mean);
	}

	auto ret_sol = inertial.get_solution();

	return utils::to_navsolution(*ret_sol);
}

void CoarseDynamicAlignment::reset(const aspn_xtensor::TypeTimestamp& time) {
	/* We have (at least) 3 separate reset scenarios that need to be handled
	 * 1. In the case that we are sitting stationary for a long time, we could reset our
	 * propagated DCM back to eye to remove any propagation errors over time
	 * 2. Once we have started moving and doing the wahba calculations to estimate the actual
	 * starting DCM, we can clear related terms (B, cross_terms) in case we brought in some bad
	 * measurements. However, the integrated DCM should stay in place. Because the bias estimates
	 * were done in the initial frame, we need the full dcm in order to correct accel measurements.
	 * This is an argument against changing the origin of the navigation frame as well, just to be
	 * consistent.
	 * 3. If we generated a starting DCM(s) and are at the stage where we are comparing solutions
	 * but that has not resolved, we reset back to step 2. The worry is that if we've gotten to this
	 * point our integrated DCM is so bad that our test filters cannot estimate the tilt errors
	 * properly. If this is the case the only real solution would be to wait for a new stationary
	 * period and completely start over. This would require some additional functionality that
	 * should probably be pushed off till later.
	 *
	 * In retrospect, in no case should the measurement buffers be cleared
	 */
	alignment_status         = AlignmentStatus::ALIGNING_COARSE;
	computed_alignment.first = false;
	time_since_last_reset    = time;
	switch (current_stage) {
	case Stage::INITIAL_STATIC: {
		C_k_to_start   = eye(3);
		C_km1_to_start = eye(3);
		break;
	}
	case Stage::WAHBA_SOLVE: {
		B           = zeros(3, 3);
		cross_terms = zeros(3);
		break;
	}
	case Stage::SOLUTION_COMPARE: {
		B           = zeros(3, 3);
		cross_terms = zeros(3);
		prospects.clear();
		current_stage = Stage::WAHBA_SOLVE;
	}
	}
}

std::pair<bool, ImuErrors> CoarseDynamicAlignment::get_imu_errors() const {
	return {alignment_status == AlignmentStatus::ALIGNED_GOOD, imu_bs};
}

MotionNeeded CoarseDynamicAlignment::motion_needed() const {
	switch (current_stage) {
	case Stage::INITIAL_STATIC:
		return MotionNeeded::NO_MOTION;
	default:
		return MotionNeeded::MOTION_NEEDED;
	}
}

double CoarseDynamicAlignment::approx_dt() {
	if (num_imu_received > 1 && num_imu_received < max_imu_count) {
		default_dt = (latest_imu_time.get_elapsed_nsec() - first_imu_time.get_elapsed_nsec()) *
		             1e-9 / num_imu_received;
	}
	return default_dt;
}

}  // namespace inertial
}  // namespace navtk
