#include <navtk/inertial/ManualAlignment.hpp>

#include <memory>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/utils/conversions.hpp>

namespace {
/**
 * @param pva Candidate PositionVelocityAttitude
 * @return Pair of a) 9 element vector indicating pva data elements that appear to be valid: p1, p2,
 * p3, v1, v2, v3, and 3 attitude (note attitude is all or nothing) and b) a boolean indicating if
 * the covariance matrix is of a size that matches the data available.
 **/
std::pair<std::vector<bool>, bool> pva_valid(
    const aspn_xtensor::MeasurementPositionVelocityAttitude& pva) {
	auto att          = pva.get_quaternion();
	auto att_valid    = !xt::any(xt::isnan(att));
	navtk::Matrix cov = pva.get_covariance();
	std::vector<bool> valid_data{!std::isnan(pva.get_p1()),
	                             !std::isnan(pva.get_p2()),
	                             !std::isnan(pva.get_p3()),
	                             !std::isnan(pva.get_v1()),
	                             !std::isnan(pva.get_v2()),
	                             !std::isnan(pva.get_v3()),
	                             att_valid,
	                             att_valid,
	                             att_valid};
	// Make sure that cov matrix matches our expected size based on presence of NANs. Mismatch
	// could occur if say one of the doubles was NAN due to a bad math operation but the cov
	// element is still present.
	navtk::Size exp_cov_size = std::count(valid_data.cbegin(), valid_data.cend(), true);
	auto cov_good =
	    (navtk::num_rows(cov) == navtk::num_cols(cov)) && (exp_cov_size == navtk::num_rows(cov));
	if (!cov_good) {
		navtk::log_or_throw<std::invalid_argument>(
		    "PVA covariance (size {}, {}) and data elements ({}) do not appear to agree.",
		    navtk::num_rows(cov),
		    navtk::num_cols(cov),
		    exp_cov_size);
	}
	return {valid_data, cov_good};
}

/**
 * @param v a 9-element vector of 3 position, 3 velocity, and 3 attitude validity markers
 * @return a 9-vector that indicates whether all position, all velocity, and all attitude elements
 * are valid. Every 3 elements are identical.
 */
std::vector<bool> pva_validity_grouped(std::vector<bool> v) {
	std::vector<bool> out;
	for (size_t k = 0; k < 3; ++k) {
		auto a = v[k * 3] && v[k * 3 + 1] && v[k * 3 + 2];
		out.insert(out.end(), 3, a);
	}
	return out;
}

/**
 * Figure out what elements from a PositionVelocityAttitude covariance should be assigned to the
 * alignment covariance and the indices to pull for each. Covariance elements that do
 * not belong to a valid 3d group are excluded (eg if a pva get_p2() value is invalid, none of the
 * covariance elements corresponding to get_p*() will be included).
 *
 * @param el_valid a 9 element vector indicating if individual position, velocity and attitude
 * elements are valid, such as that produced by pva_valid().
 *
 * @return A pair of vectors containing a) the assignment indices for a target covariance matrix,
 * and b) the indices to pull from the source covariance matrix.
 */
std::pair<std::vector<navtk::Size>, std::vector<navtk::Size>> cov_assign_indices(
    std::vector<bool> el_valid) {
	auto g_ok = pva_validity_grouped(el_valid);
	std::vector<navtk::Size> assign_left{};
	std::vector<navtk::Size> from_right{};
	size_t idx = 0;
	for (size_t k = 0; k < g_ok.size(); ++k) {
		if (g_ok[k]) {
			assign_left.push_back(k);
			from_right.push_back(idx);
			++idx;
		} else if (el_valid[k]) {
			// Cov element present but doesn't belong to a full 3d group, skip
			++idx;
		}
	}
	return {assign_left, from_right};
}

}  // namespace

namespace navtk {
namespace inertial {

ManualAlignment::ManualAlignment(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva,
                                 bool wait_for_time,
                                 bool wait_for_pos,
                                 bool wait_for_vel,
                                 bool wait_for_att,
                                 const filtering::ImuModel& model)
    : AlignBase(true, false, model),
      use_imu_time(false),
      cov(eye(9)),
      wait_for{wait_for_time, wait_for_pos, wait_for_vel, wait_for_att} {
	alignment_status = AlignmentStatus::ALIGNING_COARSE;

	auto start_good = !std::any_of(wait_for.cbegin(), wait_for.cend(), [](bool b) { return b; });
	if (start_good) {
		alignment_status = AlignmentStatus::ALIGNED_GOOD;
	}

	if (pva.get_reference_frame() !=
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC) {
		log_or_throw<std::invalid_argument>("Invalid initial PVA reference frame.");
	}
	auto pva_validity = pva_valid(pva);
	auto pva_g        = pva_validity_grouped(pva_validity.first);
	if (!pva_validity.second) {
		log_or_throw<std::invalid_argument>("Supplied alignment pva covariance was wrong shape.");
	}
	if (!pva_g[0] && !wait_for_pos) {
		log_or_throw<std::invalid_argument>(
		    "Supplied alignment position had invalid elements but aligner not instructed to "
		    "require position data.");
	}
	if (!pva_g[3] && !wait_for_vel) {
		log_or_throw<std::invalid_argument>(
		    "Supplied alignment velocity had invalid elements but aligner not instructed to "
		    "require velocity data.");
	}
	if (!pva_g[6] && !wait_for_att) {
		log_or_throw<std::invalid_argument>(
		    "Supplied alignment attitude had invalid elements but aligner not instructed to "
		    "require attitude data.");
	}
	computed_alignment.first = start_good;
	assign_from_pva(pva, false);
}

bool ManualAlignment::assign_from_pva(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva,
                                      bool set_flags) {

	Matrix supplied_cov = pva.get_covariance();
	auto pva_validity   = pva_valid(pva);
	auto g_ok           = pva_validity_grouped(pva_validity.first);
	auto p_ok           = g_ok[0];
	auto v_ok           = g_ok[3];
	auto a_ok           = g_ok[6];

	if (!pva_validity.second) {
		return false;
	}
	if (p_ok) {
		computed_alignment.second.pos = utils::extract_pos(pva);
		if (set_flags) {
			wait_for[1] = false;
		}
	}
	if (v_ok) {
		computed_alignment.second.vel = utils::extract_vel(pva);
		if (set_flags) {
			wait_for[2] = false;
		}
	}
	if (a_ok) {
		computed_alignment.second.rot_mat =
		    xt::transpose(navutils::quat_to_dcm(pva.get_quaternion()));
		if (set_flags) {
			wait_for[3] = false;
		}
	}
	if (p_ok || v_ok || a_ok) {
		auto cov_idx                   = cov_assign_indices(pva_validity.first);
		auto kp_l                      = xt::keep(cov_idx.first);
		auto kp_r                      = xt::keep(cov_idx.second);
		xt::view(cov, kp_l, kp_l)      = xt::view(supplied_cov, kp_r, kp_r);
		computed_alignment.second.time = pva.get_time_of_validity();
	}
	return true;
}

AlignBase::AlignmentStatus ManualAlignment::process(
    std::shared_ptr<aspn_xtensor::AspnBase> message) {

	auto imudata = std::dynamic_pointer_cast<aspn_xtensor::MeasurementImu>(message);
	if (imudata != nullptr) {
		last_imu_time = imudata->get_time_of_validity();
		use_imu_time  = true;
		wait_for[0]   = false;
		return post_meas_received(false, last_imu_time);
	}

	// Position
	auto data_pos = std::dynamic_pointer_cast<aspn_xtensor::MeasurementPosition>(message);
	if (data_pos != nullptr) {
		auto pos = utils::extract_pos(*data_pos);
		if (!xt::any(xt::isnan((Vector)pos))) {
			auto ref_frame = data_pos->get_reference_frame();
			if (ref_frame == ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC) {
				computed_alignment.second.pos                   = pos;
				xt::view(cov, xt::range(0, 3), xt::range(0, 3)) = data_pos->get_covariance();
				wait_for[1]                                     = false;
			} else {
				spdlog::warn(
				    "Position received for alignment but {} is not a supported reference frame",
				    ref_frame);
			}
		}
		return post_meas_received(false, data_pos->get_time_of_validity());
	}

	// Full pva
	auto data_pva =
	    std::dynamic_pointer_cast<aspn_xtensor::MeasurementPositionVelocityAttitude>(message);
	if (data_pva != nullptr) {
		auto ref_frame = data_pva->get_reference_frame();
		if (ref_frame == ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC) {
			assign_from_pva(*data_pva);
		} else {
			spdlog::warn(
			    "PositionVelocityAttitude received for alignment but {} is not a supported "
			    "reference frame",
			    ref_frame);
		}

		return post_meas_received(false, data_pva->get_time_of_validity());
	}

	// Attitude
	auto data_att = std::dynamic_pointer_cast<aspn_xtensor::MeasurementAttitude3D>(message);
	if (data_att != nullptr) {
		auto quat      = data_att->get_quaternion();
		auto ref_frame = data_att->get_reference_frame();
		Matrix t_cov   = data_att->get_tilt_error_covariance();
		if (ref_frame == ASPN_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_NED) {
			computed_alignment.second.rot_mat = xt::transpose(navutils::quat_to_dcm(quat));
			xt::view(cov, xt::range(6, 9), xt::range(6, 9)) = t_cov;
			wait_for[3]                                     = false;
		} else if (ref_frame == ASPN_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_ECEF && !wait_for[1]) {
			// ECEF but we have been supplied position so we can convert
			auto C_nav_to_ecef      = navutils::llh_to_cen(computed_alignment.second.pos);
			auto C_platform_to_ecef = navutils::quat_to_dcm(quat);
			auto C_platform_to_ned  = dot(xt::transpose(C_nav_to_ecef), C_platform_to_ecef);
			computed_alignment.second.rot_mat = xt::transpose(C_platform_to_ned);
			// Rotate cov from ecef to ned
			xt::view(cov, xt::range(6, 9), xt::range(6, 9)) =
			    dot(dot(xt::transpose(C_nav_to_ecef), t_cov), C_nav_to_ecef);
			wait_for[3] = false;
		} else {
			spdlog::warn(
			    "Attitude received for alignment but {} is not a supported reference frame",
			    ref_frame);
		}
		return post_meas_received(false, data_att->get_time_of_validity());
	}

	// Velocity
	auto data_vel = std::dynamic_pointer_cast<aspn_xtensor::MeasurementVelocity>(message);
	if (data_vel != nullptr) {
		auto vel = utils::extract_vel(*data_vel);
		if (!xt::any(xt::isnan((Vector)vel))) {
			Matrix v_cov   = data_vel->get_covariance();
			auto ref_frame = data_vel->get_reference_frame();
			if (ref_frame == ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED) {
				computed_alignment.second.vel                   = vel;
				xt::view(cov, xt::range(3, 6), xt::range(3, 6)) = v_cov;
				wait_for[2]                                     = false;
			} else if (ref_frame == ASPN_MEASUREMENT_VELOCITY_REFERENCE_FRAME_ECEF &&
			           !wait_for[1]) {
				auto C_nav_to_ecef            = navutils::llh_to_cen(computed_alignment.second.pos);
				computed_alignment.second.vel = dot(xt::transpose(C_nav_to_ecef), vel);
				xt::view(cov, xt::range(3, 6), xt::range(3, 6)) =
				    dot(dot(xt::transpose(C_nav_to_ecef), v_cov), C_nav_to_ecef);
				wait_for[2] = false;
			} else {
				spdlog::warn(
				    "Velocity received for alignment but {} is not a supported reference frame",
				    ref_frame);
			}
		}
		return post_meas_received(false, data_vel->get_time_of_validity());
	}

	return post_meas_received(true);
}

AlignBase::AlignmentStatus ManualAlignment::post_meas_received(
    bool should_throw, const aspn_xtensor::TypeTimestamp& meas_time) {
	if (should_throw) {
		log_or_throw<std::runtime_error>("Unsupported data type received");
	} else {
		computed_alignment.second.time = use_imu_time ? last_imu_time : meas_time;
	}

	if (!std::any_of(wait_for.cbegin(), wait_for.cend(), [](bool b) { return b; })) {
		alignment_status         = AlignmentStatus::ALIGNED_GOOD;
		computed_alignment.first = true;
	}
	return alignment_status;
}

std::pair<bool, Matrix> ManualAlignment::get_computed_covariance(
    const CovarianceFormat format) const {
	auto model_terms = bias_stats_from_model(format);
	switch (format) {
	case CovarianceFormat::PINSON15NEDBLOCK: {
		auto out_cov                                          = zeros(15, 15);
		xt::view(out_cov, xt::range(0, 9), xt::range(0, 9))   = cov;
		xt::view(out_cov, xt::range(9, 15), xt::range(9, 15)) = model_terms;
		return {alignment_status == AlignmentStatus::ALIGNED_GOOD, out_cov};
	}
	case CovarianceFormat::PINSON21NEDBLOCK: {
		auto out_cov                                          = zeros(21, 21);
		xt::view(out_cov, xt::range(0, 9), xt::range(0, 9))   = cov;
		xt::view(out_cov, xt::range(9, 21), xt::range(9, 21)) = model_terms;
		return {alignment_status == AlignmentStatus::ALIGNED_GOOD, out_cov};
	}
	default:
		return {false, zeros(1, 1)};
	}
}

MotionNeeded ManualAlignment::motion_needed() const { return MotionNeeded::ANY_MOTION; }

}  // namespace inertial
}  // namespace navtk
