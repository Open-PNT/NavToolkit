#include <navtk/inertial/BufferedImu.hpp>

#include <limits>
#include <memory>

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/TimestampedDataSeries.hpp>
#include <navtk/inertial/ImuErrors.hpp>
#include <navtk/inertial/Inertial.hpp>
#include <navtk/inertial/MechanizationOptions.hpp>
#include <navtk/inertial/MechanizationStandard.hpp>
#include <navtk/inertial/inertial_functions.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>
#include <navtk/utils/interpolation.hpp>
#include <navtk/utils/sortable_vectors.hpp>

using aspn_xtensor::MeasurementImu;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::filtering::TimestampedDataSeries;
using navtk::inertial::StandardPosVelAtt;
using navtk::utils::to_imu;
using navtk::utils::to_positionvelocityattitude;
using navtk::utils::to_standardposvelatt;
using std::make_shared;
using std::shared_ptr;

namespace {

constexpr double VERY_SMALL = 1e-20;

void reset_ins(navtk::inertial::Inertial& inertial,
               const MeasurementPositionVelocityAttitude& pva,
               const navtk::inertial::ImuErrors& imu_errs,
               std::shared_ptr<MeasurementPositionVelocityAttitude> old = nullptr) {
	if (old == nullptr) {
		inertial.reset(to_standardposvelatt(pva));
	} else {
		inertial.reset(std::make_shared<StandardPosVelAtt>(to_standardposvelatt(pva)),
		               std::make_shared<StandardPosVelAtt>(to_standardposvelatt(*old)));
	}
	inertial.set_imu_errors(imu_errs);
}

}  // namespace

namespace navtk {
namespace inertial {

BufferedImu::BufferedImu(const MeasurementPositionVelocityAttitude& pva,
                         shared_ptr<MeasurementImu> initial_imu,
                         double expected_dt,
                         const ImuErrors& imu_errs,
                         const MechanizationOptions& mech_options,
                         double buffer_length)
    : BufferedPva(pva, expected_dt, buffer_length),
      ins(std::make_shared<MechanizationStandard>(),
          make_shared<StandardPosVelAtt>(to_standardposvelatt(pva)),
          mech_options),
      imu_buf(buffer_length / expected_dt + 2),
      reset_err_buf(buffer_length / expected_dt + 2),
      expected_dt(expected_dt),
      dt_sum(0),
      num_dt(0) {

	auto imu_err_shared           = make_shared<ImuErrors>(imu_errs);
	imu_err_shared->time_validity = pva.get_time_of_validity();
	reset_err_buf.insert(imu_err_shared);
	// Don't insert nullptr
	if (initial_imu != nullptr) {
		imu_buf.insert(initial_imu);
	} else {
		imu_buf.insert(make_shared<MeasurementImu>(TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0),
		                                           pva.get_time_of_validity(),
		                                           ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED,
		                                           zeros(3),
		                                           zeros(3),
		                                           std::vector<aspn_xtensor::TypeIntegrity>{}));
	}
	ins.set_imu_errors(imu_errs);
}

bool BufferedImu::reset(shared_ptr<MeasurementPositionVelocityAttitude> pva,
                        shared_ptr<ImuErrors> imu_errs,
                        shared_ptr<MeasurementPositionVelocityAttitude> previous) {
	aspn_xtensor::TypeTimestamp rt = aspn_xtensor::to_type_timestamp(-1.0);

	if (pva != nullptr && in_range(pva->get_time_of_validity())) {
		rt = pva->get_time_of_validity();
	} else if (imu_errs != nullptr && in_range(imu_errs->time_validity)) {
		rt = imu_errs->time_validity;
	} else {
		spdlog::warn(
		    "Attempted reset failed; inputs either were nullptr or failed in_range() check.");
		return false;
	}

	auto reset_pva = (pva != nullptr && pva->get_time_of_validity() == rt) ? pva : calc_pva(rt);
	auto reset_err = (imu_errs != nullptr) ? *imu_errs : *get_imu_errors(rt);

	reset_err.time_validity = rt;

	if (reset_pva != nullptr && in_range(reset_pva->get_time_of_validity())) {

		// Reset the base inertial
		reset_ins(ins, *reset_pva, reset_err, previous);

		// Dump all solutions >= reset time
		auto end_time = time_span().second;

		auto sols_to_erase = pva_buf.get_in_range(reset_pva->get_time_of_validity(), end_time);

		pva_buf.erase(sols_to_erase.first, sols_to_erase.second);

		auto errs_to_erase = reset_err_buf.get_in_range(rt, end_time);
		reset_err_buf.erase(errs_to_erase.first, errs_to_erase.second);

		pva_buf.insert(reset_pva);

		// Get all imu data after reset time
		auto reprops = imu_buf.get_in_range(reset_pva->get_time_of_validity(), end_time);

		// Repropagate, scaling the initial imu meas, if required
		// don't use this->mechanize, as it will re-store imu data
		for (auto imu = reprops.first; imu != reprops.second; ++imu) {
			if (imu == reprops.first) {
				auto scale = ((*imu)->get_aspn_c()->time_of_validity.elapsed_nsec -
				              reset_pva->get_aspn_c()->time_of_validity.elapsed_nsec) *
				             1e-9 / estimated_dt();
				// If resetting to an existing record time, first imu will probably be at same
				// time. Base inertial doesn't detect a dt of zero, so need to check here.
				if (scale > VERY_SMALL) {
					ins.mechanize((*imu)->get_time_of_validity(),
					              (*imu)->get_meas_accel() * scale,
					              (*imu)->get_meas_gyro() * scale);
				}
			} else {
				ins.mechanize((*imu)->get_time_of_validity(),
				              (*imu)->get_meas_accel(),
				              (*imu)->get_meas_gyro());
			}
			pva_buf.insert(to_positionvelocityattitude(ins.get_solution()));
		}

		reset_err_buf.insert(make_shared<ImuErrors>(reset_err));
		return true;
	}
	spdlog::warn(
	    "Attempted reset failed; invalid calc_pva(imu_errs->time_validity) result at time {}.", rt);
	return false;
}

shared_ptr<MeasurementPositionVelocityAttitude> BufferedImu::calc_pva_no_reset_since(
    const aspn_xtensor::TypeTimestamp& time, const aspn_xtensor::TypeTimestamp& since) const {
	// No resets ever performed, no resets after since, or since later than time
	if (reset_err_buf.size() <= 1 || reset_err_buf.back()->time_validity <= since || since > time) {
		return calc_pva(time);
	}

	auto pva_last_reset     = calc_pva(since);
	auto err_mod_last_reset = get_imu_errors(since);

	if (pva_last_reset == nullptr) {
		return nullptr;
	}

	auto ins_clone = Inertial(ins);

	auto prior = calc_pva_no_check(since - estimated_dt());

	reset_ins(ins_clone, *pva_last_reset, *err_mod_last_reset, prior);

	auto reprops = imu_buf.get_in_range(pva_last_reset->get_time_of_validity(), time);

	// If requested time between 2 imu records need to propagate one past and interpolate
	auto stop = (reprops.second == imu_buf.cend()) ? reprops.second : reprops.second + 1;

	shared_ptr<MeasurementPositionVelocityAttitude> sol_out;

	for (auto imu = reprops.first; imu != stop; ++imu) {
		if (imu == reprops.first) {
			auto scale = ((*imu)->get_aspn_c()->time_of_validity.elapsed_nsec -
			              pva_last_reset->get_aspn_c()->time_of_validity.elapsed_nsec) *
			             1e-9 / estimated_dt();
			if (scale > 0) {
				ins_clone.mechanize((*imu)->get_time_of_validity(),
				                    (*imu)->get_meas_accel() * scale,
				                    (*imu)->get_meas_gyro() * scale);
			}
		} else if (imu == stop - 1) {
			auto pre = ins_clone.get_solution();
			ins_clone.mechanize(
			    (*imu)->get_time_of_validity(), (*imu)->get_meas_accel(), (*imu)->get_meas_gyro());
			auto post = ins_clone.get_solution();

			sol_out = utils::linear_interp_pva(
			    to_positionvelocityattitude(pre), to_positionvelocityattitude(post), time);
		} else {
			ins_clone.mechanize(
			    (*imu)->get_time_of_validity(), (*imu)->get_meas_accel(), (*imu)->get_meas_gyro());
		}
	}

	return sol_out;
}

void BufferedImu::add_data(not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) {
	auto imu_ptr = std::dynamic_pointer_cast<MeasurementImu>(data);
	if (imu_ptr == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "BufferedImu received data type other than MeasurementImu.");
		return;
	}
	mechanize(*imu_ptr);
}

void BufferedImu::mechanize(const MeasurementImu& imu) {
	mechanize(std::make_shared<MeasurementImu>(imu));
}

void BufferedImu::mechanize(std::shared_ptr<MeasurementImu> imu) {
	if (imu->get_imu_type() != ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED) {
		log_or_throw<std::invalid_argument>(
		    "Only ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED currently supported in BufferedImu.");
		return;
	}
	auto t   = imu->get_aspn_c()->time_of_validity.elapsed_nsec;
	auto dt  = (t - nsec_time_span().second) * 1e-9;
	auto est = estimated_dt();

	// Initialization could happen at any time, so first dt may be arbitrarily small
	if (num_dt > 0 && (dt < 0.5 * est || dt > 1.5 * est)) {
		spdlog::warn(
		    "Suspicious dt of {} compared against nominal of {} detected at time {}", dt, est, t);
	}

	// If this class not initialized exactly at an imu measurement time, then dt may be small while
	// delta_v and delta_theta are over the 'actual' dt. In this case, we need to scale the
	// measurements.
	if (dt > 0) {
		dt_sum += dt;
		++num_dt;
		ins.mechanize(t, imu->get_meas_accel(), imu->get_meas_gyro());
		imu_buf.insert(imu);
		pva_buf.insert(to_positionvelocityattitude(ins.get_solution()));
	}
}

void BufferedImu::mechanize(const aspn_xtensor::TypeTimestamp& time,
                            const Vector3& delta_v,
                            const Vector3& delta_theta) {
	mechanize(MeasurementImu(TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0),
	                         time,
	                         ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED,
	                         delta_v,
	                         delta_theta,
	                         std::vector<aspn_xtensor::TypeIntegrity>{}));
}

std::shared_ptr<aspn_xtensor::MeasurementImu> BufferedImu::calc_force_and_rate(
    const aspn_xtensor::TypeTimestamp& time) const {
	if (in_range(time)) {
		auto pva   = calc_pva(time);
		auto after = imu_buf.get_nearest_neighbors(time).second;
		if (pva != nullptr && safe_deref(imu_buf, after)) {
			auto force = calc_force_ned(navutils::quat_to_dcm(pva->get_quaternion()),
			                            estimated_dt(),
			                            (*after)->get_meas_gyro(),
			                            (*after)->get_meas_accel());
			auto rate  = calc_rot_rate(*pva, estimated_dt(), (*after)->get_meas_gyro());
			return std::make_shared<aspn_xtensor::MeasurementImu>(to_imu(time, force, rate));
		}
	}
	return nullptr;
}

std::shared_ptr<aspn_xtensor::MeasurementImu> BufferedImu::calc_force_and_rate(
    const aspn_xtensor::TypeTimestamp& time1, const aspn_xtensor::TypeTimestamp& time2) const {

	if (!in_range(time1) || !in_range(time2)) return nullptr;

	auto reprops      = imu_buf.get_in_range(time1, time2);
	auto total_weight = (time2.get_elapsed_nsec() - time1.get_elapsed_nsec()) * 1e-9;

	if (reprops.first == imu_buf.end() || (reprops.second - reprops.first) < 1) {
		// Nothing in-range or single imu
		return calc_force_and_rate(time2);
	}
	if (total_weight < 1e-9) {
		// Avoid divide by 0
		return calc_force_and_rate(time1);
	}

	// If time2 isn't on an exact imu message then the imu measurement that covers time2 isn't
	// included in range. Second should never == end() because of in_range.
	if (reprops.second != imu_buf.end() && time2 != (*reprops.second)->get_time_of_validity()) {
		++reprops.second;
	}

	std::vector<aspn_xtensor::MeasurementImu> vals;
	vals.reserve(reprops.second - reprops.first);

	std::transform(reprops.first,
	               reprops.second,
	               std::back_inserter(vals),
	               [&](shared_ptr<MeasurementImu> imu) -> aspn_xtensor::MeasurementImu {
		               // Halfway between imu meas would be slightly better pva, but have
		               // to work with iterators directly and special case begin() iterator
		               // Unlikely to be worth it since linearly interpolated anyway
		               auto maybe = calc_force_and_rate(imu->get_time_of_validity());
		               if (maybe == nullptr) {
			               // Error that should never happen if the iterators aren't messed with
			               spdlog::error(
			                   "Got iterator to nullptr when expecting an MeasurementImu record; "
			                   "using a zero rates for this element.");
			               return to_imu(imu->get_time_of_validity(), zeros(3), zeros(3));
		               } else {
			               return *maybe;
		               }
	               });

	// Strip out times, calculate deltas and use to weight contributions
	std::vector<aspn_xtensor::TypeTimestamp> tvec;

	tvec.push_back(time1);
	std::for_each(reprops.first, reprops.second - 1, [&tvec](const shared_ptr<MeasurementImu> i) {
		tvec.push_back(i->get_time_of_validity());
	});
	tvec.push_back(time2);

	auto weights = navtk::utils::diff(tvec);

	std::transform(vals.begin(),
	               vals.end(),
	               weights.cbegin(),
	               vals.begin(),
	               [&](aspn_xtensor::MeasurementImu v,
	                   aspn_xtensor::TypeTimestamp d) -> aspn_xtensor::MeasurementImu {
		               v.set_meas_accel(v.get_meas_accel() * to_seconds(d));
		               v.set_meas_gyro(v.get_meas_gyro() * to_seconds(d));
		               return v;
	               });

	auto summed = std::accumulate(
	    vals.begin(),
	    vals.end(),
	    to_imu(aspn_xtensor::to_type_timestamp(), zeros(3), zeros(3)),
	    [](const aspn_xtensor::MeasurementImu& f1, const aspn_xtensor::MeasurementImu& f2) {
		    auto t1                              = f1.get_time_of_validity().get_elapsed_nsec();
		    auto t2                              = f2.get_time_of_validity().get_elapsed_nsec();
		    aspn_xtensor::TypeTimestamp mid_time = aspn_xtensor::TypeTimestamp(t1 + (t2 - t1) / 2);
		    return to_imu(mid_time,
		                  f1.get_meas_accel() + f2.get_meas_accel(),
		                  f1.get_meas_gyro() + f2.get_meas_gyro());
	    });

	summed.set_meas_accel(summed.get_meas_accel() / total_weight);
	summed.set_meas_gyro(summed.get_meas_gyro() / total_weight);
	return make_shared<aspn_xtensor::MeasurementImu>(summed);
}

shared_ptr<ImuErrors> BufferedImu::get_imu_errors(const aspn_xtensor::TypeTimestamp& time) const {

	auto neighbs = reset_err_buf.get_nearest_neighbors(time);
	if (safe_deref(reset_err_buf, neighbs.first)) return *neighbs.first;
	if (safe_deref(reset_err_buf, neighbs.second)) return *neighbs.second;

	return make_shared<ImuErrors>(ins.get_accel_biases(),
	                              ins.get_gyro_biases(),
	                              ins.get_accel_scale_factors(),
	                              ins.get_gyro_scale_factors(),
	                              ins.get_solution()->time_validity);
}

double BufferedImu::estimated_dt() const { return num_dt > 10 ? dt_sum / num_dt : expected_dt; }

}  // namespace inertial
}  // namespace navtk
