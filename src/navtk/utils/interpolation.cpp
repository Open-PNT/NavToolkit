#include <navtk/utils/interpolation.hpp>

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/quaternions.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/CubicSplineModel.hpp>
#include <navtk/utils/InterpolationModel.hpp>
#include <navtk/utils/LinearModel.hpp>
#include <navtk/utils/Ordered.hpp>
#include <navtk/utils/QuadraticSplineModel.hpp>
#include <navtk/utils/sortable_vectors.hpp>

#include <spdlog/fmt/bundled/ranges.h>  // needed for spdlog to support vector<T>; must be after spdlog.h include.

using aspn_xtensor::TypeTimestamp;
using std::endl;
using std::pair;
using std::sort;
using std::vector;

namespace navtk {
namespace utils {

template <typename T>
void check_data_source_validity(const vector<T> &time_source, const vector<T> &data_source) {
	if (time_source.size() != data_source.size()) {
		log_or_throw<std::runtime_error>(
		    "Exception Occurred: Source time and source data are not matching lengths for "
		    "interpolation.");
	}
	if (time_source.size() < 2) {
		log_or_throw<std::runtime_error>(
		    "Exception Occurred: Trying to interpolate with source data that is smaller that 2 "
		    "data points.");
	}
}

template <typename T>
vector<Size> condition_source_data(vector<double> &time_source,
                                   vector<T> &data_source,
                                   vector<double> &time_interp) {

	check_data_source_validity(time_source, data_source);

	// pair up and time sort the source data
	auto check_source = pair_and_time_sort_data(time_source, data_source);

	// Duplicates in source
	auto source_dups = find_duplicates(check_source);

	// Remove duplicated source data
	remove_at_indices(check_source, source_dups);

	// Condition interpolation time tags
	sort(time_interp.begin(), time_interp.begin() + time_interp.size());

	// These are ignored indices in the interpolation time
	auto ignored_indices = find_outside(time_interp, check_source);

	// track duplicate indices in the interpolation time tags
	auto duplicate_indices = find_duplicates(time_interp);

	if (!duplicate_indices.empty()) {
		spdlog::warn(
		    "Duplicate time tag(s) found in interpolation time tags at index {}. "
		    "Duplicate will be ignored in interpolation.",
		    duplicate_indices);
	}

	ignored_indices.insert(
	    ignored_indices.end(), duplicate_indices.begin(), duplicate_indices.end());

	sort(ignored_indices.begin(), ignored_indices.end());

	// Overwrite input data
	auto splits = split_vector_pairs(check_source);
	time_source = splits.first;
	data_source = splits.second;

	remove_at_indices(time_interp, ignored_indices);
	if (time_interp.size() == 0) {
		spdlog::warn(
		    "Interpolation time tags were not within the bounds of the source time "
		    "tags; all query points rejected.");
	}

	return ignored_indices;
}

pair<vector<Size>, vector<double>> all_interpolate(
    vector<double> time_source,
    vector<double> data_source,
    vector<double> time_interp,
    std::function<not_null<std::unique_ptr<InterpolationModel>>(const vector<double> &,
                                                                const vector<double> &)> fact) {

	// check for any requirements not met by the input vectors
	vector<Size> unused_indices = condition_source_data(time_source, data_source, time_interp);

	vector<double> y_interp;  // output interpolated data
	y_interp.reserve(time_interp.size());

	auto model = fact(time_source, data_source);

	for (auto t = time_interp.cbegin(); t < time_interp.cend(); t++) {
		y_interp.push_back(model->y_at(*t));
	}

	return std::make_pair(unused_indices, y_interp);
}

pair<vector<Size>, vector<double>> linear_interpolate(const vector<double> &orig_time_source,
                                                      const vector<double> &data_source,
                                                      const vector<double> &orig_time_interp) {
	auto fact = [](const vector<double> &x, const vector<double> &y) {
		return std::make_unique<LinearModel>(x, y);
	};
	return all_interpolate(orig_time_source, data_source, orig_time_interp, fact);
}

pair<vector<Size>, vector<double>> quadratic_spline_interpolate(
    const vector<double> &orig_time_source,
    const vector<double> &data_source,
    const vector<double> &orig_time_interp) {

	auto fact = [](const vector<double> &x, const vector<double> &y) {
		return std::make_unique<QuadraticSplineModel>(x, y);
	};
	return all_interpolate(orig_time_source, data_source, orig_time_interp, fact);
}

pair<vector<Size>, vector<double>> cubic_spline_interpolate(
    const vector<double> &orig_time_source,
    const vector<double> &data_source,
    const vector<double> &orig_time_interp) {

	std::function<not_null<std::unique_ptr<InterpolationModel>>(const vector<double> &,
	                                                            const vector<double> &)>
	    fact;
	if (data_source.size() < 4) {
		spdlog::warn(
		    "Need at least 4 source data points to perform cubic interpolation. "
		    "Switching to using linear interpolation.");

		fact = [](const vector<double> &x, const vector<double> &y) {
			return std::make_unique<LinearModel>(x, y);
		};

	} else {
		fact = [](const vector<double> &x, const vector<double> &y) {
			return std::make_unique<CubicSplineModel>(x, y);
		};
	}

	return all_interpolate(orig_time_source, data_source, orig_time_interp, fact);
}

aspn_xtensor::MeasurementPositionVelocityAttitude linear_interp_pva(
    const aspn_xtensor::MeasurementPositionVelocityAttitude &pva1,
    const aspn_xtensor::MeasurementPositionVelocityAttitude &pva2,
    const aspn_xtensor::TypeTimestamp &t) {

	int64_t time1  = pva1.get_aspn_c()->time_of_validity.elapsed_nsec;
	int64_t time2  = pva2.get_aspn_c()->time_of_validity.elapsed_nsec;
	int64_t time_t = t.get_elapsed_nsec();

	if ((time1) > time2) {
		return linear_interp_pva(pva2, pva1, t);
	}

	if (time_t <= time1) {
		if (time_t != time1) {
			spdlog::warn(
			    "Requested interpolation time {} before earliest pva point at {}", time_t, time1);
		}
		return pva1;
	};
	if (time_t >= time2) {
		if (time_t != time2) {
			spdlog::warn(
			    "Requested interpolation time {} after latest pva point at {}", time_t, time2);
		}
		return pva2;
	};

	return linear_extrapolate_pva(pva1, pva2, t);
}

not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> linear_interp_pva(
    not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> pva1,
    not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> pva2,
    const aspn_xtensor::TypeTimestamp &t) {

	int64_t time1  = pva1->get_aspn_c()->time_of_validity.elapsed_nsec;
	int64_t time2  = pva2->get_aspn_c()->time_of_validity.elapsed_nsec;
	int64_t time_t = t.get_elapsed_nsec();

	if ((time1) > time2) {
		return linear_interp_pva(pva2, pva1, t);
	}

	if (time_t <= time1) {
		if (time_t != time1) {
			spdlog::warn(
			    "Requested interpolation time {} before earliest pva point at {}", time_t, time1);
		}
		return pva1;
	};
	if (time_t >= time2) {
		if (time_t != time2) {
			spdlog::warn(
			    "Requested interpolation time {} after latest pva point at {}", time_t, time2);
		}
		return pva2;
	};

	return linear_extrapolate_pva(pva1, pva2, t);
}

aspn_xtensor::MeasurementPositionVelocityAttitude linear_extrapolate_pva(
    const aspn_xtensor::MeasurementPositionVelocityAttitude &pva1,
    const aspn_xtensor::MeasurementPositionVelocityAttitude &pva2,
    const aspn_xtensor::TypeTimestamp &t) {

	aspn_xtensor::TypeTimestamp time1 = pva1.get_time_of_validity();
	aspn_xtensor::TypeTimestamp time2 = pva2.get_time_of_validity();

	if (time1 > time2) {
		return linear_extrapolate_pva(pva2, pva1, t);
	}

	if (time1 == time2) {
		return pva2;
	}

	auto l     = linear_interpolate(time1, pva1.get_p1(), time2, pva2.get_p1(), t);
	auto ln    = linear_interpolate(time1, pva1.get_p2(), time2, pva2.get_p2(), t);
	auto a     = linear_interpolate(time1, pva1.get_p3(), time2, pva2.get_p3(), t);
	Vector3 v1 = {pva1.get_v1(), pva1.get_v2(), pva1.get_v3()};
	Vector3 v2 = {pva2.get_v1(), pva2.get_v2(), pva2.get_v3()};
	auto v     = linear_interpolate(time1, v1, time2, v2, t);

	Vector3 rpy1 = navutils::quat_to_rpy(pva1.get_quaternion());
	Vector3 rpy2 = navutils::quat_to_rpy(pva2.get_quaternion());
	auto att     = linear_extrapolate_rpy(time1, rpy1, time2, rpy2, t);

	// TODO PNTOS-376 Figure out the proper, safe way to interpolate a cov matrix.
	return aspn_xtensor::MeasurementPositionVelocityAttitude(pva1.get_header(),
	                                                         t,
	                                                         pva1.get_reference_frame(),
	                                                         l,
	                                                         ln,
	                                                         a,
	                                                         v(0),
	                                                         v(1),
	                                                         v(2),
	                                                         navutils::rpy_to_quat(att),
	                                                         pva1.get_covariance(),
	                                                         pva1.get_error_model(),
	                                                         pva1.get_error_model_params(),
	                                                         pva1.get_integrity());
}

not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> linear_extrapolate_pva(
    not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> pva1,
    not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> pva2,
    const aspn_xtensor::TypeTimestamp &t) {

	aspn_xtensor::TypeTimestamp time1 = pva1->get_time_of_validity();
	aspn_xtensor::TypeTimestamp time2 = pva2->get_time_of_validity();

	if (time1 > time2) {
		return linear_extrapolate_pva(pva2, pva1, t);
	}

	if (time1 == time2) {
		return pva2;
	}

	auto l     = linear_interpolate(time1, pva1->get_p1(), time2, pva2->get_p1(), t);
	auto ln    = linear_interpolate(time1, pva1->get_p2(), time2, pva2->get_p2(), t);
	auto a     = linear_interpolate(time1, pva1->get_p3(), time2, pva2->get_p3(), t);
	Vector3 v1 = {pva1->get_v1(), pva1->get_v2(), pva1->get_v3()};
	Vector3 v2 = {pva2->get_v1(), pva2->get_v2(), pva2->get_v3()};
	auto v     = linear_interpolate(time1, v1, time2, v2, t);

	Vector3 rpy1 = navutils::quat_to_rpy(pva1->get_quaternion());
	Vector3 rpy2 = navutils::quat_to_rpy(pva2->get_quaternion());
	auto att     = linear_extrapolate_rpy(time1, rpy1, time2, rpy2, t);

	// TODO PNTOS-376 Figure out the proper, safe way to interpolate a cov matrix.
	return std::make_shared<aspn_xtensor::MeasurementPositionVelocityAttitude>(
	    pva1->get_header(),
	    t,
	    pva1->get_reference_frame(),
	    l,
	    ln,
	    a,
	    v(0),
	    v(1),
	    v(2),
	    navutils::rpy_to_quat(att),
	    pva1->get_covariance(),
	    pva1->get_error_model(),
	    pva1->get_error_model_params(),
	    pva1->get_integrity());
}

Vector3 linear_interp_rpy(const aspn_xtensor::TypeTimestamp &t1,
                          const Vector3 &rpy1,
                          const aspn_xtensor::TypeTimestamp &t2,
                          const Vector3 &rpy2,
                          const aspn_xtensor::TypeTimestamp &t) {

	if (t1 > t2) {
		return linear_interp_rpy(t2, rpy2, t1, rpy1, t);
	}

	if (t < t1) {
		spdlog::warn("Requested interpolation time {} before earliest data point at {}", t, t1);
		return rpy1;
	};
	if (t > t2) {
		spdlog::warn("Requested interpolation time {} after latest data point at {}", t, t2);
		return rpy2;
	};

	return linear_extrapolate_rpy(t1, rpy1, t2, rpy2, t);
}

Vector3 linear_extrapolate_rpy(const aspn_xtensor::TypeTimestamp &t1,
                               const Vector3 &rpy1,
                               const aspn_xtensor::TypeTimestamp &t2,
                               const Vector3 &rpy2,
                               const aspn_xtensor::TypeTimestamp &t) {
	if (t1 > t2) {
		return linear_extrapolate_rpy(t2, rpy2, t1, rpy1, t);
	}

	auto tq = (double)(t.get_elapsed_nsec() - t1.get_elapsed_nsec()) /
	          (t2.get_elapsed_nsec() - t1.get_elapsed_nsec());
	auto q1 = navutils::rpy_to_quat(rpy1);
	auto q2 = navutils::rpy_to_quat(rpy2);
	auto d  = navtk::dot(q1, q2)[0];
	if (d < 0) {
		q2 = -q2;
		d  = -d;
	}

	if (d > 0.999) {
		auto q = navutils::quat_norm(linear_interpolate(t1, q1, t2, q2, t));
		return navutils::quat_to_rpy(q);
	}

	auto theta_0 = acos(d);
	double theta = theta_0 * tq;
	auto v2      = navutils::quat_norm(q2 - q1 * d);
	auto norm_q  = navutils::quat_norm(q1 * cos(theta) + v2 * sin(theta));
	return navutils::dcm_to_rpy(navutils::ortho_dcm(navutils::quat_to_dcm(norm_q)));
}

}  // namespace utils
}  // namespace navtk
