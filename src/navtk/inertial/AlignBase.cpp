#include <navtk/inertial/AlignBase.hpp>

#include <navtk/errors.hpp>
#include <navtk/filtering/containers/Pose.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::filtering::NavSolution;
using navtk::filtering::Pose;

namespace navtk {
namespace inertial {

AlignBase::AlignBase(bool supports_static, bool supports_dynamic, const filtering::ImuModel& model)
    : computed_alignment(
          {false,
           NavSolution(Pose(Vector3(), Matrix3(), aspn_xtensor::to_type_timestamp()), Vector3())}),
      supports_static(supports_static),
      supports_dynamic(supports_dynamic),
      model(model) {}

bool AlignBase::requires_dynamic() {
	if (!supports_static && !supports_dynamic)
		log_or_throw<std::runtime_error>(
		    "Alignment class must be flagged as supporting static or dynamic data, or both.");
	return !supports_static;
}

AlignBase::AlignmentStatus AlignBase::check_alignment_status() { return alignment_status; }

std::pair<bool, NavSolution> AlignBase::get_computed_alignment() const {
	return computed_alignment;
}

std::pair<bool, Matrix> AlignBase::get_computed_covariance(const CovarianceFormat format) const {
	switch (format) {
	case CovarianceFormat::PINSON15NEDBLOCK: {
		return {false, zeros(15, 15)};
	}
	case CovarianceFormat::PINSON21NEDBLOCK: {
		return {false, zeros(21, 21)};
	}
	default:
		return {false, zeros(1)};
	}
}

std::pair<bool, ImuErrors> AlignBase::get_imu_errors() const { return {false, ImuErrors{}}; }

Matrix AlignBase::bias_stats_from_model(const CovarianceFormat format) const {
	switch (format) {
	case CovarianceFormat::PINSON15NEDBLOCK: {
		Matrix cov = zeros(6, 6);
		xt::view(cov, xt::range(0, 3), xt::range(0, 3)) =
		    xt::diag(xt::pow((Vector)model.accel_bias_initial_sigma, 2));
		xt::view(cov, xt::range(3, 6), xt::range(3, 6)) =
		    xt::diag(xt::pow((Vector)model.gyro_bias_initial_sigma, 2));
		return cov;
	}
	case CovarianceFormat::PINSON21NEDBLOCK: {
		Matrix cov = zeros(12, 12);
		xt::view(cov, xt::range(0, 3), xt::range(0, 3)) =
		    xt::diag(xt::pow((Vector)model.accel_bias_initial_sigma, 2));
		xt::view(cov, xt::range(3, 6), xt::range(3, 6)) =
		    xt::diag(xt::pow((Vector)model.gyro_bias_initial_sigma, 2));
		xt::view(cov, xt::range(6, 9), xt::range(6, 9)) =
		    xt::diag(xt::pow((Vector)model.accel_scale_factor, 2));
		xt::view(cov, xt::range(9, 12), xt::range(9, 12)) =
		    xt::diag(xt::pow((Vector)model.gyro_scale_factor, 2));
		return cov;
	}
	default:
		log_or_throw<std::runtime_error>("Covariance format for imu errors unsupported");
		return zeros(1, 1);
	}
}

}  // namespace inertial
}  // namespace navtk
