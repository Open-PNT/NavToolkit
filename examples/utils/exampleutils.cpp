#include <utils/exampleutils.hpp>

#include <pybind11/embed.h>
#include <xtensor/generators/xrandom.hpp>

#include <navtk/errors.hpp>
#include <navtk/factory.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/utils/ValidationContext.hpp>

using aspn_xtensor::TypeTimestamp;
using navtk::eye;
using navtk::Matrix;
using navtk::Vector;
using navtk::Vector3;
using navtk::zeros;
using navtk::filtering::NavSolution;
using navtk::navutils::east_to_delta_lon;
using navtk::navutils::north_to_delta_lat;
using navtk::navutils::rpy_to_dcm;
using navtk::utils::ValidationContext;
using std::vector;
using xt::all;
using xt::range;
using xt::view;
using xt::random::randn;

namespace navtk {
namespace exampleutils {

vector<NavSolution> constant_vel_pva(NavSolution start_pva, double dt, double stop_time) {
	unsigned int n         = stop_time / dt;
	double heading         = atan2(start_pva.vel[1], start_pva.vel[0]);
	auto dcm_head          = xt::transpose(rpy_to_dcm({{0.0, 0.0, heading}}));
	auto nav_vel           = dcm_head * start_pva.vel;
	double pitch           = atan2(-nav_vel[2], nav_vel[0]);
	auto C_nav_to_platform = xt::transpose(rpy_to_dcm({{0.0, pitch, heading}}));

	vector<NavSolution> pvas;
	for (unsigned int i = 0; i < n; i++) {
		auto time      = start_pva.time + i * dt;
		auto delta_pos = Vector3{{north_to_delta_lat(i * dt * start_pva.vel[0], start_pva.pos[0]),
		                          east_to_delta_lon(i * dt * start_pva.vel[1], start_pva.pos[0]),
		                          -(i * dt * start_pva.vel[2])}};

		auto pva = NavSolution(start_pva.pos + delta_pos, start_pva.vel, C_nav_to_platform, time);
		pvas.push_back(pva);
	}
	return pvas;
}


Matrix noisy_pos_meas(const vector<NavSolution>& truth, const Vector& sigma, const Matrix& err) {
	auto num_meas = truth.size();

	ValidationContext{}.add_matrix(sigma, "sigma").dim(3, 1).validate();

	Matrix adds;
	if (err.size() == 0) {
		adds = zeros(3, num_meas);
	} else {
		adds = err;
	}

	ValidationContext{}.add_matrix(adds, "adds").dim(3, num_meas).validate();

	auto lla = zeros(3, num_meas);
	for (unsigned int i = 0; i < num_meas; i++) {
		auto truth_pos = truth[i].pos;
		lla(0, i) =
		    truth_pos[0] + north_to_delta_lat(randn({1}, adds(0, i), sigma(0))[0], truth_pos[0]);
		lla(1, i) =
		    truth_pos[1] + east_to_delta_lon(randn({1}, adds(1, i), sigma(1))[0], truth_pos[0]);
	}
	view(lla, 2, all()) = noisy_alt_meas(truth, sigma(2), view(adds, 2, all()));
	return lla;
}


Vector noisy_alt_meas(const vector<NavSolution>& truth, double sigma, const Vector& err) {
	auto num_meas = truth.size();
	Vector adds;
	if (err.size() == 0) {
		adds = zeros(num_meas);
	} else {
		adds = err;
	}

	ValidationContext{}.add_matrix(adds, "adds").dim(num_meas, 1).validate();

	auto alts = zeros(num_meas);
	for (unsigned int i = 0; i < num_meas; i++) {
		auto truth_alt = truth[i].pos[2];
		alts(i)        = truth_alt + randn({1}, -adds(i), sigma)[0];
	}
	return alts;
}


Matrix noisy_vel_meas(const vector<NavSolution>& truth, const Vector& sigma, const Matrix& err) {
	auto num_meas = truth.size();
	Matrix adds;
	if (err.size() == 0) {
		adds = zeros(3, num_meas);
	} else {
		adds = err;
	}

	ValidationContext{}.add_matrix(adds, "adds").dim(3, num_meas).validate();
	ValidationContext{}.add_matrix(sigma, "sigma").dim(3, 1).validate();

	auto vel = zeros(3, num_meas);
	for (unsigned int i = 0; i < num_meas; i++) {
		auto truth_vel      = truth[i].vel;
		view(vel, all(), i) = truth_vel + view(adds, all(), i) + sigma * randn({3}, 0.0, 1.0);
	}
	return vel;
}


vector<Matrix> noisy_att_meas(const vector<NavSolution>& truth,
                              const Vector& tilt_sigma,
                              const Matrix& tilts) {
	auto num_meas = truth.size();
	Matrix actual_tilts;
	if (tilts.size() == 0) {
		actual_tilts = zeros(3, num_meas);
	} else {
		actual_tilts = tilts;
	}

	ValidationContext{}.add_matrix(actual_tilts, "actual_tilts").dim(3, num_meas).validate();
	ValidationContext{}.add_matrix(tilt_sigma, "tilt_sigma").dim(3, 1).validate();

	vector<Matrix> atts(num_meas);
	for (unsigned int i = 0; i < num_meas; i++) {
		auto C_nav_to_platform = truth[i].rot_mat;
		auto tilt_val          = view(actual_tilts, all(), i) + randn({3}, 0.0, 1.0) * tilt_sigma;
		atts[i] =
		    xt::transpose((eye(3) - navutils::skew(tilt_val)) * xt::transpose(C_nav_to_platform));
	}
	return atts;
}

}  // namespace exampleutils
}  // namespace navtk
