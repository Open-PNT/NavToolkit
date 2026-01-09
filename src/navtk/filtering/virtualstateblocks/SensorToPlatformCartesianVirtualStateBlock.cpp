#include <navtk/filtering/virtualstateblocks/SensorToPlatformCartesianVirtualStateBlock.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/derivatives.hpp>
#include <navtk/navutils/leverarms.hpp>
#include <navtk/navutils/navigation.hpp>

namespace navtk {
namespace filtering {

using navtk::navutils::dcm_to_rpy;
using navtk::navutils::rpy_to_dcm;
using xt::all;
using xt::keep;
using xt::range;
using xt::transpose;
using xt::view;

SensorToPlatformCartesianVirtualStateBlock::SensorToPlatformCartesianVirtualStateBlock(
    std::string current,
    std::string target,
    Vector3 l_ps_p,
    Matrix3 C_platform_to_sensor,
    Matrix3 C_k_to_j)
    : VirtualStateBlock(std::move(current), std::move(target)),
      l_ps_p(std::move(l_ps_p)),
      C_platform_to_sensor(std::move(C_platform_to_sensor)),
      C_k_to_j(std::move(C_k_to_j)) {}

not_null<std::shared_ptr<VirtualStateBlock>> SensorToPlatformCartesianVirtualStateBlock::clone() {
	return std::make_shared<SensorToPlatformCartesianVirtualStateBlock>(*this);
}

Matrix SensorToPlatformCartesianVirtualStateBlock::jacobian(const Vector& x,
                                                            const aspn_xtensor::TypeTimestamp&) {

	auto base_rpy      = view(x, range(3, 6));
	auto C_k_to_sensor = transpose(rpy_to_dcm(base_rpy));

	// Actual 'shifted' dcm
	auto C_sensor_to_platform = transpose(C_platform_to_sensor);
	auto C_k_to_platform      = dot(C_sensor_to_platform, C_k_to_sensor);
	auto mapped_rpy           = dcm_to_rpy(transpose(C_k_to_platform));

	// As C_platform_to_sensor is constant (no reliance on rpy) it's derivative goes to 0,
	// so we just need derivative of rpy_to_dcm of original matrix
	auto C_k_to_platform_dr = dot(C_sensor_to_platform, navutils::d_rpy_to_dcm_wrt_r(base_rpy));
	auto C_k_to_platform_dp = dot(C_sensor_to_platform, navutils::d_rpy_to_dcm_wrt_p(base_rpy));
	auto C_k_to_platform_dy = dot(C_sensor_to_platform, navutils::d_rpy_to_dcm_wrt_y(base_rpy));

	Matrix top_right                = zeros(3, 3);
	auto col4                       = dot(C_k_to_j, dot(transpose(C_k_to_platform_dr), -l_ps_p));
	auto col5                       = dot(C_k_to_j, dot(transpose(C_k_to_platform_dp), -l_ps_p));
	auto col6                       = dot(C_k_to_j, dot(transpose(C_k_to_platform_dy), -l_ps_p));
	view(top_right, all(), keep(0)) = to_matrix(col4);
	view(top_right, all(), keep(1)) = to_matrix(col5);
	view(top_right, all(), keep(2)) = to_matrix(col6);

	auto df3_d3 = (C_k_to_platform_dr(1, 2) * C_k_to_platform(2, 2) -
	               C_k_to_platform(1, 2) * C_k_to_platform_dr(2, 2)) /
	              (pow(C_k_to_platform(1, 2), 2.0) + pow(C_k_to_platform(2, 2), 2.0));
	auto df3_d4 = (C_k_to_platform_dp(1, 2) * C_k_to_platform(2, 2) -
	               C_k_to_platform(1, 2) * C_k_to_platform_dp(2, 2)) /
	              (pow(C_k_to_platform(1, 2), 2.0) + pow(C_k_to_platform(2, 2), 2.0));
	auto df3_d5 = 0.0;

	auto df4_d3 = -C_k_to_platform_dr(0, 2) / sqrt(1.0 - pow(C_k_to_platform(0, 2), 2.0));
	auto df4_d4 = -C_k_to_platform_dp(0, 2) / sqrt(1.0 - pow(C_k_to_platform(0, 2), 2.0));
	auto df4_d5 = 0.0;

	auto df5_d3 = (C_k_to_platform_dr(0, 1) * C_k_to_platform(0, 0) -
	               C_k_to_platform(0, 1) * C_k_to_platform_dr(0, 0)) /
	              (pow(C_k_to_platform(0, 1), 2.0) + pow(C_k_to_platform(0, 0), 2.0));
	auto df5_d4 = (C_k_to_platform_dp(0, 1) * C_k_to_platform(0, 0) -
	               C_k_to_platform(0, 1) * C_k_to_platform_dp(0, 0)) /
	              (pow(C_k_to_platform(0, 1), 2.0) + pow(C_k_to_platform(0, 0), 2.0));
	auto df5_d5 = (C_k_to_platform_dy(0, 1) * C_k_to_platform(0, 0) -
	               C_k_to_platform(0, 1) * C_k_to_platform_dy(0, 0)) /
	              (pow(C_k_to_platform(0, 1), 2.0) + pow(C_k_to_platform(0, 0), 2.0));

	Matrix bottom_right{
	    {df3_d3, df3_d4, df3_d5}, {df4_d3, df4_d4, df4_d5}, {df5_d3, df5_d4, df5_d5}};

	Matrix out                          = zeros(6, 6);
	view(out, range(0, 3), range(0, 3)) = eye(3);
	view(out, range(0, 3), range(3, 6)) = top_right;
	view(out, range(3, 6), range(3, 6)) = bottom_right;
	return out;
}

Vector SensorToPlatformCartesianVirtualStateBlock::convert_estimate(
    const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	std::pair<Vector3, Matrix3> start{view(x, range(0, 3)),
	                                  transpose(rpy_to_dcm(view(x, range(3, 6))))};
	auto shifted      = navutils::sensor_to_platform(start, l_ps_p, C_platform_to_sensor, C_k_to_j);
	auto out_position = shifted.first;
	auto out_rpy      = dcm_to_rpy(transpose(shifted.second));
	return Vector{
	    out_position[0], out_position[1], out_position[2], out_rpy[0], out_rpy[1], out_rpy[2]};
}

}  // namespace filtering
}  // namespace navtk
