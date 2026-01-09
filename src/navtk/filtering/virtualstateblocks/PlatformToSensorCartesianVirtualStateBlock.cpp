#include <navtk/filtering/virtualstateblocks/PlatformToSensorCartesianVirtualStateBlock.hpp>

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

PlatformToSensorCartesianVirtualStateBlock::PlatformToSensorCartesianVirtualStateBlock(
    std::string current,
    std::string target,
    Vector3 l_ps_p,
    Matrix3 C_platform_to_sensor,
    Matrix3 C_k_to_j)
    : VirtualStateBlock(std::move(current), std::move(target)),
      l_ps_p(std::move(l_ps_p)),
      C_platform_to_sensor(std::move(C_platform_to_sensor)),
      C_k_to_j(std::move(C_k_to_j)) {}

not_null<std::shared_ptr<VirtualStateBlock>> PlatformToSensorCartesianVirtualStateBlock::clone() {
	return std::make_shared<PlatformToSensorCartesianVirtualStateBlock>(*this);
}

Matrix PlatformToSensorCartesianVirtualStateBlock::jacobian(const Vector& x,
                                                            const aspn_xtensor::TypeTimestamp&) {

	Matrix3 ckj          = transpose(C_k_to_j);
	auto base_rpy        = view(x, range(3, 6));
	auto C_k_to_platform = transpose(rpy_to_dcm(base_rpy));

	std::pair<Vector3, Matrix3> orig{view(x, range(0, 3)), C_k_to_platform};
	auto shifted   = navutils::platform_to_sensor(orig, l_ps_p, C_platform_to_sensor, ckj);
	auto shift_dcm = shifted.second;

	auto C_k_to_platform_dr = navutils::d_rpy_to_dcm_wrt_r(base_rpy);
	auto C_k_to_platform_dp = navutils::d_rpy_to_dcm_wrt_p(base_rpy);
	auto C_k_to_platform_dy = navutils::d_rpy_to_dcm_wrt_y(base_rpy);

	// Bottom right
	// Delta wrt tilts. Just need to dump into the dcm_to_rpy derivatives
	auto C_k_to_sensor_dr = dot(C_platform_to_sensor, C_k_to_platform_dr);
	auto C_k_to_sensor_dp = dot(C_platform_to_sensor, C_k_to_platform_dp);
	auto C_k_to_sensor_dy = dot(C_platform_to_sensor, C_k_to_platform_dy);

	// Copy/paste. Double check.
	auto df3_d3 =
	    (C_k_to_sensor_dr(1, 2) * shift_dcm(2, 2) - shift_dcm(1, 2) * C_k_to_sensor_dr(2, 2)) /
	    (pow(shift_dcm(1, 2), 2.0) + pow(shift_dcm(2, 2), 2.0));
	auto df3_d4 =
	    (C_k_to_sensor_dp(1, 2) * shift_dcm(2, 2) - shift_dcm(1, 2) * C_k_to_sensor_dp(2, 2)) /
	    (pow(shift_dcm(1, 2), 2.0) + pow(shift_dcm(2, 2), 2.0));
	auto df3_d5 = 0.0;

	auto df4_d3 = -C_k_to_sensor_dr(0, 2) / sqrt(1.0 - pow(shift_dcm(0, 2), 2.0));
	auto df4_d4 = -C_k_to_sensor_dp(0, 2) / sqrt(1.0 - pow(shift_dcm(0, 2), 2.0));
	auto df4_d5 = 0.0;

	auto df5_d3 =
	    (C_k_to_sensor_dr(0, 1) * shift_dcm(0, 0) - shift_dcm(0, 1) * C_k_to_sensor_dr(0, 0)) /
	    (pow(shift_dcm(0, 1), 2.0) + pow(shift_dcm(0, 0), 2.0));
	auto df5_d4 =
	    (C_k_to_sensor_dp(0, 1) * shift_dcm(0, 0) - shift_dcm(0, 1) * C_k_to_sensor_dp(0, 0)) /
	    (pow(shift_dcm(0, 1), 2.0) + pow(shift_dcm(0, 0), 2.0));
	auto df5_d5 =
	    (C_k_to_sensor_dy(0, 1) * shift_dcm(0, 0) - shift_dcm(0, 1) * C_k_to_sensor_dy(0, 0)) /
	    (pow(shift_dcm(0, 1), 2.0) + pow(shift_dcm(0, 0), 2.0));

	Matrix bottom_right{
	    {df3_d3, df3_d4, df3_d5}, {df4_d3, df4_d4, df4_d5}, {df5_d3, df5_d4, df5_d5}};


	Matrix top_right                = zeros(3, 3);
	auto d_la_dr                    = dot(C_k_to_j, dot(transpose(C_k_to_platform_dr), l_ps_p));
	auto d_la_dp                    = dot(C_k_to_j, dot(transpose(C_k_to_platform_dp), l_ps_p));
	auto d_la_dy                    = dot(C_k_to_j, dot(transpose(C_k_to_platform_dy), l_ps_p));
	view(top_right, all(), keep(0)) = to_matrix(d_la_dr);
	view(top_right, all(), keep(1)) = to_matrix(d_la_dp);
	view(top_right, all(), keep(2)) = to_matrix(d_la_dy);

	Matrix out                          = zeros(6, 6);
	view(out, range(0, 3), range(0, 3)) = eye(3);
	view(out, range(0, 3), range(3, 6)) = top_right;
	view(out, range(3, 6), range(3, 6)) = bottom_right;
	return out;
}

Vector PlatformToSensorCartesianVirtualStateBlock::convert_estimate(
    const Vector& x, const aspn_xtensor::TypeTimestamp&) {
	std::pair<Vector3, Matrix3> start{view(x, range(0, 3)),
	                                  transpose(rpy_to_dcm(view(x, range(3, 6))))};
	auto shifted      = navutils::platform_to_sensor(start, l_ps_p, C_platform_to_sensor, C_k_to_j);
	auto out_position = shifted.first;
	auto out_rpy      = dcm_to_rpy(transpose(shifted.second));
	return Vector{
	    out_position[0], out_position[1], out_position[2], out_rpy[0], out_rpy[1], out_rpy[2]};
}

}  // namespace filtering
}  // namespace navtk
