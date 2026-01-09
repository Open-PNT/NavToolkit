#include <navtk/navutils/leverarms.hpp>

#include <navtk/linear_algebra.hpp>

namespace navtk {
namespace navutils {

std::pair<Vector3, Matrix3> sensor_to_platform(const std::pair<Vector3, Matrix3>& sensor_pose,
                                               const Vector3& platform_to_sensor_in_platform,
                                               const Matrix3& C_platform_to_sensor,
                                               const Matrix3& C_k_to_j) {
	Matrix3 C_k_to_platform = dot(transpose(C_platform_to_sensor), sensor_pose.second);
	Vector3 out_pos =
	    sensor_pose.first -
	    dot(C_k_to_j, dot(transpose(C_k_to_platform), platform_to_sensor_in_platform));
	return {out_pos, C_k_to_platform};
}

std::pair<Vector3, Matrix3> platform_to_sensor(const std::pair<Vector3, Matrix3>& platform_pose,
                                               const Vector3& platform_to_sensor_in_platform,
                                               const Matrix3& C_platform_to_sensor,
                                               const Matrix3& C_k_to_j) {
	Matrix3 C_k_to_sensor = dot(C_platform_to_sensor, platform_pose.second);
	Vector3 out_pos =
	    platform_pose.first +
	    dot(C_k_to_j, dot(transpose(platform_pose.second), platform_to_sensor_in_platform));
	return {out_pos, C_k_to_sensor};
}

std::pair<Vector3, Matrix3> obs_in_platform_to_sensor(
    const std::pair<Vector3, Matrix3>& obs_in_platform,
    const Vector3& platform_to_sensor_in_platform,
    const Matrix3& C_platform_to_sensor) {
	Matrix3 C_sensor_to_observed = dot(obs_in_platform.second, transpose(C_platform_to_sensor));
	Vector3 out_pos =
	    dot(C_platform_to_sensor, obs_in_platform.first - platform_to_sensor_in_platform);
	return {out_pos, C_sensor_to_observed};
}

std::pair<Vector3, Matrix3> obs_in_sensor_to_platform(
    const std::pair<Vector3, Matrix3>& obs_in_sensor,
    const Vector3& platform_to_sensor_in_platform,
    const Matrix3& C_platform_to_sensor) {
	Matrix3 C_platform_to_observed = dot(obs_in_sensor.second, C_platform_to_sensor);
	Vector3 out_pos =
	    platform_to_sensor_in_platform + dot(transpose(C_platform_to_sensor), obs_in_sensor.first);
	return {out_pos, C_platform_to_observed};
}

}  // namespace navutils
}  // namespace navtk
