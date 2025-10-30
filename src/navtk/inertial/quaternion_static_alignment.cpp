#include <navtk/inertial/quaternion_static_alignment.hpp>

#include <xtensor-blas/xlinalg.hpp>

#include <navtk/errors.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>

namespace navtk {
namespace inertial {

Matrix3 quaternion_static_alignment(const Vector3& dv_avg, const Vector3& dth_avg) {

	// navtk::norm doesn't like Vector3
	Vector accel(dv_avg);
	Vector ang_rate(dth_avg);

	// Find some rotation matrix ${\bf C}_b^{n'}$ and frame $n'$ such that ${\bf C}_b^{n'}{\bf
	// a}^b=\left[0,0,\left|{\bf a}^b\right|\right]^T$ and ${\bf z}_n={\bf z}_{n'}$ We see the frame
	// $n'$ is some frame aligned with the $n$ frame but possibly off by some rotation about the
	// ${\bf z}_n$ axis. That is, ${\bf z}_{n}$ is an eigenvector of the rotation matrix:
	// $C_{n'}^{n}$
	// We need to find a rotation matrix which will rotate the acceleration vector into the z axis.
	// Calculate normal vector for the plane formed by ${\bf z}_b^b$ and ${\bf a}^b={\bf z}_n^b$ via
	// normalized cross product
	Vector z                    = {0, 0, -1};  // this would be 0,0,1 if we use ENU instead of NED
	Vector body_to_nav_z_normal = cross(accel, z);
	body_to_nav_z_normal /= navtk::norm(body_to_nav_z_normal);

	// Find angle of rotation between ${\bf z}_b^b$ and ${\bf a}^b={\bf z}_n^b$ via formula
	// $\theta=\cos^{-1}\left(\frac{{\bf z}_b^b    \cdot {\bf z}_n^b}{|{\bf z}_b^b| |{\bf
	// z}_n^b|}\right)$
	auto z_rot = std::acos(dot(z, accel)[0] / (navtk::norm(accel) * navtk::norm(z)));

	// Construct rotation matrix ${\bf C}_b^{n'}$ such that ${\bf z}_n^{n'}={\bf C}_b^{n'}{\bf
	// z}_n^b=\left[0,0,\left|{\bf a}^b\right|\right]^T$
	Matrix3 C_sensor_to_nedprime = navutils::axis_angle_to_dcm(body_to_nav_z_normal, z_rot);

	// Verify that ${\bf C}_b^{n'}{\bf a}^b=\left[0,0,\left|{\bf a}^b\right|\right]^T$
	Vector accel_np = dot(C_sensor_to_nedprime, accel);

	if (accel_np[0] >= 1e-10 || accel_np[1] >= 1e-10) {
		log_or_throw(
		    "Assertion failed in quaternion_static_alignment: "
		    "assert(accel_np[0] < 1e-10 && accel_np[1] < 1e-10)");
	}

	// Find the rotation matrix ${\bf C}_{n'}^{n}$
	// Rotate the gyro measurement into $n'$ via ${\bf g}^{n'}={{\bf C}_b^{n'}}{\bf g}^b$
	Vector gy_np = dot(C_sensor_to_nedprime, ang_rate);

	// Find ${\bf C}_{n'}^{n}$ such that ${\bf C}_{n'}^{n}{\bf g}^{n'}=[0, g_N, g_U]^T$ for some
	// $g_N,g_U \in R$ and ${\bf z}_n$ is an eigen vector of the rotation (i.e. ${\bf
	// C}_{n'}^{n}{\bf z}_n^{n'}={\bf z}_n^{n'}$) We start by projecting the gyro measurement onto
	// the north-east plane
	Vector gy_proj = {gy_np[0], gy_np[1], 0};

	// Now find angle of rotation about the z axis
	Vector x    = {1, 0, 0};
	auto xy_rot = std::acos(dot(x, gy_proj)[0] / (navtk::norm(gy_proj) * navtk::norm(x)));

	// And axis of rotation (required to avoid quadrant issue)
	Vector xy_north_normal = cross(gy_proj, x);
	xy_north_normal /= navtk::norm(xy_north_normal);

	// Rotate about the z axis (Only choice, we have 1 degree of freedom)
	Matrix3 C_navprime_to_ned = navutils::axis_angle_to_dcm(xy_north_normal, xy_rot);

	// Calculate final vectors
	Matrix mult    = dot(C_navprime_to_ned, C_sensor_to_nedprime);
	Vector accel_n = dot(mult, accel);
	Vector gy_n    = dot(mult, ang_rate);

	if (gy_n[1] >= 1e-10) {
		log_or_throw("Assertion failed in quaternion_static_alignment: assert(gy_n[1] < 1e-10)");
	}

	return mult;
}

}  // namespace inertial
}  // namespace navtk
