#include <navtk/navutils/quaternions.hpp>

#include <xtensor/reducers/xnorm.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace navutils {

Vector4 quat_conj(const Vector4& q) { return Vector4{q(0), -q(1), -q(2), -q(3)}; }

Vector4 quat_norm(const Vector4& q) {
	// T+W 11.57 have the norm being calculated as sqrt(q, q*), where q* is the complex conjugate
	// of q. This doesn't work, and other refs (Shuster, Survey of Attitude Representations, eq 161;
	// Savage, Strapdown Analytics eq 3.2.4-21) give the more standard definition, used here.
	return q / xt::norm_l2(q);
}

Vector4 quat_mult(const Vector4& q, const Vector4& p) {
	auto q0 = q(0);
	auto q1 = q(1);
	auto q2 = q(2);
	auto q3 = q(3);
	auto p0 = p(0);
	auto p1 = p(1);
	auto p2 = p(2);
	auto p3 = p(3);
	return Vector4{q0 * p0 - q1 * p1 - q2 * p2 - q3 * p3,
	               q1 * p0 + q0 * p1 - q3 * p2 + q2 * p3,
	               q2 * p0 + q3 * p1 + q0 * p2 - q1 * p3,
	               q3 * p0 - q2 * p1 + q1 * p2 + q0 * p3};
}

Vector3 quat_rot(const Vector4& q, const Vector3& r) {
	return xt::view(quat_mult(quat_mult(q, xt::concatenate(xt::xtuple(zeros(1), r))), quat_conj(q)),
	                xt::range(1, 4));
}

Vector4 quat_prop(const Vector4& q, const Vector3& r) {
	auto r_mag = xt::norm_l2(r)[0];
	if (r_mag == 0) {
		return q;
	}
	auto ac = cos(r_mag / 2.0);
	auto as = sin(r_mag / 2.0) / r_mag;
	Vector4 v{ac, as * r(0), as * r(1), as * r(2)};
	return quat_mult(q, v);
}

Vector4 llh_to_quat_en(const Vector3& llh) { return dcm_to_quat(llh_to_cen(llh)); }

Vector4 correct_quat_with_tilt(const Vector4& q, const Vector3& t) {
	// See T+W 2nd ed eq 11.43 through 11.49 (just with swapped rates)
	auto r_mag = xt::norm_l2(t)[0];
	if (r_mag == 0) {
		return q;
	}
	auto ac = cos(r_mag / 2.0);
	auto as = sin(r_mag / 2.0) / r_mag;
	Vector4 v{ac, as * t(0), as * t(1), as * t(2)};
	return quat_mult(v, q);
}

}  // namespace navutils
}  // namespace navtk
