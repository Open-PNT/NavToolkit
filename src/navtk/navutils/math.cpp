#include <navtk/navutils/math.hpp>

#include <navtk/linear_algebra.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace navutils {

using xt::transpose;
const double PI      = 3.14159265358979323846264338327950288;
const double DEG2RAD = PI / 180.0;
const double RAD2DEG = 180.0 / PI;


Matrix3 skew(const Vector3& angles) {
	return {{0, -angles[2], angles[1]}, {angles[2], 0, -angles[0]}, {-angles[1], angles[0], 0}};
}

Matrix3 ortho_dcm(const Matrix3& dcm) {
	Matrix3 out(dcm);
	Matrix3 delta_mat = eye(3);
	double delta      = 1e-15;
	// Using Matrix3 breaks all_close call (no == for fixed size)
	Matrix z3         = zeros(3, 3);
	Matrix3 dcm_trans = transpose(dcm);

	// Same as one-step correction given by Savage (sub 7.1.1.3-1 into
	// eq 7.1.1.3-10), but iterating to drive errors down
	for (int k = 0; k < 20; k++) {
		delta_mat = 0.5 * (dot(out, dot(dcm_trans, out)) - dcm);
		out -= delta_mat;

		if (xt::allclose(delta_mat, z3, delta, delta)) {
			break;
		}
	}

	return out;
}

double wrap_to_pi(double orig) {
	if (orig <= PI && orig > -PI) {
		return orig;
	}
	auto num_wraps = std::ceil((orig - PI) / 2.0 / PI);
	return orig - num_wraps * 2.0 * PI;
}

double wrap_to_2_pi(double orig) { return fmod(fmod(orig, 2.0 * PI) + 2.0 * PI, 2.0 * PI); }

}  // namespace navutils
}  // namespace navtk
