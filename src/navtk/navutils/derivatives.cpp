#include <navtk/navutils/derivatives.hpp>

#include <limits>

#include <xtensor/xnorm.hpp>

#include <navtk/factory.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/quaternions.hpp>
#include <navtk/navutils/wgs84.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace navutils {

Matrix3 d_cen_wrt_lat(const Vector3& lla) {
	auto clat = cos(lla[0]);
	auto clon = cos(lla[1]);
	auto slat = sin(lla[0]);
	auto slon = sin(lla[1]);
	return Matrix{
	    {-clat * clon, 0, slat * clon}, {-clat * slon, 0, slat * slon}, {-slat, 0, -clat}};
}

Matrix3 d_cen_wrt_lon(const Vector3& lla) {
	auto clat = cos(lla[0]);
	auto clon = cos(lla[1]);
	auto slat = sin(lla[0]);
	auto slon = sin(lla[1]);
	return Matrix{
	    {slat * slon, -clon, clat * slon}, {-slat * clon, -slon, -clat * clon}, {0, 0, 0}};
}

Matrix3 d_cns_wrt_r(const Vector3& rpy) {
	auto cr = cos(rpy[0]);
	auto cp = cos(rpy[1]);
	auto cy = cos(rpy[2]);
	auto sr = sin(rpy[0]);
	auto sp = sin(rpy[1]);
	auto sy = sin(rpy[2]);

	return Matrix{{0, sr * sy + cr * sp * cy, cr * sy - sr * sp * cy},
	              {0, -sr * cy + cr * sp * sy, -cr * cy - sr * sp * sy},
	              {0, cr * cp, -sr * cp}};
}

Matrix3 d_cns_wrt_p(const Vector3& rpy) {
	auto cr = cos(rpy[0]);
	auto cp = cos(rpy[1]);
	auto cy = cos(rpy[2]);
	auto sr = sin(rpy[0]);
	auto sp = sin(rpy[1]);
	auto sy = sin(rpy[2]);

	return Matrix{{-sp * cy, sr * cp * cy, cr * cp * cy},
	              {-sp * sy, sr * cp * sy, cr * cp * sy},
	              {-cp, -sr * sp, -cr * sp}};
}

Matrix3 d_cns_wrt_y(const Vector3& rpy) {
	auto cr = cos(rpy[0]);
	auto cp = cos(rpy[1]);
	auto cy = cos(rpy[2]);
	auto sr = sin(rpy[0]);
	auto sp = sin(rpy[1]);
	auto sy = sin(rpy[2]);

	return Matrix{{-cp * sy, -cr * cy - sr * sp * sy, sr * cy - cr * sp * sy},
	              {cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy},
	              {0, 0, 0}};
}

Matrix3 d_llh_to_ecef_wrt_llh(const Vector3& llh) {
	double clat  = cos(llh[0]);
	double slat  = sin(llh[0]);
	double clon  = cos(llh[1]);
	double slon  = sin(llh[1]);
	double h     = llh[2];
	double slat2 = pow(slat, 2.0);

	auto R    = SEMI_MAJOR_RADIUS;
	auto e2   = ECCENTRICITY_SQUARED;
	double n  = R / (sqrt(1.0 - e2 * slat2));
	double dn = R * (e2 * slat * clat) / pow(1.0 - e2 * slat2, 1.5);

	return Matrix{{dn * clat * clon - (n + h) * slat * clon, -(n + h) * clat * slon, clat * clon},
	              {dn * clat * slon - (n + h) * slat * slon, (n + h) * clat * clon, clat * slon},
	              {h * clat + (1.0 - e2) * (dn * slat + n * clat), 0.0, slat}};
}

Matrix3 d_ecef_to_llh_wrt_ecef(const Vector3& ecef) {
	double phi0              = atan2(ecef[2], sqrt(pow(ecef[0], 2) + pow(ecef[1], 2)));
	double h0                = 0;
	double lam               = atan2(ecef[1], ecef[0]);
	int count                = 0;
	const int max_iterations = 5;

	Vector llh{phi0, lam, h0};
	Vector d_ecef{100, 0, 0};
	Matrix jac = eye(3);
	while ((sqrt(pow(d_ecef[0], 2) + pow(d_ecef[1], 2)) > 7e-6 || std::abs(d_ecef[2]) > 1e-6) &&
	       count < max_iterations) {
		auto ecef_est = llh_to_ecef(llh);
		d_ecef        = ecef - ecef_est;
		jac           = inverse(d_llh_to_ecef_wrt_llh(llh));
		llh += dot(jac, d_ecef);
		++count;
	}

	return inverse(d_llh_to_ecef_wrt_llh(llh));
}

Matrix3 d_cne_wrt_k(const Vector3& dk, const Vector3& llh) {
	auto slat  = sin(llh[0]);
	auto clat  = cos(llh[0]);
	auto slon  = sin(llh[1]);
	auto clon  = cos(llh[1]);
	double dk0 = dk[0];
	double dk1 = dk[1];
	return Matrix{{-clat * clon * dk0 + slat * slon * dk1,
	               -clat * slon * dk0 - slat * clon * dk1,
	               -slat * dk0},
	              {-clon * dk1, -slon * dk1, 0},
	              {slat * clon * dk0 + clat * slon * dk1,
	               slat * slon * dk0 - clat * clon * dk1,
	               -clat * dk0}};
}

Matrix3 d_dcm_to_rpy(const Matrix3& a,
                     const Matrix3& dadx,
                     const Matrix3& dady,
                     const Matrix3& dadz,
                     const Matrix3& b,
                     const Matrix3& dbdx,
                     const Matrix3& dbdy,
                     const Matrix3& dbdz) {
	auto d_min = std::numeric_limits<double>::min();
	auto jac   = xt::empty<double>({3, 3});
	auto ab    = dot(a, b);
	auto dx    = dot(dadx, b) + dot(a, dbdx);
	auto dy    = dot(dady, b) + dot(a, dbdy);
	auto dz    = dot(dadz, b) + dot(a, dbdz);

	auto den1 = std::max(pow(ab(2, 2), 2.0) + pow(ab(1, 2), 2.0), d_min);
	jac(0, 0) = (dx(1, 2) * ab(2, 2) - ab(1, 2) * dx(2, 2)) / den1;
	jac(0, 1) = (dy(1, 2) * ab(2, 2) - ab(1, 2) * dy(2, 2)) / den1;
	jac(0, 2) = (dz(1, 2) * ab(2, 2) - ab(1, 2) * dz(2, 2)) / den1;

	auto den2 = std::max(sqrt(1.0 - pow(ab(0, 2), 2.0)), d_min);
	jac(1, 0) = -dx(0, 2) / den2;
	jac(1, 1) = -dy(0, 2) / den2;
	jac(1, 2) = -dz(0, 2) / den2;

	auto rpy = dcm_to_rpy(xt::transpose(ab));

	if (std::fabs(rpy(1) - PI / 2.0) < 1e-12) {
		// pi/2
		auto den3 = std::max(pow(ab(2, 0) + ab(1, 1), 2.0) + pow(ab(2, 1) - ab(1, 0), 2.0), d_min);
		jac(2, 0) = ((dx(2, 1) - dx(1, 0)) * (ab(2, 0) + ab(1, 1)) -
		             (ab(2, 1) - ab(1, 0)) * (dx(2, 0) + dx(1, 1))) /
		                den3 +
		            jac(0, 0);
		jac(2, 1) = ((dy(2, 1) - dy(1, 0)) * (ab(2, 0) + ab(1, 1)) -
		             (ab(2, 1) - ab(1, 0)) * (dy(2, 0) + dy(1, 1))) /
		                den3 +
		            jac(0, 1);
		jac(2, 2) = ((dz(2, 1) - dz(1, 0)) * (ab(2, 0) + ab(1, 1)) -
		             (ab(2, 1) - ab(1, 0)) * (dz(2, 0) + dz(1, 1))) /
		                den3 +
		            jac(0, 2);
	} else if (std::fabs(rpy(1) + PI / 2.0) < 1e-12) {
		//-pi/2
		auto den3 = std::max(pow(ab(2, 0) - ab(1, 1), 2.0) + pow(ab(2, 1) + ab(1, 0), 2.0), d_min);
		jac(2, 0) = ((dx(2, 1) + dx(1, 0)) * (ab(2, 0) - ab(1, 1)) -
		             (ab(2, 1) + ab(1, 0)) * (dx(2, 0) - dx(1, 1))) /
		                den3 -
		            jac(0, 0);
		jac(2, 1) = ((dy(2, 1) + dy(1, 0)) * (ab(2, 0) - ab(1, 1)) -
		             (ab(2, 1) + ab(1, 0)) * (dy(2, 0) - dy(1, 1))) /
		                den3 -
		            jac(0, 1);
		jac(2, 2) = ((dz(2, 1) + dz(1, 0)) * (ab(2, 0) - ab(1, 1)) -
		             (ab(2, 1) + ab(1, 0)) * (dz(2, 0) - dz(1, 1))) /
		                den3 -
		            jac(0, 2);
	} else {
		// Normal
		auto den3 = std::max(pow(ab(0, 0), 2.0) + pow(ab(0, 1), 2.0), d_min);
		jac(2, 0) = (dx(0, 1) * ab(0, 0) - ab(0, 1) * dx(0, 0)) / den3;
		jac(2, 1) = (dy(0, 1) * ab(0, 0) - ab(0, 1) * dy(0, 0)) / den3;
		jac(2, 2) = (dz(0, 1) * ab(0, 0) - ab(0, 1) * dz(0, 0)) / den3;
	}
	return jac;
}

Matrix3 d_rpy_tilt_corr_wrt_tilt(const Vector3& tilts, const Matrix3& C_nav_to_platform) {
	Matrix dx{{0, 0, 0}, {0, 0, -1}, {0, 1, 0}};
	Matrix dy{{0, 0, 1}, {0, 0, 0}, {-1, 0, 0}};
	Matrix dz{{0, -1, 0}, {1, 0, 0}, {0, 0, 0}};
	auto z3 = zeros(3, 3);
	return d_dcm_to_rpy(C_nav_to_platform, z3, z3, z3, eye(3) - skew(tilts), dx, dy, dz);
}

Matrix3 d_rpy_correct_dcm_with_tilt_wrt_tilt(const Vector3& tilts,
                                             const Matrix3& C_nav_to_platform) {
	Matrix dx{{0, 0, 0}, {0, 0, -1}, {0, 1, 0}};
	Matrix dy{{0, 0, 1}, {0, 0, 0}, {-1, 0, 0}};
	Matrix dz{{0, -1, 0}, {1, 0, 0}, {0, 0, 0}};


	auto m2    = pow(tilts(0), 2) + pow(tilts(1), 2) + pow(tilts(2), 2);
	auto m4    = pow(m2, 2);
	auto m     = sqrt(m2);
	Matrix3 s  = skew(tilts);
	auto sinm  = sin(m);
	auto cosm  = cos(m);
	Matrix3 s2 = xt::linalg::matrix_power(s, 2);
	Matrix3 B  = xt::fma((1 - cosm) / m2, s2, xt::fma(sinm / m, s, eye(3)));
	auto dmdx  = tilts(0) / m;
	auto dmdy  = tilts(1) / m;
	auto dmdz  = tilts(2) / m;

	Matrix3 dBdx = (cosm * dmdx * m - dmdx * sinm) / m2 * s + sinm / m * dx +
	               (sinm * dmdx * m2 - 2 * m * dmdx * (1 - cosm)) / m4 * s2 +
	               (1 - cosm) / m2 * (dot(s, dx) + dot(dx, s));
	Matrix3 dBdy = (cosm * dmdy * m - dmdy * sinm) / m2 * s + sinm / m * dy +
	               (sinm * dmdy * m2 - 2 * m * dmdy * (1 - cosm)) / m4 * s2 +
	               (1 - cosm) / m2 * (dot(s, dy) + dot(dy, s));
	Matrix3 dBdz = (cosm * dmdz * m - dmdz * sinm) / m2 * s + sinm / m * dz +
	               (sinm * dmdz * m2 - 2 * m * dmdz * (1 - cosm)) / m4 * s2 +
	               (1 - cosm) / m2 * (dot(s, dz) + dot(dz, s));

	Matrix3 z3 = zeros(3, 3);
	return d_dcm_to_rpy(C_nav_to_platform, z3, z3, z3, B, dBdx, dBdy, dBdz);
}

Matrix d_quat_prop_wrt_r(const Vector4& q, const Vector3& r) {
	auto m = xt::norm_l2(r)[0];

	auto m3 = pow(m, 3.0);
	auto s  = sin(m / 2.0);
	auto c  = cos(m / 2.0);

	Vector3 da = -r * s / (2 * m);

	auto common = (0.5 * c * m - s) / m3;
	auto dbdx   = r(0) * r(0) * common + s / m;
	auto dbdy   = r(0) * r(1) * common;
	auto dbdz   = r(0) * r(2) * common;

	auto dcdx = r(1) * r(0) * common;
	auto dcdy = r(1) * r(1) * common + s / m;
	auto dcdz = r(1) * r(2) * common;

	auto dddx = r(2) * r(0) * common;
	auto dddy = r(2) * r(1) * common;
	auto dddz = r(2) * r(2) * common + s / m;

	Matrix out = zeros(4, 3);

	xt::view(out, xt::all(), 0) = quat_mult(q, Vector4{da(0), dbdx, dcdx, dddx});
	xt::view(out, xt::all(), 1) = quat_mult(q, Vector4{da(1), dbdy, dcdy, dddy});
	xt::view(out, xt::all(), 2) = quat_mult(q, Vector4{da(2), dbdz, dcdz, dddz});

	return out;
}

Matrix d_quat_tilt_corr_wrt_tilt(const Vector4& q) {
	return Matrix{{-q(1), -q(2), -q(3)},
	              {q(0), q(3), -q(2)},
	              {-q(3), q(0), q(1)},
	              {q(2), -q(1), q(0)}} *
	       0.5;
}

Matrix d_llh_to_quat_en_wrt_llh(const Vector3& llh) {
	Matrix jac = zeros(4, 3);

	auto q      = llh_to_quat_en(llh);
	auto slat   = sin(llh(0));
	auto clat   = cos(llh(0));
	auto slon   = sin(llh(1));
	auto clon   = cos(llh(1));
	auto q0term = pow(4 * q(0), 2.0);

	jac(0, 0) = (-clat * (clon + 1)) / (8 * q(0));
	jac(1, 0) = (-slat * slon * 4 * q(0) - 4 * jac(0, 0) * clat * slon) / q0term;
	jac(2, 0) = ((slat * clon + slat) * 4 * q(0) - 4 * jac(0, 0) * (-clat * clon - clat)) / q0term;
	jac(3, 0) = (-clat * slon * 4 * q(0) - 4 * jac(0, 0) * (-slat * slon + slon)) / q0term;


	jac(0, 1) = (slon * (slat - 1)) / (8 * q(0));
	jac(1, 1) = (clat * clon * 4 * q(0) - 4 * jac(0, 1) * clat * slon) / q0term;
	jac(2, 1) = ((clat * slon) * 4 * q(0) - 4 * jac(0, 1) * (-clat * clon - clat)) / q0term;
	jac(3, 1) = ((-slat * clon + clon) * 4 * q(0) - 4 * jac(0, 1) * (-slat * slon + slon)) / q0term;

	return jac;
}

Matrix d_quat_to_rpy_wrt_q(const Vector4& q) {
	auto a = q(0);
	auto b = q(1);
	auto c = q(2);
	auto d = q(3);

	auto t1 = 2 * (b * b + c * c);
	auto t2 = 2 * (c * c + d * d);

	auto den1  = pow(2 * (a * b + c * d), 2.0) + pow(1 - t1, 2.0);
	auto inner = std::min(pow(2 * (a * c - b * d), 2.0), 0.999);
	auto den2  = sqrt(1 - inner);
	auto den3  = pow(2 * (a * d + b * c), 2.0) + pow(1 - t2, 2.0);

	auto drda = 2 * b * (1 - t1) / den1;
	auto drdb = (2 * a * (1 - t1) + 8 * b * (a * b + c * d)) / den1;
	auto drdc = (2 * d * (1 - t1) + 8 * c * (a * b + c * d)) / den1;
	auto drdd = 2 * c * (1 - t1) / den1;

	auto dpda = 2 * c / den2;
	auto dpdb = -2 * d / den2;
	auto dpdc = 2 * a / den2;
	auto dpdd = -2 * b / den2;

	auto dyda = 2 * d * (1 - t2) / den3;
	auto dydb = 2 * c * (1 - t2) / den3;
	auto dydc = (2 * b * (1 - t2) + 8 * c * (a * d + b * c)) / den3;
	auto dydd = (2 * a * (1 - t2) + 8 * d * (a * d + b * c)) / den3;

	return Matrix{{drda, drdb, drdc, drdd}, {dpda, dpdb, dpdc, dpdd}, {dyda, dydb, dydc, dydd}};
}


Matrix d_quat_norm_wrt_q(const Vector4& q) {
	auto q0 = q(0);
	auto q1 = q(1);
	auto q2 = q(2);
	auto q3 = q(3);

	auto qn  = xt::norm_l2(q)[0];
	auto qn2 = pow(qn, 2.0);
	auto qn3 = pow(qn, 3);

	auto dqada = (qn2 - q0 * q0) / qn3;
	auto dqadb = (-q0 * q1) / qn3;
	auto dqadc = (-q0 * q2) / qn3;
	auto dqadd = (-q0 * q3) / qn3;

	auto dqbda = (-q1 * q0) / qn3;
	auto dqbdb = (qn2 - q1 * q1) / qn3;
	auto dqbdc = (-q1 * q2) / qn3;
	auto dqbdd = (-q1 * q3) / qn3;

	auto dqcda = (-q2 * q0) / qn3;
	auto dqcdb = (-q2 * q1) / qn3;
	auto dqcdc = (qn2 - q2 * q2) / qn3;
	auto dqcdd = (-q2 * q3) / qn3;

	auto dqdda = (-q3 * q0) / qn3;
	auto dqddb = (-q3 * q1) / qn3;
	auto dqddc = (-q3 * q2) / qn3;
	auto dqddd = (qn2 - q3 * q3) / qn3;

	return Matrix{{dqada, dqadb, dqadc, dqadd},
	              {dqbda, dqbdb, dqbdc, dqbdd},
	              {dqcda, dqcdb, dqcdc, dqcdd},
	              {dqdda, dqddb, dqddc, dqddd}};
}

Matrix3 d_ortho_dcm_wrt_tilt(const Matrix3& C_nav_to_platform,
                             const Vector3& tilts,
                             const Matrix3& dtilt) {
	Matrix3 c   = dot(C_nav_to_platform, eye(3) + skew(tilts));
	Matrix3 dc  = dot(C_nav_to_platform, dtilt);
	Matrix3 cdc = dot(transpose(dc), c);

	return xt::fma(
	    -0.5, (dot(dc, dot(transpose(c), c)) + dot(c, cdc) + dot(c, transpose(cdc)) - dc), dc);
}

Matrix3 d_rpy_to_dcm_wrt_r(const Vector3& rpy) {
	auto r = rpy[0];
	auto p = rpy[1];
	auto y = rpy[2];

	double cr = cos(r);
	double sr = sin(r);
	double cp = cos(p);
	double sp = sin(p);
	double cy = cos(y);
	double sy = sin(y);

	return Matrix3{{0, 0, 0},
	               {cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp},
	               {-sr * sp * cy + cr * sy, -sr * sp * sy - cr * cy, -sr * cp}};
}

Matrix3 d_rpy_to_dcm_wrt_p(const Vector3& rpy) {
	auto r = rpy[0];
	auto p = rpy[1];
	auto y = rpy[2];

	double cr = cos(r);
	double sr = sin(r);
	double cp = cos(p);
	double sp = sin(p);
	double cy = cos(y);
	double sy = sin(y);

	return Matrix3{{-sp * cy, -sp * sy, -cp},
	               {sr * cp * cy, sr * cp * sy, -sr * sp},
	               {cr * cp * cy, cr * cp * sy, -cr * sp}};
}

Matrix3 d_rpy_to_dcm_wrt_y(const Vector3& rpy) {
	auto r = rpy[0];
	auto p = rpy[1];
	auto y = rpy[2];

	double cr = cos(r);
	double sr = sin(r);
	double cp = cos(p);
	double sp = sin(p);
	double cy = cos(y);
	double sy = sin(y);

	return Matrix3{{-cp * sy, cp * cy, 0},
	               {-sr * sp * sy - cr * cy, sr * sp * cy - cr * sy, 0},
	               {-cr * sp * sy + sr * cy, cr * sp * cy + sr * sy, 0}};
}

// Shorthand for
// z = eye(4);
// for k in 0..4
//     xt::view(z, xt::all(), k) = quat_mult(xt::view(z, xt::all(), k), q)
// return xt::view(z, xt::range(1, 3), xt::all())
// which occurs commonly in quaternion related derivatives
Matrix quat_mult_id_bottom(const Vector4& p) {
	auto p0 = p(0);
	auto p1 = p(1);
	auto p2 = p(2);
	auto p3 = p(3);
	return Matrix{{p1, p0, p3, -p2}, {p2, -p3, p0, p1}, {p3, p2, -p1, p0}};
}

Matrix d_sensor_to_platform_pos_wrt_q(const Vector4& q_s_to_n,
                                      const Vector3& l_ps_p,
                                      const Matrix3& C_p_to_s) {
	// TODO: C_p_to_s typically constant, and l_ps_p Vector4 could be computed once and passed
	// to an overload to make this cheaper. In other words, if sensor mounting is constant
	// then a = 2 * quat_mult(quat_mult(q_p_to_s, r_con), quat_conj(q_p_to_s)) could be calculated
	// then this function collapses to
	// quat_mult_bottom_id(quat_mult(a, quat_conj(q_s_to_n)))
	Vector4 q_p_to_s = dcm_to_quat(C_p_to_s);
	Vector4 q_p_to_n = quat_mult(q_s_to_n, q_p_to_s);
	Vector4 r_con    = xt::concatenate(xt::xtuple(zeros(1), -l_ps_p));
	Vector4 prod     = 2 * quat_mult(quat_mult(q_p_to_s, r_con), quat_conj(q_p_to_n));
	return quat_mult_id_bottom(prod);
}

Matrix d_platform_to_sensor_pos_wrt_q(const Vector4& q_p_to_n, const Vector3& l_ps_p) {
	// Sensor mount DCM falls out of the derivative
	Vector4 r = xt::concatenate(xt::xtuple(zeros(1), l_ps_p));
	auto prod = 2 * quat_mult(r, quat_conj(q_p_to_n));
	return quat_mult_id_bottom(prod);
}

}  // namespace navutils
}  // namespace navtk
