#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>

#include <navtk/errors.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/utils/conversions.hpp>

using navtk::navutils::delta_lat_to_north;
using navtk::navutils::delta_lon_to_east;
using navtk::navutils::skew;

namespace navtk {
namespace filtering {

Pinson15NedBlock::Pinson15NedBlock(const std::string& label,
                                   ImuModel imu_model,
                                   LinearizationPointFunction lin_function,
                                   DiscretizationStrategy discretization_strategy,
                                   not_null<std::shared_ptr<GravityModel>> gravity_model)
    : StateBlock(15, label, std::move(discretization_strategy)),
      imu_model(std::move(imu_model)),
      lin_function(std::move(lin_function)),
      gravity_model(std::move(gravity_model)) {
	all_eq_gyro_randwalk =
	    (imu_model.gyro_random_walk_sigma[0] == imu_model.gyro_random_walk_sigma[1] &&
	     imu_model.gyro_random_walk_sigma[0] == imu_model.gyro_random_walk_sigma[2]);
	all_eq_accel_randwalk =
	    (imu_model.accel_random_walk_sigma[0] == imu_model.accel_random_walk_sigma[1] &&
	     imu_model.accel_random_walk_sigma[0] == imu_model.accel_random_walk_sigma[2]);
}

Pinson15NedBlock::Pinson15NedBlock(const Pinson15NedBlock& block)
    : StateBlock(block),
      imu_model(block.imu_model),
      all_eq_accel_randwalk(block.all_eq_accel_randwalk),
      all_eq_gyro_randwalk(block.all_eq_gyro_randwalk),
      q15_matrix(block.q15_matrix),
      lin_function(block.lin_function),
      gravity_model(block.gravity_model) {
	if (block.new_pva_aux != nullptr) {
		new_pva_aux = std::make_shared<Pva>(*block.new_pva_aux);
	}
	if (block.new_force_and_rate_aux != nullptr) {
		new_force_and_rate_aux = std::make_shared<Imu>(*block.new_force_and_rate_aux);
	}
	if (block.old_pva_aux != nullptr) {
		old_pva_aux = std::make_shared<Pva>(*block.old_pva_aux);
	}
	if (block.old_force_and_rate_aux != nullptr) {
		old_force_and_rate_aux = std::make_shared<Imu>(*block.old_force_and_rate_aux);
	}
}

void Pinson15NedBlock::receive_aux_data(const AspnBaseVector& aux_data) {
	for (auto aux : aux_data) {
		if (auto pva_aux = std::dynamic_pointer_cast<Pva>(aux)) {
			old_pva_aux = new_pva_aux;
			new_pva_aux = pva_aux;
			if (old_pva_aux == nullptr || !all_eq_gyro_randwalk || !all_eq_accel_randwalk) {
				generate_q_pinson15(navtk::navutils::quat_to_dcm(new_pva_aux->get_quaternion()));
			}
		} else if (auto force_and_rate_aux = std::dynamic_pointer_cast<Imu>(aux)) {
			old_force_and_rate_aux = new_force_and_rate_aux;
			new_force_and_rate_aux = force_and_rate_aux;
		} else if (auto imu_model_aux = std::dynamic_pointer_cast<ImuModel>(aux)) {
			imu_model = *imu_model_aux;
			all_eq_gyro_randwalk =
			    (imu_model.gyro_random_walk_sigma[0] == imu_model.gyro_random_walk_sigma[1] &&
			     imu_model.gyro_random_walk_sigma[0] == imu_model.gyro_random_walk_sigma[2]);
			all_eq_accel_randwalk =
			    (imu_model.accel_random_walk_sigma[0] == imu_model.accel_random_walk_sigma[1] &&
			     imu_model.accel_random_walk_sigma[0] == imu_model.accel_random_walk_sigma[2]);
			if (new_pva_aux != nullptr) {
				generate_q_pinson15(navtk::navutils::quat_to_dcm(new_pva_aux->get_quaternion()));
			}
		} else {
			StateBlock::receive_aux_data(aux_data);
		}
	}
}

not_null<std::shared_ptr<StateBlock<>>> Pinson15NedBlock::clone() {
	return std::make_shared<Pinson15NedBlock>(*this);
}

DynamicsModel Pinson15NedBlock::generate_dynamics(GenXhatPFunction,
                                                  aspn_xtensor::TypeTimestamp time_from,
                                                  aspn_xtensor::TypeTimestamp time_to) {
	if (lin_function != nullptr) {
		receive_aux_data(lin_function(time_from, time_to));
	}
	if (new_pva_aux == nullptr || new_force_and_rate_aux == nullptr)
		log_or_throw<std::runtime_error>(
		    "Pinson15 Cannot propagate unless it first receives aux_data with a Pose object");

	double dt        = (time_to.get_elapsed_nsec() - time_from.get_elapsed_nsec()) * 1e-9;
	auto F           = generate_f_pinson15(*new_pva_aux, *new_force_and_rate_aux);
	auto discretized = discretization_strategy(F, eye(15), q15_matrix, dt);
	auto Phi         = discretized.first;
	auto Qd          = discretized.second;

	// Scale N/E to account for change in frame over time
	Phi = scale_phi(Phi);

	auto g = [Phi = Phi](Vector x) { return dot(Phi, x); };
	return DynamicsModel(g, Phi, Qd);
}

Matrix Pinson15NedBlock::scale_phi(Matrix& phi) {
	if (old_pva_aux != nullptr) {
		auto pos         = navtk::utils::extract_pos(*old_pva_aux);
		auto lat_factor0 = navutils::delta_lat_to_north(1, pos[0], pos[2]);
		auto lon_factor0 = navutils::delta_lon_to_east(1, pos[0], pos[2]);

		auto new_pos     = navtk::utils::extract_pos(*new_pva_aux);
		auto lat_factor1 = navutils::delta_lat_to_north(1, new_pos[0], new_pos[2]);
		auto lon_factor1 = navutils::delta_lon_to_east(1, new_pos[0], new_pos[2]);

		double lat0Tolat1 = lat_factor1 / lat_factor0;
		double lon0Tolon1 = lon_factor1 / lon_factor0;
		xt::view(phi, xt::all(), 0) *= lat0Tolat1;
		xt::view(phi, xt::all(), 1) *= lon0Tolon1;
	}
	return phi;
}

Matrix Pinson15NedBlock::generate_q_pinson15(const Matrix& C_sensor_to_nav) {
	auto block = xt::range(3, 6);
	if (all_eq_accel_randwalk) {
		xt::view(q15_matrix, block, block) = xt::diag(pow(imu_model.accel_random_walk_sigma, 2));
	} else {
		for (size_t idx = 3; idx < 6; idx++) {
			xt::view(q15_matrix, idx, block) = xt::view(C_sensor_to_nav, (idx - 3), xt::all()) *
			                                   pow(imu_model.accel_random_walk_sigma, 2);
		}
		xt::view(q15_matrix, block, block) =
		    dot(xt::view(q15_matrix, block, block), xt::transpose(C_sensor_to_nav));
	}

	block = xt::range(6, 9);
	if (all_eq_gyro_randwalk) {
		xt::view(q15_matrix, block, block) = xt::diag(pow(imu_model.gyro_random_walk_sigma, 2));
	} else {
		for (size_t idx = 6; idx < 9; idx++) {
			xt::view(q15_matrix, idx, block) = xt::view(C_sensor_to_nav, (idx - 6), xt::all()) *
			                                   pow(imu_model.gyro_random_walk_sigma, 2);
		}
		xt::view(q15_matrix, block, block) =
		    dot(xt::view(q15_matrix, block, block), xt::transpose(C_sensor_to_nav));
	}

	xt::view(q15_matrix, xt::range(9, 12), xt::range(9, 12)) =
	    xt::diag(pow(imu_model.accel_bias_sigma, 2) * (2 / imu_model.accel_bias_tau));
	xt::view(q15_matrix, xt::range(12, 15), xt::range(12, 15)) =
	    xt::diag(pow(imu_model.gyro_bias_sigma, 2) * (2 / imu_model.gyro_bias_tau));
	return q15_matrix;
}

Matrix Pinson15NedBlock::generate_f_pinson15(const Pva& pva_aux, const Imu& force_and_rate_aux) {
	Vector3 pos            = navtk::utils::extract_pos(pva_aux);
	Vector3 vel            = navtk::utils::extract_vel(pva_aux);
	Vector3 force          = force_and_rate_aux.get_meas_accel();
	Matrix C_sensor_to_nav = navtk::navutils::quat_to_dcm(pva_aux.get_quaternion());

	// Update the earth model
	auto earth   = EarthModel(pos, vel, *gravity_model);
	double omega = EarthModel::OMEGA_E;
	double sinl  = earth.sin_l;
	double tanl  = earth.tan_l;
	double vn    = vel[0];
	double ve    = vel[1];
	double vd    = vel[2];
	double re    = earth.r_e;
	double rn    = earth.r_n;
	double cosl  = earth.cos_l;

	Matrix scalem2r =
	    Matrix{{1 / earth.lat_factor, 0, 0}, {0, 1 / earth.lon_factor, 0}, {0, 0, -1}};
	Matrix scaler2m = Matrix{{earth.lat_factor, 0, 0}, {0, earth.lon_factor, 0}, {0, 0, -1}};

	// block1: deltatilt = block1 * dtilt
	Matrix block1 = Matrix{{0, -(omega * sinl + ve / re * tanl), vn / rn},
	                       {(omega * sinl + ve / re * tanl), 0, omega * cosl + ve / re},
	                       {-vn / rn, -omega * cosl - ve / re, 0}};

	// block2: deltatilt = block2 * dvel
	Matrix block2 = Matrix{{0, 1 / re, 0}, {-1 / rn, 0, 0}, {0, -tanl / re, 0}};

	// block3: deltatilt = block3 * dpos
	Matrix block3 =
	    dot(Matrix{{-omega * sinl, 0, -ve / pow(re, 2)},
	               {0, 0, vn / pow(rn, 2)},
	               {-omega * cosl - ve / (re * cosl * cosl), 0, ve * tanl / pow(re, 2)}},
	        scalem2r);

	// block4: deltavel = block4 * dtilt
	Matrix block4 = skew(force);

	// block5: deltavel = block5*dvel
	Matrix block5 = Matrix{
	    {vd / rn, -2 * (omega * sinl + ve / re * tanl), vn / rn},
	    {2 * omega * sinl + ve / re * tanl, 1 / re * (vn * tanl + vd), 2 * omega * cosl + ve / re},
	    {-2 * vn / rn, -2 * (omega * cosl + ve / re), 0}};

	// Try adding gravity effect based on lat/north error. Derived from the 'Schwartz'
	// gravity model
	double a1 = 9.7803267715;
	double a2 = 0.0052790414;
	double a3 = 0.0000232718;
	double a4 = -3.0876910891e-6;
	double a5 = 4.3977311e-9;
	double a6 = 7.211e-13;

	auto dgdlat = 2 * a1 * a2 * cos(2 * pos[0]) +
	              a1 * a3 * (12 * (1 - cos(4 * pos[0])) / 8 - pow(sin(pos[0]), 4)) +
	              2 * a5 * (cos(2 * pos[0]) - cosl * sinl) * pos[2];
	auto dgdalt = (a4 + a5 * pow(sinl, 2)) + a6 * 2 * pos[2];

	// block6: deltavel = block6 * dpos
	Matrix block6 =
	    dot(Matrix{{-ve * (2 * omega * cosl + ve / (re * cosl * cosl)),
	                0,
	                ve * ve * tanl / pow(re, 2) - vn * vd / pow(rn, 2)},
	               {(2 * omega * (vn * cosl - vd * sinl) + vn * ve / (re * cosl * cosl)),
	                0,
	                -ve / pow(re, 2) * (vn * tanl + vd)},
	               {2 * omega * ve * sinl + dgdlat,
	                0,
	                vn * vn / pow(rn, 2) + ve * ve / pow(re, 2) + dgdalt}},
	        scalem2r);

	// block7: deltapos = block7 * dtilt (zeros/not applicable)

	// block8: deltapos = block8 * dvel
	// T+W matrix has non-unity terms that are canceled out by conversion of the LLA position errors
	// used in the book to the NED position errors used in this model. See model derivation in
	// function documentation for more detail.
	Matrix block8 = eye(3);

	// block9: deltapos = block9 * dpos

	// Terms in T+W are by-product of relationship between NED vel and LLA position errors which are
	// are not applicable to NED positions; see model derivation.
	Matrix F = zeros(15, 15);

	// Pinson 9
	xt::view(F, xt::range(0, 3), xt::range(3, 6)) = block8;
	xt::view(F, xt::range(3, 6), xt::range(0, 3)) = block6;
	xt::view(F, xt::range(3, 6), xt::range(3, 6)) = block5;
	xt::view(F, xt::range(3, 6), xt::range(6, 9)) = block4;
	xt::view(F, xt::range(6, 9), xt::range(0, 3)) = block3;
	xt::view(F, xt::range(6, 9), xt::range(3, 6)) = block2;
	xt::view(F, xt::range(6, 9), xt::range(6, 9)) = block1;

	// Pinson 15
	xt::view(F, xt::range(3, 6), xt::range(9, 12)) = C_sensor_to_nav;  // Add in accel bias to vdot
	xt::view(F, xt::range(6, 9), xt::range(12, 15)) =
	    -C_sensor_to_nav;  // Add in gyro bias to tiltdot

	// Accelerometer FOGM bias and Gyro FOGM bias
	for (Size ii = 0; ii < 3; ++ii) {
		F(ii + 9, ii + 9)   = -1.0 / imu_model.accel_bias_tau(ii);
		F(ii + 12, ii + 12) = -1.0 / imu_model.gyro_bias_tau(ii);
	}

	return F;
}

ImuModel Pinson15NedBlock::get_imu_model() const { return imu_model; }

Matrix Pinson15NedBlock::get_q15_matrix() const { return q15_matrix; }

Pinson15NedBlock::LinearizationPointFunction Pinson15NedBlock::get_lin_function() const {
	return lin_function;
}
DiscretizationStrategy Pinson15NedBlock::get_discretization_strategy() const {
	return discretization_strategy;
}
not_null<std::shared_ptr<const GravityModel>> Pinson15NedBlock::get_gravity_model() const {
	return std::const_pointer_cast<const GravityModel>(gravity_model.get());
}
std::shared_ptr<Pva> Pinson15NedBlock::get_pva_aux() { return new_pva_aux; }

std::shared_ptr<Imu> Pinson15NedBlock::get_force_and_rate_aux() { return new_force_and_rate_aux; }

}  // namespace filtering
}  // namespace navtk
