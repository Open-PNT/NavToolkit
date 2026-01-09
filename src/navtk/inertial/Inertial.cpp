#include <navtk/inertial/Inertial.hpp>

#include <navtk/inertial/MechanizationStandard.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/inertial/WanderPosVelAtt.hpp>
#include <navtk/inertial/mechanization_standard.hpp>
#include <navtk/inertial/mechanization_wander.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace inertial {

Inertial::Inertial(const not_null<std::shared_ptr<InertialPosVelAtt>> start_pva,
                   const MechanizationOptions& mech_options,
                   MechanizationFunction mech_fun)
    : integrated_alt_error(0.0),
      pva(start_pva),
      pva_old1(start_pva),
      gyro_bias({0.0, 0.0, 0.0}),
      gyro_scale_factor({0.0, 0.0, 0.0}),
      accel_bias({0.0, 0.0, 0.0}),
      accel_scale_factor({0.0, 0.0, 0.0}),
      _mech_fun(std::move(mech_fun)),
      _mech_options(mech_options) {}

Inertial::Inertial(not_null<std::shared_ptr<Mechanization>> mech_class,
                   const not_null<std::shared_ptr<InertialPosVelAtt>> start_pva,
                   const MechanizationOptions& mech_options)
    : Inertial(start_pva,
               mech_options,
               [mech_class = mech_class](
                   const Vector3& dv_s,
                   const Vector3& dth_s,
                   double dt,
                   const not_null<std::shared_ptr<InertialPosVelAtt>> pva,
                   const not_null<std::shared_ptr<InertialPosVelAtt>> old_pva,
                   const MechanizationOptions& mech_options,
                   AidingAltData* aiding_alt_data) -> not_null<std::shared_ptr<InertialPosVelAtt>> {
	               return mech_class->mechanize(
	                   dv_s, dth_s, dt, pva, old_pva, mech_options, aiding_alt_data);
               }) {}

Inertial::Inertial(const Inertial& ins)
    : integrated_alt_error(ins.integrated_alt_error),
      pva(ins.pva->clone()),
      pva_old1(ins.pva_old1->clone()),
      gyro_bias(ins.gyro_bias),
      gyro_scale_factor(ins.gyro_scale_factor),
      accel_bias(ins.accel_bias),
      accel_scale_factor(ins.accel_scale_factor),
      _mech_fun(ins._mech_fun),
      _mech_options(ins._mech_options) {}

Inertial& Inertial::operator=(const Inertial& ins) {
	if (this != &ins) {
		integrated_alt_error = ins.integrated_alt_error;
		pva                  = ins.pva->clone();
		pva_old1             = ins.pva_old1->clone();
		gyro_bias            = ins.gyro_bias;
		gyro_scale_factor    = ins.gyro_scale_factor;
		accel_bias           = ins.accel_bias;
		accel_scale_factor   = ins.accel_scale_factor;
		_mech_fun            = ins._mech_fun;
		_mech_options        = ins._mech_options;
	}
	return *this;
}

void Inertial::reset(const not_null<std::shared_ptr<InertialPosVelAtt>> new_pva,
                     const std::shared_ptr<InertialPosVelAtt> old_pva) {
	pva = new_pva;
	if (old_pva == nullptr) {
		pva_old1 = new_pva->clone();
	} else {
		pva_old1 = old_pva;
	}
}

void Inertial::reset(StandardPosVelAtt new_pva) {
	reset(std::make_shared<StandardPosVelAtt>(std::move(new_pva)));
}

void Inertial::reset(WanderPosVelAtt new_pva) {
	reset(std::make_shared<WanderPosVelAtt>(std::move(new_pva)));
}

not_null<std::shared_ptr<InertialPosVelAtt>> Inertial::get_solution() const { return pva; }

Vector3 Inertial::get_gyro_biases() const { return gyro_bias; }

void Inertial::set_gyro_biases(Vector3 gyro_biases) { gyro_bias = std::move(gyro_biases); }

Vector3 Inertial::get_gyro_scale_factors() const { return gyro_scale_factor; }

void Inertial::set_gyro_scale_factors(Vector3 gyro_scale_factors) {
	gyro_scale_factor = std::move(gyro_scale_factors);
}

Vector3 Inertial::get_accel_biases() const { return accel_bias; }

void Inertial::set_accel_biases(Vector3 accel_biases) { accel_bias = std::move(accel_biases); }

Vector3 Inertial::get_accel_scale_factors() const { return accel_scale_factor; }

void Inertial::set_accel_scale_factors(Vector3 accel_scale_factors) {
	accel_scale_factor = std::move(accel_scale_factors);
}

void Inertial::set_imu_errors(const ImuErrors& errors) {
	set_accel_biases(errors.accel_biases);
	set_accel_scale_factors(errors.accel_scale_factors);
	set_gyro_biases(errors.gyro_biases);
	set_gyro_scale_factors(errors.gyro_scale_factors);
}

void Inertial::mechanize(const aspn_xtensor::TypeTimestamp& time,
                         const Vector3& accel_meas,
                         const Vector3& gyro_meas,
                         AidingAltData* aiding_alt_data) {

	auto dt = (time.get_elapsed_nsec() - pva->time_validity.get_elapsed_nsec()) * 1e-9;
	dv_cpy  = accel_meas;
	dth_cpy = gyro_meas;
	apply_error_model(dt);

	if (aiding_alt_data != nullptr) {
		integrated_alt_error = aiding_alt_data->integrated_alt_error;
	}

	auto new_pva = _mech_fun(dv_cpy, dth_cpy, dt, pva, pva_old1, _mech_options, aiding_alt_data);
	pva_old1     = pva;
	pva          = new_pva;
}

void Inertial::apply_error_model(double dt) {
	dv_cpy[0]  = (accel_bias[0] * -dt + dv_cpy[0]) / (1.0 + accel_scale_factor[0]);
	dv_cpy[1]  = (accel_bias[1] * -dt + dv_cpy[1]) / (1.0 + accel_scale_factor[1]);
	dv_cpy[2]  = (accel_bias[2] * -dt + dv_cpy[2]) / (1.0 + accel_scale_factor[2]);
	dth_cpy[0] = (gyro_bias[0] * -dt + dth_cpy[0]) / (1.0 + gyro_scale_factor[0]);
	dth_cpy[1] = (gyro_bias[1] * -dt + dth_cpy[1]) / (1.0 + gyro_scale_factor[1]);
	dth_cpy[2] = (gyro_bias[2] * -dt + dth_cpy[2]) / (1.0 + gyro_scale_factor[2]);
}
}  // namespace inertial
}  // namespace navtk
