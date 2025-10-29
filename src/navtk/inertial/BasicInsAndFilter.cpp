#include <navtk/inertial/BasicInsAndFilter.hpp>

#include <memory>

#include <xtensor/views/xview.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/containers/ImuModel.hpp>
#include <navtk/filtering/containers/PairedPva.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/processors/PinsonPositionMeasurementProcessor.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/apply_error_states.hpp>
#include <navtk/inertial/inertial_functions.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

namespace navtk {
namespace inertial {

BasicInsAndFilter::BasicInsAndFilter(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva,
                                     const filtering::ImuModel& model_in,
                                     std::shared_ptr<aspn_xtensor::MeasurementImu> initial_imu,
                                     const ImuErrors& imu_errs,
                                     const double expected_dt,
                                     const MechanizationOptions& mech_options)
    : model(model_in),
      ins(pva, initial_imu, expected_dt, imu_errs, mech_options),
      engine(pva.get_time_of_validity()) {

	auto pins = std::make_shared<filtering::Pinson15NedBlock>(p_tag, model);
	engine.add_state_block(pins);

	Matrix cov                                      = zeros(15, 15);
	xt::view(cov, xt::range(0, 9), xt::range(0, 9)) = pva.get_covariance();
	xt::view(cov, xt::range(9, 12), xt::range(9, 12)) =
	    xt::diag(xt::pow((Vector)model.accel_bias_initial_sigma, 2));
	xt::view(cov, xt::range(12, 15), xt::range(12, 15)) =
	    xt::diag(xt::pow((Vector)model.gyro_bias_initial_sigma, 2));

	engine.set_state_block_covariance(p_tag, cov);

	auto try_f_and_r = ins.calc_force_and_rate(pva.get_time_of_validity());
	auto init_force =
	    (try_f_and_r == nullptr) ? Vector3{0, 0, -9.81} : Vector3{try_f_and_r->get_meas_accel()};
	auto init_aux = navtk::utils::to_inertial_aux(utils::to_navsolution(pva), init_force);
	engine.give_state_block_aux_data(p_tag, init_aux);
	auto mount = aspn_xtensor::TypeMounting(zeros(3), zeros(3), Vector{1, 0, 0, 0}, zeros(3, 3));
	auto proc  = std::make_shared<filtering::PinsonPositionMeasurementProcessor>(
        g_tag, std::vector<std::string>{p_tag}, mount, mount);
	engine.add_measurement_processor(proc);
}

void BasicInsAndFilter::mechanize(const aspn_xtensor::MeasurementImu& imu) {
	ins.mechanize(imu);
	auto sol = ins.calc_pva(engine.get_time());
	auto fr  = ins.calc_force_and_rate(engine.get_time());
	if (sol != nullptr && fr != nullptr) {
		auto aux = navtk::utils::to_inertial_aux(utils::to_navsolution(*sol), fr->get_meas_accel());
		engine.give_state_block_aux_data(p_tag, aux);
	}
}

void BasicInsAndFilter::update(const aspn_xtensor::MeasurementPosition& gp3d) {
	auto sol = ins.calc_pva(gp3d.get_time_of_validity());

	if (sol != nullptr) {
		auto meas = std::make_shared<filtering::PairedPva>(
		    std::make_shared<aspn_xtensor::MeasurementPosition>(gp3d), utils::to_navsolution(*sol));
		engine.update(g_tag, meas);
		auto curr_errs = ins.get_imu_errors(engine.get_time());
		auto x         = engine.get_state_block_estimate(p_tag);
		auto corrected = filtering::apply_error_states<filtering::Pinson15NedBlock>(*sol, x);
		if (curr_errs != nullptr) {
			curr_errs->accel_biases -= xt::view(x, xt::range(9, 12));
			curr_errs->gyro_biases -= xt::view(x, xt::range(12, 15));
		}
		auto accepted = ins.reset(
		    std::make_shared<aspn_xtensor::MeasurementPositionVelocityAttitude>(corrected),
		    curr_errs);
		if (accepted) {
			engine.set_state_block_estimate(p_tag, zeros(15));
		}
	}
}

std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> BasicInsAndFilter::calc_pva(
    const aspn_xtensor::TypeTimestamp& t) const {
	auto ins_sol = ins.calc_pva(t);
	if (ins_sol != nullptr) {
		ins_sol->set_covariance(xt::view(get_pinson15_cov(), xt::range(0, 9), xt::range(0, 9)));
	}
	return ins_sol;
}

Matrix BasicInsAndFilter::get_pinson15_cov() const {
	return engine.get_state_block_covariance(p_tag);
}

std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> BasicInsAndFilter::calc_pva()
    const {
	auto ins_sol = ins.calc_pva();
	ins_sol->set_covariance(
	    xt::view(engine.get_state_block_covariance(p_tag), xt::range(0, 9), xt::range(0, 9)));
	return ins_sol;
}

ImuErrors BasicInsAndFilter::calc_imu_errors() const {
	auto t       = ins.time_span().second;
	auto err_ptr = ins.get_imu_errors(t);
	return (err_ptr != nullptr) ? *err_ptr : ImuErrors{};
}

}  // namespace inertial
}  // namespace navtk
