#include <navtk/errors.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/stateblocks/DeadReckoningStateBlock.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

using xt::range;

DeadReckoningStateBlock::DeadReckoningStateBlock(const std::string& label,
                                                 double latitude_sigma,
                                                 double longitude_sigma,
                                                 Vector time_constants,
                                                 Vector process_sigmas,
                                                 double init_ref_altitude,
                                                 DiscretizationStrategy discretization_strategy)
    : FogmBlock(label, time_constants, process_sigmas, 2, discretization_strategy),
      latitude_sigma(latitude_sigma),
      longitude_sigma(longitude_sigma),
      ref_altitude(init_ref_altitude) {}

not_null<std::shared_ptr<StateBlock<>>> DeadReckoningStateBlock::clone() {
	return std::make_shared<DeadReckoningStateBlock>(*this);
}

void DeadReckoningStateBlock::receive_aux_data(const AspnBaseVector& aux_data) {
	auto altitude_aux_data =
	    std::dynamic_pointer_cast<aspn_xtensor::MeasurementAltitude>(aux_data[0]);

	// Warn and return if aux data is the wrong type.
	if (altitude_aux_data == nullptr) {
		StateBlock::receive_aux_data(aux_data);
		return;
	}

	ref_altitude = altitude_aux_data->get_altitude();

	return;
}

DynamicsModel DeadReckoningStateBlock::generate_dynamics(GenXhatPFunction gen_x_and_p_func,
                                                         aspn_xtensor::TypeTimestamp time_from,
                                                         aspn_xtensor::TypeTimestamp time_to) {
	Matrix F      = zeros(total_states, total_states);
	Matrix Q      = zeros(total_states, total_states);
	Matrix f_fogm = zeros(num_states, num_states);
	Matrix q_fogm = zeros(num_states, num_states);
	FogmBlock::populate_f_and_q(f_fogm, q_fogm);
	double dt = (time_to.get_elapsed_nsec() - time_from.get_elapsed_nsec()) * 1e-9;

	auto estimate_and_covariance = gen_x_and_p_func({get_label()});
	Vector xhat;
	if (estimate_and_covariance != nullptr)
		xhat = estimate_and_covariance->estimate;
	else {
		log_or_throw(
		    "DeadReckoningStateBlock::generate_dynamics received nullptr after calling "
		    "GenXhatPFunction with the label {}",
		    get_label());
		xhat = {0};
	}

	// Collect the FOGM block dynamics model
	F(0, 2) = navtk::navutils::north_to_delta_lat(1.0, xhat(0), ref_altitude);
	F(1, 3) = navtk::navutils::east_to_delta_lon(1.0, xhat(0), ref_altitude);
	view(F, range(2, 4), range(2, 4)) = f_fogm;
	Q(0, 0)                           = latitude_sigma * latitude_sigma;
	Q(1, 1)                           = longitude_sigma * longitude_sigma;
	view(Q, range(2, 4), range(2, 4)) = q_fogm;
	auto discretized                  = discretization_strategy(F, eye(num_rows(Q)), Q, dt);
	// Qd and Phi
	return DynamicsModel(discretized.first, discretized.second);
}

}  // namespace filtering
}  // namespace navtk
