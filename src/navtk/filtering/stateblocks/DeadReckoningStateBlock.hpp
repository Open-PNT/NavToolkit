#pragma once

#include <memory>
#include <string>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/stateblocks/FogmBlock.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/filtering/stateblocks/discretization_strategy.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A state block that may be used for a horizontal dead reckoning system.
 * The following states are represented in the state block:
 *
 *    1 - Latitude, random walk model [rad]
 *
 *    2 - Longitude, random walk model [rad]
 *
 *    3 - Velocity North, first-order gaussian-markov model [m/s]
 *
 *    4 - Velocity East, first-order gaussian-markov model [m/s]
 */
class DeadReckoningStateBlock : public FogmBlock {
public:
	virtual ~DeadReckoningStateBlock() = default;
	/**
	 * Constructor.
	 * @param label A label for the dead reckoning state block.
	 * @param latitude_sigma system noise standard deviation for latitude [rad].
	 * @param longitude_sigma system noise standard deviation for longitude [rad].
	 * @param time_constants The time constants [s] for the FOGM model of the
	 * velocity north and velocity east state blocks. The order of time
	 * constants should be velocity north first, then velocity east.
	 * @param process_sigmas The 1-sigma value at steady state [m/s] for the FOGM
	 * model of the velocity north and velocity east state blocks. The order of the process
	 * sigmas should be velocity north first, then velocity east.
	 * @param init_ref_altitude The initial reference altitude [m], required to
	 * calculate the change in latitude and longitude given delta velocity north
	 * and east.
	 * @param discretization_strategy Determines how the matrices `F` and `Q` will
	 * be linearized to produce `Phi` and `Qd` as related to the velocity north and east
	 * state blocks that use the first order gaussian-markov model. Options include a
	 * first-order, second-order, or full discretization for the velocity FOGM state blocks.
	 */
	DeadReckoningStateBlock(const std::string& label,
	                        double latitude_sigma,
	                        double longitude_sigma,
	                        Vector time_constants,
	                        Vector process_sigmas,
	                        double init_ref_altitude,
	                        DiscretizationStrategy discretization_strategy);

	/**
	 * Create a copy of the StateBlock with the same properties.
	 *
	 * @return A shared pointer to a copy of the StateBlock.
	 */
	not_null<std::shared_ptr<StateBlock<>>> clone() override;

	/**
	 * Used to process updates for reference altitude.
	 * @param aux_data An AspnBaseVector containing a aspn_xtensor::MeasurementAltitude message to
	 * update #ref_altitude.
	 */
	void receive_aux_data(const AspnBaseVector& aux_data) override;

	/**
	 * Generates the dynamics model for the filter.
	 *
	 * @param time_from The current filter time.
	 * @param time_to The target propagation time.
	 *
	 * @return The generated dynamics.
	 */
	DynamicsModel generate_dynamics(GenXhatPFunction,
	                                aspn_xtensor::TypeTimestamp time_from,
	                                aspn_xtensor::TypeTimestamp time_to) override;

protected:
	/**
	 * Latitude system standard deviation.
	 */
	double latitude_sigma;
	/**
	 * Longitude system standard deviation.
	 */
	double longitude_sigma;
	/**
	 *  Reference altitude, used in calculations for velocity to change in latitude and longitude.
	 */
	double ref_altitude;
	/**
	 *  Indicates whether SB should accept altitude aux data updates for the reference altitude.
	 */
	bool use_aux_altitude = false;
	/**
	 * The total number of internal states.
	 */
	size_t total_states = 4;
};


}  // namespace filtering
}  // namespace navtk
