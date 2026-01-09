#pragma once

#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>

namespace navtk {
namespace filtering {

/**
 * Container used to pair an aspn_xtensor::AspnBase measurement with a NavSolution instance that
 * corresponds to the approximate time and location that the measurement was taken. Typically used
 * to house a measurement that cannot be processed without additional PVA data.
 */
class PairedPva : public aspn_xtensor::AspnBase {

public:
	/**
	 * A measurement of state valid at a specific point in time.
	 */
	std::shared_ptr<aspn_xtensor::AspnBase> meas_data;

	/**
	 * Position velocity, attitude and time associated with the platform that generated `meas_data`.
	 */
	NavSolution ref_pva;

	/**
	 * Constructor.
	 * @param md The aspn_xtensor::AspnBase to store in #meas_data, specifically an instance of a
	 * class that provides some type of measurement, such as GaussianVectorData, which can be used
	 * by a MeasurementProcessor to update a filter.
	 * @param pva NavSolution valid at the time of \p md for platform to which the sensor that
	 * produced \p md was attached; initializes #ref_pva.
	 * @param message_type the ASPN message type assigned to the given template specialization of
	 * PairedPva. This will likely be specific to the running program. Defaults to
	 * ASPN_EXTENDED_BEGIN since PairedPva extends the set of defined ASPN messages. Note, if this
	 * default value is not overridden by a program using NavToolkit, then the type assigned to this
	 * template specialization of PairedPva may conflict with another type used by that program.
	 * Users should be careful to ensure that all ASPN message types used by their program have
	 * unique types if they are using aspn_xtensor::AspnBase::get_message_type to identify a message
	 * type.
	 */
	PairedPva(std::shared_ptr<aspn_xtensor::AspnBase> md,
	          NavSolution pva,
	          AspnMessageType message_type = ASPN_EXTENDED_BEGIN)
	    : aspn_xtensor::TypeHeader(message_type, 0, 0, 0, 0),
	      meas_data(md),
	      ref_pva(std::move(pva)) {}
};
}  // namespace filtering
}  // namespace navtk
