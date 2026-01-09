#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * Timestamped version of filtering::EstimateWithCovariance that works as an extended ASPN message.
 */
struct GaussianVectorData : aspn_xtensor::AspnBase, EstimateWithCovariance {
	/**
	 * Maps the parameters directly to the properties of EstimateWithCovariance using `std::move`.
	 *
	 * @param time_of_validity The time at which the data is valid.
	 * @param mean An Nx1 Vector of realizations of Gaussian distributions.
	 * @param covariance An NxN covariance matrix describing the variance and correlation between
	 * the random distributions from which the parameter mean was sampled.
	 * @param message_type the ASPN message type assigned to GaussianVectorData. This will likely be
	 * specific to the running program. Defaults to ASPN_EXTENDED_BEGIN since GaussianVectorData
	 * extends the set of defined ASPN messages. Note, if this default value is not overridden by a
	 * program using NavToolkit, then the type assigned to GaussianVectorData may conflict with
	 * another type used by that program. Users should be careful to ensure that all ASPN message
	 * types used by their program have unique types if they are using AspnBase::get_message_type to
	 * identify a message type.
	 */
	GaussianVectorData(aspn_xtensor::TypeTimestamp time_of_validity,
	                   Vector mean,
	                   Matrix covariance,
	                   AspnMessageType message_type = ASPN_EXTENDED_BEGIN)
	    : aspn_xtensor::TypeHeader(message_type, 0, 0, 0, 0),
	      EstimateWithCovariance(std::move(mean), std::move(covariance)),
	      time_of_validity(time_of_validity) {}

	/**
	 * @return Time at which the data is considered to be valid.
	 */
	aspn_xtensor::TypeTimestamp get_time_of_validity() { return time_of_validity; };

private:
	aspn_xtensor::TypeTimestamp time_of_validity;
};

}  // namespace filtering
}  // namespace navtk
