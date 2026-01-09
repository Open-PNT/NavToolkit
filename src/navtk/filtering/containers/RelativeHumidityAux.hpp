#pragma once

#include <navtk/aspn.hpp>

namespace navtk {
namespace filtering {
/**
 * Relative humidity aux data container.
 */
class RelativeHumidityAux : public aspn_xtensor::AspnBase {
public:
	/**
	 * The ratio of the current absolute humidity to the highest possible absolute humidity.
	 * Must be between 0 and 1.
	 */
	double tropo_rel_humidity;

	/**
	 * @param t The value to store in #tropo_rel_humidity.
	 * @param message_type the ASPN message type assigned to RelativeHumidityAux. This will likely
	 * be specific to the running program. Defaults to ASPN_EXTENDED_BEGIN since RelativeHumidityAux
	 * extends the set of defined ASPN messages. Note, if this default value is not overridden by a
	 * program using NavToolkit, then the type assigned to RelativeHumidityAux may conflict with
	 * another type used by that program. Users should be careful to ensure that all ASPN message
	 * types used by their program have unique types if they are using AspnBase::get_message_type to
	 * identify a message type.
	 */
	RelativeHumidityAux(double t, AspnMessageType message_type = ASPN_EXTENDED_BEGIN)
	    : aspn_xtensor::TypeHeader(message_type, 0, 0, 0, 0), tropo_rel_humidity(t) {}
};
}  // namespace filtering
}  // namespace navtk
