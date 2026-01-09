#pragma once

#include <string>
#include <utility>

#include <navtk/aspn.hpp>
#include <navtk/geospatial/sources/SpatialMapDataSource.hpp>

namespace navtk {
namespace geospatial {

/**
 * This class is the interface for all sources of elevation data.
 *
 * Derive from this class if you're writing a class that reads or calculates elevation data. Each
 * instance of this class should correspond to either a single path or no path (e.g. constant
 * undulation model).
 */
class ElevationSource : public SpatialMapDataSource {
public:
	/**
	 * Get the output vertical reference frame
	 *
	 * @return The current output frame set by the user.
	 */
	virtual AspnMeasurementAltitudeReference get_output_vertical_reference_frame() const;

	/**
	 * Set the output vertical reference frame. All values returned by `lookup_datum` will be with
	 * respect to this frame.
	 *
	 * @param new_ref the new output vertical reference frame
	 */
	virtual void set_output_vertical_reference_frame(AspnMeasurementAltitudeReference new_ref) = 0;

protected:
	/**
	 * Vertical reference frame of the input dataset.
	 */
	AspnMeasurementAltitudeReference input_reference;
	/**
	 * Vertical reference frame of all outputted elevations
	 */
	AspnMeasurementAltitudeReference output_reference;
};
}  // namespace geospatial
}  // namespace navtk
