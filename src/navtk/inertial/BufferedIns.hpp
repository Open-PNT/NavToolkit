#pragma once

#include <memory>

#include <navtk/aspn.hpp>
#include <navtk/inertial/BufferedPva.hpp>
#include <navtk/not_null.hpp>

namespace navtk {
namespace inertial {

/**
 * Class that maintains a time history of and provides access to calculated PVAs, processed
 * measurements and other values of interest.
 */
class BufferedIns : public BufferedPva {

public:
	/**
	 * Constructor.
	 *
	 * @param pva Time-tagged initial position, velocity and attitude `nullptr` if unknown.
	 * @param expected_dt The typical delta time between PVAs, in seconds.
	 * @param buffer_length Amount of data, in seconds, this class should keep in history. Max
	 * number of accessible buffer elements is `buffer_length/expected_dt + 2`.
	 */
	BufferedIns(std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude> pva = nullptr,
	            double expected_dt                                                     = 1.0,
	            double buffer_length                                                   = 60.0);

	/**
	 * Constructor.
	 *
	 * @param pva Time-tagged initial position, velocity and attitude.
	 * @param expected_dt The typical delta time between PVAs, in seconds.
	 * @param buffer_length Amount of data, in seconds, this class should keep in history. Max
	 * number of accessible buffer elements is `buffer_length/expected_dt + 2`.
	 */
	BufferedIns(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva,
	            double expected_dt   = 1.0,
	            double buffer_length = 60.0);

	/**
	 * Adds a PVA to the buffer.
	 *
	 * @param pva Time-tagged position, velocity and attitude to be added to the buffer.
	 */
	void add_pva(not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> pva);

	/**
	 * Adds a PVA to the buffer.
	 *
	 * @param pva Time-tagged position, velocity and attitude to be added to the buffer.
	 */
	void add_pva(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva);

	/**
	 * Adds a PVA to the buffer.
	 *
	 * @param data Time-tagged position, velocity and attitude to be added to the buffer. Must be of
	 * type aspn_xtensor::MeasurementPositionVelocityAttitude.
	 *
	 * @throw If error mode is DIE and \p data is a type other than
	 * aspn_xtensor::MeasurementPositionVelocityAttitude.
	 */
	void add_data(not_null<std::shared_ptr<aspn_xtensor::AspnBase>> data) override;
};

}  // namespace inertial
}  // namespace navtk
