#pragma once

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/Pinson21NedBlock.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>

namespace navtk {
namespace filtering {

/**
 * Correct a position, velocity and attitude container with a Vector of error state values, where
 * interpretation of the error states is guided by template parameter.
 *
 * @tparam Block The type of StateBlock whose state representation x matches.
 * @tparam Pva The Pva container type.
 *
 * @param pva Raw position velocity and attitude.
 * @param x Error state Vector.
 *
 * @return If the specified template overloads are implemented, a corrected version of `pva`.
 * Otherwise, `pva` is returned unmodified with a warning dumped to the user.
 */
template <typename Block, typename Pva>
Pva apply_error_states(const Pva& pva, const Vector& x) {
	spdlog::error("Cannot interpret template arguments; block size {}. No correction applied.",
	              num_rows(x));
	return pva;
}

/**
 * Correct a NavSolution container with a Pinson15NedBlock error state vector.
 *
 * @param pva Raw NavSolution.
 * @param x Error state Vector, where the first 9 states are north, east and down position error in
 * meters; north, east and down velocity error in m/s, and north, east and down tilt errors in
 * radians. Additional states are unused in the correction.
 *
 * @return Corrected NavSolution.
 */
template <>
NavSolution apply_error_states<Pinson15NedBlock>(const NavSolution& pva, const Vector& x);

/**
 * Correct a NavSolution container with a Pinson21NedBlock error state vector.
 *
 * @param pva Raw NavSolution.
 * @param x Error state Vector, where the first 9 states are north, east and down position error in
 * meters; north, east and down velocity error in m/s, and north, east and down tilt errors in
 * radians. Additional states are unused in the correction.
 *
 * @return Corrected NavSolution.
 */
template <>
NavSolution apply_error_states<Pinson21NedBlock>(const NavSolution& pva, const Vector& x);

/**
 * Correct a MeasurementPositionVelocityAttitude container with a Pinson15NedBlock error state
 * vector.
 *
 * @param pva Raw MeasurementPositionVelocityAttitude.
 * @param x Error state Vector, where the first 9 states are north, east and down position error in
 * meters; north, east and down velocity error in m/s, and north, east and down tilt errors in
 * radians. Additional states are unused in the correction.
 *
 * @return Corrected MeasurementPositionVelocityAttitude.
 */
template <>
aspn_xtensor::MeasurementPositionVelocityAttitude apply_error_states<Pinson15NedBlock>(
    const aspn_xtensor::MeasurementPositionVelocityAttitude& pva, const Vector& x);

/**
 * Correct a MeasurementPositionVelocityAttitude container with a Pinson21NedBlock error state
 * vector.
 *
 * @param pva Raw MeasurementPositionVelocityAttitude.
 * @param x Error state Vector, where the first 9 states are north, east and down position error in
 * meters; north, east and down velocity error in m/s, and north, east and down tilt errors in
 * radians. Additional states are unused in the correction.
 *
 * @return Corrected MeasurementPositionVelocityAttitude.
 */
template <>
aspn_xtensor::MeasurementPositionVelocityAttitude apply_error_states<Pinson21NedBlock>(
    const aspn_xtensor::MeasurementPositionVelocityAttitude& pva, const Vector& x);

/**
 * Correct an inertial::StandardPosVelAtt container with a Pinson15NedBlock error state vector.
 *
 * @param pva Raw inertial::StandardPosVelAtt.
 * @param x Error state Vector, where the first 9 states are north, east and down position error in
 * meters; north, east and down velocity error in m/s, and north, east and down tilt errors in
 * radians. Additional states are unused in the correction.
 *
 * @return Corrected inertial::StandardPosVelAtt.
 */
template <>
inertial::StandardPosVelAtt apply_error_states<Pinson15NedBlock>(
    const inertial::StandardPosVelAtt& pva, const Vector& x);

/**
 * Correct a inertial::StandardPosVelAtt container with a Pinson21NedBlock error state vector.
 *
 * @param pva Raw inertial::StandardPosVelAtt.
 * @param x Error state Vector, where the first 9 states are north, east and down position error in
 * meters; north, east and down velocity error in m/s, and north, east and down tilt errors in
 * radians. Additional states are unused in the correction.
 *
 * @return Corrected inertial::StandardPosVelAtt.
 */
template <>
inertial::StandardPosVelAtt apply_error_states<Pinson21NedBlock>(
    const inertial::StandardPosVelAtt& pva, const Vector& x);

/**
 * Correct an inertial::InertialPosVelAtt pointer with a Pinson15NedBlock error state vector.
 *
 * @param pva inertial::InertialPosVelAtt pointer.
 * @param x Error state Vector, where the first 9 states are north, east and down position error in
 * meters; north, east and down velocity error in m/s, and north, east and down tilt errors in
 * radians. Additional states are unused in the correction.
 *
 * @return Corrected inertial::InertialPosVelAtt; pointer is not the same as `pva`.
 */
template <>
not_null<std::shared_ptr<inertial::InertialPosVelAtt>> apply_error_states<Pinson15NedBlock>(
    const not_null<std::shared_ptr<inertial::InertialPosVelAtt>>& pva, const Vector& x);

/**
 * Correct an inertial::InertialPosVelAtt pointer with a Pinson21NedBlock error state vector.
 *
 * @param pva inertial::InertialPosVelAtt pointer.
 * @param x Error state Vector, where the first 9 states are north, east and down position error in
 * meters; north, east and down velocity error in m/s, and north, east and down tilt errors in
 * radians. Additional states are unused in the correction.
 *
 * @return Corrected inertial::InertialPosVelAtt; pointer is not the same as `pva`.
 */
template <>
not_null<std::shared_ptr<inertial::InertialPosVelAtt>> apply_error_states<Pinson21NedBlock>(
    const not_null<std::shared_ptr<inertial::InertialPosVelAtt>>& pva, const Vector& x);

}  // namespace filtering
}  // namespace navtk
