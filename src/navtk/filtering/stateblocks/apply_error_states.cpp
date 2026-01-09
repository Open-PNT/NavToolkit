#include <navtk/filtering/stateblocks/apply_error_states.hpp>

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/Pinson21NedBlock.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/inspect.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/conversions.hpp>


namespace navtk {
namespace filtering {

using aspn_xtensor::MeasurementPositionVelocityAttitude;
using navtk::inertial::InertialPosVelAtt;
using navtk::inertial::StandardPosVelAtt;
using navtk::utils::to_navsolution;
using navtk::utils::to_positionvelocityattitude;
using navtk::utils::to_standardposvelatt;
using xt::range;
using xt::view;

namespace {
void too_few_elements_warn(const std::string& block, int num) {
	spdlog::warn(
	    "Cannot interpret Vector of size {} as {}; too few elements. Corrections not applied.",
	    num,
	    block);
}
}  // namespace

template <>
NavSolution apply_error_states<Pinson15NedBlock>(const NavSolution& pva, const Vector& x) {
	if (num_rows(x) < 9) {
		too_few_elements_warn("Pinson15NedBlock", num_rows(x));
		return pva;
	}
	auto lat      = pva.pos[0] + navutils::north_to_delta_lat(x[0], pva.pos[0], pva.pos[2]);
	auto lon      = pva.pos[1] + navutils::east_to_delta_lon(x[1], pva.pos[0], pva.pos[2]);
	auto alt      = pva.pos[2] - x[2];
	auto v        = pva.vel + view(x, range(3, 6));
	auto C_n_to_s = xt::transpose(
	    navutils::correct_dcm_with_tilt(xt::transpose(pva.rot_mat), view(x, range(6, 9))));
	return NavSolution({lat, lon, alt}, v, C_n_to_s, pva.time);
}

template <>
NavSolution apply_error_states<Pinson21NedBlock>(const NavSolution& pva, const Vector& x) {
	if (num_rows(x) < 9) {
		too_few_elements_warn("Pinson21NedBlock", num_rows(x));
		return pva;
	}
	return apply_error_states<Pinson15NedBlock>(pva, x);
}

template <>
MeasurementPositionVelocityAttitude apply_error_states<Pinson15NedBlock>(
    const MeasurementPositionVelocityAttitude& pva, const Vector& x) {

	auto corrected =
	    to_positionvelocityattitude(apply_error_states<Pinson15NedBlock>(to_navsolution(pva), x));
	corrected.get_aspn_c()->header = pva.get_aspn_c()->header;
	corrected.set_covariance(pva.get_covariance());
	return corrected;
}

template <>
MeasurementPositionVelocityAttitude apply_error_states<Pinson21NedBlock>(
    const MeasurementPositionVelocityAttitude& pva, const Vector& x) {
	auto corrected =
	    to_positionvelocityattitude(apply_error_states<Pinson21NedBlock>(to_navsolution(pva), x));
	corrected.get_aspn_c()->header = pva.get_aspn_c()->header;
	corrected.set_covariance(pva.get_covariance());
	return corrected;
}

template <>
StandardPosVelAtt apply_error_states<Pinson15NedBlock>(const StandardPosVelAtt& pva,
                                                       const Vector& x) {
	return to_standardposvelatt(apply_error_states<Pinson15NedBlock>(to_navsolution(pva), x));
}

template <>
StandardPosVelAtt apply_error_states<Pinson21NedBlock>(const StandardPosVelAtt& pva,
                                                       const Vector& x) {
	return to_standardposvelatt(apply_error_states<Pinson21NedBlock>(to_navsolution(pva), x));
}

template <>
not_null<std::shared_ptr<InertialPosVelAtt>> apply_error_states<Pinson15NedBlock>(
    const not_null<std::shared_ptr<InertialPosVelAtt>>& pva, const Vector& x) {
	auto spva = StandardPosVelAtt(
	    pva->time_validity, pva->get_llh(), pva->get_vned(), pva->get_C_s_to_ned());
	return std::make_shared<StandardPosVelAtt>(apply_error_states<Pinson15NedBlock>(spva, x));
}

template <>
not_null<std::shared_ptr<InertialPosVelAtt>> apply_error_states<Pinson21NedBlock>(
    const not_null<std::shared_ptr<InertialPosVelAtt>>& pva, const Vector& x) {
	auto spva = StandardPosVelAtt(
	    pva->time_validity, pva->get_llh(), pva->get_vned(), pva->get_C_s_to_ned());
	return std::make_shared<StandardPosVelAtt>(apply_error_states<Pinson21NedBlock>(spva, x));
}

}  // namespace filtering
}  // namespace navtk
