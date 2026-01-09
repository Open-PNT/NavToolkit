#include <navtk/utils/conversions.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/inertial/InertialPosVelAtt.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace utils {

using aspn_xtensor::MeasurementPosition;
using aspn_xtensor::MeasurementPositionVelocityAttitude;
using aspn_xtensor::to_type_timestamp;
using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using navtk::filtering::NavSolution;
using navtk::inertial::InertialPosVelAtt;
using navtk::inertial::StandardPosVelAtt;
using navtk::navutils::dcm_to_quat;
using navtk::navutils::dcm_to_rpy;
using navtk::navutils::quat_to_dcm;
using navtk::navutils::quat_to_rpy;
using navtk::navutils::rpy_to_dcm;


AspnMeasurementPositionReferenceFrame convert_pva_to_pos_ref_frame(
    AspnMeasurementPositionVelocityAttitudeReferenceFrame r) {
	switch (r) {
	case ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC: {
		return ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC;
	}
	case ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_ECI: {
		return ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_ECI;
	}
	default: {
		log_or_throw<std::invalid_argument>(
		    "Unrecognized AspnMeasurementPositionVelocityAttitudeReferenceFrame {} "
		    "encountered. Pretending that it is "
		    "ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC",
		    r);
		return ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC;
	}
	}
}

NavSolution to_navsolution(const MeasurementPositionVelocityAttitude& pva) {
	return NavSolution(extract_pos(pva),
	                   extract_vel(pva),
	                   xt::transpose(quat_to_dcm(pva.get_quaternion())),
	                   pva.get_time_of_validity());
}

NavSolution to_navsolution(const InertialPosVelAtt& pva) {
	return to_navsolution(to_positionvelocityattitude(pva));
}

NavSolution to_navsolution(const Vector& pva) {
	return NavSolution(xt::view(pva, xt::range(1, 4)),
	                   xt::view(pva, xt::range(4, 7)),
	                   xt::transpose(rpy_to_dcm(xt::view(pva, xt::range(7, 10)))),
	                   to_type_timestamp(pva[0]));
}

MeasurementPositionVelocityAttitude to_positionvelocityattitude(const NavSolution& pva) {
	auto header = TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0);
	auto time   = pva.time;
	// No cov available (zeros)
	return MeasurementPositionVelocityAttitude(
	    header,
	    time,
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
	    pva.pos(0),
	    pva.pos(1),
	    pva.pos(2),
	    pva.vel(0),
	    pva.vel(1),
	    pva.vel(2),
	    dcm_to_quat(xt::transpose(pva.rot_mat)),
	    zeros(9, 9),
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
	    Vector(),
	    std::vector<aspn_xtensor::TypeIntegrity>{});
}

MeasurementPositionVelocityAttitude to_positionvelocityattitude(const InertialPosVelAtt& pva) {
	auto header     = TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0);
	auto time       = pva.time_validity;
	auto llh        = pva.get_llh();
	auto vel        = pva.get_vned();
	auto quaternion = dcm_to_quat(pva.get_C_s_to_ned());
	// No cov available (zeros)
	return MeasurementPositionVelocityAttitude(
	    header,
	    time,
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
	    llh(0),
	    llh(1),
	    llh(2),
	    vel(0),
	    vel(1),
	    vel(2),
	    quaternion,
	    zeros(9, 9),
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
	    Vector(),
	    std::vector<aspn_xtensor::TypeIntegrity>{});
}

void to_positionvelocityattitude(const InertialPosVelAtt& pva,
                                 MeasurementPositionVelocityAttitude& storage) {
	storage.set_header(TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0));
	storage.set_time_of_validity(pva.time_validity);
	auto llh = pva.get_llh();
	storage.set_p1(llh(0));
	storage.set_p2(llh(1));
	storage.set_p3(llh(2));
	auto vned = pva.get_vned();
	storage.set_v1(vned(0));
	storage.set_v2(vned(1));
	storage.set_v3(vned(2));
	storage.set_quaternion(dcm_to_quat(pva.get_C_s_to_ned()));
}

MeasurementPositionVelocityAttitude to_positionvelocityattitude(const Vector& pva) {
	return to_positionvelocityattitude(to_navsolution(pva));
}

StandardPosVelAtt to_standardposvelatt(const NavSolution& pva) {
	return to_standardposvelatt(to_positionvelocityattitude(pva));
}

StandardPosVelAtt to_standardposvelatt(const MeasurementPositionVelocityAttitude& pva) {
	return StandardPosVelAtt(pva.get_time_of_validity(),
	                         extract_pos(pva),
	                         extract_vel(pva),
	                         quat_to_dcm(pva.get_quaternion()));
}

StandardPosVelAtt to_standardposvelatt(const Vector& pva) {
	return to_standardposvelatt(to_positionvelocityattitude(pva));
}

Vector to_vector_pva(const MeasurementPositionVelocityAttitude& pva) {
	auto rpy = quat_to_rpy(pva.get_quaternion());
	return {pva.get_aspn_c()->time_of_validity.elapsed_nsec / 1e9,
	        pva.get_p1(),
	        pva.get_p2(),
	        pva.get_p3(),
	        pva.get_v1(),
	        pva.get_v2(),
	        pva.get_v3(),
	        rpy[0],
	        rpy[1],
	        rpy[2]};
}

Vector to_vector_pva(const NavSolution& pva) {
	return to_vector_pva(to_positionvelocityattitude(pva));
}

Vector to_vector_pva(const InertialPosVelAtt& pva) {
	return to_vector_pva(to_positionvelocityattitude(pva));
}

MeasurementPosition to_position(const MeasurementPositionVelocityAttitude& pva) {
	auto header = TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0);
	std::vector<double> pos{pva.get_p1(), pva.get_p2(), pva.get_p3()};
	// extract the covariance for the non-null position terms
	std::vector<Size> cov_idx;
	for (auto d : pos) {
		if (!std::isnan(d)) cov_idx.push_back(cov_idx.size());
	}
	auto kp = xt::keep(cov_idx);
	auto cv = xt::view(pva.get_covariance(), kp, kp);

	auto ref = convert_pva_to_pos_ref_frame(pva.get_reference_frame());
	return MeasurementPosition(header,
	                           pva.get_time_of_validity(),
	                           ref,
	                           pos[0],
	                           pos[1],
	                           pos[2],
	                           cv,
	                           ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
	                           pva.get_error_model_params(),
	                           pva.get_integrity());
}

Vector3 extract_pos(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva) {
	return {pva.get_p1(), pva.get_p2(), pva.get_p3()};
}

Vector3 extract_vel(const aspn_xtensor::MeasurementPositionVelocityAttitude& pva) {
	return {pva.get_v1(), pva.get_v2(), pva.get_v3()};
}

Vector3 extract_pos(const aspn_xtensor::MeasurementPosition& pos) {
	return {pos.get_term1(), pos.get_term2(), pos.get_term3()};
}

Vector3 extract_vel(const aspn_xtensor::MeasurementVelocity& vel) {
	return {vel.get_x(), vel.get_y(), vel.get_z()};
}

aspn_xtensor::MeasurementImu to_imu(aspn_xtensor::TypeTimestamp time,
                                    const Vector& forces,
                                    const Vector& rates) {
	return aspn_xtensor::MeasurementImu(aspn_xtensor::TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0),
	                                    time,
	                                    ASPN_MEASUREMENT_IMU_IMU_TYPE_SAMPLED,
	                                    forces,
	                                    rates,
	                                    std::vector<aspn_xtensor::TypeIntegrity>{});
}

AspnBaseVector to_inertial_aux(const filtering::NavSolution& nav_sol,
                               const Vector& forces,
                               const Vector& rates) {
	auto pva = std::make_shared<aspn_xtensor::MeasurementPositionVelocityAttitude>(
	    navtk::utils::to_positionvelocityattitude(nav_sol));
	auto f_and_r = std::make_shared<aspn_xtensor::MeasurementImu>(
	    to_imu(pva->get_time_of_validity(), forces, rates));
	return AspnBaseVector{pva, f_and_r};
}

}  // namespace utils
}  // namespace navtk
