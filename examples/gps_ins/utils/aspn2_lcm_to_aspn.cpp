#include <gps_ins/utils/aspn2_lcm_to_aspn.hpp>

#include <navtk/aspn.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/utils/conversions.hpp>

using aspn_xtensor::TypeHeader;
using aspn_xtensor::TypeTimestamp;
using std::vector;

namespace navtk {
namespace exampleutils {

aspn_xtensor::TypeTimestamp to_aspn(
    const datasources::lcm::messages::aspn::types::timestamp &time) {
	return aspn_xtensor::TypeTimestamp(time.sec * navtk::utils::NANO_PER_SEC + time.nsec);
}

void store_truth_lcm(TruthResults &truth,
                     const datasources::lcm::messages::aspn::positionvelocityattitude &truth_raw) {
	// Pull out data from class fields
	const auto &t_val = truth_raw.header.timestamp_valid;
	truth.time.push_back(t_val.sec + t_val.nsec / 1e9);
	truth.lat.push_back(truth_raw.position.latitude);
	truth.lon.push_back(truth_raw.position.longitude);
	truth.alt.push_back(truth_raw.position.altitude);
	truth.vel_n.push_back(truth_raw.velocity[0]);
	truth.vel_e.push_back(truth_raw.velocity[1]);
	truth.vel_d.push_back(truth_raw.velocity[2]);
	truth.att_r.push_back(truth_raw.attitude[0]);
	truth.att_p.push_back(truth_raw.attitude[1]);
	truth.att_y.push_back(truth_raw.attitude[2]);
}

aspn_xtensor::MetadataGpsLnavEphemeris to_aspn(
    const datasources::lcm::messages::aspn::gpsephemeris &t) {
	// pass as aux data to the filter, change to ASPN ephemeris type
	// create the ASPN ephemeris class
	auto header = TypeHeader(ASPN_METADATA_GPS_CNAV_EPHEMERIS, 0, 0, 0, 0);
	auto metadata_header =
	    aspn_xtensor::TypeMetadataheader(header, "converted from ASPN2 LCM", NAN, 3, 9);
	auto time         = TypeTimestamp(t.t_oe * navtk::utils::NANO_PER_SEC);
	auto satnav_clock = aspn_xtensor::TypeSatnavClock(t.t_oc, t.af_0, t.af_1, t.af_2);
	auto orbit        = aspn_xtensor::TypeKeplerOrbit(t.m_0,
                                               t.delta_n,
                                               t.e,
                                               t.sqrt_a,
                                               t.omega_0,
                                               t.i_0,
                                               t.i_dot,
                                               t.omega,
                                               t.omega_dot,
                                               t.c_uc,
                                               t.c_us,
                                               t.c_rc,
                                               t.c_rs,
                                               t.c_ic,
                                               t.c_is,
                                               t.t_oe);
	aspn_xtensor::MetadataGpsLnavEphemeris ephemeris_data(
	    metadata_header, time, t.wn_t_oe, t.prn, satnav_clock, orbit, t.t_gd, 0, 0);
	return ephemeris_data;
}

aspn_xtensor::MeasurementSatnav to_aspn(const datasources::lcm::messages::aspn::gnss &t) {
	// stores the converted ASPN gnss class
	vector<aspn_xtensor::TypeSatnavObs> obs;
	// convert the observations
	for (auto &observation : t.obs) obs.push_back(to_aspn(observation));

	auto header      = TypeHeader(ASPN_MEASUREMENT_SATNAV, 0, 0, 0, 0);
	auto lcm_time    = t.header.timestamp_valid;
	auto time        = TypeTimestamp(lcm_time.sec * navtk::utils::NANO_PER_SEC + lcm_time.nsec);
	auto satnav_time = aspn_xtensor::TypeSatnavTime(
	    t.week_number, t.seconds_of_week, ASPN_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GPS);
	return aspn_xtensor::MeasurementSatnav{header, time, satnav_time, 1, obs, {}};
}

aspn_xtensor::TypeSatnavObs to_aspn(
    const datasources::lcm::messages::aspn::nested::gnss::gnss_obs &t) {

	aspn_xtensor::TypeSatnavSignalDescriptor signal_descriptor{
	    ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1C};

	if (t.band == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::BAND1 &&
	    t.attribute == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::SIG_C) {
		signal_descriptor = {ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1C};
	}
	if (t.band == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::BAND2 &&
	    t.attribute == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::SIG_C) {
		signal_descriptor = {ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2C};
	}
	if (t.band == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::BAND1 &&
	    t.attribute == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::SIG_P) {
		signal_descriptor = {ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1P};
	}
	if (t.band == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::BAND2 &&
	    t.attribute == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::SIG_P) {
		signal_descriptor = {ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2P};
	}
	if (t.band == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::BAND1 &&
	    t.attribute == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::SIG_Y) {
		signal_descriptor = {ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1Y};
	}
	if (t.band == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::BAND2 &&
	    t.attribute == datasources::lcm::messages::aspn::nested::gnss::gnss_obs::SIG_Y) {
		signal_descriptor = {ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2Y};
	}

	double pseudorange            = NAN;
	double pseudorange_variance   = NAN;
	double doppler                = NAN;
	double doppler_variance       = NAN;
	double carrier_phase          = NAN;
	double carrier_phase_variance = NAN;
	double signal_strength        = NAN;

	// ASPN2 GNSS observation can container *either* a pseudorange, doppler, carrier phase, signal
	// strength, or ionospheric correction measurement. ASPN23 GNSS observation can contain all in
	// one message. For each signal observations, map it but leave the others as NaNs.
	switch (t.type) {
	case datasources::lcm::messages::aspn::nested::gnss::gnss_obs::OBS_C:
		pseudorange = t.observation;
		break;
	case datasources::lcm::messages::aspn::nested::gnss::gnss_obs::OBS_D:
		doppler = t.observation;
		break;
	case datasources::lcm::messages::aspn::nested::gnss::gnss_obs::OBS_L:
		carrier_phase = t.observation;
		break;
	case datasources::lcm::messages::aspn::nested::gnss::gnss_obs::OBS_S:
		signal_strength = t.observation;
		break;
	case datasources::lcm::messages::aspn::nested::gnss::gnss_obs::OBS_I:
		throw std::runtime_error(
		    "Cannot convert ASPN 2 ionospheric correction to ASPN 3 GNSS observation");
	}

	return aspn_xtensor::TypeSatnavObs(ASPN_TYPE_SATNAV_SATELLITE_SYSTEM_SATELLITE_SYSTEM_SYS_GPS,
	                                   signal_descriptor,
	                                   t.prn,
	                                   NAN,
	                                   pseudorange,
	                                   pseudorange_variance,
	                                   ASPN_TYPE_SATNAV_OBS_PSEUDORANGE_RATE_TYPE_PSR_RATE_DOPPLER,
	                                   doppler,
	                                   doppler_variance,
	                                   carrier_phase,
	                                   carrier_phase_variance,
	                                   signal_strength,
	                                   t.lock_count,
	                                   ASPN_TYPE_SATNAV_OBS_IONO_CORRECTION_SOURCE_UNKNOWN,
	                                   false,
	                                   false,
	                                   false,
	                                   {});
}

aspn_xtensor::MeasurementPosition to_aspn(
    const datasources::lcm::messages::aspn::geodeticposition3d &t) {
	auto header = TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0);
	auto time   = to_aspn(t.header.timestamp_valid);
	return aspn_xtensor::MeasurementPosition(header,
	                                         time,
	                                         ASPN_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
	                                         t.position.latitude,
	                                         t.position.longitude,
	                                         t.position.altitude,
	                                         to_matrix(t.covariance),
	                                         ASPN_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
	                                         Vector(),
	                                         std::vector<aspn_xtensor::TypeIntegrity>{});
}

aspn_xtensor::MeasurementImu to_aspn(const datasources::lcm::messages::aspn::imu &t) {
	// pull out delta V's and theta's
	Vector3 d_vel{t.delta_v[0], t.delta_v[1], t.delta_v[2]};
	Vector3 d_theta{t.delta_theta[0], t.delta_theta[1], t.delta_theta[2]};

	auto header = TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0);
	auto time   = to_aspn(t.header.timestamp_valid);
	return aspn_xtensor::MeasurementImu(
	    header, time, ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED, d_vel, d_theta, {});
}

aspn_xtensor::MeasurementPositionVelocityAttitude to_aspn(
    const datasources::lcm::messages::aspn::positionvelocityattitude &t) {
	auto header     = TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0);
	auto time       = to_aspn(t.header.timestamp_valid);
	auto quaternion = navtk::navutils::rpy_to_quat({t.attitude[0], t.attitude[1], t.attitude[2]});

	return aspn_xtensor::MeasurementPositionVelocityAttitude(
	    header,
	    time,
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
	    t.position.latitude,
	    t.position.longitude,
	    t.position.altitude,
	    t.velocity[0],
	    t.velocity[1],
	    t.velocity[2],
	    quaternion,
	    to_matrix(t.covariance),
	    ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
	    {},
	    {});
}

}  // namespace exampleutils
}  // namespace navtk
