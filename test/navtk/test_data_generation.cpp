#include <test_data_generation.hpp>

#include <navtk/aspn.hpp>
#include <navtk/factory.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/navutils/gravity.hpp>
#include <navtk/navutils/navigation.hpp>
#include <navtk/navutils/wgs84.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

using aspn_xtensor::TypeSatnavObs;

namespace navtk {
namespace testing {

aspn_xtensor::MeasurementImu stationary_imu(
    not_null<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>> pva, double dt) {
	// As we are assuming stationary, l_dot and lambda_dot are both 0
	auto Csn  = xt::transpose(navutils::quat_to_dcm(pva->get_quaternion()));
	auto wnie = Vector3{cos(pva->get_p1()), 0, -sin(pva->get_p1())} * navutils::ROTATION_RATE;
	auto dth  = dot(Csn, wnie * dt);
	auto g    = navutils::calculate_gravity_schwartz(pva->get_p3(), pva->get_p1());
	// Inverse of the rotation correction from the calc_force_ned function
	auto inv_corr = inverse(eye(3) + 0.5 * navutils::skew(dth));
	auto dv       = dot(inv_corr, dot(Csn, Vector3{0.0, 0.0, -g[2]} * dt));
	auto new_time = pva->get_time_of_validity() + dt;
	auto header   = aspn_xtensor::TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0);
	return aspn_xtensor::MeasurementImu(
	    header, new_time, ASPN_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED, dv, dth, {});
}

std::vector<TypeSatnavObs> generate_gnss_observations(
    const std::vector<int32_t>& prns_to_use,
    const std::vector<aspn_xtensor::TypeSatnavSatelliteSystem>& sat_sys_to_use,
    const std::vector<AspnTypeSatnavSignalDescriptorSignalDescriptor>& descriptors_to_use) {

	std::vector<TypeSatnavObs> meas;
	for (auto prn : prns_to_use) {
		for (auto sat_sys : sat_sys_to_use) {
			for (auto descriptor : descriptors_to_use) {

				auto attr_val   = 0;
				double band_val = 0;
				switch (descriptor) {
				case ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1P:
					attr_val = 1;
					band_val = 0.01;
					break;
				case ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L1C:
					attr_val = 2;
					band_val = 0.01;
					break;
				case ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2P:
					attr_val = 1;
					band_val = 0.02;
					break;
				case ASPN_TYPE_SATNAV_SIGNAL_DESCRIPTOR_SIGNAL_DESCRIPTOR_L2C:
					attr_val = 2;
					band_val = 0.02;
					break;
				default:
					break;
				}

				double pseudorange   = attr_val + 0.1 + band_val;
				double doppler       = attr_val + 0.3 + band_val;
				double carrier_phase = attr_val + 0.2 + band_val;
				auto obs =
				    TypeSatnavObs(sat_sys,
				                  descriptor,
				                  prn,
				                  0,
				                  pseudorange,
				                  0,
				                  ASPN_TYPE_SATNAV_OBS_PSEUDORANGE_RATE_TYPE_PSR_RATE_DOPPLER,
				                  doppler,
				                  0,
				                  carrier_phase,
				                  0,
				                  0,
				                  0,
				                  ASPN_TYPE_SATNAV_OBS_IONO_CORRECTION_SOURCE_UNKNOWN,
				                  false,
				                  false,
				                  false,
				                  {});
				meas.push_back(obs);
			}
		}
	}
	return meas;
}

}  // namespace testing
}  // namespace navtk
