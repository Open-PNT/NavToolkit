import lcm
import numpy as np
import matplotlib.pyplot as plt

from utils.plots import (
    plot_trajectory,
    plot_delta_results,
    plot_pos_vel_profile,
    plot_rpy,
    plot_acc_gyro_err,
)
from utils.TimestampedDataSeries import TimestampedData, TimestampedDataSeries
from utils.aspn2_to_aspn import lcm_geodeticposition3d_to_aspn, lcm_imu_to_aspn
from navtk.filtering import apply_error_states
from navtk.inertial import StandardPosVelAtt
from navtk.navutils import (
    rpy_to_quat,
    quat_to_dcm,
    rpy_to_dcm,
    delta_lat_to_north,
    delta_lon_to_east,
    wrap_to_pi,
)
from navtk.utils import to_vector_pva, cubic_spline_interpolate

from aspn23_xtensor import to_type_timestamp, to_seconds

try:
    import datasources.lcm.messages.aspn as lcm_messages
except ImportError:
    print('Missing ASPN-LCM python module!')
    print('Try adding <navtk>/subprojects/lcm-generated/python to PYTHONPATH')
    exit(1)


class TruthResults:
    def __init__(self):
        self.time = []
        self.time_double = []
        self.lat = []
        self.lon = []
        self.alt = []
        self.vel_n = []
        self.vel_e = []
        self.vel_d = []
        self.att_r = []
        self.att_p = []
        self.att_y = []


class LcmDataHandler:
    def __init__(self, url, truth_channel, imu_channel, gps_channel):

        # Set up the lcm connection
        self.lcm_connection = lcm.LCM(url)

        # Set up containers for storing data
        self.imu_series = TimestampedDataSeries()
        self.imu_time_validity = to_type_timestamp(0)
        self.gps_series = TimestampedDataSeries()
        self.truth_results = TruthResults()

        # Subscribe to data channels
        self.lcm_connection.subscribe(truth_channel, self.handle_truth)
        self.lcm_connection.subscribe(imu_channel, self.handle_imu)
        self.lcm_connection.subscribe(gps_channel, self.handle_gps)

        # Parameters for reading data
        self.lcm_eof = False
        self.keep_reading_data = True

    def handle_truth(self, _, data):
        data = lcm_messages.positionvelocityattitude.decode(data)
        truth_time = to_type_timestamp(
            data.header.timestamp_valid.sec, data.header.timestamp_valid.nsec
        )
        self.truth_results.time.append(truth_time)
        self.truth_results.time_double.append(to_seconds(truth_time))
        self.truth_results.lat.append(data.position.latitude)
        self.truth_results.lon.append(data.position.longitude)
        self.truth_results.alt.append(data.position.altitude)
        self.truth_results.vel_n.append(data.velocity[0])
        self.truth_results.vel_e.append(data.velocity[1])
        self.truth_results.vel_d.append(data.velocity[2])
        self.truth_results.att_r.append(data.attitude[0])
        self.truth_results.att_p.append(data.attitude[1])
        self.truth_results.att_y.append(wrap_to_pi(data.attitude[2]))

    def handle_imu(self, _, data):
        data = lcm_imu_to_aspn(lcm_messages.imu.decode(data))
        imu_time = data.get_time_of_validity()
        imu_data = TimestampedData(imu_time, data)
        self.imu_series.insert_timestamped_data(imu_data)
        self.imu_time_validity = imu_time

    def handle_gps(self, _, data):
        data = lcm_geodeticposition3d_to_aspn(
            lcm_messages.geodeticposition3d.decode(data)
        )

        gps_time = data.get_time_of_validity()
        gps_data = TimestampedData(gps_time, data)
        self.gps_series.insert_timestamped_data(gps_data)


class ResultsHandler:
    def __init__(self, C_INS_TO_PLATFORM, PINSON15):
        self.filter_output = []
        self.sigma_output = []
        self.ins_output = []
        self.output_filter_time = []
        self.C_INS_TO_PLATFORM = C_INS_TO_PLATFORM
        self.PINSON15 = PINSON15

    def generate_output(self, engine, ins):
        ins_pva = ins.get_solution()
        max_filter_size = 24
        max_sigma_size = 15
        temp_filter = np.zeros(max_filter_size)
        temp_sigma = np.zeros(max_sigma_size)

        # Pull out the current solution state
        x_pinson = engine.get_state_block_estimate(self.PINSON15)
        p_pinson = engine.get_state_block_covariance(self.PINSON15)
        C_b_to_n = ins_pva.get_C_s_to_ned() @ self.C_INS_TO_PLATFORM.T
        spva = StandardPosVelAtt(
            ins_pva.time_validity,
            ins_pva.get_llh(),
            ins_pva.get_vned(),
            C_b_to_n,
        )
        corr_pva = apply_error_states(spva, x_pinson)

        temp_filter[0:9] = to_vector_pva(corr_pva)[1:10]
        temp_filter[9:max_filter_size] = x_pinson
        temp_sigma[0:max_sigma_size] = np.sqrt(np.diag(p_pinson))

        temp_ins = to_vector_pva(spva)[1:10]

        self.filter_output.append(temp_filter)
        self.sigma_output.append(temp_sigma)
        self.ins_output.append(temp_ins)
        self.output_filter_time.append(to_seconds(engine.get_time()))

    def convert_truth(self, truth, interp_time):
        interp_truth = TruthResults()
        interp_time = np.array(interp_time)
        interp_truth.time_double = interp_time
        interp_truth.lat = cubic_spline_interpolate(
            np.array(truth.time_double), np.array(truth.lat), interp_time
        )[1]
        interp_truth.lon = cubic_spline_interpolate(
            np.array(truth.time_double), np.array(truth.lon), interp_time
        )[1]
        interp_truth.alt = cubic_spline_interpolate(
            np.array(truth.time_double), np.array(truth.alt), interp_time
        )[1]
        interp_truth.vel_n = cubic_spline_interpolate(
            np.array(truth.time_double), np.array(truth.vel_n), interp_time
        )[1]
        interp_truth.vel_e = cubic_spline_interpolate(
            np.array(truth.time_double), np.array(truth.vel_e), interp_time
        )[1]
        interp_truth.vel_d = cubic_spline_interpolate(
            np.array(truth.time_double), np.array(truth.vel_d), interp_time
        )[1]
        return interp_truth

    def calculate_truth_tilts(
        self, truth, truth_interp, filter_results, ins_results, time_tags
    ):
        quat_0 = []
        quat_1 = []
        quat_2 = []
        quat_3 = []
        truth_size = len(truth.time_double)
        quat_temp = np.zeros((4, truth_size))
        rpy_temp = np.zeros(3)
        for jj in range(truth_size):
            rpy_temp[0] = truth.att_r[jj]
            rpy_temp[1] = truth.att_p[jj]
            rpy_temp[2] = truth.att_y[jj]
            quat_temp[:, jj] = rpy_to_quat(rpy_temp)
            quat_0.append(quat_temp[0, jj])
            quat_1.append(quat_temp[1, jj])
            quat_2.append(quat_temp[2, jj])
            quat_3.append(quat_temp[3, jj])

        quat_interp_0 = cubic_spline_interpolate(
            np.array(truth.time_double), np.array(quat_0), time_tags
        )[1]
        truth_interp_len = len(quat_interp_0)
        truth_interp.quat_matrix = np.zeros((4, truth_interp_len))
        truth_interp.quat_matrix[0, :] = quat_interp_0
        truth_interp.quat_matrix[1, :] = cubic_spline_interpolate(
            np.array(truth.time_double), np.array(quat_1), time_tags
        )[1]
        truth_interp.quat_matrix[2, :] = cubic_spline_interpolate(
            np.array(truth.time_double), np.array(quat_2), time_tags
        )[1]
        truth_interp.quat_matrix[3, :] = cubic_spline_interpolate(
            np.array(truth.time_double), np.array(quat_3), time_tags
        )[1]

        ins_rpy_index = 6
        tilt_index = 15
        delta_tilts = np.zeros((3, truth_interp_len))
        true_tilts = np.zeros((3, truth_interp_len))

        for kk in range(truth_interp_len):
            c_b_to_n_true = quat_to_dcm(truth_interp.quat_matrix[:, kk])
            c_n_to_b_ins = rpy_to_dcm(
                ins_results[ins_rpy_index : ins_rpy_index + 3, kk]
            ).T
            tilt_mat = c_b_to_n_true @ c_n_to_b_ins
            true_tilts[0, kk] = tilt_mat[1, 2]
            true_tilts[1, kk] = tilt_mat[2, 0]
            true_tilts[2, kk] = tilt_mat[0, 1]
            delta_tilts[:, kk] = (
                filter_results[tilt_index : tilt_index + 3, kk]
                - true_tilts[:, kk]
            )

        return true_tilts, delta_tilts

    def calculate_position_profile(self, filter_results, truth_interp):
        filter_data_size = len(filter_results[0, :])
        d_ned_true = np.zeros((3, filter_data_size))
        d_ned_filt = np.zeros((3, filter_data_size))
        lat_index = 0

        for jj in range(filter_data_size):
            d_ned_true[0, jj] = delta_lat_to_north(
                truth_interp.lat[jj] - truth_interp.lat[0],
                truth_interp.lat[0],
                truth_interp.alt[0],
            )
            d_ned_true[1, jj] = delta_lon_to_east(
                truth_interp.lon[jj] - truth_interp.lon[0],
                truth_interp.lat[0],
                truth_interp.alt[0],
            )
            d_ned_true[2, jj] = truth_interp.alt[jj] - truth_interp.alt[0]

            d_ned_filt[0, jj] = delta_lat_to_north(
                filter_results[lat_index, jj] - filter_results[lat_index, 0],
                truth_interp.lat[0],
                truth_interp.alt[0],
            )
            d_ned_filt[1, jj] = delta_lon_to_east(
                filter_results[lat_index + 1, jj]
                - filter_results[lat_index + 1, 0],
                truth_interp.lat[0],
                truth_interp.alt[0],
            )
            d_ned_filt[2, jj] = (
                filter_results[lat_index + 2, jj]
                - filter_results[lat_index + 2, 0]
            )

        return d_ned_true, d_ned_filt

    def plot_results(self, truth_raw):
        truth_interp = self.convert_truth(truth_raw, self.output_filter_time)

        filter_results = np.array(self.filter_output).T
        sigma_results = np.array(self.sigma_output).T
        ins_results = np.array(self.ins_output).T
        time_tag_results = np.array(self.output_filter_time)
        true_tilts, delta_tilts = self.calculate_truth_tilts(
            truth_raw,
            truth_interp,
            filter_results,
            ins_results,
            time_tag_results,
        )

        d_ned_true, d_ned_filt = self.calculate_position_profile(
            filter_results, truth_interp
        )

        plot_trajectory(d_ned_true, d_ned_filt)
        plot_delta_results(
            filter_results,
            sigma_results,
            truth_interp,
            delta_tilts,
            time_tag_results,
        )
        plot_pos_vel_profile(
            filter_results,
            d_ned_true,
            d_ned_filt,
            truth_interp,
            time_tag_results,
        )
        plot_rpy(truth_raw, filter_results, time_tag_results)

        plot_acc_gyro_err(filter_results, sigma_results, time_tag_results)

        plt.show()
