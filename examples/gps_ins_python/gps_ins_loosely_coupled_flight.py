#!/usr/bin/env python3

import numpy as np
import sys
from navtk.filtering import (
    StandardFusionEngine,
    DirectMeasurementProcessor,
    Pinson15NedBlock,
    GaussianVectorData,
    ImuModel,
    NavSolution,
)
from navtk.inertial import (
    CoarseDynamicAlignment,
    AlignBase,
    Inertial,
    StandardPosVelAtt,
)
from navtk.navutils import (
    dcm_to_rpy,
    rpy_to_dcm,
    delta_lat_to_north,
    delta_lon_to_east,
)
from navtk.utils import to_inertial_aux
from utils.data_handling import LcmDataHandler, ResultsHandler
from utils.utils import (
    create_pinson15_ins_aux_data,
    create_pva_estimate,
    interpolate_inertial_lla,
    time_min,
)

from aspn23_xtensor import to_type_timestamp

# Some constants to control filter behavior

# How long to wait before processing a GPS message (messages are buffered for
# this long, to handle data arriving out of order.
GPS_PROCESS_DELAY = to_type_timestamp(5.0)

# Maximum Filter Propagation Time
MAX_PROP_DT = to_type_timestamp(0.25)

# Our IMU in the sample data recorded data at 100Hz.
IMU_DT = 0.01

# INS sensor to platform DCM, required for s3 data file
C_INS_TO_PLATFORM = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

# Apply feedback to INS
APPLY_FEEDBACK = True

# Feedback Threshold, apply feedback after total error reaches this value
FEEDBACK_THRESHOLD = 10.0

# Create string label for creating/calling pinson15 state block
PINSON15 = "pinson15"

# These are the names of the channels in the log file
IMU_CHANNEL = "aspn://vikingnav/novatel/2001.0/0"
GPS_CHANNEL = "aspn://vikingnav/novatel/1001.3/0"
TRUTH_CHANNEL = "aspn://vikingnav/novatel/1010.0/0"

# When true reads initial alignment from log file; false uses dynamic alignment
# algorithm
ALIGN_FROM_TRUTH = False

# A stationary IMU feels the force of gravity, which we're assuming here is
# constant.
GRAVITY = np.array([0.0, 0.0, -9.81])

# Number of states in a Pinson15 block
NUM_PINSON_STATES = 15


# Script to run a gps and ins loosely coupled example given the
# ../subprojects/navtk-data/afrl_s3_flight_20180618_v2.log data file. The GPS
# and IMU measurements are stored in time ordered buffers even if the
# measurements are received out of order. The buffers are processed in close to
# realtime (5 second delay). Note: the buffers are not thread safe, but this
# example is single threaded. The filter is loosely coupled, using only
# position measurements from GPS and IMU measurements. Usage: python3
#  gps_ins_loosely_coupled_flight.py /path/to/afrl_s3_flight_20180618_v2.log
# [OPTIONAL: plot_flag] [OPTIONAL: number_of_seconds_to_process]
#
# Note: to run this example, the PYTHONPATH environment variable must include
# the path to the bindings, as well as the path to the
# subprojects/lcm-generated/python folder.
def main():

    # Pull out location of data file
    if len(sys.argv) < 2:
        raise Exception(
            "Usage: {} [input-data-file]\n\n".format(sys.argv[0])
            + "No input data file specified"
        )

    data_file = sys.argv[1]
    plot_results_flag = True
    if len(sys.argv) > 2:
        plot_results_flag = sys.argv[2] == "1"

    # Make the assumption this won't process more than a week of data
    number_of_seconds_to_process = to_type_timestamp(604800)
    if len(sys.argv) > 3:
        number_of_seconds_to_process = to_type_timestamp(float(sys.argv[3]))

    # Create url to lcm file location
    url = "file://" + data_file + "?speed=0"

    # Hand the url and channel names to the LcmDataHandler, which will read and
    # store the data in buffers from the lcm file
    data = LcmDataHandler(url, TRUTH_CHANNEL, IMU_CHANNEL, GPS_CHANNEL)

    # Initialize a ResultsHandler to store results
    results = ResultsHandler(C_INS_TO_PLATFORM, PINSON15)

    # Initialize IMU model
    imu_model = ImuModel(
        np.zeros(3) + 0.0095,  # accel_random_walk_sigma
        np.zeros(3) + 0.0000873,  # gyro_random_walk_sigma
        np.zeros(3) + 0.0098,  # accel_bias_sigma
        np.zeros(3) + 3600,  # accel_bias_tau
        np.zeros(3) + 4.8481e-6,  # gyro_bias_sigma
        np.zeros(3) + 3600,  # gyro_bias_tau
    )

    # Process log file till we either hit a truth message to align from or the
    # dynamic alignment generates a solution, depending on which method was
    # selected
    if ALIGN_FROM_TRUTH:
        try:
            while data.lcm_connection.handle_timeout(10) > 0:
                if data.truth_results.time:
                    break
        except OSError:
            raise Exception(
                "Error: Could not find truth data in logfile. Exiting program."
            )
        if len(data.truth_results.time) == 0:
            raise Exception(
                "Error: Could not find truth data in logfile. Exiting program."
            )
        t_start = data.truth_results.time[0]
        data.imu_time_validity = t_start
        t_last_prop = t_start
        t_last_imu = t_start
        lla_initial = np.array(
            [
                data.truth_results.lat[0],
                data.truth_results.lon[0],
                data.truth_results.alt[0],
            ]
        )
        vned_initial = np.array(
            [
                data.truth_results.vel_n[0],
                data.truth_results.vel_e[0],
                data.truth_results.vel_d[0],
            ]
        )
        attitude_initial = np.array(
            [
                data.truth_results.att_r[0],
                data.truth_results.att_p[0],
                data.truth_results.att_y[0],
            ]
        )

    else:
        aligner = CoarseDynamicAlignment(imu_model)
        try:
            while data.lcm_connection.handle_timeout(10) > 0:
                if (
                    aligner.check_alignment_status()
                    == AlignBase.AlignmentStatus.ALIGNED_GOOD
                ):
                    break
                if not data.gps_series.empty() and not data.imu_series.empty():
                    gps_time = data.gps_series.front().timestamp
                    if data.imu_series.back().timestamp >= gps_time:
                        imu_to_process = data.imu_series.get_in_range(
                            0, gps_time + IMU_DT
                        )
                        for imu in imu_to_process:
                            aligner.process(imu.data)
                        aligner.process(data.gps_series.front().data)
                        data.imu_series.erase(imu_to_process)
                        data.gps_series.pop_front()

        except OSError:
            raise Exception(
                "Error: Could not find valid alignment while playing logfile. "
                + "Exiting program."
            )
        computed_alignment = aligner.get_computed_alignment()
        sol = computed_alignment[1]
        t_start = sol.time
        data.imu_time_validity = t_start
        t_last_prop = t_start
        t_last_imu = t_start
        lla_initial = sol.pos
        vned_initial = sol.vel
        attitude_initial = dcm_to_rpy(np.transpose(sol.rot_mat))

    # Initialize the fusion engine model. This, along with the fusion strategy
    # (which defaults to an EKF), state blocks, and measurement processors acts
    # as the navigation filter.
    engine = StandardFusionEngine(t_start)

    # Create a Pinson15Ned stateblock, labeled as "pinson15", and provide the
    # required IMU modeling parameters generated.
    imu_block = Pinson15NedBlock(PINSON15, imu_model)
    # Add the IMU state block to the main state block
    engine.add_state_block(imu_block)
    # Initialize estimate and covariance for the pinson15 state block
    x0_pinson = np.zeros(15)  # initialize the error to zero
    pos_variance = 9.0
    vel_variance = 0.1
    tilt_variance = 0.01
    accel_bias_variance = 9.604e-5
    gyro_bias_variance = 2.3504074e-11
    p0_diagonal = np.array(
        [
            pos_variance,
            pos_variance,
            pos_variance,
            vel_variance,
            vel_variance,
            vel_variance,
            tilt_variance,
            tilt_variance,
            tilt_variance,
            accel_bias_variance,
            accel_bias_variance,
            accel_bias_variance,
            gyro_bias_variance,
            gyro_bias_variance,
            gyro_bias_variance,
        ]
    )
    p0_pinson = np.diag(p0_diagonal)

    # Set initial conditions on the state blocks
    engine.set_state_block_estimate(PINSON15, x0_pinson)
    engine.set_state_block_covariance(PINSON15, p0_pinson)

    # Create measurement processor for GPS position measurements
    measurement_function_jacobian_gps = np.zeros((3, 15))
    measurement_function_jacobian_gps[0:3, 0:3] = np.eye(3)
    gps_processor_label = "gps"
    # This program uses GPS as position updates only and therefore uses the
    # DirectPositionMeasurementProcessor. This measurement processor will be
    # called 'gps', and measurement objects processed by the fusion engine
    # which are labeled 'gps' will be sent to this measurement processor.
    gps_processor = DirectMeasurementProcessor(
        gps_processor_label, PINSON15, measurement_function_jacobian_gps
    )
    engine.add_measurement_processor(gps_processor)

    # Next, create an ins object, giving it the initial position, attitude and
    # initial velocity.

    # Adjust the initial inertial frame to match truth frame. Required for the
    # s3 file data.
    C_platform_to_ned = rpy_to_dcm(attitude_initial)

    # In contrast to using the truth data, the alignment algorithm returns the
    # IMU sensor to nav DCM directly, meaning we do not need to do the platform
    # to sensor correction (to account for the offset between the platform to
    # nav DCM recorded in truth and the IMU sensor orientation on the platform
    # when aligning from the truth messages)
    C_sensor_to_ned = C_platform_to_ned
    if ALIGN_FROM_TRUTH:
        C_sensor_to_ned = C_platform_to_ned @ C_INS_TO_PLATFORM

    ins_nav_solution = NavSolution(
        lla_initial, vned_initial, C_sensor_to_ned.T, t_start
    )
    ins = Inertial(
        StandardPosVelAtt(t_start, lla_initial, vned_initial, C_sensor_to_ned)
    )
    engine.give_state_block_aux_data(
        PINSON15, to_inertial_aux(ins_nav_solution, GRAVITY, np.zeros((3)))
    )

    data.imu_series.erase(data.imu_series.get_in_range(0, t_start))
    data.gps_series.erase(data.gps_series.get_in_range(0, t_start))

    # The lcm python implementation throws an OSError at the end of the log
    # file
    try:
        while data.lcm_connection.handle_timeout(10) > 0:
            # Stop the example if the specified number of seconds has been
            # processed
            if (
                data.imu_time_validity - t_start
            ) > number_of_seconds_to_process:
                break
            # Propagate any GPS messages (older than 5s for now)
            gps_to_process = data.gps_series.get_in_range(
                t_last_prop, data.imu_time_validity - GPS_PROCESS_DELAY
            )
            for gps_meas in gps_to_process:
                gps = gps_meas.data
                lla_gps = np.array(
                    [gps.get_term1(), gps.get_term2(), gps.get_term3()]
                )

                # Propagate the filter to GPS measurement time at MAX_PROP_DT
                # intervals
                while t_last_prop < gps.get_time_of_validity():
                    # If measurements are out of order in the log file then
                    # some IMU measurements that are valid before the start
                    # time can slip in. This clears them out.
                    if t_last_prop == t_start:
                        data.imu_series.erase(
                            data.imu_series.get_in_range(0, t_last_prop)
                        )
                    t_this_prop = time_min(
                        t_last_prop + MAX_PROP_DT, gps.get_time_of_validity()
                    )
                    imu_to_process = data.imu_series.get_in_range(
                        t_last_prop, t_this_prop
                    )
                    if len(imu_to_process) > 0:

                        # Mechanize any IMU messages up to the next interval
                        for imu_meas in imu_to_process:
                            imu = imu_meas.data
                            ins.mechanize(
                                imu.get_time_of_validity(),
                                imu.get_meas_accel(),
                                imu.get_meas_gyro(),
                            )
                        last_imu = imu_to_process[-1].data
                        t_last_imu = last_imu.get_time_of_validity()
                        engine.give_state_block_aux_data(
                            PINSON15,
                            create_pinson15_ins_aux_data(
                                ins, last_imu, IMU_DT
                            ),
                        )
                        data.imu_series.erase(imu_to_process)
                    engine.propagate(t_this_prop)
                    t_last_prop = t_this_prop

                # Get the inertial solution position (mechanized at IMU time <=
                # GPS time)
                lla_ins = ins.get_solution().get_llh()
                ned_pos_err = lla_gps - lla_ins

                # Mechanize (using a temporary INS), get the inertial solution,
                # interpolate the LLA
                if (
                    t_last_imu < gps.get_time_of_validity()
                ) and not data.imu_series.empty():
                    temp_ins = Inertial(ins)
                    interpolated_lla = interpolate_inertial_lla(
                        data.imu_series.front().data,
                        temp_ins,
                        gps.get_time_of_validity(),
                    )
                    ned_pos_err = lla_gps - interpolated_lla
                    lla_ins = interpolated_lla

                # Convert latitude-longitude-altitude (rad-rad-meters) to
                # north-east-down in meters
                ned_pos_err[0] = delta_lat_to_north(
                    ned_pos_err[0], lla_gps[0], lla_gps[2]
                )
                ned_pos_err[1] = delta_lon_to_east(
                    ned_pos_err[1], lla_gps[0], lla_gps[2]
                )
                ned_pos_err[2] *= -1

                # Update the filter using the position (GPS) update
                engine.update(
                    gps_processor_label,
                    GaussianVectorData(
                        gps.get_time_of_validity(),
                        ned_pos_err,
                        gps.get_covariance(),
                    ),
                )

                # Mechanize an additional IMU sample and propagate the filter
                # to the IMU sample time Mechanization ensures INS state stored
                # in generate_output is at same time tag as filter state
                if (
                    t_last_imu < gps.get_time_of_validity()
                ) and not data.imu_series.empty():
                    imu = data.imu_series.front().data
                    ins.mechanize(
                        imu.get_time_of_validity(),
                        imu.get_meas_accel(),
                        imu.get_meas_gyro(),
                    )
                    t_last_imu = imu.get_time_of_validity()
                    engine.give_state_block_aux_data(
                        PINSON15,
                        create_pinson15_ins_aux_data(ins, imu, IMU_DT),
                    )
                    data.imu_series.pop_front()
                    engine.propagate(t_last_imu)
                    t_last_prop = t_last_imu
                    lla_ins = ins.get_solution().get_llh()

                # Calculate output results of filter
                results.generate_output(engine, ins)

                # Lastly, apply state feedback if desired. State feedback is
                # always needed in the vertical channel, and may be in the
                # horizontal channel. In this example, the full feedback of
                # position, velocity, and attitude is implemented but only when
                # the estimated position error exceeds a predefined threshold
                # (FEEDBACK_THRESHOLD).
                if APPLY_FEEDBACK:
                    x_pinson = engine.get_state_block_estimate(PINSON15)
                    temp_x = x_pinson[0:3]
                    if np.linalg.norm(temp_x, 2) > FEEDBACK_THRESHOLD:
                        # Reset the inertial solution
                        ins.reset(
                            create_pva_estimate(ins, engine, lla_ins, PINSON15)
                        )
                        # Reset the filter error state
                        x_pinson[0:9] = np.zeros(9)
                        engine.set_state_block_estimate(PINSON15, x_pinson)

            data.gps_series.erase(gps_to_process)

    # The lcm python implementation throws an error at the end of the log file
    except OSError:
        pass

    # Plot the results
    if plot_results_flag:
        results.plot_results(data.truth_results)


if __name__ == "__main__":
    main()
