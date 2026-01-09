import numpy as np
import matplotlib.pyplot as plt
from navtk.navutils import delta_lat_to_north, delta_lon_to_east


def plot_trajectory(d_ned_true, d_ned_filt):
    plt.figure()
    plt.plot(d_ned_true[1, :], d_ned_true[0, :], "k", label="True")
    plt.plot(d_ned_filt[1, :], d_ned_filt[0, :], "m-", label="Filter")
    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title("Trajectory")
    plt.legend()
    plt.grid()


def plot_delta_results(
    filter_results, sigma_results, truth_interp, delta_tilts, time_tags
):
    lat_index = 0
    vned_index = 3
    truth_size = len(time_tags[:])
    state_names = [
        "North Pos Error",
        "East Pos Error",
        "Down Pos Error",
        "North Vel Error",
        "East Vel Error",
        "Down Vel Error",
        "North Tilt Error",
        "East Tilt Error",
        "Down Tilt Error",
    ]
    figure_titles = ["Position Error", "Velocity Error", "Tilt Error"]
    state_y_labels = [
        "meters",
        "meters",
        "meters",
        "m/s",
        "m/s",
        "m/s",
        "radians",
        "radians",
        "radians",
    ]
    lat_factor = delta_lat_to_north(
        1, truth_interp.lat[0], truth_interp.alt[0]
    )
    lon_factor = delta_lon_to_east(1, truth_interp.lat[0], truth_interp.alt[0])
    delta_pos_vel = np.zeros((6, truth_size))
    delta_pos_vel[0, :] = lat_factor * (
        filter_results[lat_index, :] - truth_interp.lat
    )
    delta_pos_vel[1, :] = lon_factor * (
        filter_results[lat_index + 1, :] - truth_interp.lon
    )
    delta_pos_vel[2, :] = -1 * (
        filter_results[lat_index + 2, :] - truth_interp.alt
    )
    delta_pos_vel[3, :] = filter_results[vned_index, :] - truth_interp.vel_n
    delta_pos_vel[4, :] = (
        filter_results[vned_index + 1, :] - truth_interp.vel_e
    )
    delta_pos_vel[5, :] = (
        filter_results[vned_index + 2, :] - truth_interp.vel_d
    )

    delta = [0, 0]
    delta_index = delta[0]
    stand_dev = [0, 6]
    sd_index = stand_dev[0]
    delta_data = delta_pos_vel

    for state in [0, 3, 6]:
        plt.figure()
        plt.suptitle(figure_titles[int(state / 3)])
        for p_idx in range(state, state + 3):
            plt.subplot(3, 1, 1 + p_idx % 3)
            plt.plot(
                time_tags,
                delta_data[delta_index, :],
                "k",
                label=state_names[p_idx],
            )
            plt.plot(
                time_tags,
                sigma_results[sd_index, :],
                "b",
                label="Filter-Computed 1-Sigma",
            )
            plt.plot(time_tags, -1 * sigma_results[sd_index, :], "b")
            plt.xlabel("Time (s)")
            plt.ylabel(state_y_labels[p_idx])
            plt.legend()
            plt.grid()
            if p_idx == 5:
                delta_index = delta[1]
                delta_data = delta_tilts
            else:
                delta_index += 1
            sd_index += 1


def plot_pos_vel_profile(
    filter_results, d_ned_true, d_ned_filt, truth_interp, time_tags
):
    vned_index = 3
    time_tags_plot = time_tags - time_tags[0]
    subplot_idx = [311, 312, 313]
    true_colors = ["k", "b", "m"]
    filt_colors = ["y-", "c-", "g-"]

    # Plot Position Profile
    plt.figure()
    plt.suptitle("Position Profile (Relative to Starting Point)")
    true_labels = ["dNorth (true)", "dEast (true)", "dAlt (true)"]
    filt_labels = [
        "dNorth (estimated)",
        "dEast (estimated)",
        "dAlt (estimated)",
    ]
    for idx in range(3):
        plt.subplot(subplot_idx[idx])
        plt.plot(
            time_tags_plot,
            d_ned_true[idx, :],
            true_colors[idx],
            label=true_labels[idx],
        )
        plt.plot(
            time_tags_plot,
            d_ned_filt[idx, :],
            filt_colors[idx],
            label=filt_labels[idx],
        )
        plt.ylabel("Position (meters)")
        plt.grid()
        plt.legend()
    plt.xlabel("Time (s)")

    # Plot Velocity Profile
    plt.figure()
    plt.suptitle("Velocity Profile")
    filt_labels = ["N Estimated", "E Estimated", "D Estimated"]
    for idx in range(3):
        plt.subplot(subplot_idx[idx])
        if idx == 0:
            plt.plot(
                time_tags_plot,
                truth_interp.vel_n,
                true_colors[idx],
                label="N True",
            )
        if idx == 1:
            plt.plot(
                time_tags_plot,
                truth_interp.vel_e,
                true_colors[idx],
                label="E True",
            )
        if idx == 2:
            plt.plot(
                time_tags_plot,
                truth_interp.vel_d,
                true_colors[idx],
                label="D True",
            )
        plt.plot(
            time_tags_plot,
            filter_results[vned_index, :],
            filt_colors[idx],
            label=filt_labels[idx],
        )
        vned_index = vned_index + 1
        plt.ylabel("Velocity (m/s)")
        plt.grid()
        plt.legend()
    plt.xlabel("Time (s)")


def plot_rpy(truth, filter_results, time_tags):
    truth_size = len(truth.time)
    # Roll index in filter_results
    filter_r_index = 6
    plot_truth_rpy = np.zeros((4, truth_size))
    plot_truth_rpy[0, :] = np.array(truth.time_double) - time_tags[0]
    plot_truth_rpy[1, :] = truth.att_r
    plot_truth_rpy[2, :] = truth.att_p
    plot_truth_rpy[3, :] = truth.att_y

    time_tags_plot = time_tags - time_tags[0]
    state_names = ["Roll", "Pitch", "Yaw"]

    plt.figure()
    plt.suptitle("Attitude Profile")
    for p_idx in range(3):
        plt.subplot(3, 1, p_idx + 1)
        plt.plot(
            plot_truth_rpy[0, :],
            plot_truth_rpy[1 + p_idx, :],
            "b",
            label=state_names[p_idx] + " Truth",
        )
        plt.plot(
            time_tags_plot,
            filter_results[filter_r_index + p_idx, :],
            "k",
            label=state_names[p_idx] + "Estimate",
        )
        plt.xlabel("Time (s)")
        plt.ylabel("radians")
        plt.legend()
        plt.grid()


def plot_acc_gyro_err(filter_results, sigma_results, time_tags):
    bias_index = [18, 21]  # acc, gyro
    sigma_index = [9, 12]
    num_plots = 2
    time_tags_plot = time_tags - time_tags[0]

    title_labels = [
        "Accelerometer Bias Error State Estimates",
        "Gyro Bias Error State Estimates",
    ]
    y_labels = ["Bias in (m/s^2)", "Bias in (rad/s)"]

    filter_colors = ["k", "b", "g"]
    sigma_colors = ["-r", "-y", "-m"]
    filter_legend = ["X", "Y", "Z"]
    sigma_legend = [
        "estimate +/- X Sigma",
        "estimate +/- Y Sigma",
        "estimate +/- Z Sigma",
    ]
    subplot_idx = [311, 312, 313]

    for figure_idx in range(num_plots):
        plt.figure()
        plt.suptitle(title_labels[figure_idx])

        for axis_idx in range(3):
            plt.subplot(subplot_idx[axis_idx])
            plt.plot(
                time_tags_plot,
                filter_results[bias_index[figure_idx] + axis_idx, :],
                filter_colors[axis_idx],
                label=filter_legend[axis_idx],
            )
            plt.plot(
                time_tags_plot,
                filter_results[bias_index[figure_idx] + axis_idx, :]
                + sigma_results[sigma_index[figure_idx] + axis_idx, :],
                sigma_colors[axis_idx],
                label=sigma_legend[axis_idx],
            )
            plt.plot(
                time_tags_plot,
                filter_results[bias_index[figure_idx] + axis_idx, :]
                - sigma_results[sigma_index[figure_idx] + axis_idx, :],
                sigma_colors[axis_idx],
            )
            plt.grid()
            plt.legend()
            plt.ylabel(y_labels[figure_idx])
        plt.xlabel("Time (s)")
