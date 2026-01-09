#!/usr/bin/env python3
"""
An example to compare particle filter w/ update to EKF filter w/o update.
In this example, the EKF essentially functions as a faster PF w/o update.
Thus, this example compares update vs. no update, not PF vs. EKF.

NOTE: This example is heavily tuned to a specific rng seed. Changing the
parameters may negatively impact filter performance.
"""

import os
import sys
import random
from numpy import (
    zeros,
    zeros_like,
    eye,
    arange,
    linspace,
    sqrt,
    diag,
    array,
    sin,
    cos,
    pi,
    meshgrid,
    arctan2,
    mean,
    ceil,
    linalg,
    dot,
)
import matplotlib.pyplot as plt
from matplotlib.pyplot import plot, axis, close, title, xlabel, ylabel, legend
from matplotlib.animation import FFMpegWriter

from navtk.filtering import (
    StandardFusionEngine,
    StateBlock,
    GaussianVectorData,
    StandardDynamicsModel,
    first_order_discretization_strategy,
    StandardMeasurementModel,
    MeasurementProcessor,
)
from navtk.filtering.experimental import RbpfStrategy
from navtk.geospatial import SpatialMapDataSource
from navtk.experimental import s_rand

from aspn23_xtensor import to_type_timestamp

ANIMATION_AVAILABLE = FFMpegWriter.isAvailable()

# CONFIGURABLE PARAMETERS #

# NOTE: This example is heavily tuned to a specific rng seed. Changing the
# parameters may negatively impact filter performance.

# How long we are filtering for
RUN_TIME = 400
# When to switch from curved to linear trajectory
TIME_TO_BEGIN_LINEAR_TRAJECTORY = 275
# Average delta position, meters
AVG_STRIDE_LENGTH = 2.5

# Number of particles for RbpfStrategy filter
NUM_PARTICLES = 500

# Make movie animation
MAKE_MOVIE = False
# Output path for the movie
MOVIE_OUTPUT_PATH = 'examples/output/'
# How many sigma ellipse to show on movie
NUM_SIGMA_ELLIPSE = 2.5
# Show results on figures
SHOW_PLOTS_FLAG = True
# Show particle states on map
SHOW_PARTICLES = True
# Steps between showing the particle estimates
PARTICLE_LAPSE_TIME = 30

# Distance from center to start at
STARTING_RADIUS = 70
# Distance from center to end at
ENDING_RADIUS = 170
# Angle from west-east axis to start at
STARTING_ANGLE = -1.5 * pi

# How many hills to model the topology
# Hills are shaped like Gaussian distribution
NUM_HILLS_IN_TOPOLOGY = 18
# Max amplitude of hills in topology (m)
MAX_HILL_AMPLITUDE = 200
# Max cone amplitude (m)
MAX_CONE_AMPLITUDE = 100

# Covariance of altitude "measurements" (m^2)
MEASUREMENT_COVARIANCE = [[5]]
# Noise added during propagation (m)
DELTA_P_NOISE_STD_DEV = 5.0
# Number of steps between updates
UPDATE_PERIOD = 1
# Amount to spread out resampled particles (m)
RESAMPLING_JITTER = 0.1

# NON-CONFIGURABLE PARAMETERS #

# This example is tuned for this specific seed. Changing the seed (or even the
# other parameters) will risk throwing off the simulation.
RNG_SEED = 876543210

# Set RNG Seeds for this simulation and internal libviper RNG
random.seed(RNG_SEED)
s_rand(RNG_SEED)

# Topology bounds based on max distance travelled (m)
HILL_BOUND = (RUN_TIME - TIME_TO_BEGIN_LINEAR_TRAJECTORY) * AVG_STRIDE_LENGTH
if HILL_BOUND < 0:
    HILL_BOUND = ENDING_RADIUS + 50
# Make sure hills fit inside the map boundaries
PLOT_BOUND = HILL_BOUND + 50

NUM_STATES = 2
STATE_BLOCK_LABEL = 'position_SB'
MEASUREMENT_MODEL_LABEL = 'altitude_map_MP'
INITIAL_STATE_COVARIANCE = zeros((NUM_STATES, NUM_STATES))


def particle_filter_example(run_time):
    """
    An example simulating movement along a random topology with only elevation
    observations.

    The goal is to showcase the particle filter with a highly nonlinear update
    model.

    The truth simulates moving at a variable rate in a circular trajectory,
    while the state model is simulated by the addition of white gaussian noise
    to the truth. The observation model is a simulated altitude update which
    gets compared to a predicted measurement derived from a topology map.
    """

    radius = linspace(STARTING_RADIUS, ENDING_RADIUS, run_time)
    # compute final angle based on average step size, move clockwise
    ending_angle = STARTING_ANGLE - AVG_STRIDE_LENGTH * run_time / mean(radius)
    angle = linspace(STARTING_ANGLE, ending_angle, run_time)

    # generate topology
    location_2d = zeros((run_time, 2))
    for loc, rad, ang in zip(location_2d, radius, angle):
        loc[0] = rad * cos(ang)
        loc[1] = rad * sin(ang)

    topology_source = Topology(location_2d)

    (
        state_results,
        state_truth,
        state_particles_history,
        state_covariance_history,
        rmse,
    ) = run_monte_carlo_simulation(location_2d, topology_source, run_time)

    state_naming = ["East Position", "North Position"]

    state_y_labels = ["m", "m"]

    # Get elevation at each grid point on the map for contour plotting
    map_bounds = [-PLOT_BOUND, PLOT_BOUND, -PLOT_BOUND, PLOT_BOUND]
    topo_grid_step = 1
    scaled_east_range = arange(map_bounds[0], map_bounds[1], topo_grid_step)
    scaled_north_range = arange(map_bounds[2], map_bounds[3], topo_grid_step)
    grid_east, grid_north = meshgrid(scaled_east_range, scaled_north_range)

    grid_elevations = topology_source.lookup_datum_array(grid_north, grid_east)
    grid_locs_3d = [grid_east, grid_north, grid_elevations]

    if ANIMATION_AVAILABLE and MAKE_MOVIE:
        make_animation(
            grid_locs_3d,
            state_truth,
            state_particles_history,
            state_covariance_history,
            state_results,
            map_bounds,
            run_time,
        )

    if SHOW_PLOTS_FLAG:
        make_plots(
            state_truth,
            state_results,
            state_covariance_history,
            rmse,
            state_y_labels,
            state_naming,
        )

    if SHOW_PLOTS_FLAG:
        make_trajectory_plots(
            grid_locs_3d, state_truth, state_results, run_time
        )
    if SHOW_PARTICLES and SHOW_PLOTS_FLAG:
        make_particle_trajectory_plot(
            grid_locs_3d,
            state_truth,
            state_results,
            state_particles_history,
            run_time,
        )

    if SHOW_PLOTS_FLAG:
        plt.show()


def run_monte_carlo_simulation(location_2d, topology_source, run_time):

    # determine elevation
    location_3d = zeros((run_time, 3))
    alt_change = zeros(run_time)
    for cur_time in range(run_time):
        location_3d[cur_time, 0] = location_2d[cur_time, 0]
        location_3d[cur_time, 1] = location_2d[cur_time, 1]
        location_3d[cur_time, 2] = topology_source.lookup_datum(
            location_2d[cur_time, 1], location_2d[cur_time, 0]
        )
        if cur_time > 0:
            alt_change[cur_time - 1] = (
                location_3d[cur_time, 2] - location_3d[cur_time - 1, 2]
            )
    alt_change[run_time - 1] = alt_change[run_time - 2]

    # Adust location so velocity increases downhill
    stride_profile = (
        AVG_STRIDE_LENGTH
        - alt_change
        / (1 + max(alt_change) - min(alt_change))
        * 0.5
        * AVG_STRIDE_LENGTH
    )
    stride_profile = stride_profile * AVG_STRIDE_LENGTH / mean(stride_profile)

    state_truth = zeros((run_time, NUM_STATES))
    elevation_measurement = zeros(run_time)

    state_results = zeros((run_time, NUM_STATES))

    # init truth
    true_state = zeros(NUM_STATES)
    initial_angle = arctan2(
        location_3d[1, 1] - location_3d[0, 1],
        location_3d[1, 0] - location_3d[0, 0],
    )
    true_state[0] = location_3d[0, 0]
    true_state[1] = location_3d[0, 1]

    # run the experiment
    random.seed(RNG_SEED)  # reset seed so truth is always generated the same
    print('Simulating truth trajectory...')
    for cur_time in range(run_time):
        if 0 < cur_time < TIME_TO_BEGIN_LINEAR_TRAJECTORY:
            initial_angle = arctan2(
                location_3d[cur_time, 1] - location_3d[cur_time - 1, 1],
                location_3d[cur_time, 0] - location_3d[cur_time - 1, 0],
            )
        true_state[0] += stride_profile[cur_time] * cos(initial_angle)
        true_state[1] += stride_profile[cur_time] * sin(initial_angle)

        state_truth[cur_time] = true_state

        elevation = topology_source.lookup_datum(true_state[1], true_state[0])

        # add noise to elevation observation to simulate barometer measurement
        elevation_measurement[cur_time] = elevation + random.gauss(
            mu=0, sigma=sqrt(MEASUREMENT_COVARIANCE[0][0])
        )

    state_particles_history = zeros((run_time, NUM_STATES, NUM_PARTICLES))
    state_propagate_particles_history = zeros(
        (run_time, NUM_STATES, NUM_PARTICLES)
    )
    state_covariance_history = zeros((run_time, NUM_STATES, NUM_STATES))

    rmse = zeros((NUM_STATES))

    # reset seed so noise for each filter is always the same
    random.seed(RNG_SEED)

    print('Running the filter...')

    # Create the fusion engine with a strategy that supports
    # particle-filtering.
    filter_strategy = RbpfStrategy(NUM_PARTICLES, 0.75, True)
    filter = StandardFusionEngine(to_type_timestamp(), filter_strategy)

    position_SB = MotionStateBlock(STATE_BLOCK_LABEL)
    filter.add_state_block(position_SB)
    filter.set_state_block_estimate(STATE_BLOCK_LABEL, state_truth[0])
    filter.set_state_block_covariance(
        STATE_BLOCK_LABEL, INITIAL_STATE_COVARIANCE
    )

    # mark all as particle states for the model
    filter_strategy.set_marked_states(range(NUM_STATES), [RESAMPLING_JITTER])

    altitude_map_MP = AltitudeMapMeasurementProcessor(
        MEASUREMENT_MODEL_LABEL, [STATE_BLOCK_LABEL]
    )
    filter.add_measurement_processor(altitude_map_MP)
    altitude_map_MP.receive_aux_data(topology_source)

    # run the experiment through the filter
    delta_p = [0, 0]
    for cur_time in range(run_time):

        # simulate delta_p measurements by calculate delta_p from truth state
        # and adding gaussian white noise
        if cur_time > 0:
            delta_p = [
                state_truth[cur_time, 0] - state_truth[cur_time - 1, 0],
                state_truth[cur_time, 1] - state_truth[cur_time - 1, 1],
            ]
            delta_p[0] += random.gauss(mu=0, sigma=DELTA_P_NOISE_STD_DEV)
            delta_p[1] += random.gauss(mu=0, sigma=DELTA_P_NOISE_STD_DEV)

            position_SB.receive_aux_data(delta_p)

        filter.propagate(to_type_timestamp(cur_time))

        state_propagate_particles_history[
            cur_time
        ] = filter_strategy.get_state_particles()

        if cur_time % UPDATE_PERIOD == 0:
            filter.update(
                MEASUREMENT_MODEL_LABEL,
                GaussianVectorData(
                    to_type_timestamp(cur_time),
                    [elevation_measurement[cur_time]],
                    MEASUREMENT_COVARIANCE,
                ),
            )

        state_estimate = filter.get_state_block_estimate(STATE_BLOCK_LABEL)
        state_covariance = filter.get_state_block_covariance(STATE_BLOCK_LABEL)

        state_covariance_history[cur_time] = state_covariance

        state_particles_history[
            cur_time
        ] = filter_strategy.get_state_particles()

        state_results[cur_time] = state_estimate

    for state_idx in range(NUM_STATES):
        for cur_time in range(run_time):
            rmse[state_idx] += (
                state_truth[cur_time][state_idx]
                - state_results[cur_time][state_idx]
            ) ** 2
        rmse[state_idx] /= run_time

    rmse = sqrt(rmse)

    return (
        state_results,
        state_truth,
        state_particles_history,
        state_covariance_history,
        rmse,
    )


def make_animation(
    grid_locs_3d,
    state_truth,
    state_particles_history,
    state_covariance_history,
    state_results,
    map_bounds,
    run_time,
):

    os.makedirs(MOVIE_OUTPUT_PATH, exist_ok=True)

    print("Rendering movie...")

    metadata = dict(
        title='Movie ' + 'particle_filter',
        artist='particle_filter_example.py',
        comment='Movie support!',
        b='800k',
    )
    writer = FFMpegWriter(fps=10, metadata=metadata)
    fig_movie = plt.figure()
    name_concat = 'particle_filter.mp4'
    file_output = MOVIE_OUTPUT_PATH + name_concat

    with writer.saving(fig_movie, file_output, dpi=300):
        for cur_time in range(run_time):
            plt.clf()

            # Basic contour plot
            contour = plt.contour(
                grid_locs_3d[0], grid_locs_3d[1], grid_locs_3d[2]
            )
            # Recast levels to new class
            contour.levels = [NumberFormat(val) for val in contour.levels]

            # Label levels with specially formatted floats
            if plt.rcParams["text.usetex"]:
                fmt = r'%r '
            else:
                fmt = '%r '

            plt.clabel(
                contour, contour.levels, inline=True, fmt=fmt, fontsize=10
            )
            plot(
                state_truth[:cur_time, 0],
                state_truth[:cur_time, 1],
                label="True State",
                color="k",
            )

            particles_east_loc = state_particles_history[cur_time, 0, :]
            particles_north_loc = state_particles_history[cur_time, 1, :]
            plt.scatter(
                particles_east_loc, particles_north_loc, marker=".", color='y'
            )

            # compute error ellipse parameters
            sub_covariance = state_covariance_history[cur_time][0:2, 0:2]
            d_p, v_p = linalg.eig(sub_covariance)
            sin_cos_data = zeros((2, 21))
            sin_cos_data[0, :] = cos(
                arange(0, 2 * pi + 2 * pi / 20, 2 * pi / 20)
            )
            sin_cos_data[1, :] = sin(
                arange(0, 2 * pi + 2 * pi / 20, 2 * pi / 20)
            )
            ell = NUM_SIGMA_ELLIPSE * dot(
                dot(sqrt(diag(abs(d_p))), v_p), sin_cos_data
            )
            ellx = ell[0, :] + state_results[cur_time][0]
            elly = ell[1, :] + state_results[cur_time][1]
            plot(ellx, elly, '-', color='g')
            plot(
                state_results[cur_time][0],
                state_results[cur_time][1],
                '*',
                color='g',
                label='Particle Filter',
            )

            xlabel('East Position (m)')
            ylabel('North Position (m)')
            axis([map_bounds[0], map_bounds[1], map_bounds[2], map_bounds[3]])
            legend(loc='upper right')
            title('North-East Plane')
            writer.grab_frame()
    close(fig_movie)

    print(f'Movie written to: {os.path.abspath(file_output)}')


def make_plots(
    state_truth,
    state_results,
    state_covariance_history,
    rmse,
    state_y_labels,
    state_naming,
):

    print("Generating plots...")
    for state_idx in range(NUM_STATES):

        # make one plot with the state of each filter overlayed with true state
        fig, ax = plt.subplots(2, sharex=True)
        fig.suptitle(state_naming[state_idx])
        ax[0].plot(state_truth[:, state_idx], label="True State", color="k")
        ax[0].set_ylabel(state_y_labels[state_idx])
        xlabel("Time (s)")

        state = state_results[:, state_idx]
        label = f'RMSE = {rmse[state_idx]}'
        ax[0].plot(state, label=label, color='g')

        # make subplots for error of each filter
        error = state_results[:, state_idx] - state_truth[:, state_idx]
        error_label = 'Error'
        std_dev = sqrt(state_covariance_history[:, state_idx, state_idx])
        std_dev_label = 'Std Dev'
        ax[1].plot(error, label=error_label, color='g')
        ax[1].plot(std_dev, label=std_dev_label, color='r')
        ax[1].plot(-std_dev, color='r')
        ax[1].legend()
        ax[1].set_ylabel(f'Error ({state_y_labels[state_idx]})')

        ax[0].legend()
        fig.show()


def generate_contour_map(grid_locs_3d, state_truth, run_time):
    # Basic contour plot
    fig, ax = plt.subplots()

    contour = ax.contour(grid_locs_3d[0], grid_locs_3d[1], grid_locs_3d[2])
    # Recast levels to new class
    contour.levels = [NumberFormat(val) for val in contour.levels]

    # Label levels with specially formatted floats
    if plt.rcParams["text.usetex"]:
        fmt = r'%r '
    else:
        fmt = '%r '

    ax.clabel(contour, contour.levels, inline=True, fmt=fmt, fontsize=10)

    plot(
        state_truth[0:run_time, 0],
        state_truth[0:run_time, 1],
        label="True State",
        color="k",
    )

    return fig


def make_trajectory_plots(grid_locs_3d, state_truth, state_results, run_time):

    fig = generate_contour_map(grid_locs_3d, state_truth, run_time)

    # plot trajectory
    data_east = state_results[:, 0]
    data_north = state_results[:, 1]
    plot(data_east, data_north, label='Particle Filter', color='g')

    xlabel('East Position (m)')
    ylabel('North Position (m)')
    title('North-East Plane')
    legend()
    fig.show()


def make_particle_trajectory_plot(
    grid_locs_3d, state_truth, state_results, state_particles_history, run_time
):

    fig = generate_contour_map(grid_locs_3d, state_truth, run_time)

    # plot particle filter trajectory
    data_east = state_results[:, 0]
    data_north = state_results[:, 1]
    plot(data_east, data_north, label='Particle Filter', color='g')

    # show particles at steps of PARTICLE_LAPSE_TIME
    for cur_time in arange(0, run_time, PARTICLE_LAPSE_TIME):
        p_east = state_particles_history[cur_time, 0, :]
        p_north = state_particles_history[cur_time, 1, :]
        p_label = "Time %d" % cur_time
        plt.scatter(p_east, p_north, label=p_label, marker=".")
    # show particles at end
    p_east = state_particles_history[run_time - 1, 0, :]
    p_north = state_particles_history[run_time - 1, 1, :]
    p_label = "Time %d" % (run_time - 1)

    plt.scatter(p_east, p_north, label=p_label, marker=".")
    xlabel('East Position (m)')
    ylabel('North Position (m)')
    legend()
    title('North-East Plane Particles')

    fig.show()


class Topology(SpatialMapDataSource):
    def __init__(self, location_2d):
        SpatialMapDataSource.__init__(self)

        # reset seed so that Topology is always generated the same
        random.seed(RNG_SEED)

        # create an overall cone-like topology, sloping downward from the
        # starting location
        self.cone_center = [location_2d[0, 0], location_2d[0, 1]]
        # elevation at distance of cone_radius from start is 0
        self.cone_radius = sqrt(2 * (2 * HILL_BOUND) ** 2)
        self.cone_amplitude = MAX_CONE_AMPLITUDE

        # Bounds of map in which to generate hills
        topo_hill_bounds = [-HILL_BOUND, HILL_BOUND, -HILL_BOUND, HILL_BOUND]
        hill_bounds_delta_east = topo_hill_bounds[1] - topo_hill_bounds[0]
        hill_bounds_delta_north = topo_hill_bounds[3] - topo_hill_bounds[2]

        # compute starting delta_p to set criteria for near Gaussian peak
        delta_p = sqrt(
            (location_2d[1, 0] - location_2d[0, 0]) ** 2
            + (location_2d[1, 1] - location_2d[0, 1]) ** 2
        )

        # Generate num_hills_east x num_hills_north = num_hills
        num_hills_east = int(round(sqrt(NUM_HILLS_IN_TOPOLOGY)))
        num_hills_north = (
            ceil(NUM_HILLS_IN_TOPOLOGY / num_hills_east)
            if NUM_HILLS_IN_TOPOLOGY > 0
            else 0
        )
        hill_east_step = round(hill_bounds_delta_east / (1 + num_hills_east))
        hill_north_step = round(
            hill_bounds_delta_north / (1 + num_hills_north)
        )

        radius_scale = sqrt(hill_east_step**2 + hill_north_step**2)

        self.hill_centers = zeros((NUM_HILLS_IN_TOPOLOGY, 2))
        self.hill_amplitudes = zeros(NUM_HILLS_IN_TOPOLOGY)
        self.hill_radii = zeros(NUM_HILLS_IN_TOPOLOGY)
        hill_count = 0
        for ii in range(num_hills_east):
            for jj in range(0, int(num_hills_north)):
                east_loc = topo_hill_bounds[0] + (ii + 1) * hill_east_step
                north_loc = topo_hill_bounds[2] + (jj + 1) * hill_north_step

                if hill_count < NUM_HILLS_IN_TOPOLOGY:
                    hill_center_east = (
                        east_loc
                        + random.gauss(mu=0, sigma=1)
                        * radius_scale
                        / num_hills_east**2
                    )
                    hill_center_north = (
                        north_loc
                        + random.gauss(mu=0, sigma=1)
                        * radius_scale
                        / num_hills_north**2
                    )

                    distance_from_hill_to_locs = sqrt(
                        (hill_center_east - location_2d[:, 0]) ** 2
                        + (hill_center_north - location_2d[:, 1]) ** 2
                    )
                    if min(distance_from_hill_to_locs) < 5 * delta_p:
                        # avoid very small amplitudes
                        amp = (
                            0.85 + 0.15 * (random.random() - 0.1)
                        ) * MAX_HILL_AMPLITUDE
                        rad = 0.01 * radius_scale**1.5
                    else:
                        # employ smaller amplitudes
                        amp = random.random() * MAX_HILL_AMPLITUDE
                        random_value = 10 * random.random()
                        rad = random_value * radius_scale**1.5

                    self.hill_centers[hill_count] = [
                        hill_center_east,
                        hill_center_north,
                    ]
                    self.hill_amplitudes[hill_count] = amp
                    self.hill_radii[hill_count] = rad
                    hill_count += 1

    # lookup each point in (north_locs, east_locs) and return a numpy array of
    # the resulting elevations
    def lookup_datum_array(self, north_locs, east_locs):
        elevations = zeros_like(east_locs)
        for hill_center_loc, hill_amp, hill_rad in zip(
            self.hill_centers, self.hill_amplitudes, self.hill_radii
        ):
            distance_from_hill_to_locs = (
                east_locs - hill_center_loc[0]
            ) ** 2 + (north_locs - hill_center_loc[1]) ** 2
            hill_effect = (
                hill_amp * (hill_rad - distance_from_hill_to_locs) / hill_rad
            )
            hill_effect[hill_effect < 0] = 0
            elevations = elevations + hill_effect
        return elevations

    # lookup a single point (north_loc, east_loc) and return the resulting
    # elevation
    def lookup_datum(self, north_loc, east_loc):
        # print(north_loc, east_loc)
        return self.lookup_datum_array(array([north_loc]), array([east_loc]))[
            0
        ]


class MotionStateBlock(StateBlock):
    delta_p = [0, 0]

    def __init__(self, name):
        StateBlock.__init__(self, 2, name)  # num_states, label

    def generate_dynamics(self, x_hat, time_from, time_to):
        def g(state_estimates):
            states = zeros(2)

            states[0] = state_estimates[0] + self.delta_p[0]
            states[1] = state_estimates[1] + self.delta_p[1]
            return states

        Q = diag(array([DELTA_P_NOISE_STD_DEV, DELTA_P_NOISE_STD_DEV]) ** 2)

        F = zeros((2, 2))

        [Phi, Qd] = first_order_discretization_strategy(
            F, G=eye(2, 2), Q=Q, dt=1
        )
        return StandardDynamicsModel(g, Phi, Qd)

    def receive_aux_data(self, delta_p):
        self.delta_p = delta_p


class AltitudeMapMeasurementProcessor(MeasurementProcessor):
    def __init__(self, name, state_block_labels):
        MeasurementProcessor.__init__(self, name, state_block_labels)

    def generate_model(self, meas, generate_function):
        def h(state_estimates):
            east = state_estimates[0]
            north = state_estimates[1]

            return [self.topology_source.lookup_datum(north, east)]

        # A closed-form solution to calculate the Jacobian of h() does not
        # exist because it is a very nonlinear function. Until
        # SampledMeasurementModel is supported, it is necessary to fill in H
        # with something but these values will not be used by the filter.
        H = zeros((1, 2))
        return StandardMeasurementModel((meas.estimate), h, H, meas.covariance)

    def receive_aux_data(self, topology_source):
        self.topology_source = topology_source


class NumberFormat(float):
    def __repr__(self):
        num = f'{self:.1f}'
        return f'{self:.0f}' if num[-1] == '0' else num


if __name__ == "__main__":
    if len(sys.argv) > 2:
        sys.exit("Usage: particle_filter_example.py [run_time]")
    elif len(sys.argv) == 2:
        run_time = min(int(sys.argv[1]), RUN_TIME)
    else:
        run_time = RUN_TIME
    particle_filter_example(run_time)
