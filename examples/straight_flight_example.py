#!/usr/bin/env python3

from numpy import (
    zeros,
    ones,
    eye,
    array,
    arange,
    random,
    sqrt,
    linalg,
    diag,
    concatenate,
)
from matplotlib.pyplot import (
    plot,
    figure,
    xlabel,
    ylabel,
    legend,
    show,
    suptitle,
    subplot,
    grid,
)

from navtk.filtering import (  # noqa
    NavSolution,
    StandardFusionEngine,
    hg1700_model,
    hg9900_model,
    ImuModel,
    Pinson15NedBlock,
    GaussianVectorData,
    DirectMeasurementProcessor,
    apply_error_states,
)
from navtk.navutils import delta_lat_to_north, delta_lon_to_east
from navtk.exampleutils import (
    constant_vel_pva,
    noisy_pos_meas,
    noisy_alt_meas,
    noisy_vel_meas,
)
from navtk.utils import to_inertial_aux

from aspn23_xtensor import TypeTimestamp, to_type_timestamp

# Some convenient constants (in seconds)
MINUTE = 60.0
HOUR = MINUTE * 60.0

# Choose which aiding sources to enable
VELO_ENABLED = True
POS_ENABLED = True
BARO_ENABLED = True

# Turn feedback on/off
FEEDBACK_ENABLED = True

# If the source is enabled, choose the measurement sigma
# in meters
VELO_SIGMA = 10.0
POS_SIGMA = 5.0
BARO_SIGMA = 30.0

# If the source is enabled, this is the interval (sec)
# at which we receive measurements
# (NOTE: needs to be a multiple of dt)
VELO_INTERVAL = 20.0
POS_INTERVAL = 5 * MINUTE
BARO_INTERVAL = 5.0

# Choose interval to apply feedback
FEEDBACK_INTERVAL = 10 * MINUTE

# How long we are filtering for
RUNTIME = 20 * MINUTE

# How often we propagate our solution
dt = 1.0


# A no-op function passed to generate_dynamics to fulfill parameter
# requirements since the Pinson state block doesn't need this information to
# generate its model and because it's not actually running inside a filter.
# Please see the StandardFusionEngine for a proper example of how to define the
# callback function, where it returns the relevant state estimate and
# covariance associated with the given state block labels.
def gen_xhat_p(state_block_labels):
    pass


def straight_flight_example():
    """
    An example filter with an aircraft flying const velocity 1m/s north
    straight and level.  This example has toggleable variables to
    enable/disable velocity aiding, baro aiding, and GPS (3D position)
    aiding, as well as toggleable INS grade and sensor noise levels, with
    all measurements and errors simulated.

    This example is intended to illustrate usage of the pinson15 block and
    how it interacts with various aiding sensor types, as well as the ease
    of rapidly plugging in new aiding sensors into a working filter.
    Inertial mechanization is simulated by adding modeled inertial errors
    to a perfect trajectory.  Feedback to this 'inertial' is optionally
    employed to keep the reference trajectory from becoming so incorrect
    during longer examples that the model breaks down.

    A walk through of this example is available in the official
    documentation.  Please look there for a step-by-step tutorial.
    """
    # Create the engine and add appropriate modules to setup a navigation
    # filter.
    engine = StandardFusionEngine()

    # Choose INS model for the Pinson state block
    model = hg1700_model()

    """
    #Navigation Grade
    model = filtering.hg9900_model()

    #Custom Model
    model = filtering.ImuModel(
        accel_random_walk_sigma = [[1,2,3]],
        gyro_random_walk_sigma = [[1,2,3]],
        accel_bias_sigma = [[1,2,3]],
        accel_bias_tau = [[1,2,3]],
        gyro_bias_sigma = [[1,2,3]],
        gyro_bias_tau = [[1,2,3]]
        )
    """

    # Create our desired state block and add to the engine/filter
    block = Pinson15NedBlock("pinson15", model)
    engine.add_state_block(block)
    engine.get_state_block_names_list()

    # An identical state block to model the true errors for comparison against
    # the filter
    true_error_block = Pinson15NedBlock("trueError", model)

    # Initialize filter uncertainty
    # Units are m, m/s, rad, m/s^2, and rad/s
    s0 = array(
        [
            3,
            3,
            3,
            0.03,
            0.03,
            0.03,
            0.0002,
            0.0002,
            0.0002,
            model.accel_bias_initial_sigma[0],
            model.accel_bias_initial_sigma[1],
            model.accel_bias_initial_sigma[2],
            model.gyro_bias_initial_sigma[0],
            model.gyro_bias_initial_sigma[1],
            model.gyro_bias_initial_sigma[2],
        ]
    )

    P0 = diag(s0**2)
    engine.set_state_block_covariance("pinson15", P0)

    # Create MeasurementProcessors and add to the engine/filter
    # TODO: Re-enable these when aliased versions exist
    hAlt = zeros((1, 15))
    hAlt[0, 2] = 1.0
    engine.add_measurement_processor(
        DirectMeasurementProcessor("altimeter", "pinson15", hAlt)
    )

    hPos = zeros((3, 15))
    hPos[0:3, 0:3] = eye(3)
    engine.add_measurement_processor(
        DirectMeasurementProcessor("gps++", "pinson15", hPos)
    )

    hVel = zeros((3, 15))
    hVel[0:3, 3:6] = eye(3)
    engine.add_measurement_processor(
        DirectMeasurementProcessor("odometer", "pinson15", hVel)
    )

    # Setup our simulation runtime and output variables for plotting
    times = arange(0.0, RUNTIME, dt)
    N = times.size
    out_states = zeros((15, N))
    out_cov = zeros((15, N))
    true_errors = zeros((15, N))

    # Create an initial trajectory point to kick off measurement simulation
    vel = [1, 0, 0]  # m/s
    pos0 = [0.0, 0.0, 0.0]  # This is lat (rad), lon (rad), alt (m).
    Dcm0 = eye(3)  # Attitude DCM
    fNed = [0.0, 0.0, -9.81]  # Measured specific force (m/s^2)
    nav_sol = NavSolution(pos0, vel, Dcm0, TypeTimestamp(0))

    # Generate a simple 'truth' trajectory and initialize the
    # reference/inertial trajectory
    truth = constant_vel_pva(nav_sol, dt, RUNTIME)
    ref_pva = constant_vel_pva(nav_sol, dt, RUNTIME)

    # Generate measurements by adding noise to truth
    pos_meas = noisy_pos_meas(truth, ones(3) * POS_SIGMA)
    vel_meas = noisy_vel_meas(truth, ones(3) * VELO_SIGMA)
    alt_meas = noisy_alt_meas(truth, BARO_SIGMA)

    # Filter loop
    for i in range(0, N):
        if i > 0:
            time = float(times[i])

            # Propagate the 'true' errors forward

            aux = to_inertial_aux(ref_pva[i - 1], fNed, zeros((3)))
            true_error_block.receive_aux_data(aux)

            dyn = true_error_block.generate_dynamics(
                gen_xhat_p, to_type_timestamp(), to_type_timestamp(dt)
            )

            unscaledErrors = concatenate((zeros(9), random.randn(6)))
            true_errors[0:15, i] = (
                dyn.Phi.dot(true_errors[0:15, i - 1].reshape(15))
                + linalg.cholesky(dyn.Qd).dot(unscaledErrors)
            ).reshape(15)

            # The available 'inertial' solution is the true location +
            # simulated inertial errors
            errors = -(true_errors[0:9, i].reshape(-1))
            ref_pva[i] = apply_error_states(truth[i], errors)

            pBlock = engine.get_state_block('pinson15')
            pBlock.receive_aux_data(
                to_inertial_aux(ref_pva[i], fNed, zeros(3))
            )

            engine.propagate(to_type_timestamp(time))

            if BARO_ENABLED and abs(time % BARO_INTERVAL) < 0.0001:
                time_validity = to_type_timestamp(time)
                measurement_data = [ref_pva[i].pos[2] - alt_meas[i]]
                measurement_covariance = [[BARO_SIGMA * BARO_SIGMA]]

                engine.update(
                    "altimeter",
                    GaussianVectorData(
                        time_validity, measurement_data, measurement_covariance
                    ),
                )

            if POS_ENABLED and abs(time % POS_INTERVAL) < 0.0001:

                time_validity = to_type_timestamp(time)
                delta_llh = pos_meas[0:3, i] - ref_pva[i].pos
                measurement_data = [
                    delta_lat_to_north(
                        delta_llh[0], pos_meas[0, i], pos_meas[2, i]
                    ),
                    delta_lon_to_east(
                        delta_llh[1], pos_meas[0, i], pos_meas[2, i]
                    ),
                    -delta_llh[2],
                ]
                measurement_covariance = eye(3) * POS_SIGMA * POS_SIGMA

                engine.update(
                    "gps++",
                    GaussianVectorData(
                        time_validity, measurement_data, measurement_covariance
                    ),
                )

            if VELO_ENABLED and abs(time % VELO_INTERVAL) < 0.0001:
                time_validity = to_type_timestamp(time)
                measurement_data = vel_meas[0:3, i] - ref_pva[i].vel
                measurement_covariance = eye(3) * VELO_SIGMA * VELO_SIGMA

                engine.update(
                    "odometer",
                    GaussianVectorData(
                        time_validity, measurement_data, measurement_covariance
                    ),
                )

            if FEEDBACK_ENABLED and abs(time % FEEDBACK_INTERVAL) < 0.0001:
                true_errors[0:15, i] = (
                    true_errors[0:15, i].reshape(15)
                    - engine.get_state_block_estimate("pinson15")
                ).reshape(15)

                engine.set_state_block_estimate("pinson15", zeros((15,)))

        # Save output after updating
        out_cov[:, i] = engine.get_state_block_covariance(
            "pinson15"
        ).diagonal()
        out_states[:, i] = engine.get_state_block_estimate("pinson15").reshape(
            15
        )

    # Plot results
    state_names = [
        "North Pos Error",
        "East Pos Error",
        "Down Pos Error",
        "North Vel Error",
        "East Vel Error",
        "Down Vel Error",
    ]
    title_names = ["Position Error", "Velocity Error"]
    state_y_labels = ["meters", "meters", "meters", "m/s", "m/s", "m/s"]

    for p_idx in range(0, 6):
        if p_idx % 3 == 0:
            figure()
            suptitle(title_names[int(p_idx / 3)])
        subplot(3, 1, p_idx % 3 + 1)
        if p_idx % 3 == 2:
            xlabel("Time (s)")
        plot(
            times,
            out_states[p_idx, 0:N] - true_errors[p_idx, 0:N],
            label=state_names[p_idx],
            color="k",
        )
        plot(
            times,
            sqrt(out_cov[p_idx, 0:N]),
            label="Filter-Computed 1-Sigma",
            color="b",
        )
        plot(times, -sqrt(out_cov[p_idx, 0:N]), color="b")
        ylabel(state_y_labels[p_idx])
        legend()
        grid()
    show()


if __name__ == "__main__":
    straight_flight_example()
