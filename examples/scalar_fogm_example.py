#!/usr/bin/env python3

from numpy import array, zeros, sqrt, remainder
from matplotlib.pyplot import (
    plot,
    figure,
    title,
    xlabel,
    ylabel,
    show,
    legend,
    grid,
)

from navtk.filtering import (
    FogmBlock,
    StandardFusionEngine,
    DirectMeasurementProcessor,
    GaussianVectorData,
)

from aspn23_xtensor import to_type_timestamp


def scalar_fogm_example():
    """
    A simple example creating a filter with 1-state (FOGM dynamics). This
    example demonstrates the FOGM estimate decay back to zero, even though
    measurements are periodically resetting it towards 10
    """
    # Arguments are block name, timeConstant, stateSigma, and num_states
    block = FogmBlock('myblock', 50.0, 10.0, 1)

    # Create a fusion engine which, along with a fusion strategy, measurement
    # processors, and state blocks acts as a navigation filter.
    engine = StandardFusionEngine()

    # Args are label, H, and blockLabel(s)
    processor = DirectMeasurementProcessor(
        'myprocessor', 'myblock', array([[1.0]])
    )
    engine.add_state_block(block)
    engine.add_measurement_processor(processor)

    engine.set_state_block_estimate('myblock', zeros((1,)))
    engine.set_state_block_covariance('myblock', array([[100.0]]))

    # Args are block name, startTime, endTime, dt
    t = [x / 10 for x in range(6000)]
    state = zeros(len(t))
    sigma = zeros(len(t))

    for j in range(len(t)):
        engine.propagate(to_type_timestamp(t[j]))
        if abs(remainder(t[j], 100.0) < 0.001):
            engine.update(
                'myprocessor',
                GaussianVectorData(
                    to_type_timestamp(t[j]), array([10]), array([[100.0]])
                ),
            )

        state[j] = engine.get_state_block_estimate('myblock')[0]
        sigma[j] = sqrt(engine.get_state_block_covariance('myblock'))[0][0]

    figure(1)
    plot(t, state + sigma, color='k', label='Pos Cov Bound')
    plot(t, state - sigma, color='k', label='Neg Cov Bound')
    plot(t, state, color='b', label='State Value')
    xlabel('Time (s)')
    ylabel('Sigma (m)')
    title('Position State, Sigma')
    legend()
    grid()

    figure(2)
    plot(t, state, color='k', label='North Position')
    xlabel('Time (s)')
    ylabel('Value (m)')
    title('Position Solution')
    legend()
    grid()
    show()


if __name__ == "__main__":
    scalar_fogm_example()
