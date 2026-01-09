#!/usr/bin/env python3

from BiasBlock import BiasBlock
from BiasMeasurementProcessor import BiasMeasurementProcessor

from navtk.filtering import StandardFusionEngine, GaussianVectorData

from aspn23_xtensor import to_type_timestamp


def bias_example_with_update():
    '''
    Example showing use of custom state block and measurement processor
    for a simple bias-estimating situation.
    '''
    # Create our state block, measurement processor, and fusion engine
    # instance. All together these act as a navigation filter.
    block = BiasBlock("mybiasblock")
    processor = BiasMeasurementProcessor("myprocessor", "mybiasblock")
    engine = StandardFusionEngine()

    # Add the block and processor to the engine/filter
    engine.add_state_block(block)
    engine.add_measurement_processor(processor)

    # Get and print the initial state estimate and covariance
    out = engine.get_state_block_estimate("mybiasblock")
    out_cov = engine.get_state_block_covariance("mybiasblock")
    print("Initial:")
    print("The state estimate is {}".format(out))
    print("The state covariance is {}\n".format(out_cov))

    # Propagate to 0.1 seconds
    engine.propagate(to_type_timestamp(0.1))

    # Get and print the state estimate and covariance after propagation
    out = engine.get_state_block_estimate("mybiasblock")
    out_cov = engine.get_state_block_covariance("mybiasblock")
    print("After propagation:")
    print("The state estimate is {}".format(out))
    print("The state covariance is {}\n".format(out_cov))

    # Make a measurement at 0.1 seconds of sensed value 1.0 with cov 1.001
    time_validity = to_type_timestamp(0.1)
    measurement_data = [1.0]
    measurement_covariance = [[1.001]]
    raw_meas = GaussianVectorData(
        time_validity, measurement_data, measurement_covariance
    )

    # Update the filter estimate with the measurement
    engine.update("myprocessor", raw_meas)

    # Get and print the state estimate and covariance after the update
    out = engine.get_state_block_estimate("mybiasblock")
    out_cov = engine.get_state_block_covariance("mybiasblock")
    print("After update:")
    print("The state estimate is {}".format(out))
    print("The state covariance is {}".format(out_cov))


if __name__ == "__main__":
    bias_example_with_update()
