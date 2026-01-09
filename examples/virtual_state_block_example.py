#!/usr/bin/env python3

from numpy import array

from navtk.filtering import (
    FogmBlock,
    StandardFusionEngine,
    VirtualStateBlock,
    ScaleVirtualStateBlock,
    ChainedVirtualStateBlock,
    FirstOrderVirtualStateBlock,
)


def virtual_state_block_example():
    """
    Quick example showing how to use VirtualStateBlocks (VSB) from python.
    """

    # Create an engine and add 3 sample state blocks that we'll use to
    # demonstrate various types of VirtualStateBlocks. Once this engine is
    # fully configured it acts as a navigation filter.
    my_engine = StandardFusionEngine()

    block = FogmBlock('block', 50.0, 10.0, 1)
    my_engine.add_state_block(block)
    my_engine.set_state_block_estimate('block', array([10.0]))
    my_engine.set_state_block_covariance('block', array([[100.0]]))

    block2 = FogmBlock('block2', 50.0, 10.0, 1)
    my_engine.add_state_block(block2)
    my_engine.set_state_block_estimate('block2', array([10.0]))
    my_engine.set_state_block_covariance('block2', array([[100.0]]))

    block3 = FogmBlock('block3', 50.0, 10.0, 3)
    my_engine.add_state_block(block3)
    my_engine.set_state_block_estimate('block3', array([10.0, 10.0, 10.0]))
    my_engine.set_state_block_covariance(
        'block3', array([[100.0, 0, 0], [0, 100.0, 0], [0, 0, 100.0]])
    )

    # Create the virtual state block and add it to the engine/filter- this one
    # scales the state block named 'block' by 0.5 to get a new state block
    # 'halfblock' See how ScaledVsb is defined below.
    my_vsb = ScaledVsb('block', 'halfblock', 0.5)
    my_engine.add_virtual_state_block(my_vsb)

    # Now we have access to the original 'block' as well as the 'halfblock'
    orig_est = my_engine.get_state_block_estimate('block')
    print(orig_est)
    scaled_est = my_engine.get_state_block_estimate('halfblock')
    print(scaled_est)

    # Other VirtualStateBlocks already implemented in filtering
    # There is a VSB that scales state vectors of a given size
    vec_scale_vsb = ScaleVirtualStateBlock(
        'block3', 'otherscaled', array([0.2, 0.3, 0.4])
    )
    my_engine.add_virtual_state_block(vec_scale_vsb)
    other_scaled = my_engine.get_state_block_estimate('otherscaled')
    print(other_scaled)

    # There is one that allows for 'chaining' of individual VSB's to do
    # multiple transforms
    vec_scale_vsb2 = ScaleVirtualStateBlock(
        'otherscaled', 'doublescaled', array([0.2, 0.3, 0.4])
    )
    chained = ChainedVirtualStateBlock([vec_scale_vsb, vec_scale_vsb2])
    my_engine.add_virtual_state_block(chained)
    double_scaled = my_engine.get_state_block_estimate('doublescaled')
    print(double_scaled)

    # The engine can do this automatically under certain conditions
    vec_scale_vsb3 = ScaleVirtualStateBlock(
        'otherscaled', 'newscaled', array([0.2, 0.3, 0.4])
    )
    my_engine.add_virtual_state_block(vec_scale_vsb3)
    new_scaled = my_engine.get_state_block_estimate('newscaled')
    # Should be the same as 'doublescaled' above
    print(new_scaled)

    # Another just accepts a mapping function fx, and optionally a function
    # that generates the Jacobian of jx (otherwise it is calculated
    # numerically). See some_mapping and map_jac implemented below.
    first_order = FirstOrderVirtualStateBlock(
        'block3', 'first_order', some_mapping, map_jac
    )
    first_order_no_jac = FirstOrderVirtualStateBlock(
        'block3', 'first_order_no_jac', some_mapping, None
    )

    my_engine.add_virtual_state_block(first_order)
    my_engine.add_virtual_state_block(first_order_no_jac)
    first_order_est = my_engine.get_state_block_estimate('first_order')
    first_order_cov = my_engine.get_state_block_covariance('first_order')
    first_order_no_jac_est = my_engine.get_state_block_estimate(
        'first_order_no_jac'
    )
    first_order_no_jac_cov = my_engine.get_state_block_covariance(
        'first_order_no_jac'
    )
    print(first_order_est)
    print(first_order_cov)
    print(first_order_no_jac_est)
    print(first_order_no_jac_cov)


# You can implement a VirtualStateBlock directly- just need a ctor and
# convert(), convert_estimate(), and jacobian() implementations
class ScaledVsb(VirtualStateBlock):
    def __init__(self, current, target, scale):
        # Note: this explicit __init__ call is required by pybind11
        # when implementing inheritance
        VirtualStateBlock.__init__(self, current, target)
        self.scale = scale

    def convert(self, ec, time):
        # Expect a 1 element state, just do scalar multiplication
        ec.estimate = ec.estimate * self.scale
        ec.covariance = ec.covariance * self.scale * self.scale
        return ec

    def convert_estimate(self, x, time):
        return x * self.scale

    def jacobian(self, x, time):
        return array([[self.scale]])


def some_mapping(orig):
    cp = orig
    cp[0] *= 3
    cp[1] *= 4
    cp[2] *= 5
    return cp


def map_jac(x):
    return array([[3.0, 0, 0], [0.0, 4.0, 0.0], [0.0, 0.0, 5.0]])


if __name__ == "__main__":
    virtual_state_block_example()
