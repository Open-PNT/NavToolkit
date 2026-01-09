#!/usr/bin/env python3

import unittest
from numpy import dot, transpose, eye
from numpy.linalg import inv
from numpy.testing import assert_allclose

from navtk.filtering import (
    StandardModelStrategy,
    LinearizedStrategyBase,
    StandardDynamicsModel,
    StandardMeasurementModel,
    EkfStrategy,
    StandardFusionEngine,
    StateBlock,
)
from aspn23_xtensor import TypeTimestamp


class PyEkfStrategy(StandardModelStrategy, LinearizedStrategyBase):
    def __init__(self):
        LinearizedStrategyBase.__init__(self)
        StandardModelStrategy.__init__(self)

    def propagate(self, dynamics_model):
        self.symmetricize_covariance()
        self.validate_linearized_propagate(
            dynamics_model.Phi, dynamics_model.Qd
        )

        self.estimate = dynamics_model.g(self.estimate)
        self.covariance = (
            dot(
                dot(dynamics_model.Phi, self.covariance),
                transpose(dynamics_model.Phi),
            )
            + dynamics_model.Qd
        )

    def update(self, measurement_model):
        h = measurement_model.h
        H = measurement_model.H
        R = measurement_model.R
        z = measurement_model.z
        P = self.covariance
        estimate = self.estimate
        hx = h(estimate)

        self.validate_linearized_update(H, R, z, hx)

        I = eye(len(P), len(P))
        res = z - hx
        K1 = dot(P, transpose(H))
        K2 = inv(dot(dot(H, P), transpose(H)) + R)
        K = dot(K1, K2)
        p_new = dot(I - dot(K, H), P)
        estimate_new = estimate + dot(K, res)

        self.estimate = estimate_new
        self.covariance = p_new


class EkfStrategySubclass(EkfStrategy):
    last_event = None

    def __init__(self):
        super().__init__()
        self.last_event = '__init__'

    def on_fusion_engine_state_block_added_impl(self, first_index, count):
        super().on_fusion_engine_state_block_added_impl(first_index, count)
        self.last_event = 'stateblock add'


class FusionStrategy(unittest.TestCase):
    def test_python_EkfStrategy(self):
        # Test tolerance, both absolute and relative
        TOL = 1e-14

        # Create strategies
        initial_estimate = [1]
        initial_covariance = [[10]]
        py_strategy = PyEkfStrategy()
        py_strategy.on_fusion_engine_state_block_added(1)
        py_strategy.set_estimate_slice(initial_estimate)
        py_strategy.set_covariance_slice(initial_covariance)

        cpp_strategy = EkfStrategy()
        cpp_strategy.on_fusion_engine_state_block_added(1)
        cpp_strategy.set_estimate_slice(initial_estimate)
        cpp_strategy.set_covariance_slice(initial_covariance)

        # Define propagation model
        def g(x):
            return x * 1.1

        Phi = [[1.1]]
        Qd = [[1]]
        dynamics_model = StandardDynamicsModel(g, Phi, Qd)

        # Propagate
        for ii in range(10):
            py_strategy.propagate(dynamics_model)
            cpp_strategy.propagate(dynamics_model)
        assert_allclose(py_strategy.estimate, cpp_strategy.estimate, TOL)
        assert_allclose(py_strategy.covariance, cpp_strategy.covariance, TOL)

        # Define update model
        z = [1.0]
        H = [[1.0]]
        R = [[2.0]]
        measurement_model = StandardMeasurementModel(z, H, R)

        # Update
        py_strategy.update(measurement_model)
        cpp_strategy.update(measurement_model)
        assert_allclose(py_strategy.estimate, cpp_strategy.estimate, TOL)
        assert_allclose(py_strategy.covariance, cpp_strategy.covariance, TOL)

    def test_python_subclass_of_EkfStrategy_virtual_funcs(self):
        strategy = EkfStrategySubclass()
        self.assertEqual(strategy.last_event, '__init__')
        engine = StandardFusionEngine(TypeTimestamp(0), strategy)
        block = StateBlock(1, "block")
        engine.add_state_block(block)
        self.assertEqual(strategy.last_event, 'stateblock add')


if __name__ == '__main__':
    unittest.main()
