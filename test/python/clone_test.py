#!/usr/bin/env python3

import unittest
from navtk.filtering import (
    StateBlock,
    MeasurementProcessor,
    FogmBlock,
    DirectMeasurementProcessor,
)


class PyStateBlock(StateBlock):
    def __init__(self, label):
        StateBlock.__init__(self, 1, label)

    def clone(self):
        return PyStateBlock(self.get_label())


class PyMeasurementProcessor(MeasurementProcessor):
    def __init__(self, label, state_block_label):
        MeasurementProcessor.__init__(self, label, state_block_label)

    def clone(self):
        return PyMeasurementProcessor(
            self.get_label(), self.get_state_block_labels()[0]
        )


class CloneTests(unittest.TestCase):
    def test_clone_cpp_block(self):
        block = FogmBlock('block', 1.0, 1.0, 1)
        self.assertIsInstance(block.clone(), StateBlock)

    def test_clone_cpp_processor(self):
        processor = DirectMeasurementProcessor('processor', 'block', [[1]])
        self.assertIsInstance(processor.clone(), MeasurementProcessor)

    def test_clone_python_block(self):
        block = PyStateBlock('block')
        self.assertIsInstance(block.clone(), StateBlock)

    def test_clone_python_processor(self):
        processor = PyMeasurementProcessor('processor', 'block')
        self.assertIsInstance(processor.clone(), MeasurementProcessor)


if __name__ == '__main__':
    unittest.main()
