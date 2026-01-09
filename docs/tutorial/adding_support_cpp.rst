.. _rst_adding_support_cpp:

Adding State/Sensor Support (C++)
=================================

While NavToolkit has a few measurement processors and state blocks available
off the shelf, you'll most likely want to build your own for custom states or
sensors. This section will discuss the process of creating one of each of
these in C++. For more information on creating a state block or measurement
processor from Python, see :ref:`rst_adding_support_python`.

Creating the StateBlock
---------------------------

The :class:`~navtk::filtering::StateBlock` represents a block of one or more
states, along with their dynamics over time. To create a new state block,
simply subclass :class:`~navtk::filtering::StateBlock` with your own
implementation.

When a fusion engine is asked to
:func:`~navtk::filtering::StandardFusionEngine::propagate` forward its
estimates in time, it will in turn ask each of the state blocks that have been
added to it for information on how the block's states should be propagated. It
does this by calling the block's
:func:`~navtk::filtering::StateBlock::generate_dynamics` function, which
produces the information the fusion engine needs to provide to the
:class:`~navtk::filtering::FusionStrategy` to perform the propagation.

Let's take a look at an example block that we might write. Suppose we wanted a
:class:`~navtk::filtering::StateBlock` with one state which has constant
dynamics. Then we might write ``BiasBlock.hpp``:

.. literalinclude:: ../../examples/bias_with_update/BiasBlock.hpp
   :language: C++

The :class:`~navtk::filtering::StateBlock` constructor takes in two parameters:
the number of states you have and the label of your block. Here we pass it ``1,
label`` for those parameters respectively. We receive the ``label`` from the
user in our own constructor and pass it through to the super constructor.

You'll also notice our implementation of :class:`~navtk::filtering::StateBlock`
has two methods implemented:

* :func:`~navtk::filtering::StateBlock::clone`:
  This is a method which is generally only used for advanced
  functionality, but it can also be used as a convenient way to duplicate
  :class:`~navtk::filtering::StateBlock`\s. Note that since it returns a copy
  of itself with the same label, we would either have to change the label of
  the cloned :class:`~navtk::filtering::StateBlock` or add it to a different
  fusion engine.

* :func:`~navtk::filtering::StateBlock::generate_dynamics`:
  When the fusion engine needs to propagate forward the states this block
  represents, it will call this method. The method is given the current state
  estimate (``xhat``), the time we are propagating from, and the time we are
  propagating to. It is required to return an instance of
  :class:`~navtk::filtering::StandardDynamicsModel`, which is a structure
  containing all the information the fusion engine needs to calculate the
  propagated estimates. Recalling the formulation of the problem from the
  :ref:`rst_introduction`, dynamics contains the function ``g`` (which returns
  the matrix product of ``xhat`` and a `1x1` identity matrix), the first-order
  Taylor series approximation of ``g`` named :math:`\Phi`, and the covariance
  matrix of :math:`v_k` named :math:`Q_d`. In the code above, we fill out the
  dynamics struct with a function ``g`` that returns its input, :math:`\Phi`
  as the identity matrix, and a covariance equal to the elapsed time on the
  dynamics noise.

Other things to note:

* Our ``BiasBlock`` requires a unique label associated with it. This is
  important as its uniqueness makes it possible to query the fusion engine for
  blocks by label. If a fusion engine has multiple ``BiasBlock``\s added to it
  they should each have a different label.

* All of the matrices :func:`~navtk::filtering::StateBlock::generate_dynamics`
  inputs or outputs are *only* with respect to the states that belong to the
  :class:`~navtk::filtering::StateBlock`. For example, the ``xhat`` argument is
  guaranteed to be a `1`-length vector consisting of *only* the bias state,
  even if other states are in the fusion engine from other sensors. This
  approach makes it easier to write modules in isolation, as adding new modules
  to a fusion engine won't affect the input/output of the individual modules.
  This also means that it is generally recommended that any states that have
  coupled dynamics be a part of the same :class:`~navtk::filtering::StateBlock`.

* If our :class:`~navtk::filtering::StateBlock` required data to be brought in
  through a side channel, we could have implemented
  :func:`~navtk::filtering::StateBlock::receive_aux_data`


Creating the MeasurementProcessor
---------------------------------

A :class:`~navtk::filtering::MeasurementProcessor` contains the information a
:class:`~navtk::filtering::FusionStrategy` needs to process a specific type of
raw measurement that it receives. When someone calls a fusion engine's
:func:`~navtk::filtering::StandardFusionEngine::update` method, the fusion
engine will look at the ``processor_label`` parameter and try to find a
:class:`~navtk::filtering::MeasurementProcessor` with the same label that it
can send the measurement to for processing. If it finds one, the fusion engine
will send the measurement to the processor, calling its
:func:`~navtk::filtering::MeasurementProcessor::generate_model` method which
returns the information the :class:`~navtk::filtering::FusionStrategy` needs to
incorporate the measurement. The processor's job is then to receive the
measurement from the fusion engine and extract the information from it that the
:class:`~navtk::filtering::FusionStrategy` needs to perform an update.

Let's look at an example of a simple processor implementation. We'll create a
processor object for a measurement of the bias state, called
``BiasMeasurementProcessor.hpp``:

.. literalinclude:: ../../examples/bias_with_update/BiasMeasurementProcessor.hpp
   :language: C++

Implementing a :class:`~navtk::filtering::MeasurementProcessor` subclass
requires you to implement two methods:

* :func:`~navtk::filtering::MeasurementProcessor::clone`:
  This is a method which is generally only used for advanced
  functionality, but it can also be used as a convenient way to duplicate
  :class:`~navtk::filtering::MeasurementProcessor`\s. Note that since it
  returns a copy of itself with the same label, we would either have to change
  the label of the cloned :class:`~navtk::filtering::MeasurementProcessor`\s or
  add it to a different fusion engine.

* :func:`~navtk::filtering::MeasurementProcessor::generate_model`:
  This method is passed in the measurement coming from the sensor along with
  the current snapshot of the :class:`~navtk::filtering::FusionStrategy`'s
  estimate and estimate covariance. It is required to produce the processed
  measurement (:math:`\mathbf{z}`) and measurement model
  (:math:`\mathbf{h,H,R}`) which will be be passed into the
  :class:`~navtk::filtering::FusionStrategy`. In our example, we'll not need to
  process the measurement further so we'll simply pass through the raw
  measurement.

Other things to note:

* Our ``BiasMeasurementProcessor`` requires a label to uniquely identify it, as
  well as an array of labels of :class:`~navtk::filtering::StateBlock`\s it
  relates to. When :func:`~navtk::filtering::MeasurementProcessor::generate_model`
  is called, its input/output will be the set of concatenated states from the set
  of states in this list. This allows measurement processors to be written
  independently of the other sensors in the fusion engine, but still be able to
  relate the measurement to any states as needed.

* If our ``BiasMeasurementProcessor`` required data to be brought in through a
  side channel, we could have implemented
  :func:`~navtk::filtering::MeasurementProcessor::receive_aux_data`.

Using the StateBlock and MeasurementProcessor with a Fusion Engine
---------------------------------------------------------------------

Once we have a library of modules for the states and measurements we are
interested in, using them with a fusion engine is simple. Consider the file
``bias_example_with_update.cpp``:

.. literalinclude:: ../../examples/bias_with_update/bias_example_with_update.cpp
   :language: C++

If you'd like to try this example out you can find it (as well as the
:class:`~navtk::filtering::StateBlock` and
:class:`~navtk::filtering::MeasurementProcessor`) in the
``examples/bias_with_update`` directory. It can be compiled and run by:

.. code-block:: bash

   ninja -C build run_bias_example_with_update
