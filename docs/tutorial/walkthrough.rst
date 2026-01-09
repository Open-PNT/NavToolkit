.. _rst_walkthrough:

Filter Construction & Usage Walkthrough
=======================================

This section will walk you through building and running a filter. To get a
complete and runnable version of this code with simulated measurements and
data being received from multiple sensors over time, look in the ``examples``
folder of NavToolkit for the ``straight_flight_example.cpp``. The
README has step-by-step instructions on how to run this example.

Setting Up a Filter
-----------------------

The following headers will be included for this walkthrough:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: INCLUDES
   :end-before: END
   :tab-width: 0

And the following ``using`` statements are included for convenience:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: USINGS
   :end-before: END
   :tab-width: 0

The first thing to do is choose the fusion engine and
:class:`~navtk::filtering::FusionStrategy` type that we want and make them.
NavToolkit currently can implement an unscented Kalman filter or an extended
Kalman filter depending on the chosen
:class:`~navtk::filtering::FusionStrategy` type. For now we'll build a fusion
engine which uses an EKF. Since the default
:class:`~navtk::filtering::FusionStrategy` for
:class:`~navtk::filtering::StandardFusionEngine` is EKF, the default
constructor can be used here:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: ENGINE CREATION
   :end-before: END
   :tab-width: 0

The fusion engine we just created is initially empty, containing zero states to
estimate and no way to process any measurements. The next step is to fill in
the fusion engine with the set of states we want. Once the fusion engine has
had :class:`~navtk::filtering::MeasurementProcessor`\s and
:class:`~navtk::filtering::StateBlock`\s added to it, it can act as a
navigation filter.

In NavToolkit, a set of :math:`N` states is represented by a
:class:`~navtk::filtering::StateBlock`. The
:class:`~navtk::filtering::StateBlock` contains details of how those states
propagate forward through time. With NavToolkit we provide a set of premade
:class:`~navtk::filtering::StateBlock`\s. For this walkthrough we'll create a
:class:`~navtk::filtering::Pinson15NedBlock` which contains 15 states that
model the errors of an INS in the North-East-Down (NED) frame:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: BLOCK CREATION
   :end-before: END
   :tab-width: 0

Notice that when we created the block, we had to give it two parameters: a
``label`` (the name of the :class:`~navtk::filtering::StateBlock`, in this case
we're calling it ``"pinson15"``) and a ``model``. Examining the constructor of
:class:`~navtk::filtering::Pinson15NedBlock` we see that the ``label`` is a
unique identifier for this block that we'll use to refer to it later on, and
the ``model`` is an instance of :class:`~navtk::filtering::ImuModel` which
describes the grade of the INS. In the example above we used a preset model for
the ``HG1700`` inertial, but we could just as easily have built a custom
:class:`~navtk::filtering::ImuModel` by passing in the needed parameters
manually.

To add our newly created Pinson block to our fusion engine, we can simply call
the :func:`~navtk::filtering::StandardFusionEngine::add_state_block` method of
the fusion engine:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: ADD BLOCK TO ENGINE
   :end-before: END
   :tab-width: 0

At this point, the fusion engine will add the 15 states from ``block`` to its
internal :class:`~navtk::filtering::FusionStrategy`, initializing them to zero.
If the initial covariance of your states are non-zero, you can tell the fusion
engine what you'd like the Pinson states initialized to. To do so, construct a
`15x15` matrix of the initial :math:`P_0` matrix and pass them into the fusion
engine's
:func:`~navtk::filtering::StandardFusionEngine::set_state_block_covariance`
method:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: SET COVARIANCE
   :end-before: END
   :tab-width: 0

Notice that we passed in the label ``"pinson15"``. This is how the fusion
engine knows that we are updating the covariance of the Pinson states. If our
fusion engine had two blocks named ``"a"`` and ``"b"``, we could choose which
one we wanted to update the covariance of by passing in either ``"a"`` or
``"b"`` here along with the matrix of values. If we wanted to set a block's
estimate (as opposed to its covariance) to something, we could use the
:func:`~navtk::filtering::StandardFusionEngine::set_state_block_estimate`
method in a similar manner as we did here, but passing in a
:type:`~navtk::Vector` of length `15` instead (since our state block has 15
states).

The fusion engine now has a state block loaded and it knows the initial
conditions of the state block. For most state blocks, we would now be ready to
propagate the fusion engine forward, estimating the states in ``"pinson15"`` as
they change over time. However, the
:class:`~navtk::filtering::Pinson15NedBlock` requires a bit of additional
information in order to estimate inertial errors accurately. The errors of an
INS change as a function of the vehicle's motion. In particular, to estimate
the errors the INS will contain, the Pinson block must approximately know the
vehicle's position, rotation, velocity, and specific force. In order to pass
this sort of additional information to the Pinson block, we can call the fusion
engine's
:func:`~navtk::filtering::StandardFusionEngine::give_state_block_aux_data`
method, passing in the label of the receiving
:class:`~navtk::filtering::StateBlock` and the required data.

In order to know if a given state block needs aux data and what sort of aux
data you need to give it, you can consult its documentation. In the case of the
:class:`~navtk::filtering::Pinson15NedBlock` we can see that it requires an aux
data of the type :class:`~navtk::filtering::PinsonAux` which contains specific
force and a :class:`~navtk::filtering::NavSolution`, which is a structure
containing the position, rotation, and current time. Thus we will pass the
current pose and specific force to our Pinson block:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: AUX DATA
   :end-before: END
   :tab-width: 0

Propagating a Filter
------------------------

The filter (via the fusion engine) now has all the information it needs to
propagate forward estimates of inertial error over time. To do so, we call the
fusion engine's :func:`~navtk::filtering::StandardFusionEngine::propagate`
method:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: PROPAGATE
   :end-before: END
   :tab-width: 0

The fusion engine starts out at time ``0.0`` by default, so the call above will
propagate the fusion engine's estimate one second into the future. If we wanted
to get the estimate of our Pinson block after propagation, we could call

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: GET STATE INFO
   :end-before: END
   :tab-width: 0

which will return the 15-element :type:`~navtk::Vector` state estimate and
`15x15` estimate covariance matrix, respectively.

A subsequent call to ``propagate(to_type_timestamp(3.0))`` would propagate the filter to
``t=3`` seconds, ``propagate(to_type_timestamp(5.0))`` to ``t=5`` seconds, and so on. The
selected times do not need to be evenly spaced or integers as the state block
will provide the information to the :class:`~navtk::filtering::FusionStrategy`
on how to update the states for an arbitrary time. Note that the aux data
should be refreshed as often as possible (by calling
:func:`~navtk::filtering::StandardFusionEngine::give_state_block_aux_data`)
when using INS error states, as doing so is critical to get a correct estimate.

Updating a Filter with Measurements
---------------------------------------

By default, the filter knows nothing about how to process measurements coming
from sensors. In order to learn, the fusion engine must be given one or more
:class:`~navtk::filtering::MeasurementProcessor`, which are objects that
contain the algorithms to process measurements coming from a particular sensor
type. NavToolkit contains some off the shelf
:class:`~navtk::filtering::MeasurementProcessor` implementations for common
sensor types, and allows users to design custom ones for specific algorithms by
implementing the :class:`~navtk::filtering::MeasurementProcessor` interface
themselves. For this walkthrough, we'll add a
:class:`~navtk::filtering::DirectMeasurementProcessor`, configured to update
the vehicle's altitude, to the fusion engine. First we build an instance of the
processor:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: MP CREATION
   :end-before: END
   :tab-width: 0

The first parameter is the name (``label``) of the processor itself (so we can
refer to it later on), the second parameter is the name of the state block it
will relate its measurements to, and the third is a :type:`~navtk::Matrix`
which maps the states to the measurement space. In this case, we want the
measurement processor to take the altitude measurements it receives and
update our estimate of the Pinson states, so we pass in the label of the
Pinson states here and a `1x15` :type:`~navtk::Matrix` of zeros except
for a one at the index corresponding to the vertical position state.

Next we add the processor to the fusion engine:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: ADD MP TO ENGINE
   :end-before: END
   :tab-width: 0

At this point the fusion engine has been informed of how to process altitude
measurements from a sensor, so we can proceed with sending a measurement to
the fusion engine:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: UPDATE
   :end-before: END
   :tab-width: 0

We call the fusion engine's :func:`~navtk::filtering::StandardFusionEngine::update` method, which
requires the label of the receiving :class:`~navtk::filtering::MeasurementProcessor` and the
measurement (:class:`~aspn_xtensor::AspnBase`) as parameters.

Thus, in our code above, we told the filter we have a measurement at time ``t=10.0``, with a raw
value of the reference altitude minus ``0.5``, and a measurement covariance of ``BARO_SIGMA``
squared. The ``processor_label`` passed to ``update`` was set to ``"altimeter"`` which is the name
of the measurement processor that was previously added to the fusion engine.

It was necessary to difference the measured altitude against a reference
altitude because we are updating an *error-state* filter in this case. An error
state contains the estimate of the difference between a 'true' value and some
system's best guess at the same value- in this case it is estimating the level
of error of our simulated reference altitude. To update this state, the
:class:`~navtk::filtering::MeasurementProcessor` must be able to form a
*delta measurement*, which is the difference between the measured altitude and
the reference altitude.

When the fusion engine receives the above measurement, it will first note that
the timestamp of ``10.0`` doesn't match the current filter time (remember, we
propagated to ``t=1.0`` previously). The first thing it will do is propagate
the estimate forward to the measurement's time stamp.

Next, it will examine the ``processor_label`` and determine that the correct
measurement processor to process the measurement is the ``"altimeter"``
processor we added earlier. The fusion engine will then pass the measurement to
that processor, which will in turn generate the matrices and functions the
:class:`~navtk::filtering::FusionStrategy` needs to perform the update. After
the call to :func:`~navtk::filtering::StandardFusionEngine::update`, the
filter's current estimate will have incorporated the information in the
measurement, and calls to
:func:`~navtk::filtering::StandardFusionEngine::get_state_block_estimate` and
:func:`~navtk::filtering::StandardFusionEngine::get_state_block_covariance`
will produce the post-update estimate and covariance respectively.

Adding a Bias State
-----------------------

The filter we just set up only contains the 15-state INS error model states.
However, it is common to model a barometric altitude sensor as having a bias
on the measurements it collects. Because NavToolkit is designed to be
modular and (live) pluggable, it is straightforward to modify how
we modeled the baro update without touching other parts of the filter (e.g.
the inertial error states).

The first step is to add a new state block to the fusion engine that contains
a single state representing our baro's bias. We'll model the bias here as a
First-Order Gauss-Markov bias (:class:`~navtk::filtering::FogmBlock`):

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: CREATE BIAS BLOCK
   :end-before: END
   :tab-width: 0

Note that we've given this state block the name ``"baro_bias"`` with a time
constant of ``50``, a state sigma of ``10``, and set it to one state. Now we
add the new bias state block to the fusion engine:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: ADD BIAS BLOCK TO ENGINE
   :end-before: END
   :tab-width: 0

The fusion engine now contains two state blocks: the original
:class:`~navtk::filtering::Pinson15NedBlock` and the
:class:`~navtk::filtering::FogmBlock`. At this point you could set the new
state block's initial estimate and covariance as done with the
:class:`~navtk::filtering::Pinson15NedBlock` above.

We previously added a :class:`~navtk::filtering::DirectMeasurementProcessor` to
the fusion engine, which didn't know anything about a bias state and thus won't
use our new ``"baro_bias"`` state. To rectify that situation, we'll need to add
another :class:`~navtk::filtering::DirectMeasurementProcessor` configured to
update both the altitude and bias state:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: ADD BIAS MP
   :end-before: END
   :tab-width: 0

Originally our filter only contained the 15 states from the
:class:`~navtk::filtering::Pinson15NedBlock`, but now it has an additional bias
state, bringing the total to 16 states. While the first
:class:`~navtk::filtering::MeasurementProcessor` only mapped the third state
(the :class:`~navtk::filtering::Pinson15NedBlock` vertical position state) to
the measurement, this :class:`~navtk::filtering::MeasurementProcessor` also
maps to the 16th state, which is the bias state we added. Also in contrast to
the original processor, note that this one is constructed using a vector of
:class:`~navtk::filtering::StateBlock` labels. The name (``label``) of the new
processor is ``"biased_altimeter"``, which is how we'll refer to this processor
in the future.

We now have a filter that is capable of processing biased altimeter
measurements. When we send a new measurement to the fusion engine, we decide
which measurement processor the measurement is destined for.
For example, if we write:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: UPDATE BIAS
   :end-before: END
   :tab-width: 0

we would be telling the fusion engine that a new biased measurement of altitude
is available at time ``t=11.0`` and that it should be sent to the
``"biased_altimeter"`` measurement processor for processing.
However, if we received an unbiased altimeter measurement at time ``t=15``
(say from some other altimeter sensor that didn't have a bias) we could still
send it to the previously added ``"altimeter"`` processor:

.. literalinclude:: ../src/walkthrough.cpp
   :language: C++
   :start-after: UPDATE #2
   :end-before: END
   :tab-width: 0

All processors added to the fusion engine will remain available for the
lifetime of the fusion engine, which is also true of all state blocks added to
the fusion engine (unless you remove them). Thus you can add blocks
and processors at will to model any number of states or measurements that
you need.

Next Steps
--------------

In this walkthrough we showed the basics of building a filter, propagating it,
and updating it with measurements. In the ``StraightFlightExample`` source
code example, we expand on this walkthrough code to demonstrate a filter that
runs on simulated measurements that are fed over time, with three
different measurement types optionally enabled.

The :ref:`rst_adding_support_cpp` and :ref:`rst_adding_support_python` sections
illustrate how to make your own custom :class:`~navtk::filtering::StateBlock`\s
and :class:`~navtk::filtering::MeasurementProcessor`\s in C++ and Python,
respectively.
