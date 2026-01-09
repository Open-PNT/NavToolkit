.. _rst_introduction:

Introduction
================

What is NavToolkit?
-----------------------------

NavToolkit (navtk) is a modular navigation software library, designed to assist
users in the creation of navigation filters in an efficient, pluggable, agile
manner. To do so, NavToolkit exposes an API for developing pluggable filter
components:

* **State Blocks:** A stochastic model for a set of unknown quantities of
  interest, represented in state-space form.

* **Measurement Processors:** A description of the error model of a measurement
  from a sensor and instructions on how to relate the measurement to the state
  blocks.

* **Fusion Strategies:** Contains the state estimates of the filter and uses
  models provided by :class:`~navtk::filtering::StateBlock`\s and
  :class:`~navtk::filtering::MeasurementProcessor`\s to propagate and update
  the state estimates.

* **Fusion Engines:** An online iterative sensor fusion engine which can take
  in a list of one or more :class:`~navtk::filtering::StateBlock`\s and
  :class:`~navtk::filtering::MeasurementProcessor`\s as well as the raw measurements
  (as :class:`~aspn_xtensor::AspnBase`\s) from a sensor and produce an
  estimate of all the states in every :class:`~navtk::filtering::StateBlock`
  at current, past, and (via error-propagation) future times. It is the master
  component which the user interacts with. It does bookkeeping, passes
  measurements to :class:`~navtk::filtering::MeasurementProcessor`\s, and the
  models from :class:`~navtk::filtering::StateBlock`\s and
  :class:`~navtk::filtering::MeasurementProcessor`\s to the
  :class:`~navtk::filtering::FusionStrategy`.


To build a filter, you select a fusion engine, (e.g.
:class:`~navtk::filtering::StandardFusionEngine`), a
:class:`~navtk::filtering::FusionStrategy` (e.g.
:class:`~navtk::filtering::EkfStrategy`), one or more
:class:`~navtk::filtering::StateBlock`\s (to represent all the states modeled
by your filter), and one or more
:class:`~navtk::filtering::MeasurementProcessor`\s (one for each kind of
measurement you will be receiving). These components assembled together
comprise your filter.  The fusion engine which coordinates the components of
the filter is then able to accept measurements from the sensors and your filter
can start estimating your chosen states (jump straight to the
:ref:`rst_adding_support_cpp` section to see an example).

How is NavToolkit Different than AFIT Scorpion or pntOS?
------------------------------------------------------------------

For an in-depth explanation of the differences between these systems, please
see the :ref:`rst_faq`.

Formal Description
----------------------

NavToolkit is designed to estimate a set of time-varying values
:math:`\mathbf{x}` given a set of observations :math:`\mathbf{z}` which contain
information about :math:`\mathbf{x}`. Let :math:`\mathbf{x}_k` be the
`Mx1` vector representing the value of :math:`\mathbf{x}` at time
:math:`t_k` and :math:`\mathbf{z}_k` be the `Nx1` set of observations
collected at :math:`t_k`. Then NavToolkit assumes that the way
:math:`\mathbf{x}` changes from one time epoch to the next is well-modeled by

.. math::

    \mathbf{x}_{k}=\mathbf{g}(\mathbf{x}_{k-1})\mathbf{+w}_{k},\ \ \
    \mathbf{w}_{k}\overset{\mathrm{iid}}{\sim}N(0,\sigma_{w})

where :math:`\mathbf{g}` is the *discrete-time propagation function*, and
:math:`\mathbf{w}` is a  white Gaussian noise source. It also assumes that the
observations are related to the state vector :math:`\mathbf{x}` by

.. math::

    \mathbf{z}_{k}\mathbf{=h}(\mathbf{x}_{k})\mathbf{+v}_{k},\ \ \
    \mathbf{v}_{k}\overset{\mathrm{iid}}{\sim}N(0,\sigma_{v})


where :math:`\mathbf{h}` is the *measurement model function*, and
:math:`\mathbf{v}` is a white Gaussian noise source.

Estimators that are able to estimate problems modeled as above are known as
*filters*. NavToolkit's goal is to build filters that estimate
:math:`\mathbf{x}` at :math:`t_k` (:math:`\mathbf{x}_{k})` given the
measurement at :math:`t_k` (:math:`\mathbf{z}_{k}`) along with all prior
received measurements (:math:`\mathbf{z}_{k-1},\ldots)`. As measurements come
in, we can continually refine our estimate of :math:`\mathbf{x}` using the new
information, calculating a new estimate of :math:`\mathbf{x}_{k+1}` when the
measurement :math:`\mathbf{z}_{k+1}` is received, and so forth. The primary
task a user of NavToolkit must complete is defining and describing
:math:`\mathbf{g},\mathbf{w},\mathbf{h}`, and :math:`\mathbf{v}` for their
particular problem of interest.

In the special case of a linear model, these equations can be rewritten as:

.. math::

    \mathbf{x}_{k}=\mathbf{\Phi}_{k-1}\mathbf{x}_{k-1}\mathbf{+w}_{k},\ \ \
    \mathbf{w}_{k}\overset{\mathrm{iid}}{\sim}N(0,\sigma_{w})

where :math:`\mathbf{\Phi}_{k-1}` is the Jacobian matrix of
:math:`\mathbf{g}(\mathbf{x}_{k-1})`, and the covariance of
:math:`\mathbf{w}_{k}` is :math:`\mathbf{Qd}_{k}` (the discrete-time process
noise covariance matrix), defined as :math:`\mathbf{Qd}_{k}= E[\mathbf{w}_{k}
\mathbf{w}_{k}^{T} ]`.

.. math::

    \mathbf{z}_{k}=\mathbf{H}_{k}\mathbf{x}_{k}\mathbf{+v}_{k},\ \ \
    \mathbf{v}_{k}\overset{\mathrm{iid}}{\sim}N(0,\sigma_{v})

where :math:`\mathbf{H}_{k}` is the Jacobian matrix of
:math:`\mathbf{h}(\mathbf{x}_{k})`, and the covariance of
:math:`\mathbf{v}_{k}` is :math:`\mathbf{R}_{k}` (the measurement noise
covariance matrix), defined as :math:`\mathbf{R}_{k}= E[\mathbf{v}_{k}
\mathbf{v}_{k}^{T} ]`.

Targeted Problems
---------------------

NavToolkit aims to be flexible enough to solve a large portion of real-world
complimentary navigation problems, including:

* Classic GPS/INS fusion.
* Problems with highly non-linear :math:`\mathbf{h}` functions, solved via
  Monte-Carlo methods.
* Simultaneous Localization and Mapping (SLAM) problems, including live state
  insertion, deletion, and maintenance.
* Post-processed non-causal solutions, including forward/backward smoothing.
* Both inertial-present (error state) and inertial-free (direct state) sensor
  fusion approaches.
