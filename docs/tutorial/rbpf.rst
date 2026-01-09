.. _rst_rbpf:

Using the Rao-Blackwellized Particle Filter (RBPF)
======================================================

What is the RBPF?
---------------------

The RBPF (:class:`~navtk::filtering::experimental::RbpfStrategy`) is a
:class:`~navtk::filtering::FusionStrategy` that employs sampling to interrogate
data for system state estimation. This method, commonly called Particle
Filtering, uses many particles to generate state hypotheses that are tested
against observations to achieve an estimate of the true posterior distribution.
In a typical particle filter all states are sampled. The RBPF allows certain
states to be marginalized out and treated as linearized Gaussian states,
reducing the space from which particles must be generated. The 
:class:`~navtk::filtering::experimental::RbpfStrategy` allows the user to
select which states are particle states (nonlinear) and which are marginalized
(linear).


The RBPF is useful for models that have highly nonlinear states, non-Gaussian
distributions, or states that may have multi-modal situations. These nonlinear
states are challenging to estimate using linear and linearized methods like the
Kalman and Extended Kalman Filters respectively. The RBPF operates as a bank of
EKFs, where linear states are propagated and updated with the Kalman Filter
equations, and nonlinear states are propagated and updated exploiting Particle
Filter sampling.

To guarantee filter convergence the Particle Filter requires the number of
particles to satisfy the Law of Large Numbers. This requirement is relaxed in
order to process data in real time. The relaxation is mitigated by a resampling
step that pulls particles from low confidence estimates to higher ones.
Marginalization of the linear states also reduces the dimensionality of the
filter to save processing resources.

The RBPF uses the same :class:`~navtk::filtering::FusionStrategy` interface as
the EKF and UKF with the addition of a step to mark particle states.

Target Problems
-------------------

The RBPF may be used to solve problems with nonlinear states, non-Gaussian
statistics, and multimodal distributions. RBPF performance for these problems
will likely exceed EKF and UKF.

Marking Particle States
---------------------------

The following example shows how to mark particle states.


Setting Up a Fusion Engine With an RBPF Strategy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We'll include the following headers for this example:

.. literalinclude:: ../src/rbpf_walkthrough.cpp
   :language: C++
   :start-after: INCLUDES
   :end-before: END
   :tab-width: 0

From here we can create an RBPF :class:`~navtk::filtering::FusionStrategy` and
create a fusion engine using this strategy.

.. literalinclude:: ../src/rbpf_walkthrough.cpp
   :language: C++
   :start-after: CREATE ENGINE
   :end-before: END
   :tab-width: 0

Adding a State Block
^^^^^^^^^^^^^^^^^^^^^^^^

Next we'll create a block and add it to the fusion engine.

.. literalinclude:: ../src/rbpf_walkthrough.cpp
   :language: C++
   :start-after: ADD BLOCK
   :end-before: END
   :tab-width: 0

Marking Particle States
^^^^^^^^^^^^^^^^^^^^^^^^^^^

States in the RBPF default to standard (linear) Gaussian states. This indicates
that the states are propagated and updated with standard Kalman Filter
operations.

You must provide the state marking function with a list of states to mark as
particle states. Marked states will be treated by the strategy as if they were
states in a Particle Filter while the others are propagated and updated using
EKF equations. This list references the states within the strategy. The first
state in a given state block is 0, the second is 1, etc. The following code
snippet would mark the first two states of the
:class:`~navtk::filtering::Pinson15NedBlock` as particle states.

.. literalinclude:: ../src/rbpf_walkthrough.cpp
   :language: C++
   :start-after: MARK STATE
   :end-before: END
   :tab-width: 0

Particle states may also be marked with jitter levels. Adding jitter is a
filter design approach to mitigate particle degeneration; the condition when
all of the sampling occurs at a single point. With jitter the resampling step
distributes slightly altered copies of the high confidence samples. The amount
of high confidence state alteration is proportional to the jitter scale and
estimated covariance. The following code snippet would mark the first two
states of the :class:`~navtk::filtering::Pinson15NedBlock` as particle states
and assign jitter scaling of 0.075.

.. literalinclude:: ../src/rbpf_walkthrough.cpp
   :language: C++
   :start-after: MARK STATE WITH JITTER
   :end-before: END
   :tab-width: 0

Tuning the Rao-Blackwellized Particle Filter
------------------------------------------------

The RBPF implementation has three user parameters to customize the filter
design:

1. Number of Particles
2. Single Jacobian Mode
3. Resampling Threshold

Number of Particles
^^^^^^^^^^^^^^^^^^^^^^^

The number of particles is a critical parameter; if the number is too low
then the state may be undersampled and experience a 'lost lock' condition
requiring the filter to be reinitialized. If the number is too large then
the processing may exceed time budgets. Computational time increases
approximately linearly with the number of particles. Higher dimensional
filtering problems will require more particles than low dimensional problems.
The number of particles is passed in as a parameter in one of the
:class:`~navtk::filtering::FusionStrategy` constructors but defaults to 100.

The number of particles can also be set after construction.

.. literalinclude:: ../src/rbpf_walkthrough.cpp
   :language: C++
   :start-after: SET NUM PARTICLES
   :end-before: END
   :tab-width: 0

Single Jacobian Mode
^^^^^^^^^^^^^^^^^^^^^^^^

The RBPF is designed to employ an independent EKF for each particle. This
design is a benefit for challenging problems but may be an unnecessary
computational burden for less demanding cases. For less demanding situations we
may use a
single Jacobian matrix, with linearization around the global mean, applied to
all particles instead of each particle innovating its own. The computational
savings may be significant depending on the number of particles and state model
size.

The user can select which mode to operate with using a parameter in one of the
constructors.

The single Jacobian mode can also be set after construction.

.. literalinclude:: ../src/rbpf_walkthrough.cpp
   :language: C++
   :start-after: SET CALC SINGLE JACOBIAN
   :end-before: END
   :tab-width: 0

The default mode for the single Jacobian is false. Setting it to true uses a
single Jacobian applied to each particle which provides a significant
computational savings.

Resampling Threshold
^^^^^^^^^^^^^^^^^^^^^^^^

Particle Filtering may fail to sample the true state and diverge. The Particle
Filter uses the sum of the normalized particle weights to detect a potentially
failed condition in order to remedy by reinitializing particle states. The user
may set the resampling threshold for the reinitialization decision. Low
threshold values express a confidence in the filter to maintain lock, whereas
high values are more prone to reinitialization. The resampling threshold can be
set using a parameter in one of the constructors.

The resampling threshold can also be set after construction.

.. literalinclude:: ../src/rbpf_walkthrough.cpp
   :language: C++
   :start-after: SET RESAMPLING THRESHOLD
   :end-before: END
   :tab-width: 0

The default value is 0.75 and the :class:`~navtk::filtering::FusionStrategy`
will throw an error if the number is not between 0 and 1.

Other Common Tuning Practices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is very common in Particle Filtering applications to increase either the
dynamics noise, denoted in NavToolkit as the state block **Q** matrix,
and/or the measurement noise, denoted in a measurement processor as the
**R** matrix.
This process is sometimes called roughening in the literature. This tuning is
not built into the RBPF, but instead would need to be modified directly in the
provided state blocks and measurement processors.

.. TODO: Add a link/reference to the RBPF example when it exists  ..
