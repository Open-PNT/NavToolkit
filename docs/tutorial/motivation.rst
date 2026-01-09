.. _rst_motivation:

Motivation
==========

Goals
-----

The primary goals of NavToolkit are to:

* Develop a framework for building estimators that is both intuitive enough for
  a beginner/student to use while also being flexible enough to support
  advanced research in sensor integration technology.

* Build an architecture which enables both simulated/post-processing of data
  sets and real-time processing of live sensor data coming from a live sensor
  platform.

* Enable different organizations to easily share code. Because the sensor API
  is fully modular, it is easy for two organizations to independently write
  their own modules for their own sensors and then later add them both to the
  same filter.

* Let users quickly and easily test the effects of different sensor integration
  strategies, filter types, or sensor fidelity by using pluggable modules.

* Allow easy access to the library from Python.

* Support filter deployment on performance-critical projects, such as embedded
  platforms, distributed scalable cloud estimators, and everything in-between.

* Provide a set of off-the-shelf sensor modules for common sensors than can be
  easily plugged into a new filter project. This will eliminate the need to
  reinvent the wheel for common problems like GPS/INS integration or barometric
  aiding.

* Make the core classes abstracted to support usages other than navigation
  (e.g. using a particle filter to estimate the frequency jitter of a phase
  lock loop (PLL)).

What is Plug-and-play?
----------------------

NavToolkit aims to be pluggable in three ways:

* **Pluggable algorithms:** A sensor module conceptually integrates both a
  sensor type (e.g., "relative range") as well as an algorithm for using
  measurements from that sensor (the measurement model). As a result, over time
  a library of many different modules for a single sensor type will be
  developed. For example, there are many different ways to integrate a camera
  into a filter in the literature (e.g., visual odometry, map matching, neural
  networks, etc.). Once a set of modules is developed for camera integration,
  when a new data set is collected all of the different algorithms can be run
  on the data set by simply swapping out modules. This allows for a rapid
  comparison of algorithms' relative performance.

* **Pluggable filters:** Once a sensor module is defined, it can be used with
  any filter which supports the module type. For example, a set of modules
  written for interoperating with
  :class:`~navtk::filtering::StandardFusionEngine` can be used with both an EKF
  and a UKF filter, switching between the two by changing one line of code.
  This allows for users to rapidly explore the effects of filter order without
  unnecessary code rewriting.

* **Pluggable sensors:** We allow for the development of sensor modules which
  work with any sensor from a generic class of sensors. For example, you could
  write a generic GPS module which will work with any GPS provider, regardless
  of the vendor/model of device actually being used.
