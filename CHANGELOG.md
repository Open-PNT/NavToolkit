# Changelog

All major changes to the project will be annotated in this file.

## v2.0 - 2024-05-02

### Added

- `navtk::magnetic` namespace to Python bindings
- `navtk::inertial::AlignBase::motion_needed()` added to query whether motion is needed for alignment
- An outlier detection algorithm in `navtk::utils::OutlierDetection` and related classes
- `navtk::filtering::StandardFusionEngine::give_virtual_state_block_aux_data()` and
  `navtk::filtering::VirtualStateBlock::receive_aux_data()` and `navtk::filtering::VirtualStateBlock::convert_estimate()`
- `navtk::filtering::VirtualStateBlockManager::get_virtual_state_block()` using a label
- `get_virtual_state_block_target_labels()` to `navtk::filtering::StandardFusionEngine` and to
  `navtk::filtering::VirtualStateBlockManager`
- `has_virtual_state_block()` to `navtk::filtering::StandardFusionEngine` using a label
- `remove_virtual_state_block()` to `navtk::filtering::StandardFusionEngine` and to
  `navtk::filtering::VirtualStateBlockManager`
- `navtk::get_time()` which extracts a timestamp from an `aspn_xtensor::AspnBase` object
- Instructions on how to set up a Python Virtual Environment to install Python binding dependencies
- Extra clock choices to `navtk::filtering::ClockBiasesStateBlock`
- Ability for `navtk::inertial::ManualAlignment` to override the default IMU model
- `navtk::inertial::BufferedImu` now warns if a measurement from an unsupported IMU type is received
- Additional reference frame arguments to a number of measurement processor constructors
(e.g. `navtk::filtering::Attitude3dMeasurementProcessor`) and warnings regarding frame mismatches.

### Breaking Changes

- Removed `navtk::navutils::tilt_to_rpy()`
- Removed deprecated `navtk::navutils::rad_to_meter()` function. Please use delta_lon_to_east() and
  delta_lat_to_north() or meridian_radius() and transverse_radius() instead.
- Removed deprecated `BufferedInertial` class. Please use `BufferedImu` instead.
- Replaced `navtk::aspnplaceholder` and `navtk::aspn3placeholder` with ASPN 2023.1 C++ generated
  code based on the ASPN 2023.1 ICD
- The three `navtk::filtering::Velocity*dMeasurementProcessor` classes were replaced by
  `navtk::filtering::VelocityMeasurementProcessor`
- The two `navtk::filtering::DeltaPosition*dMeasurementProcessor` classes were replaced by
  `navtk::filtering::DeltaPositionMeasurementProcessor`
- `navtk::filtering::DirectionToKnownFeature3dMeasurementProcessor` was replaced by
  `navtk::filtering::DirectionToPoints3dMeasurementProcessor`
- `navtk::filtering::OverhauserMagnetometerMeasurementProcessor` was replaced by
  `navtk::filtering::MagneticFieldMagnitudeMeasurementProcessor`
- `navtk::filtering::PositionVelocityMeasurementProcessor` was replaced by
  `navtk::filtering::PositionVelocityAttitudeMeasurementProcessor`
- Removed `navtk::filtering::DimensionTag` enum
- Replaced `navtk::filtering::Time` with `aspn_xtensor::TypeTimestamp`, removed `navtk::filtering::TimeStandard`
- Replaced public property `navtk::filtering::StateBlock::num_states` with function
  `navtk::filtering::StateBlock::get_num_states()`
- Removed `navtk::filtering::StandardFusionEngine::give_fusion_strategy_aux_data()` and
  `navtk::filtering::FusionStrategy::receive_aux_data()`
- Replaced several auxiliary data containers from `navtk::filtering` with ASPN 2023.1 objects:
  `AltitudeAux`, `Attitude1dAux`, `AuxData`, `DeltaPosition1dAux`, `DeltaPosition2dPolarAux`,
  `GeodeticPosition3dAux`, `GpsEphemerisAux`, `MountingAux`, `PinsonAux`
- Replaced `navtk::geospatial::ElevationReference` with ASPN 2023.1 object
  `aspn_xtensor::AspnMeasurementAltitudeReference`
- Replaced `navtk::gnssutils` containers with ASPN 2023.1 objects: `GnssObservations`, `GpsTime`,
  `IonoData`, `SvInfo`, `SvPosInfo`
- Replaced `navtk::inertial` containers with ASPN 2023.1 objects: `CalculatedForces`, `ForceAndRate`
- All `receive_aux_data()` functions and functions that forward aux data now take a vector of ASPN
  2023.1 objects
- Replaced `navtk::filtering::Measurement` with `aspn_xtensor::AspnBase`.  As a consequence,
  `navtk::filtering::StandardFusionEngine::update()` now requires a label argument.
- Removed class template from `navtk::filtering::PairedPva`
- `navtk::geospatial::get_shared()` now returns a shared pointer instead of a reference
- Removed `navtk::filtering::TimestampedData` class, added time of validity to containers as needed
- Moved `navtk::filtering::GenXhatPFunction` declaration from MeasurementProcessor.hpp to new file
  GenXhatPFunction.hpp
- `navtk::filtering::StateBlock::generate_dynamics()` no longer accepts an estimate, but rather a
  `GenXhatPFunction` as an argument
- `setup.py` is removed; building and installing the Python bindings can still be done through Meson
- `navtk::geospatial::GdalRaster()` signature has changed to remove elevation reference arguments
- `set_output_vertical_reference_frame()` removed from `navtk::geospatial::Raster` and `navtk::geospatial::Tile`

### Other Changes

- Fixed a bug in `navtk::gnssutils::assemble_obs_ephem()` which used incorrect PRNs
- Fixed bug where it was possible to pass an invalid capacity to `navtk::utils::RingBuffer`
  constructor, causing undefined behavior
- Bug fixes and cleanup of `navtk::magnetic::MagnetometerCalibration` and related classes
- `navtk::filtering::StandardFusionEngine::generate_x_and_p()` now accepts virtual state block
  labels as well as real state block labels
- `navtk::filtering::StandardFusionEngine::get_cross_term_covariance()` now accepts virtual state
  block labels as well as real state block labels
- Fixed a bug in a couple of measurement processors which were treating vectors like matrices,
  potentially causing undefined behavior
- Updated subprojects to newer versions
- Documentation updates

## v1.4 - 2023-06-08

### Added

- Iterator begin() and end() functions for `MeasurementBufferBase`
- Several additional Python bindings
- A GPS/INS loosely couple flight example written in Python
- Additional ASPN placeholders
- Support for Python 3.11

### Changed

- `MeasurementBuffer` has moved from the `navtk::filtering::experimental` namespace to the
  `navtk::filtering` namespace.
- Generalized `NonlinearAltitudeProcessor` to work for both tightly-coupled and loosely-coupled cases
- Fixed a bug where `CoarseDynamicAlign` used an incorrect estimated time difference
- Fixed a bug in the IMU movement detector that would cause it to stay stuck in the INVALID state
  under certain conditions
- Updated to LCM version 1.5
- Now using a special branch of Pybind11 that allows passing/returning smart pointers (the `smart_holder`
  branch); Pybind11 installed on the host system will no longer be used
- Fixed a down position error bias in the tightly coupled ground example
- Fixed an edge case in `BufferedPva::calc_force_and_rate()` which returned the wrong values if the
  buffer is full
- Fixed a bug in `GdalRaster` where it would reference a tile that was already cleared
- Fixed a bug in `ManualHeadingAlignment` delta-time calculation
- Documentation updates

## v1.3 - 2022-10-13

### Added

- A new namespace `navtk::magnetic` holding magnetic conversion and calibration functions
- A Dead Reckoning State Block
- A formatting target in Meson for Python files: `format_py`

### Changed

- Fixed implicit lossy conversions between `double` and the `navtk::filtering::Time` class
- The `navtk::filtering::Time::operator double` is now deprecated.  Please use `navtk::filtering::Time::to_double()`.
- LCM is now a subproject and no longer requires being installed as a system dependency
- Changed python extension module version to follow PEP440
- Fixed bug involving `calc_sv_azimuth` in `gnssutils`
- Documentation updates

## v1.2 - 2022-06-07

### Added

- Added placeholder classes for ASPN3 datatypes. However, the codebase has not fully incorporated
  them and ASPN2 is still in use throughout.
- Added the `BufferedIns` class which is used to work with `PositionVelocityAttitude` (PVA) outputs
  of an inertial navigation system, as opposed to the `BufferedImu` class which generates a PVA
  by integrating delta velocity and rotation measurements. Both share a common base class,
  `BufferedPva`. Each child class now implements an `add_data` method for incorporating new
  measurements. This may be used in place of the `mechanize` function in the case of `BufferedImu`.
- Added support for using `PositionVelocityAttitude` messages as sources of position information
  in alignment algorithms.
- Added a zero-velocity update (ZUPT) measurement processor, `ZuptMeasurementProcessor`.
- Added an ASPN2 placeholder message for `Heading` measurements.
- Added Python bindings for WGS-84 constants.

### Changed

- Deprecated `BufferedInertial`. Please use `BufferedImu` instead.
- Added support for Ubuntu 22.04 Jammy, and dropped support for 18.04 Bionic.
- Updated Fedora support from 34 to 35.
- Switched pybind subproject to an upstream branch to reduce object slicing issues (issue #138).
- Switched time representation in `BaronavMeasurementProcessor` from `double` to `Time` (issue #627).
- Fixed memory overflow in `GdalSource` (issue #685).
- Changed gravitation and rotation rate constants in use in various `Gnss` related
  `MeasurementProcessors` (issue #551, see also MR !176).
- Changed signature of `BaronavMeasurementProcessor`, `NonlinearAltitudeMeasurementProcessor`,
  `GdalRaster` and `GeoidUndulationSource` constructors to incorporate path to geoid file.

## v1.1 - 2022-02-28

### Added

- Added tutorial documentation page for virtual state blocks.
- Added `particle_filter_example.py`, an experimental example which runs the RBPF as purely a
  particle filter, using simulated data to propagate a 2-state (2D horizontal position) filter via a
  linear model (delta position measurements with white Gaussian noise errors) and update via a very
  nonlinear model (simulated altitude measurements compared to a simulated elevation map lookup).
- Added `SimpleElevationProvider` and `ElevationSource` as abstract bases for any elevation-related
  data classes. These classes currently allow for elevation frame conversions, specifically
  converting between height-above-ellipsoid (HAE) and mean-sea-level (MSL), also known as
  height-above-geoid.
- Added a dynamic inertial alignment algorithm, which uses an initial stationary period to estimate
  inertial sensor biases followed by a dynamic period where multiple possible alignments initialize
  EKFs that are run simultaneously until one can be chosen as the best alignment.
- Added `BaronavMeasurementProcessor`, a measurement processor that wraps a 4-state stand-alone
  BaroNav filter into a measurement processor that can receive barometer data and return geodetic
  position 3D measurements.
- Added `MovementDetector` detector and supporting classes, which uses a voting scheme to leverage
  multiple movement detectors (currently position and inertial measurements supported). This is
  currently used by the dynamic inertial alignment algorithm but also has good potential for
  detecting stationary periods when a zero-velocity update (ZUPT) could be applied.
- Added custom comparison operators to the `Time` class.

### Changed

- Deprecated `rad_to_meter`. Please use `delta_lat_to_north` and `delta_lon_to_east` instead.

## v1.0 - 2022-01-18

### Added

- Added abstract `SpatialMapdataProvider` class to support future, more advanced geospatial
  algorithms, and a simple implementation, `SimpleProvider` (issue #578).
- Added experimental classes to support a future BaroNav implementation, including:
  - `NonlinearAltitudeProcessor`, a measurement processor that generates a non-linear map-based
  altitude measurement model for updates to a 4-state, particle-based altitude map-matching
  algorithm, intended for ground vehicle use (issue #605).
  - `SampledFogmBlock`, a state block whose propagation function `g` propagates a First-Order Gauss
  Markov (FOGM) process.
  - `BaronavMotionBlock`, a state block whose propagation function `g` propagates local-level
  horizontal position states and a heading bias state using 1-D delta-position and heading
  measurements.
  - `MeasurementBuffer` and `MeasurementBuffer3d`, buffer classes for storing and retrieving sensor
  measurements necessary for propagating using `BaronavMotionBlock`.
- Added `StandardFusionEngine` interface class to support pntOS (issue #589).
- Added aux data implementations (`Attitude1dAux`, `DeltaPosition1dAux`, `GeodeticPosition3dAux`) to
  support future a BaroNav implementation.
- Added `BasicInsAndFilter`, a sort of "minimum viable" GPS/INS filter to support a future dynamic
  alignment implementation.
- Added `solve_wahba_davenport`, an alternative algorithm for solving Wahba's problem using
  Davenport's q-Method.
- Added new `PseudorangeType`s: `PL1`, `PL2`, `ML1`, `ML2`, `ALT`, `CAL1_PYL2_COMBINATION`,
  `ML1_ML2_COMBINATION` (issue #616).
- The generated documentation is now hosted at `https://pntos.is4s.io/navtk/`.
- `flexiblas` is now supported as an alternative to `openblas`.
- Added `+` and `-` arithmetic operators to `Time`.
- Added `Attitude3dMeasurementProcessor`, which updates NED tilt-error states using a `Attitude3d`
  measurement.
- Now supports installing the python bindings via `setup.py`.
- Added an inertial alignment algorithm, `ManualAlignment`, which can provide an inertial with an
  alignment given a combination of user-provided position, velocity, or attitude and position,
  velocity, or attitude taken from sensor measurements.
- Added a tightly-coupled GPS measurement processor which updates position and velocity states using
  pseudorange and Doppler measurements.
- Inertial mechanization now supports altitude aiding.
- Added `GeoidUndulationSource`, which does database lookups to return the difference between the
  ellipsoid and geoid at a given latitude and longitude (issue #378). Also added related
  `geoid_minus_ellipsoid`, `hae_to_msl`, `msl_to_hae` conversion functions to `navutils`.
- Added `BarometricPressure` to ASPN placeholder classes.
- Now generates `navtoolkit.hpp`, a meta-header that, when included, includes all headers in
  NavToolkit (issue #528).
- `navtoolkit_python_dep` to support projects that want to use NavToolkit as a subproject and write
  Python bindings that compile on OSX.

### Changed

- Moved GDAL, geoid undulation, and related classes from `navutils` namespace to `geospatial` (issue
  #578).
- Fixed a bug where `inverse` failed on large matrices.
- Viper Library (libviper) renamed to NavToolkit (navtk). Namespace and directory structure has been
  changed from `viper` to `navtk`.
- Moved random number generation to `experimental` namespace (issue #615).
- The `gpsutils` namespace was renamed to `gnssutils` to support future GNSS development.
- `GdalSource` will now attempt to read data from multiple data sources rather than just the first
  one that contains the queried point. This is useful for datasets that contain overlapping sources
  with blank regions.
- Replaced `pylab` with `matplotlib.pyplot` and `numpy`(issue #612).
- `RbpfStrategy` and its related classes has been moved to the `experimental` namespace as its
  functionality is likely to change significantly (issue #594).
- `rpy_to_dcm` and `dcm_to_rpy` now return and accept the transposed DCM from their previous
  behavior to be consistent with the DCM convention used in other coordinate frame transition
  functions.
- Fixed and added coverage for many Python bindings.
- Fixed a bug where the tightly-coupled GPS measurement processors mixed up their bias states.
- Synced up validation and inspect behavior in `FusionStrategy` (issue #591).
- Fixed a bug where `assemble_prs.cpp` was grabbing `SIG_Y` measurements when it should have been
  grabbing `SIG_P` measurements (issue 582).
- `GdalSource` now ignores "hidden" files (files that start with `.`).
- Renamed `PseudorangeType` `CAL1_PYL1_COMBINATION` to `CAL1_PYL2_COMBINATION`, since it uses a
  combination of C/A L1 and P(Y) L2 codes (issue #579).
- Fixed a bug in the dual frequency ionospheric delay calculation (issue #576).
- Refactored the GPS/INS tightly-coupled example to make it easier for users to adapt to their own
  datasets.
- Various `RbpfStrategy` improvements and bugfixes (issues #62, #291, #294, #298, #310, #383).
- `Time` class is used more widely to represent absolute times in order to provide more precision
  than `double` for large timestamps.
- Measurement processors now `log_or_throw` when they are sent an unsupported measurement type
  (issue #552).
- `StateBlock` is no longer an abstract/interface class. It now models a constant value.
- Moved Python bindings into `src/` tree (issue #543).
- Fixed a bug in DCM exponential integration (commit ae99fbc601f).
- Fixed a bug where tilt errors were being applied incorrectly, particularly in virtual state blocks
  (issue #538).
- Removed `gpsutils.hpp` and `navutils.hpp`.
- Reorganized `test/` file tree to better match `src/` (issue #535).
- Switched to Ubuntu 20.04 from 18.04 as the "canonical" OS.
- `StandardFusionEngine`'s `get_num_states` method is now public instead of protected.
- Fixed a bug in `ChainedVirtualStateBlock`'s caching (issue #530).

## v0.2 - 2021-03-01

### Added

- Viper Library can now be installed.
- `StateBlock`, `VirtualStateBlock`, `MeasurementProcessor`, and `FusionStrategy` can now be cloned.
- `Pinson21NedBlock` `StateBlock`, which adds states to estimate IMU scale factor errors.
- `VirtualStateBlock` examples.
- Port of remaining `MeasurementProcessor`s from ANT Center Kotlin Scorpion codebase.
- Port of inertial static alignment algorithm.
- GeoTIFF support to DTED utilities.
- Helper functions to convert between `viper::filtering::NavSolution`,
  `viper::inertial::StandardPosVelAtt`, and `viper::aspnplaceholder::PositionVelocityAttitude`.
- Logging framework, using spdlog.
- Helper functions to apply Pinson error state estimates to PVA.
- Port of remaining `ImuModel`s from ANT Center Kotlin Scorpion codebase.
- The `BufferedInertial` class, from which historic PVA, forces, and rates can be queried.
- A `receive_aux_data` method on `FusionStrategy`.
- Version embedded in library.
- Global error modes, which determine whether the library throws an exception, logs an error, or
  quietly attempts to move on when it encounters a problem.

### Changed

- Renamed "Scorpion Toolkit" to "Viper Library" and `stk` namespace to `viper`.
- Renamed `StandardFilter` to `StandardFusionEngine`, and `FilterStrategy` to `FusionStrategy`.
- `FusionStrategy` estimate and covariance can be lazily evaluated (property rather than field).
- Removed `Ref` class.
- LCM 1.3 is now supported.
- The "body" frame is now referred to as the "platform" frame.
- `SinglePointPseudorangeProcessor` now estimates each satellite's leftover error using a FOGM
  state.
- Several `StandardFusionEngine` methods have been added or made public (`has_processor`,
  `get_measurement_processor_names_list`, `has_block`, `generate_x_and_p`).
- `Buffer` and `BufferStrategy` were removed; `StandardFusionEngine` no longer buffers measurements.
- `ElevationProvider` was removed and `ElevationSource` was renamed to `SpatialMapData` and its
  implementations should now be used directly.
- `add_states` renamed to `on_filter_state_block_added` (similar for `remove_states`).
- The tightly-coupled GPS/INS example now uses `BufferedInertial` and `VirtualStateBlock`s.
- Reworked coordinate frames definitions and descriptions.
- `StandardFusionEngine::peekahead` now returns `nullptr` if no labels are passed in.
- `ValidationContext::is_validation_enabled()` has been removed in favor of
  `viper::get_global_error_mode()`.

## v0.1 - 2020-03-31

### Added

- Initial port of functionality from JVM Kotlin to C++.
- AFIT Scorpion compatibility layer (Python only).
- Python bindings.
- New approach to INS mechanization via wander azimuth.
- New VirtualStateBlock approach (aliased states).
- Placeholder data model files (aspnplaceholder) based on ASPN.
- Enhancements to CI pipelines.
- New DTED support library.

### Changed

- Naming conventions, hierarchy, and organization of files.
- Filter delegation approach (composition over inheritance).
