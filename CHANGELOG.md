# Changelog

All notable changes to ins_sim. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the version
numbers [Semantic Versioning](https://semver.org/): before 1.0, a new minor
version may change the API.

## [0.3.0] - 2026-11-18

Version 0.3.0 adopts the notation of Groves, *Principles of GNSS, Inertial,
and Multisensor Integrated Navigation Systems* (2nd ed., 2013), in the code
and the documentation, measures and models the navigation errors more
carefully, and adds complete documentation. **The Python API changes, and
results differ from 0.2.0.** The
[Migrating from 0.2](https://jorgieo.github.io/ins_sim/0.3/project/migrating-from-0.2/)
guide lists every change.

### Changed

- **The Python API is renamed after the Groves symbols.** For example,
  `pos_n` becomes `r_lb_l`, `lat`, `lon` and `alt` become `L_b`,
  `lambda_b` and `h_b`, `R_b2n` becomes `C_b_n`, `omega_b` and `f_b`
  become `omega_ib_b` and `f_ib_b`, and `dt` becomes `tau_i`. The radii of
  curvature become `R_N` (meridian) and `R_E` (transverse): in 0.2, `R_N`
  was the transverse radius. Functions that returned tuples return named
  tuples.
- **`run_monte_carlo` returns a `MonteCarloResult`** with seven fields,
  including the new `est_h_b` and `delta_psi_nb_n`, and
  `percentile_envelope` takes the position-error array.
- **Bias instability is the datasheet value**: the floor of the sensor's
  Allan deviation. The spread of the in-run bias is derived from it so
  that the simulated Allan deviation peaks there; for the same IMU spec,
  it is 1.62 times the spread of 0.2. `IMUSpec.gyro_bi_std` and
  `accel_bi_std` become `gyro_bias_instability` and
  `accel_bias_instability`, with the spreads as the new properties
  `gyro_gm_std` and `accel_gm_std`.
- **Attitude errors are the small rotation between the estimated and true
  attitude**, as North tilt, East tilt and heading error, instead of
  differences of roll, pitch and heading. The app's *Attitude Errors* tab
  and the command-line summary show them.
- **Position errors are the latitude, longitude and height errors in
  metres** along North, East and Down at the true position, in every
  result.
- The in-run bias starts at its steady-state spread instead of zero.
- Each trial draws its alignment, gyro and accelerometer errors from
  separate random streams, so changing one error source leaves the
  others' draws unchanged. The same seed draws different errors than in
  0.2.
- The ±3σ bands use the sample standard deviation (dividing by N − 1).
- ins_sim requires NumPy 1.25 or later.

### Added

- Documentation, with a version selector: tutorials for the app and for
  Python, a user guide, the theory and mathematics of every part of the
  simulator, a validation of the simulator against textbook results, the
  Python API reference, references for the mission and IMU-spec files,
  and a glossary and bibliography.
- IMU specs for a tactical-grade and a MEMS unit, and an example mission,
  in the documentation.
- `ins_sim.evaluation.errors`, with the three error definitions:
  `attitude_error`, `velocity_error` and `position_error`.
- Runnable examples in the docstrings.

### Fixed

- Missions can descend. A waypoint leg to a lower altitude used to stay
  level, and a `climb` phase to a lower altitude climbed.
- A mission whose first phase is not a `ground_roll` fails with a clear
  message instead of `cannot convert float NaN to integer`.
- The stated accuracy of the gravity model's height correction.
- The downloadable app leaves out the Qt modules it does not use, some of
  which Qt offers only under the GPL-3.0, so the downloads are smaller.

## [0.2.0] - 2026-07-22

### Added

- A Linux ARM64 build, for the Raspberry Pi 4 and 5 running the 64-bit
  Raspberry Pi OS.
- `trajectories` and `imu_specs` folders next to the app, holding copies
  of the packaged files. Missions and IMU specs saved there appear in the
  app's lists, and **Browse…** opens them.
- A documentation site: download and installation, a user guide and the
  theory, illustrated with figures produced by the simulator.
- Third-party license notices and texts in the downloadable app.

## [0.1.1] - 2026-07-20

### Changed

- A smaller downloadable app: packages the app does not use are left out.

## [0.1.0] - 2026-07-19

The first release.

- A desktop app: choose a mission and an IMU spec, run a Monte Carlo
  ensemble in the background, and explore interactive results: the CEP
  with an hourly percentile table, the attitude, velocity and position
  errors with ±3σ bands, a 3D trajectory with its 95th-percentile error
  tube, and a ground-track map.
- An IMU error model with angle and velocity random walk, Gauss–Markov
  bias drift, turn-on bias, scale-factor and misalignment errors, and
  initial alignment errors.
- A baro-damped or free-inertial vertical channel.
- A command-line run with a summary figure and an interactive map.
- Downloadable apps for Windows and Linux.

[0.3.0]: https://github.com/jorgieo/ins_sim/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/jorgieo/ins_sim/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/jorgieo/ins_sim/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/jorgieo/ins_sim/releases/tag/v0.1.0
