# Python API

The simulator is a small Python package; the GUI and the command-line tool
are thin layers on top of it. Each page below is generated from the
docstrings in the source code.

Physics quantities are named after their symbols on the
[Notation & conventions](../../theory/notation.md) page. For example,
`v_eb_n` is \(\vvec{eb}{n}\), the velocity of the body relative to the Earth
resolved in NED, and `C_b_n` is \(\Cmat{b}{n}\), the body-to-NED attitude.
Prefixes mark the role of a value: `meas_` for a sensor measurement, `est_`
for a navigation estimate, `delta_` for an error.

| Page | What it covers | Start with |
| ---- | -------------- | ---------- |
| [Earth model](earth-model.md) | WGS-84 radii of curvature, Earth rate, transport rate, gravity | `normal_gravity`, `wgs84_radii` |
| [Trajectory](trajectory.md) | The truth trajectory from a mission YAML, and the ideal IMU outputs along it | `build_trajectory`, `TruthTrajectory` |
| [Sensors](sensors.md) | The IMU error model | `load_imu_spec`, `generate_imu_samples` |
| [Navigation](navigation.md) | The strapdown mechanization | `strapdown_navgrade` |
| [Evaluation](evaluation.md) | Monte Carlo runs and the navigation-error definitions | `run_monte_carlo`, `position_error` |
| [Configuration](config.md) | Paths to the packaged default YAML files | `default_trajectory_path` |

## A complete run

This script runs ten Monte Carlo trials on the packaged mission, prints the
CEP after one hour and plots every trial's horizontal error. It is the same
file the test suite runs, so it stays in step with the code, and the
[Your first script](../../get-started/first-script.md) tutorial goes
through it step by step.

```python
--8<-- "docs/examples/first_script.py"
```
