# ins_sim

**Inertial Navigation Simulation & GUI** — an open-source 6-DOF, WGS-84
strapdown INS Monte Carlo simulator with a PySide6 desktop front end.

[![Release](https://github.com/jorgieo/ins_sim/actions/workflows/release.yml/badge.svg)](https://github.com/jorgieo/ins_sim/actions/workflows/release.yml)
[![Docs](https://github.com/jorgieo/ins_sim/actions/workflows/docs.yml/badge.svg)](https://github.com/jorgieo/ins_sim/actions/workflows/docs.yml)
[![Latest release](https://img.shields.io/github/v/release/jorgieo/ins_sim)](https://github.com/jorgieo/ins_sim/releases/latest)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

[Documentation](https://jorgieo.github.io/ins_sim/) ·
[Download](https://github.com/jorgieo/ins_sim/releases/latest)

| Desktop GUI | Monte Carlo output |
| :---------: | :----------------: |
| ![Main window](docs/assets/gui/main_window.png) | ![Circular Error Probable Results](docs/assets/plots/cep.png) |

## Overview

`ins_sim` generates a truth flight trajectory, derives the gyro and
accelerometer outputs a perfect IMU would produce along that trajectory
(`ω_ib_b`, `f_b`), corrupts them with a navigation-grade IMU error model
(angular/velocity random walk, Gauss-Markov bias drift, turn-on bias
repeatability), and runs a strapdown mechanization to recover position,
velocity, and attitude. A Monte Carlo ensemble over independent noise
realizations characterizes how navigation error grows over time for a given
IMU grade.

The simulation models rotating-Earth effects explicitly: Earth rate, transport
(craft) rate, WGS-84 radii of curvature, and Somigliana normal gravity are all
evaluated at the current latitude/altitude at every step, so the strapdown
mechanization is consistent with true inertial navigation rather than a flat,
non-rotating Earth approximation. The vertical channel runs either baro-damped
(third-order fixed-gain loop) or free-inertial, where it exhibits the expected
instability.

Results are explored in a desktop GUI whose visualizations are fully
interactive plotly pages rendered in-app.

## Download

Prebuilt portable bundles for **Windows** and **Linux** (x86-64 and ARM64) are
attached to every
[GitHub Release](https://github.com/jorgieo/ins_sim/releases/latest) — no
Python installation required:

- `ins_sim-vX.Y.Z-windows-x86_64.zip` — extract, then run `ins_sim.exe`
  inside the extracted folder.
- `ins_sim-vX.Y.Z-linux-x86_64.tar.gz` — `tar xzf`, then run
  `./ins_sim/ins_sim`.
- `ins_sim-vX.Y.Z-linux-aarch64.tar.gz` — ARM64 / aarch64 build (e.g.
  **Raspberry Pi 4/5 on the 64-bit Raspberry Pi OS**); `tar xzf`, then run
  `./ins_sim/ins_sim`. Check your CPU with `uname -m` (`aarch64`).

See the [Download & Install](https://jorgieo.github.io/ins_sim/download/)
page for platform notes (Windows SmartScreen, Linux runtime libraries,
Raspberry Pi).

## The GUI

Launch from source with:

```bash
pip install ".[gui]"
python -m ins_sim.gui
```

- Select a mission trajectory and IMU error specification (packaged YAMLs or
  your own files with the same schema).
- Set the trial count, integration time step, and baro altitude aiding
  (uncheck it to run the vertical channel free-inertial).
- Run the Monte Carlo ensemble in the background with live log output.
- Explore the results in interactive plotly tabs:
  - **CEP** — horizontal error growth for every trial, ensemble CEP, linear
    fit, and a percentile table by hour.
  - **Attitude / Velocity / Position errors** — ensemble mean and 3σ bands
    per axis.
  - **3D Trajectory** — all trial tracks, the truth track, and a 95th
    percentile error tube.
  - **Map** — ground track over OpenStreetMap tiles with a horizontal error
    envelope (map tiles are the only feature that needs an internet
    connection).

## CLI

A scripted run of the same engine (matplotlib summary figure plus a
standalone ground-track map HTML):

```bash
pip install -e ".[dev]"

# Default run: packaged BQN departure mission, nav-grade IMU, 20 trials
python main.py

# Custom mission, IMU spec, and ensemble size
python main.py --config path/to/mission.yaml --imu-spec path/to/imu.yaml \
               --trials 100 --dt 0.1 --seed 7
```

To use a different mission profile or IMU grade, point `build_trajectory` /
`load_imu_spec` at your own YAML files (same schema as the defaults):

```python
from ins_sim.trajectory.kinematics import build_trajectory
from ins_sim.sensors.imu import load_imu_spec

truth, v_sprint, R_turn = build_trajectory("path/to/your_mission.yaml", dt=0.1)
spec = load_imu_spec("path/to/your_imu_spec.yaml")
```

## Architecture

| Module | Role | Key implementations |
| ------ | ---- | ------------------- |
| `ins_sim/core` | Earth & gravity model | WGS-84 ellipsoid, radii of curvature, Earth/transport rates, Somigliana gravity + free-air correction |
| `ins_sim/trajectory` | Truth generation | Phase-based mission builder (ground roll, takeoff, climb, waypoints, loiters, speed ramps), ISA Mach conversion |
| `ins_sim/sensors` | IMU error model | ARW/VRW white noise, Gauss-Markov bias instability, turn-on repeatability, scale factor & misalignment |
| `ins_sim/navigation` | Strapdown mechanization | Quaternion attitude propagation, Coriolis/transport-rate compensation, baro-damped or free-inertial vertical channel |
| `ins_sim/evaluation` | Monte Carlo statistics | Empirical ensemble runs, percentile envelopes, CEP |
| `ins_sim/gui` | Desktop interface | PySide6 + QThread background workers, interactive plotly views in QtWebEngine |

Data flows from truth generation, through noise injection, into the strapdown
navigator, and is summarized by the Monte Carlo / evaluation layer:

```mermaid
flowchart TD
    CFG["Config (YAML)<br/>trajectory + IMU spec"]
    TRUTH["Truth generation<br/>trajectory/kinematics.py"]
    SENSOR["Noise addition<br/>sensors/imu.py"]
    NAV["Strapdown mechanization<br/>navigation/strapdown.py"]
    EVAL["Monte Carlo + plots<br/>evaluation/"]
    EARTH(["Earth model<br/>core/earth_model.py"])

    CFG --> TRUTH
    TRUTH -->|"true ω, specific force"| SENSOR
    SENSOR -->|"measured ω, f (+ noise)"| NAV
    NAV -->|"pos, vel, attitude"| EVAL
    EARTH -.->|"ω_ie, ω_en, g, R_M/R_N"| TRUTH
    EARTH -.-> NAV
```

## Conventions

### Frames

- **NED (n-frame)** — local-level North-East-Down navigation frame, origin at
  the current position. Position is reported relative to the trajectory start
  unless otherwise integrated as geodetic (lat, lon, alt).
- **Body (b-frame)** — vehicle-fixed frame: x forward, y right, z down.
- Rotating-Earth quantities (Earth rate `ω_ie`, transport rate `ω_en`) are
  evaluated in NED and resolved into body via `C_n^b` where needed.

### Attitude

- Attitude is propagated and stored as a **scalar-last quaternion**
  `[x, y, z, w]` (`scipy.spatial.transform.Rotation` convention), representing
  the body-to-NED rotation `C_b^n`.
- Euler angles, when used (e.g. for plotting), are `[roll φ, pitch θ, heading ψ]`
  applied in body axis order `Z-Y-X` (heading, then pitch, then roll).

### Units — strict SI

- Angles: radians (`rad`), angular rates: `rad/s`.
- Length: meters (`m`), velocity: `m/s`, acceleration/specific force: `m/s²`.
- Time: seconds (`s`).
- YAML mission/IMU configs are expressed in conventional datasheet/aviation
  units (`deg`, `kt`, `ft`, `°/hr`, `µg`, etc.) and are converted to SI at load
  time (`build_trajectory`, `load_imu_spec`) — no non-SI unit crosses into the
  simulation core.

## Development

```bash
# Editable install with dev dependencies
pip install -e ".[dev,gui]"

# Test suite
python -m pytest

# Local desktop bundle (onedir, at dist/ins_sim/)
pyinstaller ins_sim.spec --noconfirm

# Documentation site (http://127.0.0.1:8000)
pip install ".[docs]"
mkdocs serve
```

## License

This project's own source code is licensed under the [MIT License](LICENSE).

The downloadable release binaries additionally bundle third-party
components, most notably **Qt / PySide6**, which are redistributed under the
**GNU LGPL v3**. Attribution and the full license texts for all bundled
components are provided in [THIRD-PARTY-NOTICES.txt](THIRD-PARTY-NOTICES.txt)
and the [`licenses/`](licenses/) directory, both of which ship inside every
release bundle. Installing the library from source without the `gui` extra
pulls no LGPL-licensed dependencies.
