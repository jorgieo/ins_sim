# User Guide

!!! warning "Under construction"
    Detailed tutorials are being written. The outline below shows what is
    planned; the [Home](index.md) page covers the essentials in the meantime.

## Planned sections

- **Running your first simulation** — choosing a trajectory and IMU spec,
  setting trial count and time step, baro aiding on/off.
- **Writing mission profiles** — the trajectory YAML schema: departure point,
  waypoint legs, speed ramps, climbs, and turns; how aviation units map to SI.
- **Writing IMU specifications** — the IMU YAML schema: angular/velocity
  random walk, Gauss-Markov bias parameters, turn-on repeatability.
- **Reading the results** — how to interpret each visualization tab: CEP and
  its percentile table, the 3σ error bands, the 3D error tube, and the
  ground-track error envelope.
- **Scripted workflows** — driving the simulation engine from Python without
  the GUI.
