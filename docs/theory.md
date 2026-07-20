# Theory & Math

!!! warning "Under construction"
    The full formulations are being written up. The outline below shows what
    is planned.

## Planned sections

- **Strapdown mechanization** — NED navigation equations with explicit
  rotating-Earth terms: Earth rate, transport rate, WGS-84 radii of
  curvature, and Somigliana normal gravity.
- **IMU stochastic error model** — angular/velocity random walk, first-order
  Gauss-Markov bias drift, and turn-on bias repeatability, and how datasheet
  units map onto the simulation parameters.
- **Schuler oscillation** — why horizontal INS errors oscillate with period

    $$
    T_s = 2\pi\sqrt{\frac{R}{g}} \approx 84.4\ \text{min}
    $$

- **Vertical-channel instability and baro damping** — the unstable
  free-inertial altitude solution and the third-order fixed-gain baro loop
  used to damp it.
- **Monte Carlo error characterization** — CEP, percentile envelopes, and
  ensemble statistics.

Kalman-filter-based aiding (e.g. GNSS/INS integration) is **not implemented**
in `ins_sim` today; it is a candidate for future work and would be documented
here.
