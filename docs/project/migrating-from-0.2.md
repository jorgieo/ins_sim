# Migrating from 0.2

Version 0.3 renames the Python API after the symbols of
[Groves (2013)](../reference/bibliography.md#groves2013), measures and
models the navigation errors more carefully, and reads bias instability
as the datasheet value. Mission and IMU files load as before, and the app
works the same way, but the results differ from 0.2, and Python scripts
need updating.

## Results change

Each of these changes alters the numbers you get from the same files:

| Change | Effect |
| ------ | ------ |
| Bias instability is the datasheet value, the floor of the Allan deviation. 0.2 used it directly as the spread of the in-run bias. | The in-run bias spread is 1.62 times that of 0.2 for the same file. See [Modelling an IMU](../guide/imu-specs.md#bias-instability-is-the-datasheet-value). |
| The in-run bias starts at its full spread. In 0.2 it started at zero and grew over its correlation time. | Larger errors early in a flight. |
| Attitude errors are the small rotation between the estimated and true attitude, as North tilt, East tilt and heading error. 0.2 plotted differences of roll, pitch and heading. | A different *Attitude Errors* tab; its checkbox, formerly *Attitude/Euler Angle Errors*, is renamed. |
| Position errors are the latitude, longitude and height errors in metres along North, East and Down at the true position, in every result. 0.2 subtracted positions relative to the starting point. | The two agree near the start and differ more the further the flight goes. |
| Each trial draws its alignment, gyro and accelerometer errors from three separate random streams. | The same seed draws different errors than in 0.2. |
| The ±3σ bands use the sample standard deviation (dividing by N − 1). | Slightly wider bands with few trials. |
| Missions can descend. In 0.2 a waypoint leg to a lower altitude stayed level, and a `climb` to a lower altitude climbed. | Missions with descents now fly as written. |
| A mission must start with a `ground_roll`. 0.2 crashed on such a file with an obscure error. | A clear error message instead. |

## Mission and IMU files

The keys are unchanged, so every 0.2 file loads in 0.3. One value changes
meaning: `bias_instab_deg_per_hr` and `bias_instab_ug` are now read as the
datasheet bias instability.

- If you took them from a datasheet, there is nothing to do: 0.3 now
  simulates them as the datasheet means them.
- If you had tuned them as the spread of the in-run bias, as 0.2 used
  them, multiply them by 0.617 to keep the same simulated bias.

The app never overwrites the files in its `imu_specs` folder, so the copy
of `imu_spec.yaml` it made under 0.2 keeps comments that describe bias
instability as a Gauss–Markov spread. Its values are those of 0.3's
packaged spec. To get the new comments, delete the copy and restart the
app, which makes a fresh one.

## Python API

Physics quantities are named after their symbols: `v_eb_n` is the velocity
\(\vvec{eb}{n}\), `C_b_n` the attitude \(\Cmat{b}{n}\). The
[Notation & conventions](../theory/notation.md#symbols-and-code-names)
page lists each symbol with its 0.2 name. In practice:

- Function names are unchanged, but many argument names change. Calls that
  pass arguments by position keep working; keyword arguments need the new
  names.
- Functions that returned a tuple now return a named tuple. Unpacking by
  position works as before, except for `run_monte_carlo`, which now
  returns seven fields, and every field can also be read by name.
- ins_sim now needs NumPy 1.25 or later.

### Earth model

| 0.2 | 0.3 |
| --- | --- |
| `wgs84_radii(lat)` returned `(R_M, R_N)` | `wgs84_radii(L_b)` returns `Radii(R_N, R_E)` |
| `earth_rate_n(lat)` | `earth_rate_n(L_b)` |
| `transport_rate_n(v_n, lat, h)` | `transport_rate_n(v_eb_n, L_b, h_b)` |
| `normal_gravity(lat, h)` | `normal_gravity(L_b, h_b)` |

!!! warning "`R_N` changed meaning"
    0.2 called the meridian radius of curvature `R_M` and the transverse
    one `R_N`. 0.3 follows Groves: `R_N` is the **meridian** radius and
    `R_E` the **transverse** one. `wgs84_radii` still returns the meridian
    radius first, so `R_M, R_N = wgs84_radii(lat)` still runs, but its
    `R_N` is now what the docs call `R_E`. Rename it to avoid confusion.

### Trajectory

| 0.2 | 0.3 |
| --- | --- |
| `build_trajectory(yaml_path, dt=None)` | `build_trajectory(yaml_path, tau_i=None)`, returning `MissionTrajectory(truth, v_sprint, R_turn)` |
| `TruthTrajectory(pos_n, vel_n, euler, dt, lat0_deg, lon0_deg, alt0)` | `TruthTrajectory(pos_profile, v_eb_n, euler_nb, tau_i, L_b0_deg, lambda_b0_deg, h_b0)` |
| `TruthTrajectory.from_spline(path, speed, dt, lat0_deg, lon0_deg, alt0)` | `TruthTrajectory.from_spline(path, speed, tau_i, L_b0_deg, lambda_b0_deg, h_b0)` |

The truth trajectory's attributes:

| 0.2 | 0.3 | Quantity |
| --- | --- | -------- |
| `dt` | `tau_i` | Time step |
| `pos_n` | `r_lb_l` | Position relative to the starting point |
| `vel_n` | `v_eb_n` | Velocity, North-East-Down |
| `acc_n` | `v_eb_n_dot` | Rate of change of the velocity |
| `lat`, `lon`, `alt` | `L_b`, `lambda_b`, `h_b` | Latitude, longitude, height |
| `euler` | `euler_nb` | Roll, pitch, heading |
| `R_b2n` | `C_b_n` | Attitude, body to North-East-Down |
| `f_b` | `f_ib_b` | Specific force a perfect accelerometer measures |
| `omega_b` | `omega_ib_b` | Angular rate a perfect gyro measures |
| `g_loc` | `g` | Gravity |

`t` is unchanged.

### Sensors

| 0.2 | 0.3 |
| --- | --- |
| `IMUSpec.gyro_bi_std` | `IMUSpec.gyro_bias_instability`, now the datasheet value. The in-run bias spread is the new property `gyro_gm_std`. |
| `IMUSpec.gyro_bi_tau` | `IMUSpec.gyro_bias_tau` |
| `IMUSpec.accel_bi_std` | `IMUSpec.accel_bias_instability`, with the new property `accel_gm_std` |
| `IMUSpec.accel_bi_tau` | `IMUSpec.accel_bias_tau` |
| `generate_imu_samples(truth, spec, rng)` returned `(omega_meas, f_meas)` | returns `IMUSamples(meas_omega_ib_b, meas_f_ib_b)` |

The other `IMUSpec` fields keep their names.

### Navigation

| 0.2 | 0.3 |
| --- | --- |
| `strapdown_navgrade(omega_meas, f_meas, init_state, dt, alt_truth=None, baro_tau=100.0)` | `strapdown_navgrade(meas_omega_ib_b, meas_f_ib_b, init_state, tau_i, meas_h_b=None, baro_tau=100.0)` |
| `init_state` as `(lat0, lon0, alt0, v_n0, q_b2n_0)` | `NavState(L_b, lambda_b, h_b, v_eb_n, q_b_n)`; a plain tuple in that order still works |
| returned `(pos_ned, lat, lon, alt, vel, quat)` | returns `NavSolution(r_lb_l, L_b, lambda_b, h_b, v_eb_n, q_b_n)` |

### Evaluation

| 0.2 | 0.3 |
| --- | --- |
| `run_monte_carlo(...)` returned `(pos_runs, euler_runs, lat_runs, lon_runs, vel_runs)` | returns `MonteCarloResult` with the fields `est_r_lb_l`, `est_euler_nb`, `est_L_b`, `est_lambda_b`, `est_v_eb_n`, `est_h_b` and `delta_psi_nb_n`. Read them by name. |
| `percentile_envelope(pos_runs, truth_pos, q=95)` | `percentile_envelope(delta_r_eb_n, q=95)`, taking the position error from `position_error` |
| – | `ins_sim.evaluation.errors`: `attitude_error`, `velocity_error` and `position_error` define the three errors |
| `build_summary_figure(truth, pos_runs, euler_runs, r95, n_trials)` | `build_summary_figure(truth, mc, r95)` |
| `plot_error_tube(ax, truth_pos, r95, ...)` | `plot_error_tube(ax, r_lb_l, r95, ...)` |

In `main.py`, `simulate` returns `(mc, r95)` instead of five arrays. The
command-line options are unchanged.

For a complete script written for 0.3, see
[Your first script](../get-started/first-script.md).

## The documentation

The single theory page is now the [Theory & Math](../theory/index.md)
section, with a page per part of the simulator, and the user guide is now
the [User guide](../guide/index.md) section. Their addresses, `/theory/`
and `/guide/`, still work.
