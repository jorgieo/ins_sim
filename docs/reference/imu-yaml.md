# IMU spec YAML

Every key of an IMU spec file, with its unit and the SI value it becomes.
The [Modelling an IMU](../guide/imu-specs.md) guide explains how to fill
them in from a datasheet, and the
[IMU error model](../theory/imu-errors.md) page what each one does.

## Top level

| Key | Required | Contents |
| --- | -------- | -------- |
| `name` | no | Free text for your reference. ins_sim does not read it. |
| `gyro` | yes | The gyro errors: see [gyro](#gyro). |
| `accel` | yes | The accelerometer errors: see [accel](#accel). |
| `alignment` | no | The initial attitude error: see [alignment](#alignment). Without it, each trial starts from the true attitude. |

The app lists a file as an IMU spec only if it has `gyro` and `accel`
keys.

All spreads are one standard deviation (1σ). Each applies to every axis
separately: the three gyros, or the three accelerometers, draw independent
errors of that size.

## gyro

| Key | Required | Unit | Meaning | `IMUSpec` field, SI unit |
| --- | -------- | ---- | ------- | ------------------------ |
| `arw_deg_per_rt_hr` | yes | °/√h | Angle random walk: the density of the white noise | `gyro_arw`, rad/√s |
| `bias_instab_deg_per_hr` | yes | °/h | Bias instability, as on the datasheet: the floor of the Allan deviation | `gyro_bias_instability`, rad/s |
| `bias_tau_s` | yes, above 0 | s | Correlation time of the in-run bias. Its Allan deviation peaks at 1.89 times this. | `gyro_bias_tau`, s |
| `repeatability_deg_per_hr` | yes | °/h | Turn-on bias repeatability: the constant bias of each run | `gyro_br_std`, rad/s |
| `scale_factor_ppm` | no, default 0 | ppm | Scale-factor error | `gyro_sf_std`, dimensionless |
| `misalignment_urad` | no, default 0 | µrad | Axis misalignment, per angle | `gyro_ma_std`, rad |

## accel

| Key | Required | Unit | Meaning | `IMUSpec` field, SI unit |
| --- | -------- | ---- | ------- | ------------------------ |
| `vrw_m_per_s_per_rt_hr` | yes | m/s/√h | Velocity random walk: the density of the white noise | `accel_vrw`, m/s/√s |
| `bias_instab_ug` | yes | µg | Bias instability, as on the datasheet | `accel_bias_instability`, m/s² |
| `bias_tau_s` | yes, above 0 | s | Correlation time of the in-run bias | `accel_bias_tau`, s |
| `repeatability_ug` | yes | µg | Turn-on bias repeatability | `accel_br_std`, m/s² |
| `scale_factor_ppm` | no, default 0 | ppm | Scale-factor error | `accel_sf_std`, dimensionless |
| `misalignment_urad` | no, default 0 | µrad | Axis misalignment, per angle | `accel_ma_std`, rad |

## alignment

| Key | Required | Unit | Meaning | `IMUSpec` field, SI unit |
| --- | -------- | ---- | ------- | ------------------------ |
| `tilt_std_deg` | no, default 0 | ° | Initial tilt error, about North and about East | `align_tilt_std`, rad |
| `heading_std_deg` | no, default 0 | ° | Initial heading error | `align_heading_std`, rad |

## Rules

- Every value must be 0 or more. Set a value to 0 to remove that error.
- `bias_tau_s` must be above 0, even when its bias instability is 0.
- A missing required key stops the run with a message naming it, such as
  `KeyError: 'bias_tau_s'`.

## The packaged spec

The app's default IMU spec, `imu_spec.yaml`, a navigation-grade unit:

```yaml
--8<-- "ins_sim/config/imu_spec.yaml"
```

Two more presets, for a tactical-grade and a MEMS unit, are on the
[Modelling an IMU](../guide/imu-specs.md#three-presets) page.

In Python, [`load_imu_spec`][ins_sim.sensors.imu.load_imu_spec] reads a
file into an [`IMUSpec`][ins_sim.sensors.imu.IMUSpec], in SI units. An
`IMUSpec` can also be built directly. Its defaults are the packaged spec's
noise and bias values, with no scale-factor, misalignment or alignment
errors.
