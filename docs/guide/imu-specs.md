# Modelling an IMU

An IMU spec file describes how a unit's gyros and accelerometers err, in
the units of a sensor datasheet. This page starts from three ready-made
specs, one per grade of IMU, then shows how to enter your own sensor's
datasheet. The [IMU spec YAML](../reference/imu-yaml.md) reference lists
every key, and the [IMU error model](../theory/imu-errors.md) page explains
the mathematics.

## Three presets

| Grade | Typical use | Gyro bias instability | Accelerometer turn-on bias |
| ----- | ----------- | --------------------- | -------------------------- |
| Navigation | Airliners and ships, with ring-laser gyros | 0.01 °/h | 25 µg |
| Tactical | Missiles and large drones, with fibre-optic gyros | 0.5 °/h | 500 µg |
| MEMS | Small drones and vehicles, with silicon sensors | 10 °/h | 5,000 µg |

The values are illustrative of each class; real products vary within it.
The navigation-grade spec is the one the app ships with. To use another,
copy it into a new `.yaml` file in the app's `imu_specs` folder (the copy
button at the top right of each file copies it):

=== "Navigation grade"

    ```yaml
    --8<-- "ins_sim/config/imu_spec.yaml"
    ```

=== "Tactical grade"

    ```yaml
    --8<-- "docs/examples/imu/tactical_grade.yaml"
    ```

=== "MEMS"

    ```yaml
    --8<-- "docs/examples/imu/mems_grade.yaml"
    ```

### How fast each grade drifts

This script flies the three specs through the half-hour training flight of
[Writing a mission](missions.md):

```python
--8<-- "docs/examples/imu_grades.py"
```

```text
--8<-- "docs/examples/imu_grades.txt"
```

Each step down in grade costs more than an order of magnitude. After ten
minutes the tactical-grade unit is about 25 times further off than the
navigation-grade one, and the MEMS unit about 400 times. The MEMS unit is
kilometres off within five minutes. That is why such units are always
paired with GNSS or other aiding, which ins_sim does not model.

## From a datasheet to the YAML

Most datasheets give every value the spec needs, though not always in the
same units:

| Datasheet parameter | YAML key | Unit | From other units |
| ------------------- | -------- | ---- | ---------------- |
| Angle random walk (gyro noise) | `arw_deg_per_rt_hr` | °/√h | °/√s or (°/s)/√Hz: × 60. (°/h)/√Hz: ÷ 60 |
| Gyro bias instability | `bias_instab_deg_per_hr` | °/h | °/s: × 3,600 |
| Gyro turn-on bias repeatability | `repeatability_deg_per_hr` | °/h | °/s: × 3,600 |
| Velocity random walk (accelerometer noise) | `vrw_m_per_s_per_rt_hr` | m/s/√h | µg/√Hz: × 0.000588. m/s/√s or (m/s²)/√Hz: × 60 |
| Accelerometer bias instability | `bias_instab_ug` | µg | mg: × 1,000 |
| Accelerometer turn-on bias repeatability | `repeatability_ug` | µg | mg: × 1,000 |
| Scale-factor error | `scale_factor_ppm` | ppm | %: × 10,000 |
| Axis misalignment | `misalignment_urad` | µrad | mrad: × 1,000. arcsec: × 4.848. °: × 17,453 |

Two more values rarely appear as numbers: the bias correlation time
`bias_tau_s`, explained below, and the alignment block, which depends on
how the system aligns rather than on the sensors alone.

Every spread is one standard deviation (1σ) per axis. If a datasheet
gives a bound instead, convert it: divide a 3σ bound by three.

### Bias instability is the datasheet value

Enter the bias instability exactly as the datasheet gives it: the floor of
the sensor's Allan deviation, the lowest point of its Allan-deviation
plot. ins_sim models the wandering bias as a Gauss–Markov process and
picks its spread so that the simulated sensor's Allan deviation peaks at
exactly that value. The [Validation](../validation/index.md#allan-deviation)
page shows the match.

- Some datasheets quote the IEEE 952 bias-instability *coefficient*
  instead of the floor. The floor is 0.664 times the coefficient, so
  multiply by 0.664 before entering it.
- The **correlation time** `bias_tau_s` sets where the simulated Allan
  deviation peaks: at 1.89 times the correlation time. If the datasheet
  has an Allan-deviation plot, read off the averaging time \(T\) at the
  floor and enter \(T/1.89\). Without a plot you have to choose; the
  presets use one hour for navigation grade, 300 s for tactical grade and
  100 s for MEMS.

### Turn-on bias and in-run bias

The two bias terms behave differently in a run. The **turn-on bias
repeatability** sets a constant bias, different every time the unit is
switched on: ins_sim draws it once per trial. The **bias instability**
sets how far the bias then wanders during the run. MEMS datasheets often
call the turn-on value the *initial bias error* or *bias repeatability*,
and the bias instability the *in-run bias stability*.

### Scale factor and misalignment

A scale-factor error is a gain error: the sensor reads 100 ppm high, say.
A misalignment means the sensor's axes are not exactly at right angles to
each other and to the body. Both errors are proportional to what the
sensors measure, so they never vanish. At rest the accelerometers still
measure gravity, and a misaligned one reads part of it along a horizontal
axis: the packaged spec's 50 µrad reads 50 µg, as much as a bias. In
flight the errors grow with the turn rates and accelerations, so they
matter most in manoeuvres. Each trial draws a scale factor per axis and
six independent misalignment angles, two for each pair of axes.

## The alignment block

Before it navigates, an INS works out its initial attitude, and the
`alignment` block gives how far off that is. It has two values: the tilt
(about the North and East axes) and the heading error.

- **Self-alignment, for navigation grade.** At rest, the unit levels itself
  by finding the direction of gravity, and finds North by sensing the
  Earth's rotation. The accelerometer bias limits the tilt and the gyro
  bias limits the heading. As a guide, the tilt in degrees is 0.000057
  times the accelerometer turn-on bias in µg, and the heading error in
  degrees is about \(3.8\,b_g/\cos L_b\), with \(b_g\) the gyro turn-on
  bias in °/h and \(L_b\) the latitude.
- **Heading from elsewhere, for tactical grade and MEMS.** With 1 °/h
  gyros, sensing the Earth's rotation would leave about 4° of heading
  error, and MEMS gyros, whose biases are as large as the Earth's rotation
  rate, cannot sense it usefully. These systems take their heading from a
  magnetometer, from GNSS, or from another navigator. Enter the accuracy
  of that source.

Leave the block out to start every trial from the true attitude. That is
useful when you want to see the sensor errors alone.

## Switching errors off

Set any value to 0 to remove that error. `scale_factor_ppm`,
`misalignment_urad` and the whole alignment block are optional and default
to 0. Keep `bias_tau_s` positive, even when its bias instability is 0.

With every error at 0 the IMU is perfect, and the navigator follows the
truth exactly. Switching errors off one group at a time shows which one
matters most: see [Recipes](recipes.md#find-the-dominant-error-source).

## What the model leaves out

The model covers the errors that dominate a navigation-grade INS. It does
not include g-dependent gyro bias, quantization, rate random walk,
scale-factor nonlinearity, temperature effects or vibration. The
[IMU error model](../theory/imu-errors.md#assumptions-limitations) page
lists these and the model's other assumptions.
