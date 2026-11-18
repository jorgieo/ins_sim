# Recipes

Short answers to questions that come up once the simulator is running.
Most need only the app; the error budget also has a script.

## Get a quick answer, then a precise one

Explore with **10 trials at a 1 s time step**: the packaged mission then
runs in about 15 seconds, quick enough to try several missions or IMU
specs. Once you know what you want to show, rerun it at the defaults, **50
trials at 0.1 s**, or with more trials (see below) for the final numbers.

## Choose the time step

The time step decides how finely the truth and the navigator are sampled;
the run time is proportional to the number of steps. It hardly changes how
fast the errors grow, because that comes from the sensor errors, not from
the sampling. On the packaged mission, 50 trials give a CEP growth of 1.24
nmi/h at a 1 s step and 1.28 nmi/h at 0.1 s, and ±3σ bands that end within
10% of each other.

A 1 s step is therefore fine for studying error growth over tens of
minutes. Use 0.1 s, the default and the setting of the figures in these
docs, for final numbers and for short, fast manoeuvres.

## How many trials?

Every statistic comes from a finite sample of trials, and is uncertain
accordingly:

| Trials | Uncertainty of a ±3σ band | Good for |
| ------ | ------------------------- | -------- |
| 10 | about 25% | a first look at the shape of the errors |
| 50 | about 10% | the CEP and the ±3σ bands: the default |
| 200 | about 5% | the 95th percentile, the 3D tube and the map band |

The 95th percentile needs the most trials, because it rests on the few
worst ones: with 50 trials, on the worst two or three. The
[Monte Carlo evaluation](../theory/monte-carlo.md#sampling-uncertainty)
page derives these numbers.

## Compare two IMUs fairly

Run both specs on the same mission with the same time step and number of
trials. The app always uses the same random seed, and every error is drawn
the same way whatever its size. The two runs therefore use the same
underlying random numbers, scaled by each spec's values, so the difference
between them comes from the specs, not from luck. This makes the
*difference* between two specs much more precise than either result on its
own, even with few trials.

The same holds for any other setting, such as the alignment block or the
mission. In a script, pass the same `seed` to `run_monte_carlo`.

## Find the dominant error source

To find which error limits a navigator, run it with one group of errors at
a time and compare. Each group draws exactly the same errors in these runs
as in the run with every error on (see above), so luck does not blur the
comparison.

**In the app:** copy the IMU spec once per group, keep that group's values
in each copy and set every other value to 0, as described in
[Modelling an IMU](imu-specs.md#switching-errors-off). Run each copy with
the same settings and compare the CEP tables.

**In a script**, `dataclasses.replace` makes the copies:

```python
--8<-- "docs/examples/error_budget.py"
```

On the half-hour training flight of [Writing a mission](missions.md) it
prints:

```text
--8<-- "docs/examples/error_budget.txt"
```

On this flight the initial alignment and the gyro biases dominate; the
accelerometer biases matter least. A better accelerometer would barely
help, while a better alignment or better gyros would. The contributions do
not add up directly: they combine roughly as the square root of the sum of
their squares, 732 m here, against 799 m for all sources together.

Which source dominates depends on the flight: errors that the Schuler
oscillation keeps bounded, such as the accelerometer biases, matter less
on long flights than errors that keep growing, such as the gyro biases.
Rerun the budget for your own mission; the
[INS error model](../theory/error-model.md) page explains how each error
grows.

## Check that a mission flies as planned

Run it with 2 trials at a 1 s step and look at the **3D Trajectory** and
**Map** tabs, or use the checking script in
[Writing a mission](missions.md#check-a-mission-before-running-it).

## Fly a path of your own from Python

A mission file is not the only way to make a truth trajectory.
`TruthTrajectory.from_spline` flies a smooth path through a list of points
at constant speed, and `run_monte_carlo` accepts the result like any
other. See its example on the
[Trajectory](../reference/api/trajectory.md) page of the Python API.
