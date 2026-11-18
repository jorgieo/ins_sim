# Writing a mission

A mission file describes a flight the way a pilot would plan it: where it
departs, then a list of phases (take-off, climbs, legs to waypoints,
holding patterns, speed changes) in feet, knots and degrees. The simulator
flies the plan smoothly, and the result becomes the truth that every
Monte Carlo trial tries to follow. This page builds an example mission and
explains how the simulator flies each part. The
[Mission YAML](../reference/mission-yaml.md) reference lists every key.

## Start from a copy

Mission files are plain text in the YAML format. Open the app's
`trajectories` folder (see [User guide](index.md#missions-and-imu-specs)),
copy `bqn_departure.yaml` or the example below to a new name, and edit it
in any text editor. It appears in the app's **Trajectory** list the next
time you open the list.

Three rules of YAML matter here: indent with spaces, never tabs; start
each list item, here each phase, with a dash; and anything after a `#` is
a comment.

## An example: a training flight

This half-hour flight from Rafael Hernández Airport (BQN) uses every kind
of phase:

```yaml
--8<-- "docs/examples/missions/training_flight.yaml"
```

The file has four parts:

- **`name`** is for your reference; the app lists missions by file name.
- **`departure`** is the starting point, on the runway, and the bank angle
  for the turns onto each leg. The runway elevation is in feet above mean
  sea level.
- **`simulation_time`** holds a time step that only a Python script uses,
  and only when it gives no time step of its own. The app and the command
  line always use their own setting.
- **`phases`** lists the flight, in order. Each phase starts where the
  previous one ended, with its position, heading, speed and altitude.

## The phases

| Phase | What the aircraft does |
| ----- | ---------------------- |
| `ground_roll` | Accelerates from rest along the runway heading, reaching `speed_final_kt` after `run_length_m` metres. Every mission starts with one, and only one. |
| `takeoff` | Pitches up from level to `pitch_deg` over `duration_s` seconds, on `heading_deg`, while the speed changes to `speed_kt`. Give it the runway heading. |
| `climb` | Climbs straight ahead at `speed_kt`, at a flight-path angle of `pitch_deg`, to `to_altitude_ft`. If the target is lower, it descends at that angle instead. |
| `waypoint` | Turns toward the fix at `lat_deg`, `lon_deg`, then flies straight to it, climbing or descending steadily to `alt_ft` on the way. `speed_kt` sets the leg's speed; without it the aircraft keeps its current speed. An optional `loiter` block then flies `n_turns` full circles at `bank_angle_deg`, to the `right` (the default) or `left`. |
| `speed_ramp` | Changes speed steadily, over `duration_s` seconds, to `speed_end_kt` or to the Mach number `speed_end_mach`, keeping heading and altitude. |

A Mach number is converted to a speed with the speed of sound of the
International Standard Atmosphere at the altitude of the last `climb` or
`waypoint`: Mach 0.8 at 15,000 ft is 501 kt.

## Check a mission before running it

The quickest check is in the app: run 2 trials at a 1 s time step and look
at the **3D Trajectory** and **Map** tabs. The log also reports the
mission's length and duration.

For more detail, this script flies a mission and reports its duration,
distance, limits, and how close the track passes each waypoint:

```python
--8<-- "docs/examples/mission_summary.py"
```

For the training flight it prints:

```text
--8<-- "docs/examples/mission_summary.txt"
```

The aircraft passes right over the first waypoint, where its holding
pattern starts, and within a kilometre of the last one. It misses the
second by 3 km, for a reason the next section explains, and so passes it
at 3,433 ft, still descending: its closest point to the waypoint comes
before the end of the leg, which reaches 3,000 ft later. At the other two
waypoints the altitudes are 30–40 ft high because of the 1 s time step;
at 0.1 s they are within a few feet.

## How the simulator flies a plan

The simulator turns the plan into a smooth flight that obeys the laws of
motion, rather than one that passes exactly through every point:

- **Turns.** The aircraft turns onto each leg at the departure's
  `nav_bank_angle_deg`, in a coordinated turn: the bank angle follows from
  the speed and the turn rate, and there is no sideslip.
- **Waypoints are approached, not hit.** The bearing and distance to a
  waypoint are worked out from where the previous phase ended, before the
  turn onto the leg. The turn moves the aircraft sideways, so after a large
  turn the leg runs parallel to the planned one, offset by up to the turn's
  diameter: about 9 km at 280 kt and 25° of bank. The leg is then flown at
  a constant heading, which drifts from the great circle on long legs, by
  about 11 km over 1,000 km. On the packaged mission the track passes 3.5
  to 5.6 km from each waypoint. To pass closer, add an intermediate
  waypoint so that the turn before the final leg is small.
- **Climbs and descents on a leg are steady.** A waypoint leg changes
  altitude at a constant angle, set by the leg's length. A short leg with a
  large altitude change therefore makes a steep climb or dive: a 3°
  descent needs about 19 km for every 1,000 m.
- **Speed changes belong in a `speed_ramp`.** A waypoint's `speed_kt`
  takes effect at the end of its turn, so a large change there is abrupt.
- **Transitions are smoothed.** The velocity is smoothed over about two
  seconds, so the bank and pitch change over a few seconds at every phase
  boundary, as in a real aircraft.
- **Altitudes are close, not exact.** Each change of phase can leave the
  altitude off by up to one time step's worth of climb: a few tens of feet
  at a 1 s time step, a few feet at 0.1 s.
- **What is not modelled.** There is no wind, no angle of attack (the
  pitch equals the flight-path angle) and no sideslip. Heights are
  measured from the WGS-84 ellipsoid, and the departure elevation is used
  as such, since ins_sim has no geoid model.

The [Truth trajectory & ideal IMU](../theory/truth-kinematics.md) page
explains how the flown plan becomes the truth and the perfect IMU outputs.

## When a mission fails to load

A mission that does not appear in the **Trajectory** list either has a
YAML syntax error (often a tab, or a missing colon or dash) or has no
`phases` section. **Browse…** says so when you pick it.

A mission that appears but has a mistake stops the run with a
*Simulation failed* message:

| Message | Cause |
| ------- | ----- |
| `ValueError: The first phase must be a 'ground_roll'...` | The first phase is something else, or the phases are out of order. |
| `ValueError: Only the first phase can be a 'ground_roll'` | A second `ground_roll`: there is only one take-off run. |
| `ValueError: Unknown phase type: 'clmb'` | A misspelt phase type. |
| `KeyError: 'speed_kt'` | A phase lacks a required key, here `speed_kt`. The [reference](../reference/mission-yaml.md) lists each phase's keys. |
| `ValueError: speed_end_mach needs an earlier 'climb' or 'waypoint' phase...` | A Mach `speed_ramp` before any phase has set the altitude. |
| `KeyError: 'simulation_time'` | The `simulation_time` section is missing. |
