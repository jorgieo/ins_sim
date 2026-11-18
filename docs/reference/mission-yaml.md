# Mission YAML

Every key of a mission file, with its unit and meaning. The
[Writing a mission](../guide/missions.md) guide explains how the simulator
flies a mission, with a worked example.

Angles are in degrees, altitudes in feet above mean sea level, speeds in
knots, run lengths in metres and durations in seconds. ins_sim converts
them all to SI units when it reads the file; the
[Units & constants](units-constants.md) page gives the factors.

## Top level

| Key | Required | Contents |
| --- | -------- | -------- |
| `name` | no | Free text for your reference. ins_sim does not read it. |
| `departure` | yes | The starting point: see [departure](#departure). |
| `simulation_time` | yes | A default time step: see [simulation_time](#simulation_time). |
| `phases` | yes | The flight, as a list of phases flown in order: see [Phases](#phases). |

The app lists a file as a mission only if it has a `phases` key.

## departure

| Key | Required | Unit | Meaning |
| --- | -------- | ---- | ------- |
| `lat_deg` | yes | ° | Latitude of the starting point, positive north |
| `lon_deg` | yes | ° | Longitude, positive east |
| `alt_ft` | yes | ft | Elevation of the starting point above mean sea level. With no geoid model, ins_sim uses it as the height above the WGS-84 ellipsoid. |
| `nav_bank_angle_deg` | no, default 25 | ° | Bank angle of the turn onto each waypoint leg |

## simulation_time

| Key | Required | Unit | Meaning |
| --- | -------- | ---- | ------- |
| `dt_s` | see below | s | Time step used when a Python script calls `build_trajectory` without one |

The section must be present. `dt_s` is read only by a script that gives
no time step of its own; the app and the command line always use their
own setting.

## Phases

Each phase is a mapping with a `type` key and the keys of its type. Phases
are flown in the order listed, and each starts from the position, heading,
speed and altitude where the previous one ended.

- The **first phase must be a `ground_roll`**, and no other phase can be
  one: every mission starts from rest on the runway.
- Headings are measured clockwise from true north.

### ground_roll

Accelerates steadily from rest at the departure point, along the runway.

| Key | Required | Unit | Meaning |
| --- | -------- | ---- | ------- |
| `heading_deg` | yes | ° | Runway heading |
| `run_length_m` | yes | m | Distance covered while accelerating |
| `speed_final_kt` | yes | kt | Speed at the end of the roll |

### takeoff

Rotates for lift-off: the pitch rises steadily from level, and the speed
changes steadily, over the phase.

| Key | Required | Unit | Meaning |
| --- | -------- | ---- | ------- |
| `heading_deg` | yes | ° | Heading, held throughout: normally the runway heading |
| `pitch_deg` | yes | ° | Pitch at the end of the phase |
| `speed_kt` | yes | kt | Speed at the end of the phase |
| `duration_s` | yes | s | Duration |

### climb

Climbs, or descends, straight ahead on the current heading at a constant
speed and flight-path angle, until it reaches the target altitude.

| Key | Required | Unit | Meaning |
| --- | -------- | ---- | ------- |
| `pitch_deg` | yes | ° | Flight-path angle. Only its size is used: the phase climbs or descends toward `to_altitude_ft`. Must not be 0. |
| `speed_kt` | yes | kt | Speed along the flight path |
| `to_altitude_ft` | yes | ft | Target altitude above mean sea level |

### waypoint

Turns toward the fix at the departure's `nav_bank_angle_deg`, the shorter
way round, then flies straight toward it, changing altitude at a constant
angle. The bearing and distance to the fix are worked out before the
turn, so the leg ends near the fix rather than on it: see
[How the simulator flies a plan](../guide/missions.md#how-the-simulator-flies-a-plan).

| Key | Required | Unit | Meaning |
| --- | -------- | ---- | ------- |
| `lat_deg` | yes | ° | Latitude of the fix |
| `lon_deg` | yes | ° | Longitude of the fix |
| `alt_ft` | yes | ft | Altitude at the end of the leg, above mean sea level |
| `speed_kt` | no | kt | Speed on the leg, from the end of the turn. Without it, the current speed. |
| `loiter` | no | – | Circles to fly at the end of the leg: see below |

The `loiter` block:

| Key | Required | Unit | Meaning |
| --- | -------- | ---- | ------- |
| `n_turns` | yes | – | Number of full circles, a whole number |
| `bank_angle_deg` | yes | ° | Bank angle of the circles. With the speed, it sets their radius. |
| `direction` | no, default `right` | – | `right` (clockwise seen from above) or `left` |

### speed_ramp

Changes speed steadily over the phase, keeping the heading and altitude.
Give the final speed either in knots or as a Mach number.

| Key | Required | Unit | Meaning |
| --- | -------- | ---- | ------- |
| `speed_end_kt` | one of the two | kt | Speed at the end of the phase |
| `speed_end_mach` | one of the two | – | Mach number at the end of the phase. Converted with the speed of sound of the International Standard Atmosphere at the altitude of the last `climb` or `waypoint`, so one of those must come first. Used if both keys are given. |
| `duration_s` | yes | s | Duration |

## Errors

A mistake in the file stops the run with a message that names it. The
[Writing a mission](../guide/missions.md#when-a-mission-fails-to-load)
guide lists the common ones.

## The packaged mission

The app's default mission, `bqn_departure.yaml`:

```yaml
--8<-- "ins_sim/config/bqn_departure.yaml"
```

In Python, [`build_trajectory`][ins_sim.trajectory.kinematics.build_trajectory]
reads a mission file and returns its truth trajectory, and
[`default_trajectory_path`][ins_sim.config.default_trajectory_path] gives
the path of the packaged one.
