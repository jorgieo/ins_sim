# Units & constants

Inside the simulator every quantity is in SI units: metres, seconds,
radians and their combinations. Mission files, IMU specs and the app use
the units of aviation and of sensor datasheets instead, and ins_sim
converts them when it reads them. This page lists the conversions and the
physical constants the simulator uses.

## Units in mission files

| Quantity | Unit in the file | SI unit | Conversion |
| -------- | ---------------- | ------- | ---------- |
| Latitude, longitude, heading, angles | degree (°) | rad | × π/180 |
| Altitude | foot (ft) | m | × 0.3048 |
| Speed | knot (kt) | m/s | × 0.514444 (1,852 m per hour) |
| Speed as a Mach number | – | m/s | × the speed of sound: see [Atmosphere](#atmosphere) |
| Ground-roll length | metre (m) | m | – |
| Duration | second (s) | s | – |

## Units in IMU specs

| Quantity | Unit in the file | SI unit | Conversion |
| -------- | ---------------- | ------- | ---------- |
| Angle random walk | °/√h | rad/√s | × π/180/60 = 2.9089 × 10⁻⁴ |
| Gyro bias | °/h | rad/s | × π/180/3600 = 4.8481 × 10⁻⁶ |
| Velocity random walk | m/s/√h | m/s/√s | × 1/60 |
| Accelerometer bias | µg | m/s² | × 9.80665 × 10⁻⁶ |
| Scale factor | ppm | dimensionless | × 10⁻⁶ |
| Misalignment | µrad | rad | × 10⁻⁶ |
| Alignment errors | ° | rad | × π/180 |
| Correlation time | s | s | – |

A microgram-force, µg, is a millionth of standard gravity, 9.80665 m/s²,
whatever the local gravity. The [Modelling an IMU](../guide/imu-specs.md#from-a-datasheet-to-the-yaml)
guide converts other datasheet units.

## Units in the results

The app plots attitude errors in degrees, velocity errors in m/s and
position errors in metres, against time in minutes; the CEP tab plots
nautical miles (NM, 1,852 m) against hours. The 3D view is in kilometres.

## The Earth

ins_sim uses the WGS-84 ellipsoid and its gravity model, described on the
[Earth & gravity](../theory/earth-gravity.md) page:

| Constant | Symbol | Value | In the code |
| -------- | ------ | ----- | ----------- |
| Equatorial radius | \(R_0\) | 6,378,137.0 m | [`WGS84_A`][ins_sim.core.earth_model.WGS84_A] |
| Flattening | \(f\) | 1/298.257223563 | [`WGS84_F`][ins_sim.core.earth_model.WGS84_F] |
| First eccentricity squared | \(e^2\) | 6.69437999 × 10⁻³ | [`WGS84_E2`][ins_sim.core.earth_model.WGS84_E2] |
| Earth rotation rate | \(\omega_{ie}\) | 7.2921151467 × 10⁻⁵ rad/s | [`WGS84_OMEGA`][ins_sim.core.earth_model.WGS84_OMEGA] |
| Gravity at the equator | \(g_e\) | 9.7803253359 m/s² | [`normal_gravity`][ins_sim.core.earth_model.normal_gravity] |
| Somigliana constant | \(k\) | 1.93185265241 × 10⁻³ | [`normal_gravity`][ins_sim.core.earth_model.normal_gravity] |

Standard gravity, 9.80665 m/s², is used only to convert µg, and to turn a
bank angle into a turn radius when a mission is built. Elsewhere the
simulator uses normal gravity, which varies with latitude and height.

## Atmosphere

A Mach number in a mission file becomes a speed through the speed of sound
of the International Standard Atmosphere at the altitude \(h\) of the last
`climb` or `waypoint`:

$$
a = 340.294\ \text{m/s} \times \sqrt{\frac{T}{288.15\ \text{K}}},
\qquad
T = 288.15\ \text{K} - 0.0065\ \text{K/m} \times h
$$

where \(a\) is the speed of sound, \(T\) the air temperature and \(h\) the
altitude in metres. The formula is that of the lowest layer of the
atmosphere, valid up to 11 km (36,089 ft). At 15,000 ft it gives
322.3 m/s, so Mach 0.8 is 501 kt.

## Fixed settings

| Setting | Value | Where it is explained |
| ------- | ----- | --------------------- |
| Random seed of the app | 42 | [User guide](../guide/index.md#running) |
| Time constant of the barometric height loop | 100 s | [Vertical channel](../theory/vertical-channel.md) |
| Smoothing of the mission's velocity | 2 s | [Writing a mission](../guide/missions.md#how-the-simulator-flies-a-plan) |
| Allan-deviation peak of a Gauss–Markov bias, per unit spread | 0.6173642831 | [`GM_ALLAN_PEAK`][ins_sim.sensors.imu.GM_ALLAN_PEAK] |
