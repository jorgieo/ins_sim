# Notation & conventions

Inertial navigation keeps track of several reference frames at once: the
spinning Earth, the local North-East-Down directions, and the vehicle's own
axes. Most confusion in INS mathematics comes from losing track of *which*
frame a quantity describes, *relative to what*, and *along which axes* its
components are written. These docs use the notation of
[Groves (2013)](../reference/bibliography.md#groves2013), which writes all
three facts into every symbol.

This page is the reference for every symbol used elsewhere in the docs. Each
symbol has one meaning on every page. From v0.3.0 the Python code uses the
same scheme, so an equation and the line of code that implements it share
their names.

!!! abstract "Key points"
    - A kinematic symbol reads "**motion of** the object frame, **relative
      to** the reference frame, **resolved in** the axes frame":
      \(\wvec{ib}{b}\) is the angular rate of the body (b) relative to
      inertial space (i), expressed along the body axes (b).
    - \(\Cmat{b}{n}\) transforms a vector's components from body axes to
      North-East-Down axes.
    - A tilde marks a sensor measurement and a hat marks an estimate. Errors
      are always **estimate minus truth**.
    - Everything inside the simulator is in SI units.

## Reading a Groves symbol

Every kinematic vector carries three frame labels:

$$
\huge \boldsymbol{\omega}_{\textcolor[RGB]{232,89,12}{i}\,\textcolor[RGB]{28,126,214}{b}}^{\textcolor[RGB]{47,158,68}{b}}
$$

| Slot | In \(\wvec{ib}{b}\) | Meaning |
| ---- | ------------------- | ------- |
| Main symbol | \(\boldsymbol{\omega}\) | The quantity: here, angular rate. |
| First subscript | **i** | The *reference* frame the motion is measured relative to. Here: inertial space. |
| Second subscript | **b** | The *object* frame whose motion is described. Here: the vehicle body. |
| Superscript | **b** | The *resolving axes*: the frame whose axes the three components are written along. Here: the body axes. |

The resolving axes can change without changing the physical quantity. The
same velocity vector, resolved in two different frames, is related by a
transformation matrix:

$$
\vvec{eb}{n} = \Cmat{b}{n}\,\vvec{eb}{b}
$$

Three symbols are worth reading aloud, because they are what the sensors
measure and what the navigator outputs:

- \(\wvec{ib}{b}\) is what a **gyro triad measures**: the body's rotation
  rate relative to inertial space, along the sensor axes.
- \(\fvec{ib}{b}\) is what an **accelerometer triad measures**: specific
  force (acceleration relative to inertial space, minus gravitation), along
  the sensor axes.
- \(\vvec{eb}{n}\) is the **ground velocity** the navigator reports: the
  body's velocity relative to the Earth, in North, East and Down components.

## Frames

| Frame | Label | Origin | Axes | Role in ins_sim |
| ----- | ----- | ------ | ---- | --------------- |
| Earth-centred inertial (ECI) | i | Earth's centre | Fixed relative to the distant stars | The frame gyros and accelerometers measure motion against. |
| Earth-centred, Earth-fixed (ECEF) | e | Earth's centre | Rotate with the Earth at \(\omega_{ie}\) | Velocity is Earth-referenced. |
| Local navigation (NED) | n | The vehicle | North, East, Down at the current position | Attitude and velocity are resolved here. |
| Body | b | The IMU | x forward, y right, z down | The sensor axes. |
| Local tangent plane | l | The start point | North, East, Down at the start point | Drawing tracks relative to the start. |

## Building blocks

### Transformation matrices

\(\Cmat{\beta}{\alpha}\) transforms a vector's components **from** the
\(\beta\)-frame axes **to** the \(\alpha\)-frame axes. These matrices are
orthonormal, so the inverse transformation is the transpose:

$$
\Cmat{n}{b} = \left(\Cmat{b}{n}\right)^{\mathsf T}
$$

### Skew-symmetric matrices

The cross product \(\mathbf{a}\times\mathbf{b}\) can be written as a matrix
product, \(\skew{\mathbf{a}}\,\mathbf{b}\), with

$$
\skew{\mathbf{a}} =
\begin{bmatrix}
0 & -a_3 & a_2 \\
a_3 & 0 & -a_1 \\
-a_2 & a_1 & 0
\end{bmatrix}.
$$

The upper-case Greek symbol denotes the skew-symmetric form of an angular
rate, e.g. \(\Wmat{ie}{n} = \skew{\wvec{ie}{n}}\).

### Decorations

| Notation | Meaning | Example |
| -------- | ------- | ------- |
| Tilde | Measured by a sensor | \(\wvec[\tilde]{ib}{b}\), the gyro output |
| Hat | Estimated or computed by the navigator | \(\Cmat[\hat]{b}{n}\), the computed attitude |
| \(\delta\) | Error, always **estimate minus truth**: \(\delta x = \hat{x} - x\) | \(\delta\vvec{eb}{n}\) |
| \((-)\), \((+)\) | Value before / after an update step | \(\Cmat{b}{n}(+)\) |
| Subscript \(k\) | Sample (epoch) index | \(\mathbf{x}_k\) |
| \(\tau_i\) | IMU output and integration interval | 0.1 s by default |

## Attitude

The vehicle's attitude is the transformation \(\Cmat{b}{n}\) from body axes
to NED axes. It can also be described by three Euler angles: roll
\(\phi_{nb}\), pitch \(\theta_{nb}\) and heading (yaw) \(\psi_{nb}\).
\(\Cmat{b}{n}\) is built by rotating through heading about the Down axis,
then pitch about the new y axis, then roll about the new x axis: a z-y-x
sequence.

!!! warning "Quaternion component order"
    Groves writes the attitude quaternion \(\qvec{b}{n}\) **scalar-first**,
    \((q_0, q_1, q_2, q_3)\), with \(q_0\) the scalar part. The code stores it
    **scalar-last**, `[x, y, z, w]`, because that is the convention of
    `scipy.spatial.transform.Rotation`. The two describe the same rotation;
    only the storage order differs.

## Position, Earth shape and gravity

| Symbol | Meaning |
| ------ | ------- |
| \(L_b\) | Geodetic latitude of the body |
| \(\lambda_b\) | Longitude of the body |
| \(h_b\) | Geodetic height above the WGS-84 ellipsoid |
| \(R_0\), \(e\), \(f\) | Equatorial radius, eccentricity and flattening of the ellipsoid |
| \(\omega_{ie}\) | Earth rotation rate (a scalar). Its vector form in NED is \(\wvec{ie}{n}\). |
| \(R_N\) | **Meridian** radius of curvature: governs north-south motion |
| \(R_E\) | **Transverse** radius of curvature: governs east-west motion |
| \(\gvec{b}{n}\) | Acceleration due to gravity at the body, resolved in NED (Down-positive) |

$$
R_N = \frac{R_0\,(1-e^2)}{\left(1-e^2\sin^2 L_b\right)^{3/2}},
\qquad
R_E = \frac{R_0}{\sqrt{1-e^2\sin^2 L_b}}
$$

!!! danger "Same letters, different meaning"
    Many geodesy and navigation texts write the meridian radius as \(M\) or
    \(R_M\), and the prime-vertical (normal) radius as \(N\) or \(R_N\). In
    those texts, \(R_N\) is Groves' \(R_E\). ins_sim v0.2 and earlier
    followed that convention. From v0.3.0 the code and these docs use
    Groves' \(R_N\) (meridian) and \(R_E\) (transverse).

!!! note "Height"
    \(h_b\) is height above the ellipsoid. The simulator has no geoid model,
    so mission altitudes given above mean sea level (for example the YAML
    `alt_ft` key) are used directly as ellipsoidal height.

## Errors

Errors are always the estimate minus the truth.

- **Attitude error** \(\delta\psivec{nb}{n}\) is the small rotation that
  takes the true attitude to the computed one:
  \(\Cmat[\hat]{b}{n} \approx \left(\Ithree + \skew{\delta\psivec{nb}{n}}\right)\Cmat{b}{n}\).
  Its components are the North tilt, the East tilt and the Down (heading)
  error.
- **Velocity error** \(\delta\vvec{eb}{n} = \vvec[\hat]{eb}{n} - \vvec{eb}{n}\).
- **Position error** \(\delta\rvec{eb}{n}\), resolved in NED at the true
  position:

$$
\delta\rvec{eb}{n} =
\begin{bmatrix}
\left(R_N + h_b\right)\delta L_b \\
\left(R_E + h_b\right)\cos L_b\,\delta\lambda_b \\
-\delta h_b
\end{bmatrix}
$$

## IMU errors

The sensors' outputs are modelled as

$$
\fvec[\tilde]{ib}{b} = \mathbf{b}_a + \left(\Ithree + \mathbf{M}_a\right)\fvec{ib}{b} + \mathbf{w}_a,
\qquad
\wvec[\tilde]{ib}{b} = \mathbf{b}_g + \left(\Ithree + \mathbf{M}_g\right)\wvec{ib}{b} + \mathbf{w}_g
$$

where each bias is split into a static and a dynamic part,
\(\mathbf{b}_a = \mathbf{b}_{as} + \mathbf{b}_{ad}\) and
\(\mathbf{b}_g = \mathbf{b}_{gs} + \mathbf{b}_{gd}\).

| Term | Meaning | Drawn | `IMUSpec` field | YAML key |
| ---- | ------- | ----- | --------------- | -------- |
| \(\mathbf{b}_{gs}\), \(\mathbf{b}_{as}\) | Static (turn-on) bias | Once per trial | `gyro_br_std`, `accel_br_std` | `repeatability_*` |
| \(\mathbf{b}_{gd}\), \(\mathbf{b}_{ad}\) | Dynamic (in-run) bias: a first-order Gauss–Markov process whose Allan deviation peaks at the datasheet bias instability | Every sample | `*_bias_instability`, `*_bias_tau` | `bias_instab_*`, `bias_tau_s` |
| \(\mathbf{M}_g\), \(\mathbf{M}_a\) | Scale factor (diagonal) and misalignment (off-diagonal) | Once per trial | `*_sf_std`, `*_ma_std` | `scale_factor_ppm`, `misalignment_urad` |
| \(\mathbf{w}_g\), \(\mathbf{w}_a\) | White noise: ARW and VRW | Every sample | `gyro_arw`, `accel_vrw` | `arw_deg_per_rt_hr`, `vrw_m_per_s_per_rt_hr` |
| \(\delta\psivec{nb}{n}(0)\) | Initial alignment error | Once per trial | `align_tilt_std`, `align_heading_std` | `alignment:` block |

Groves' gyro model also includes a g-dependent bias, proportional to
specific force. ins_sim does not model it.

## Symbols and code names

The Python column lists the identifiers used from v0.3.0. They follow one
pattern:

- Vectors are `<quantity>_<reference><object>_<axes>`: \(\vvec{eb}{n}\) is
  `v_eb_n`.
- Transformations are `C_<from>_<to>`: \(\Cmat{b}{n}\) is `C_b_n`.
- Prefixes `true_`, `meas_` and `est_` mark truth, measurement and estimate
  where the role would otherwise be ambiguous.

| Symbol | Meaning | SI unit | Python (v0.3.0) | v0.2 name | Explained in |
| ------ | ------- | ------- | --------------- | --------- | ------------ |
| \(L_b\), \(\lambda_b\), \(h_b\) | Latitude, longitude, height | rad, rad, m | `L_b`, `lambda_b`, `h_b` | `lat`, `lon`, `alt` | Earth & gravity |
| \(R_N\) | Meridian radius of curvature | m | `R_N` | `R_M` ⚠ | Earth & gravity |
| \(R_E\) | Transverse radius of curvature | m | `R_E` | `R_N` ⚠ | Earth & gravity |
| \(\wvec{ie}{n}\) | Earth rate, NED | rad/s | `omega_ie_n` | `w_ie` | Earth & gravity |
| \(\wvec{en}{n}\) | Transport rate, NED | rad/s | `omega_en_n` | `w_en` | Earth & gravity |
| \(\gvec{b}{n}\) | Gravity, NED | m/s² | `g_b_n` | `g_n` | Earth & gravity |
| \(\Cmat{b}{n}\) | Attitude (scipy `Rotation`) | – | `C_b_n` | `R_b2n` | Frames & attitude |
| \(\qvec{b}{n}\) | Attitude quaternion, scalar-last | – | `q_b_n` | `quat` | Frames & attitude |
| \(\phi_{nb}\), \(\theta_{nb}\), \(\psi_{nb}\) | Roll, pitch, heading | rad | `euler_nb` | `euler` | Frames & attitude |
| \(\wvec{ib}{b}\) | True angular rate | rad/s | `omega_ib_b` | `omega_b` | Truth & ideal IMU |
| \(\fvec{ib}{b}\) | True specific force | m/s² | `f_ib_b` | `f_b` | Truth & ideal IMU |
| \(\wvec[\tilde]{ib}{b}\) | Gyro output | rad/s | `meas_omega_ib_b` | `omega_meas` | IMU errors |
| \(\fvec[\tilde]{ib}{b}\) | Accelerometer output | m/s² | `meas_f_ib_b` | `f_meas` | IMU errors |
| \(\wvec{in}{n}\) | Rate of n relative to i | rad/s | `omega_in_n` | `w_in` | Mechanization |
| \(\wvec{nb}{b}\) | Rate of b relative to n | rad/s | `omega_nb_b` | `w_nb_b` | Mechanization |
| \(\vvec{eb}{n}\) | Velocity, NED | m/s | `v_eb_n` | `vel_n`, `v_n` | Mechanization |
| \(\rvec{lb}{l}\) | Position relative to the start point | m | `r_lb_l` | `pos_n` | Mechanization |
| \(\tau_i\) | IMU and integration interval | s | `tau_i` | `dt` | Mechanization |
| \(\avec{ib}{b}\), \(\uvec{ib}{b}\) | Attitude and velocity increments | rad, m/s | `alpha_ib_b`, `upsilon_ib_b` | – | Mechanization |
| \(\delta\psivec{nb}{n}\) | Attitude error | rad | `delta_psi_nb_n` | `mis_rotvec` (initial only) | INS error model |
| \(\delta\vvec{eb}{n}\) | Velocity error | m/s | `delta_v_eb_n` | – | INS error model |
| \(\delta\rvec{eb}{n}\) | Position error, local NED | m | `delta_r_eb_n` | – | INS error model |

User-facing names do not change: YAML keys, command-line flags and GUI
labels keep their datasheet-style names. For example, the GUI's
*Time step (dt)* is \(\tau_i\).

## Units

Everything inside the simulator is SI: radians, metres, seconds, and
quantities derived from them. Mission and IMU YAML files use the units of
aviation charts and sensor datasheets (degrees, feet, knots, °/h, µg), and
are converted to SI as they are loaded. The GUI converts back to familiar
display units (degrees, nautical miles) only when plotting.
