# Strapdown mechanization

The mechanization is the navigator itself: the equations that turn the gyro
and accelerometer outputs back into attitude, velocity and position. It
integrates the measured angular rate into attitude. It uses that attitude to
resolve the measured specific force along North, East and Down, adds
gravity back, and removes the apparent forces of the rotating Earth. It then
integrates the result twice, into velocity and position. Every sensor error
passes through these equations, and the [INS error model](error-model.md)
follows from them. ins_sim implements the local-navigation-frame form of
Groves[^groves].

!!! abstract "Key results"
    - Each step performs four updates (attitude, specific-force resolution,
      velocity, position) after re-evaluating Earth rate, transport rate,
      gravity and the radii of curvature at the current state.
    - The attitude update applies the exact rotation-vector exponential of
      the body's rate relative to NED.
    - The basic velocity and position updates are first-order (forward
      Euler), on purpose: the default truth generator inverts exactly these
      steps.
    - The optional [precision mechanization](#precision-mechanization) adds
      coning, rotation and sculling corrections and a trapezoidal position
      update, for the outputs of a real, integrating IMU. On the packaged
      mission at a 0.1 s step it cuts the navigator's own error from
      1.7 km to about 2 m.

## Physical picture

```mermaid
flowchart LR
    G["Gyros<br/>ω̃<sub>ib</sub><sup>b</sup>"] --> ATT["Attitude update<br/>C<sub>b</sub><sup>n</sup>"]
    A["Accelerometers<br/>f̃<sub>ib</sub><sup>b</sup>"] --> RES["Resolve in NED<br/>f<sub>ib</sub><sup>n</sup> = C<sub>b</sub><sup>n</sup> f̃<sub>ib</sub><sup>b</sup>"]
    ATT --> RES
    RES --> VEL["Velocity update<br/>+ gravity, − Coriolis"]
    VEL --> POS["Position update<br/>L<sub>b</sub>, λ<sub>b</sub>, h<sub>b</sub>"]
    EARTH(["Earth model<br/>ω<sub>ie</sub><sup>n</sup>, ω<sub>en</sub><sup>n</sup>, g, R<sub>N</sub>, R<sub>E</sub>"])
    POS -.-> EARTH
    VEL -.-> EARTH
    EARTH -.-> ATT
    EARTH -.-> VEL
```

The sensors are strapped to the vehicle, so they measure along the body
axes. The navigator keeps track of the attitude and uses it to resolve each
specific-force measurement in NED. What it measures is not acceleration,
though. Specific force excludes gravity, which the navigator must add from
its [gravity model](earth-gravity.md#gravity). It also includes the
apparent accelerations of a rotating, curved frame, which the navigator must
remove: the Coriolis term, and the transport term that keeps a vehicle
following the curved Earth. The Earth model needs the current position and
velocity, so every update feeds the next.

## Continuous-time model

In the local navigation frame, the navigation equations are:

$$
\Cmat[\dot]{b}{n} = \Cmat{b}{n}\Wmat{ib}{b} - \left(\Wmat{ie}{n} + \Wmat{en}{n}\right)\Cmat{b}{n}
$$

$$
\vvec[\dot]{eb}{n} = \Cmat{b}{n}\fvec{ib}{b} + \gvec{b}{n}
- \left(\Wmat{en}{n} + 2\,\Wmat{ie}{n}\right)\vvec{eb}{n}
$$

$$
\dot L_b = \frac{v_N}{R_N + h_b},
\qquad
\dot\lambda_b = \frac{v_E}{(R_E + h_b)\cos L_b},
\qquad
\dot h_b = -v_D
$$

where:

- \(\Cmat{b}{n}\) is the attitude, and \(\vvec{eb}{n} = (v_N, v_E, v_D)\)
  the velocity relative to the Earth, resolved in NED [m/s];
- \(\Wmat{ib}{b}\) and \(\fvec{ib}{b}\) hold the gyro and accelerometer
  outputs [rad/s, m/s²];
- \(\Wmat{ie}{n}\) and \(\Wmat{en}{n}\) are the skew-symmetric matrices of
  Earth rate and transport rate [rad/s];
- \(\gvec{b}{n}\) is gravity [m/s²], and \(L_b\), \(\lambda_b\), \(h_b\),
  \(R_N\), \(R_E\) are as on [Earth & gravity](earth-gravity.md).

The attitude equation is explained on
[Frames & attitude](frames-attitude.md#attitude-kinematics). In the velocity
equation, \(2\Wmat{ie}{n}\vvec{eb}{n}\) is the Coriolis acceleration of a
body moving over the rotating Earth, and \(\Wmat{en}{n}\vvec{eb}{n}\) the
acceleration needed to follow the Earth's curvature. A vehicle at rest
senses only the Earth's rotation and the reaction to gravity, and every
derivative is then zero.

## As implemented

Each step takes the state at sample \(k\) to sample \(k+1\), one IMU
interval \(\tau_i\) later. A tilde marks the sensor outputs; every other
quantity is the navigator's own.

**1. Frame rates and gravity** at the current state:

=== "Equation"

    $$
    \wvec{in}{n} = \wvec{ie}{n}(L_{b,k}) + \wvec{en}{n}(\vvec{eb}{n}(k), L_{b,k}, h_{b,k}),
    \qquad
    \gvec{b}{n} = \begin{bmatrix} 0 & 0 & g(L_{b,k}, h_{b,k}) \end{bmatrix}^{\mathsf T}
    $$

=== "Code"

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:frame_rates"
    ```

**2. Body rate relative to NED.** Remove the NED frame's own rotation,
resolved in body axes, from what the gyros measured:

=== "Equation"

    $$
    \wvec{nb}{b} = \wvec[\tilde]{ib}{b} - \Cmat{n}{b}\,\wvec{in}{n}
    $$

=== "Code"

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:body_rate"
    ```

**3. Attitude update.** Rotate the attitude by the body's rotation over the
step, as an exact rotation vector:

=== "Equation"

    $$
    \Cmat{b}{n}(k+1) = \Cmat{b}{n}(k)\,\exp\left(\skew{\wvec{nb}{b}\,\tau_i}\right)
    $$

    This is the exact solution of the attitude equation when
    \(\wvec{nb}{b}\) is constant over the step.

=== "Code"

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:attitude_update"
    ```

**4. Specific force in NED**, with the attitude at the start of the step:

=== "Equation"

    $$
    \fvec{ib}{n} = \Cmat{b}{n}(k)\,\fvec[\tilde]{ib}{b}
    $$

=== "Code"

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:specific_force_transform"
    ```

**5. Velocity update**, forward Euler. The code also holds the barometric
damping terms, explained on [Vertical channel](vertical-channel.md):

=== "Equation"

    $$
    \vvec{eb}{n}(k+1) = \vvec{eb}{n}(k) + \left[\fvec{ib}{n}
    - \left(2\wvec{ie}{n} + \wvec{en}{n}\right)\times\vvec{eb}{n}(k)
    + \gvec{b}{n}\right]\tau_i
    $$

=== "Code"

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:velocity_update"
    ```

**6. Position update**, forward Euler, with the radii of curvature at the
start of the step:

=== "Equation"

    $$
    L_{b,k+1} = L_{b,k} + \frac{v_{N,k}\,\tau_i}{R_N + h_{b,k}},
    \quad
    \lambda_{b,k+1} = \lambda_{b,k} + \frac{v_{E,k}\,\tau_i}{(R_E + h_{b,k})\cos L_{b,k}},
    \quad
    h_{b,k+1} = h_{b,k} - v_{D,k}\,\tau_i
    $$

=== "Code"

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:position_update"
    ```

Finally, the new position is also converted to metres from the start point,
\(\rvec{lb}{l}\), for drawing tracks (see
[Frames & attitude](frames-attitude.md#as-implemented)).

!!! note "Compared with Groves (2013)"
    This is Groves' local-navigation-frame mechanization[^groves], with
    three deliberate differences:

    - **Attitude.** Groves' basic attitude update is the first-order
      truncation \(\Cmat{b}{n}(\Ithree + \Wmat{ib}{b}\tau_i) -
      \Wmat{in}{n}\Cmat{b}{n}\tau_i\). ins_sim applies the exact
      exponential of the same rotation, which agrees with it to first
      order and keeps the attitude a pure rotation.
    - **Position.** Groves averages the velocities at the start and end of
      the step; ins_sim uses the velocity at the start.
    - **Precision corrections are optional.** Groves' precision
      mechanization corrects for the body turning within each step. ins_sim
      offers it as a [separate mode](#precision-mechanization).

    The first-order choices keep the navigator the exact inverse of the
    default [truth generator](truth-kinematics.md#as-implemented).

## Precision mechanization

A real IMU does not sample the angular rate and specific force at an
instant. It integrates them over each interval and reports the increments:
the attitude increment \(\avec{ib}{b} = \int \wvec{ib}{b}\,dt\) and the
velocity increment \(\uvec{ib}{b} = \int \fvec{ib}{b}\,dt\), both in body
axes. The basic update treats them as rates held constant over the step,
and that misses three effects of the body turning while the IMU
integrates:

- **Rotation.** The specific force is integrated in body axes that keep
  turning, so resolving the whole increment with the start attitude is
  wrong by about \(\tfrac12\,\avec{ib}{b}\times\uvec{ib}{b}\). In a turn at
  rate \(\dot\psi\) and speed \(v\) this is a velocity error along the
  flight path that grows at \(\tfrac12 v\dot\psi^2\tau_i\), 0.03 m/s² for an
  airliner's 3 °/s turn at a 0.1 s step.
- **Coning.** When the rotation axis itself moves within the step, the
  rotation is not \(\avec{ib}{b}\): rotations do not commute. A vibration
  that cones the IMU drifts the attitude even though no gyro has a bias.
- **Sculling.** When a rotation oscillates in phase with a specific force
  along another axis, the true velocity increment differs from
  \(\uvec{ib}{b}\) in a way that does not average out.

ins_sim's precision mechanization corrects all three with Savage's
one-sample-plus-previous algorithms[^savage], which estimate how the rates
change within the step from the previous step's increments. It also
updates the position with the velocities at both ends of the step. The
steps below replace steps 2 to 6; step 1 is unchanged.

**P1. Midpoint rates.** The nav-frame rate, gravity and Coriolis term are
needed over the whole step, so each is extrapolated to its middle from this
step and the previous one:

=== "Equation"

    $$
    \bar{\mathbf{x}} = \tfrac32\,\mathbf{x}_k - \tfrac12\,\mathbf{x}_{k-1},
    \qquad \mathbf{x} \in \left\{\wvec{in}{n},\ \gvec{b}{n} - \left(2\wvec{ie}{n} + \wvec{en}{n}\right)\times\vvec{eb}{n}\right\}
    $$

=== "Code"

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:precision_midpoint"
    ```

**P2. Attitude**, with the coning correction on the body side and the
nav frame's rotation over the step on the other:

=== "Equation"

    $$
    \boldsymbol{\phi}_k = \avec{ib,k}{b} + \tfrac{1}{12}\,\avec{ib,k-1}{b}\times\avec{ib,k}{b},
    \qquad
    \Cmat{b}{n}(k+1) = \exp\left(-\skew{\boldsymbol{\zeta}}\right)\Cmat{b}{n}(k)\exp\left(\skew{\boldsymbol{\phi}_k}\right),
    \qquad
    \boldsymbol{\zeta} = \wvec[\bar]{in}{n}\,\tau_i
    $$

=== "Code"

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:coning"
    ```

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:precision_attitude"
    ```

**P3. Velocity.** The rotation and sculling corrections give the velocity
increment in the body axes at the start of the step, which is resolved in
NED and corrected for half the nav frame's rotation:

=== "Equation"

    $$
    \Delta\mathbf{v}^{b} = \uvec{ib,k}{b} + \tfrac12\,\avec{ib,k}{b}\times\uvec{ib,k}{b}
    + \tfrac{1}{12}\left(\avec{ib,k-1}{b}\times\uvec{ib,k}{b} + \uvec{ib,k-1}{b}\times\avec{ib,k}{b}\right)
    $$

    $$
    \vvec{eb}{n}(k+1) = \vvec{eb}{n}(k) + \Cmat{b}{n}(k)\,\Delta\mathbf{v}^{b}
    - \tfrac12\,\boldsymbol{\zeta}\times\Cmat{b}{n}(k)\,\uvec{ib,k}{b}
    + \overline{\gvec{b}{n} - \left(2\wvec{ie}{n} + \wvec{en}{n}\right)\times\vvec{eb}{n}}\;\tau_i
    $$

    where the overlined term is the midpoint value from P1.

=== "Code"

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:sculling"
    ```

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:precision_velocity"
    ```

**P4. Position**, by the trapezoidal rule:

=== "Equation"

    $$
    h_{b,k+1} = h_{b,k} - \tfrac12\left(v_{D,k} + v_{D,k+1}\right)\tau_i,
    \qquad
    L_{b,k+1} = L_{b,k} + \tfrac12\left(\frac{v_{N,k}}{R_{N,k} + h_{b,k}} + \frac{v_{N,k+1}}{R_{N,k+1} + h_{b,k+1}}\right)\tau_i
    $$

    and likewise for \(\lambda_b\), with \(R_E\) and \(\cos L_b\) at each
    end. The radii at the end of the step are taken at a latitude predicted
    with the start values, which keeps the update second order.

=== "Code"

    ```python
    --8<-- "ins_sim/navigation/strapdown.py:precision_position"
    ```

At the first step there is no previous step: the corrections that need one
are left out, and the midpoint rates are the start values.

!!! warning "Pair the precision mechanization with an integrating IMU"
    The default truth's outputs are the exact inverse of the *basic*
    update, so the basic mechanization reproduces that truth exactly and
    the precision corrections would only add an error. They pay off on
    the outputs of an integrating IMU, which ins_sim generates when the
    truth is [oversampled](truth-kinematics.md#integrating-imu): set
    `oversample` in [`build_trajectory`][ins_sim.trajectory.kinematics.build_trajectory],
    `--oversample` on the command line, or *Truth oversampling* under
    *Advanced* in the app. The GUI and the command line print a note when
    the precision mechanization runs without it.

On an integrating IMU, the basic mechanization's error is first order in
\(\tau_i\): it halves when the step halves. The precision mechanization's
falls at least four times as fast. The
[Validation](../validation/index.md#precision-mechanization) page checks
each correction against the classical coning and sculling motions, and
compares the two mechanizations on the packaged mission.

## What it looks like

![Navigator minus truth, on a logarithmic scale, when the navigator is fed the truth's own IMU outputs](../assets/theory/round_trip.png)

The navigator, fed the ideal outputs of the packaged mission's truth,
reproduces the truth to within \(2.3 \times 10^{-8}\) m over the whole
80 minutes. The difference is floating-point rounding, which accumulates in
small steps. This is the
[zero-noise round trip](../validation/index.md#zero-noise-round-trip) of the
Validation page. It proves that the navigator and the truth generator are
consistent. It cannot show how accurately the navigator integrates
continuous motion, because the truth is built from the navigator's own
discrete steps.

The example below runs the navigator on a level IMU at rest at 45° N. With
perfect sensors it stays exactly at rest: the gyros' Earth rate is removed
by step 2, and the accelerometers' reaction to gravity is cancelled by
step 5. A 0.01 °/h gyro bias moves it 1.3 km in an hour.

## Assumptions & limitations

!!! warning "Assumptions & limitations"
    - **First-order basic update.** The basic velocity and position updates
      are forward Euler, the specific force is resolved with the attitude at
      the start of each step, and there are no coning or sculling
      corrections. Against the default truth these errors are invisible,
      because the truth inverts them exactly (see
      [Truth trajectory & ideal IMU](truth-kinematics.md#assumptions-limitations)).
      An oversampled truth exposes them, and the precision mechanization
      removes most of them.
    - **Precision update to second order.** The precision mechanization's
      remaining error falls with the square of the step or faster. Its
      corrections assume the rates change smoothly over two steps, so a
      manoeuvre much shorter than that, such as a 6 °/s roll-in sampled at
      1 s, still leaves an error.
    - **Known start.** Each run starts from the true position and velocity.
      Only the attitude starts in error, by the drawn alignment error; the
      alignment itself is not simulated.
    - **Vertical channel.** Without a height reference the vertical channel
      diverges. By default a barometric loop holds it (see
      [Vertical channel](vertical-channel.md)).
    - **No aiding.** Apart from the barometer, nothing corrects the
      navigator: there is no GNSS or other measurement update.
    - **Not over the poles**, where the NED frame is undefined.

## In the code

| Quantity | Equation | Implemented in |
| -------- | -------- | -------------- |
| The whole mechanization | Steps 1 to 6, or 1 and P1 to P4 with `mechanization="precision"` | [`strapdown_navgrade`][ins_sim.navigation.strapdown.strapdown_navgrade] |
| Coning correction | \(\boldsymbol{\phi}_k\) | [`coning_correction`][ins_sim.navigation.strapdown.coning_correction] |
| Rotation and sculling corrections | \(\Delta\mathbf{v}^{b}\) | [`sculling_correction`][ins_sim.navigation.strapdown.sculling_correction] |
| Initial state | \(L_b\), \(\lambda_b\), \(h_b\), \(\vvec{eb}{n}\), \(\qvec{b}{n}\) | [`NavState`][ins_sim.navigation.strapdown.NavState] |
| Output history | Every state, plus \(\rvec{lb}{l}\) | [`NavSolution`][ins_sim.navigation.strapdown.NavSolution] |
| Earth rate, transport rate, gravity, radii | Step 1 | [`earth_rate_n`][ins_sim.core.earth_model.earth_rate_n], [`transport_rate_n`][ins_sim.core.earth_model.transport_rate_n], [`normal_gravity`][ins_sim.core.earth_model.normal_gravity], [`wgs84_radii`][ins_sim.core.earth_model.wgs84_radii] |

This example runs the navigator for an hour on a level IMU at rest at 45° N,
first with perfect sensors and then with a 0.01 °/h bias on the gyro that
points East:

=== "Script"

    ```python
    --8<-- "docs/examples/stationary_navigator.py"
    ```

=== "Output"

    ```text
    --8<-- "docs/examples/stationary_navigator.txt"
    ```

The 1.3 km is close to the single-channel prediction for a gyro bias,
\(R\,b_g\,(t - \sin\omega_s t/\omega_s)\) = 1.35 km after an hour (see
[INS error model](error-model.md#gyro-bias)).

This example runs both mechanizations on the packaged mission at 1 s steps
with perfect sensors, first on the default truth and then on an
integrating IMU's outputs:

=== "Script"

    ```python
    --8<-- "docs/examples/precision_mechanization.py"
    ```

=== "Output"

    ```text
    --8<-- "docs/examples/precision_mechanization.txt"
    ```

Each mechanization is exact on the outputs it was built for and about
20 km off on the other's. On the integrating IMU the precision update is
100 times closer; the 198 m it keeps comes from the roll into each turn,
which at 6 °/s is fast for a 1 s step. At the app's 0.1 s it is about 2 m.

## References

[^groves]: [Groves (2013)](../reference/bibliography.md#groves2013), Chapter 5,
    for the inertial navigation equations in the local navigation frame,
    and their precision versions.

[^savage]: [Savage (1998a)](../reference/bibliography.md#savage1998a) and
    [Savage (1998b)](../reference/bibliography.md#savage1998b) derive the
    coning and sculling algorithms, including the one-sample-plus-previous
    forms used here. Groves (2013), Chapter 5, presents them within the
    precision navigation equations.
