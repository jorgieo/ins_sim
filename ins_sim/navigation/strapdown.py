"""Strapdown inertial navigation in the local North-East-Down frame.

`strapdown_navgrade` integrates gyro and accelerometer outputs into
attitude, velocity and curvilinear position on the rotating WGS-84 Earth,
accounting for Earth rate, transport rate, Coriolis and normal gravity.
An optional barometric height measurement stabilizes the vertical channel,
which is unstable when the navigator runs on inertial sensors alone. See the
[Strapdown mechanization][strapdown-mechanization] theory page.
"""

from collections.abc import Sequence
from typing import NamedTuple

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from ins_sim.core.earth_model import earth_rate_n, transport_rate_n, normal_gravity, wgs84_radii


class NavState(NamedTuple):
    """Navigation state at one epoch: the initial condition of the mechanization.

    Attributes:
        L_b: Geodetic latitude [rad].
        lambda_b: Longitude [rad].
        h_b: Geodetic height [m].
        v_eb_n: Velocity of the body relative to the Earth, resolved in
            NED, shape (3,) [m/s].
        q_b_n: Body-to-NED attitude quaternion, scalar-last [x, y, z, w]
            (scipy convention), shape (4,).
    """

    L_b: float
    lambda_b: float
    h_b: float
    v_eb_n: np.ndarray
    q_b_n: np.ndarray


class NavSolution(NamedTuple):
    """Navigation solution history from the strapdown mechanization.

    Attributes:
        r_lb_l: Position relative to the start point, resolved in the
            start point's NED axes, shape (M, 3) [m].
        L_b: Geodetic latitude, shape (M,) [rad].
        lambda_b: Longitude, shape (M,) [rad].
        h_b: Geodetic height, shape (M,) [m].
        v_eb_n: Velocity resolved in NED, shape (M, 3) [m/s].
        q_b_n: Body-to-NED attitude quaternion, scalar-last, shape (M, 4).
    """

    r_lb_l: np.ndarray
    L_b: np.ndarray
    lambda_b: np.ndarray
    h_b: np.ndarray
    v_eb_n: np.ndarray
    q_b_n: np.ndarray


def strapdown_navgrade(meas_omega_ib_b: np.ndarray, meas_f_ib_b: np.ndarray,
                       init_state: NavState | Sequence, tau_i: float,
                       meas_h_b: np.ndarray | None = None,
                       baro_tau: float = 100.0) -> NavSolution:
    """Runs local-level (NED) strapdown integration with rotating-Earth corrections.

    Each step (Groves notation):

    1. Evaluate omega_ie^n, omega_en^n and g(L_b, h_b) at the current state.
    2. Body rate relative to the navigation frame:
       `omega_nb^b = omega_ib^b − C_n^b·(omega_ie^n + omega_en^n)`.
    3. Update C_b^n with the exact rotation-vector exponential of
       omega_nb^b·tau_i.
    4. Resolve specific force into NED: `f_ib^n = C_b^n·f_ib^b`.
    5. Velocity update, forward Euler:
       `dv_eb^n/dt = f_ib^n − (2·omega_ie^n + omega_en^n) × v_eb^n + g_b^n`.
    6. Curvilinear position update, forward Euler.

    Args:
        meas_omega_ib_b: Gyro output, angular rate of the body relative to
            inertial space resolved in body axes, shape (M, 3) [rad/s].
        meas_f_ib_b: Accelerometer output, specific force resolved in body
            axes, shape (M, 3) [m/s²].
        init_state: Initial state (L_b, lambda_b, h_b, v_eb_n, q_b_n): a
            NavState, or any sequence of those five values in that order.
        tau_i: Integration interval [s].
        meas_h_b: Optional barometric height measurement, shape (M,) [m].
            When provided it drives a third-order baro-damped vertical
            channel: the INS-minus-baro height residual feeds proportional
            corrections into the height and vertical-velocity updates plus
            an integral state that absorbs steady vertical accelerometer
            error. This stabilises the inherently unstable vertical channel
            of a free-inertial navigator (the +√(2g/R) eigenvalue from the
            free-air gravity gradient). When None the vertical channel runs
            free-inertial and diverges accordingly. Defaults to None.
        baro_tau: Time constant of the baro-damping loop [s]; the three
            gains place a triple pole at −1/baro_tau. Only used when
            meas_h_b is given. Defaults to 100.0.

    Returns:
        NavSolution: (r_lb_l, L_b, lambda_b, h_b, v_eb_n, q_b_n) at every
            sample; unpacks positionally in that order.

    Examples:
        A level IMU at rest senses only Earth rate and gravity, and the
        navigator stays exactly at rest:

        >>> import numpy as np
        >>> from scipy.spatial.transform import Rotation
        >>> from ins_sim.core.earth_model import earth_rate_n, normal_gravity
        >>> from ins_sim.navigation.strapdown import NavState, strapdown_navgrade
        >>> L_b = np.deg2rad(45.0)
        >>> gyro = np.tile(earth_rate_n(L_b), (100, 1))
        >>> accel = np.tile([0.0, 0.0, -normal_gravity(L_b, 0.0)], (100, 1))
        >>> start = NavState(L_b, 0.0, 0.0, np.zeros(3), Rotation.identity().as_quat())
        >>> nav = strapdown_navgrade(gyro, accel, start, tau_i=0.1)
        >>> float(np.abs(nav.v_eb_n).max())
        0.0
    """
    M = len(meas_omega_ib_b)
    L_b = np.zeros(M); lambda_b = np.zeros(M); h_b = np.zeros(M)
    v_eb_n = np.zeros((M, 3)); q_b_n = np.zeros((M, 4))
    r_lb_l = np.zeros((M, 3))          # for plotting against truth

    L_b[0], lambda_b[0], h_b[0], v_eb_n[0], q_b_n[0] = init_state
    C_b_n = Rot.from_quat(q_b_n[0])
    L_b0 = L_b[0]                      # reference for the local NED display
    h_b0 = h_b[0]
    R_N0, R_E0 = wgs84_radii(L_b0)     # loop-invariant: L_b0 never changes
    g_b_n = np.zeros(3)                # reused each iteration

    # --8<-- [start:baro_gains]
    # Third-order baro-damping gains: triple pole at -1/baro_tau for the
    # (height error, vertical-velocity error, accel-error integrator) loop.
    if meas_h_b is not None:
        K1 = 3.0 / baro_tau
        K2 = 3.0 / baro_tau ** 2
        K3 = 1.0 / baro_tau ** 3
        acc_corr = 0.0                 # integral state [m/s²], → -δa_D
    # --8<-- [end:baro_gains]

    for k in range(M - 1):
        # --8<-- [start:frame_rates]
        # --- 1. Local rotating-Earth quantities at step k --------------
        omega_ie_n = earth_rate_n(L_b[k])
        omega_en_n = transport_rate_n(v_eb_n[k], L_b[k], h_b[k])
        omega_in_n = omega_ie_n + omega_en_n                   # nav-frame rate
        g_b_n[2] = normal_gravity(L_b[k], h_b[k])
        # --8<-- [end:frame_rates]

        # --8<-- [start:body_rate]
        # --- 2. Body rate relative to the nav frame ------------------
        # The gyro saw omega_ib^b. Subtract the nav frame's own rotation
        # (resolved in body axes) to get what drives the b→n attitude update.
        C_n_b = C_b_n.inv()
        omega_nb_b = meas_omega_ib_b[k] - C_n_b.apply(omega_in_n)
        # --8<-- [end:body_rate]

        # --8<-- [start:attitude_update]
        # --- 3. Attitude update (exact rotation-vector exponential) ---
        dC = Rot.from_rotvec(omega_nb_b * tau_i)
        C_b_n_next = C_b_n * dC                                # right-mult: body increment
        q_b_n[k+1] = C_b_n_next.as_quat() # pyright: ignore[reportCallIssue]
        # --8<-- [end:attitude_update]

        # --8<-- [start:specific_force_transform]
        # --- 4. Specific force in NED (forward Euler — consistent with
        #        the truth IMU, which uses the same first-order scheme) ---
        f_ib_n = C_b_n.apply(meas_f_ib_b[k])
        # --8<-- [end:specific_force_transform]

        # --8<-- [start:velocity_update]
        # --- 5. Rotating-frame velocity update ------------------------
        coriolis = np.cross(2.0 * omega_ie_n + omega_en_n, v_eb_n[k])
        v_eb_n_dot = f_ib_n - coriolis + g_b_n
        # Baro damping: the height residual corrects vertical velocity
        # (K2) and updates the accel-error integrator (K3); a positive
        # residual (INS high) commands positive Down velocity.
        if meas_h_b is not None:
            baro_residual = h_b[k] - meas_h_b[k]
            v_eb_n_dot[2] += K2 * baro_residual + acc_corr
            acc_corr += K3 * baro_residual * tau_i
        v_eb_n[k+1] = v_eb_n[k] + v_eb_n_dot * tau_i
        # --8<-- [end:velocity_update]

        # --8<-- [start:position_update]
        # --- 6. Curvilinear position (forward Euler — matches truth) --
        R_N, R_E = wgs84_radii(L_b[k])
        L_b[k+1] = L_b[k] + (v_eb_n[k][0] / (R_N + h_b[k])) * tau_i
        lambda_b[k+1] = lambda_b[k] + (v_eb_n[k][1] /
                                       ((R_E + h_b[k]) * np.cos(L_b[k]))) * tau_i
        h_b[k+1] = h_b[k] - v_eb_n[k][2] * tau_i
        if meas_h_b is not None:
            h_b[k+1] -= K1 * baro_residual * tau_i
        # --8<-- [end:position_update]

        # Position relative to the start point (for plotting against
        # truth): linearized curvilinear-to-metres conversion using the
        # start-point radii R_N0, R_E0.
        # --8<-- [start:relative_position]
        r_lb_l[k+1, 0] = (L_b[k+1] - L_b0) * (R_N0 + h_b0)
        r_lb_l[k+1, 1] = (lambda_b[k+1] - lambda_b[0]) * (R_E0 + h_b0) * np.cos(L_b0)
        r_lb_l[k+1, 2] = -(h_b[k+1] - h_b0)
        # --8<-- [end:relative_position]

        C_b_n = C_b_n_next

    return NavSolution(r_lb_l, L_b, lambda_b, h_b, v_eb_n, q_b_n)
