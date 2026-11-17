from typing import NamedTuple

import numpy as np
import yaml
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation as Rot

from ins_sim.core.earth_model import (
    WGS84_A, wgs84_radii, earth_rate_n, transport_rate_n, normal_gravity,
)
from ins_sim.trajectory.spline import NEDSplinePath


# =========================================================================
# 1. Navigation-grade truth trajectory
# =========================================================================
def _euler_from_velocity(v_eb_n, tau_i, g_ref):
    """Derives roll/pitch/heading Euler angles from a NED velocity profile.

    Heading and pitch come directly from the velocity direction; roll
    is back-solved from the coordinated-turn condition tan(phi_nb) =
    v_h·psi_dot / g, so that the resulting attitude matches what a
    constant-altitude, slip-free turn would produce.

    Args:
        v_eb_n: Velocity resolved in NED, shape (M, 3) [m/s].
        tau_i: Sample interval [s].
        g_ref: Reference local gravity used in the coordinated-turn
            bank-angle formula [m/s²].

    Returns:
        numpy.ndarray: Euler angles [phi_nb (roll), theta_nb (pitch),
            psi_nb (heading)], shape (M, 3) [rad].
    """
    psi   = np.unwrap(np.arctan2(v_eb_n[:, 1], v_eb_n[:, 0]))
    v_h   = np.linalg.norm(v_eb_n[:, :2], axis=1)
    theta = np.arctan2(-v_eb_n[:, 2], np.maximum(v_h, 1e-6))
    psi_dot = np.gradient(psi, tau_i)
    phi   = np.arctan2(v_h * psi_dot, g_ref)
    return np.column_stack([phi, theta, psi])


class TruthTrajectory:
    """Truth state derived from a raw kinematic profile, consistent with rotating-Earth navigation kinematics.

    Computed at every sample (Groves notation):
      • Position relative to the start point, r_lb^l (linearized
        geodetic reprojection, so it is directly comparable to the
        strapdown output)
      • Velocity v_eb^n and its time derivative (input / numerical
        derivative)
      • Curvilinear position (L_b, lambda_b, h_b) integrated from start
      • Euler angles (input)
      • Angular rate omega_ib^b that a perfect gyro would output —
        this includes Earth rate and transport rate, resolved in body
        axes, via the exact discrete body-to-NED rotation
      • Specific force f_ib^b that a perfect accelerometer would output
        — derived from the forward-difference rotating-frame velocity
        equation so that a forward-Euler strapdown can recover the
        truth exactly

    Attributes:
        tau_i (float): Sample interval [s].
        t (numpy.ndarray): Time vector, shape (M,) [s].
        r_lb_l (numpy.ndarray): Position relative to the start point,
            resolved in the start point's NED axes, shape (M, 3) [m].
        v_eb_n (numpy.ndarray): Velocity of the body relative to the
            Earth, resolved in NED, shape (M, 3) [m/s].
        v_eb_n_dot (numpy.ndarray): Time derivative of v_eb_n (central
            differences, diagnostic only), shape (M, 3) [m/s²].
        L_b (numpy.ndarray): Geodetic latitude, shape (M,) [rad].
        lambda_b (numpy.ndarray): Longitude, shape (M,) [rad].
        h_b (numpy.ndarray): Geodetic height, shape (M,) [m].
        euler_nb (numpy.ndarray): Euler angles [phi_nb (roll), theta_nb
            (pitch), psi_nb (heading)], shape (M, 3) [rad].
        C_b_n (scipy.spatial.transform.Rotation): Body-to-NED attitude
            C_b^n, vectorized over all M samples.
        omega_ib_b (numpy.ndarray): Truth gyro output omega_ib^b, shape
            (M, 3) [rad/s].
        f_ib_b (numpy.ndarray): Truth accelerometer output f_ib^b, shape
            (M, 3) [m/s²].
        g (numpy.ndarray): Local gravity magnitude g(L_b, h_b), shape
            (M,) [m/s²].
    """
    def __init__(self, pos_profile: np.ndarray, v_eb_n: np.ndarray,
                 euler_nb: np.ndarray, tau_i: float,
                 L_b0_deg: float = 38.97, lambda_b0_deg: float = -76.49,
                 h_b0: float = 100.0):
        """Builds the truth trajectory from raw position/velocity/Euler-angle profiles.

        Args:
            pos_profile: Raw NED position profile, shape (M, 3) [m]. Only
                used to determine the sample count; the exposed `r_lb_l`
                attribute is recomputed from the integrated curvilinear
                position (see class docstring).
            v_eb_n: Velocity profile resolved in NED, shape (M, 3) [m/s].
            euler_nb: Euler angles [roll, pitch, heading], shape (M, 3)
                [rad].
            tau_i: Sample interval [s].
            L_b0_deg: Initial geodetic latitude [deg]. Defaults to 38.97.
            lambda_b0_deg: Initial longitude [deg]. Defaults to -76.49.
            h_b0: Initial geodetic height [m]. Defaults to 100.0.
        """
        self.tau_i = tau_i
        L_b0 = np.deg2rad(L_b0_deg)
        lambda_b0 = np.deg2rad(lambda_b0_deg)

        M = len(pos_profile)
        self.t = np.arange(M) * tau_i
        self.v_eb_n = v_eb_n
        self.v_eb_n_dot = np.gradient(v_eb_n, tau_i, axis=0)
        self.euler_nb = euler_nb

        # ---- Curvilinear position by integration --------------------------
        # Integrate (L_b, lambda_b, h_b) using current radii of curvature.
        # Inherently sequential (L_b[k+1] depends on L_b[k]), so this stays
        # a loop.
        # --8<-- [start:truth_position]
        L_b = np.zeros(M); lambda_b = np.zeros(M); h_b = np.zeros(M)
        L_b[0], lambda_b[0], h_b[0] = L_b0, lambda_b0, h_b0
        for k in range(M - 1):
            R_N, R_E = wgs84_radii(L_b[k])
            L_b[k+1] = L_b[k] + (v_eb_n[k, 0] / (R_N + h_b[k])) * tau_i
            lambda_b[k+1] = lambda_b[k] + (v_eb_n[k, 1] /
                                           ((R_E + h_b[k]) * np.cos(L_b[k]))) * tau_i
            h_b[k+1] = h_b[k] - v_eb_n[k, 2] * tau_i
        # --8<-- [end:truth_position]
        self.L_b, self.lambda_b, self.h_b = L_b, lambda_b, h_b

        # Body-to-NED attitude as a vectorized scipy Rotation stack
        self.C_b_n = Rot.from_euler(
            'ZYX', np.column_stack([euler_nb[:, 2], euler_nb[:, 1], euler_nb[:, 0]]))
        C_n_b = self.C_b_n.inv()

        # ---- Truth IMU outputs (exact discrete, vectorized) --------------
        # Consistent with strapdown_navgrade's forward-Euler recursion:
        #   omega_ib_b[k] = exact discrete rotation vector C[k]→C[k+1] / tau_i
        #                   plus Earth/transport rate in body axes
        #   f_ib_b[k]     = forward-difference specific force in NED,
        #                   rotated to body axes:
        #                     f_ib_n[k] = (v[k+1]-v[k])/tau_i + Coriolis[k] - g[k]
        omega_ie_n = earth_rate_n(L_b)                       # (M, 3)
        omega_en_n = transport_rate_n(v_eb_n, L_b, h_b)      # (M, 3)
        g = normal_gravity(L_b, h_b)                         # (M,)
        self.g = g
        g_b_n = np.zeros((M, 3)); g_b_n[:, 2] = g

        # --8<-- [start:truth_specific_force]
        v_eb_n_rate = np.empty((M, 3))
        v_eb_n_rate[:-1] = (v_eb_n[1:] - v_eb_n[:-1]) / tau_i
        v_eb_n_rate[-1]  = v_eb_n_rate[-2]
        f_ib_n = v_eb_n_rate + np.cross(2.0 * omega_ie_n + omega_en_n, v_eb_n) - g_b_n
        self.f_ib_b = C_n_b.apply(f_ib_n)
        # --8<-- [end:truth_specific_force]

        # --8<-- [start:truth_angular_rate]
        dC_b = self.C_b_n[:-1].inv() * self.C_b_n[1:]       # C_b(k+1)^b(k)
        omega_nb_b_fwd = dC_b.as_rotvec() / tau_i            # (M-1, 3)
        omega_nb_b = np.vstack([omega_nb_b_fwd, omega_nb_b_fwd[-1:]])
        self.omega_ib_b = omega_nb_b + C_n_b.apply(omega_ie_n + omega_en_n)
        # --8<-- [end:truth_angular_rate]

        # Reproject truth position through the same linearized geodetic
        # formula that strapdown_navgrade uses, so error =
        # estimated r_lb_l - truth.r_lb_l measures actual navigation error
        # rather than flat-earth vs. curved-earth bias.
        R_N0, R_E0 = wgs84_radii(L_b0)
        r_lb_l = np.zeros((M, 3))
        r_lb_l[:, 0] = (L_b - L_b0)   * (R_N0 + h_b0)
        r_lb_l[:, 1] = (lambda_b - lambda_b[0]) * (R_E0 + h_b0) * np.cos(L_b0)
        r_lb_l[:, 2] = -(h_b - h_b0)
        self.r_lb_l = r_lb_l

    @classmethod
    def from_spline(cls, path: NEDSplinePath, speed: float, tau_i: float,
                     L_b0_deg: float = 38.97, lambda_b0_deg: float = -76.49,
                     h_b0: float = 100.0) -> "TruthTrajectory":
        """Builds a truth trajectory by traversing a spline path at constant speed.

        Args:
            path: NED spline path to traverse.
            speed: Along-path speed [m/s], assumed constant.
            tau_i: Sample interval [s].
            L_b0_deg: Initial geodetic latitude [deg]. Defaults to 38.97.
            lambda_b0_deg: Initial longitude [deg]. Defaults to -76.49.
            h_b0: Initial geodetic height [m]. Defaults to 100.0.

        Returns:
            TruthTrajectory: Truth trajectory over the spline.
        """
        T = path.length / speed
        t = np.arange(0.0, T + tau_i, tau_i)
        s_of_t = np.minimum(speed * t, path.length)
        pos_profile = path.position(s_of_t)
        v_eb_n = np.gradient(pos_profile, tau_i, axis=0)
        # Coordinated-turn bank uses local gravity at the start; for short
        # trajectories using a mean g is well within the rounding error
        # of the heading-rate derivative itself.
        g_ref = normal_gravity(np.deg2rad(L_b0_deg), h_b0)
        euler_nb = _euler_from_velocity(v_eb_n, tau_i, g_ref)
        return cls(pos_profile, v_eb_n, euler_nb, tau_i,
                   L_b0_deg=L_b0_deg, lambda_b0_deg=lambda_b0_deg, h_b0=h_b0)


# =========================================================================
# 2. Phase helpers
# =========================================================================

def _ground_roll(hdg_deg, v_final, run_len, dt):
    """Generates a constant-acceleration ground-roll (takeoff run) segment.

    Args:
        hdg_deg: Ground-roll heading [deg].
        v_final: Speed at end of roll [m/s].
        run_len: Ground-roll distance [m].
        dt: Sample interval [s].

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]: (pos, vel), NED position
            and velocity arrays, each shape (N, 3).
    """
    a     = v_final ** 2 / (2.0 * run_len)
    t_end = v_final / a
    N     = max(2, int(t_end / dt) + 1)
    t_    = np.arange(N) * dt
    spd   = np.minimum(a * t_, v_final)
    dist  = np.where(t_ >= t_end, run_len, 0.5 * a * t_ ** 2)
    hdg   = np.deg2rad(hdg_deg)
    pos   = np.zeros((N, 3))
    pos[:, 0] = dist * np.cos(hdg)
    pos[:, 1] = dist * np.sin(hdg)
    vel   = np.zeros((N, 3))
    vel[:, 0] = spd * np.cos(hdg)
    vel[:, 1] = spd * np.sin(hdg)
    return pos, vel


def _climb(entry, hdg_deg, speed, alt_ned_start, alt_ned_end, pitch_deg=10.0, dt=0.1):
    """Generates a constant-pitch, constant-speed climb (or descent) segment.

    Args:
        entry: Entry NED position, shape (3,) [m].
        hdg_deg: Ground-track heading [deg].
        speed: Airspeed along the climb path [m/s].
        alt_ned_start: Starting NED Down coordinate [m].
        alt_ned_end: Ending NED Down coordinate [m].
        pitch_deg: Pitch angle γ [deg]. Defaults to 10.0.
        dt: Sample interval [s]. Defaults to 0.1.

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]: (pos, vel), NED position
            and velocity arrays, each shape (N, 3). The Down component
            of velocity is negative while climbing.
    """
    hdg   = np.deg2rad(hdg_deg)
    gamma = np.deg2rad(pitch_deg)
    v_h   = speed * np.cos(gamma)
    v_d   = -speed * np.sin(gamma)   # Down < 0 while climbing
    dur   = abs(alt_ned_end - alt_ned_start) / (speed * np.sin(gamma))
    N     = max(2, int(dur / dt) + 1)
    t_    = np.arange(N) * dt
    pos   = np.zeros((N, 3))
    pos[:, 0] = entry[0] + v_h * np.cos(hdg) * t_
    pos[:, 1] = entry[1] + v_h * np.sin(hdg) * t_
    pos[:, 2] = alt_ned_start + v_d * t_
    vel   = np.tile([v_h * np.cos(hdg), v_h * np.sin(hdg), v_d], (N, 1))
    return pos, vel


def _straight(entry, hdg_deg, dist_m, speed, dt):
    """Generates a straight, level, constant-speed segment.

    Args:
        entry: Entry NED position, shape (3,) [m].
        hdg_deg: Heading [deg].
        dist_m: Distance to travel [m].
        speed: Speed [m/s].
        dt: Sample interval [s].

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]: (pos, vel), NED position
            and velocity arrays, each shape (N, 3).
    """
    hdg = np.deg2rad(hdg_deg)
    N   = max(2, int(dist_m / (speed * dt)) + 1)
    t_  = np.arange(N) * dt
    pos = np.zeros((N, 3))
    pos[:, 0] = entry[0] + speed * np.cos(hdg) * t_
    pos[:, 1] = entry[1] + speed * np.sin(hdg) * t_
    pos[:, 2] = entry[2]
    vel = np.tile([speed * np.cos(hdg), speed * np.sin(hdg), 0.0], (N, 1))
    return pos, vel


def _turn(entry, hdg_start_deg, hdg_end_deg, speed, alt_ned, R_turn, dt):
    """Generates a coordinated, constant-altitude horizontal turn.

    Direction is chosen as the shortest arc from hdg_start_deg to
    hdg_end_deg.

    Args:
        entry: Entry NED position, shape (3,) [m].
        hdg_start_deg: Initial heading [deg].
        hdg_end_deg: Target heading [deg].
        speed: Speed [m/s], assumed constant through the turn.
        alt_ned: Constant NED Down coordinate during the turn [m].
        R_turn: Turn radius [m].
        dt: Sample interval [s].

    Returns:
        tuple[numpy.ndarray, numpy.ndarray, float]: (pos, vel,
            exit_hdg), NED position and velocity arrays (each shape
            (N, 3)), and the exiting heading [deg].
    """
    turn_rate = speed / R_turn
    hdg0  = np.deg2rad(hdg_start_deg)
    delta = np.deg2rad(hdg_end_deg) - hdg0
    delta = (delta + np.pi) % (2 * np.pi) - np.pi   # wrap to (−π, π]
    sign  = float(np.sign(delta)) if delta != 0.0 else 1.0
    N     = max(2, int(abs(delta) / turn_rate / dt) + 1)
    t_    = np.arange(N) * dt
    hdg_t = hdg0 + sign * turn_rate * t_
    vR    = speed / (sign * turn_rate)
    pos   = np.zeros((N, 3))
    pos[:, 0] = entry[0] + vR * (np.sin(hdg_t) - np.sin(hdg0))
    pos[:, 1] = entry[1] + vR * (np.cos(hdg0)  - np.cos(hdg_t))
    pos[:, 2] = alt_ned
    vel   = np.zeros((N, 3))
    vel[:, 0] = speed * np.cos(hdg_t)
    vel[:, 1] = speed * np.sin(hdg_t)
    exit_hdg = float(np.rad2deg(hdg_t[-1]))
    return pos, vel, exit_hdg


def _loiter(entry, hdg_deg, speed, alt_ned, n_revs, R_turn, direction='right', dt=0.1):
    """Generates n complete horizontal circles at constant altitude.

    Args:
        entry: Entry NED position, shape (3,) [m].
        hdg_deg: Initial heading [deg].
        speed: Speed [m/s], assumed constant through the loiter.
        alt_ned: Constant NED Down coordinate during the loiter [m].
        n_revs: Number of complete revolutions.
        R_turn: Turn radius [m].
        direction: Turn direction, 'right' or 'left'. Defaults to
            'right'.
        dt: Sample interval [s]. Defaults to 0.1.

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]: (pos, vel), NED position
            and velocity arrays, each shape (N, 3).
    """
    turn_rate = speed / R_turn
    sign  = 1.0 if direction == 'right' else -1.0
    hdg0  = np.deg2rad(hdg_deg)
    dur   = n_revs * 2.0 * np.pi / turn_rate
    N     = max(2, int(dur / dt) + 1)
    t_    = np.arange(N) * dt
    hdg_t = hdg0 + sign * turn_rate * t_
    vR    = speed / (sign * turn_rate)
    pos   = np.zeros((N, 3))
    pos[:, 0] = entry[0] + vR * (np.sin(hdg_t) - np.sin(hdg0))
    pos[:, 1] = entry[1] + vR * (np.cos(hdg0)  - np.cos(hdg_t))
    pos[:, 2] = alt_ned
    vel   = np.zeros((N, 3))
    vel[:, 0] = speed * np.cos(hdg_t)
    vel[:, 1] = speed * np.sin(hdg_t)
    return pos, vel


def _pitch_transition(entry, hdg_deg, speed, pitch_start_deg, pitch_end_deg,
                      v_end=None, dur=20.0, dt=0.1):
    """Smoothly ramps pitch angle, and optionally speed, over `dur` seconds at constant heading.

    Args:
        entry: Entry NED position, shape (3,) [m].
        hdg_deg: Heading [deg], held constant.
        speed: Initial speed [m/s].
        pitch_start_deg: Initial pitch angle [deg].
        pitch_end_deg: Final pitch angle [deg].
        v_end: Final speed [m/s]. Defaults to `speed` (no speed
            change) when None.
        dur: Duration of the ramp [s]. Defaults to 20.0.
        dt: Sample interval [s]. Defaults to 0.1.

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]: (pos, vel), NED position
            and velocity arrays, each shape (N, 3).
    """
    if v_end is None:
        v_end = speed
    N    = max(2, int(dur / dt) + 1)
    t_   = np.arange(N) * dt
    alpha = t_ / t_[-1]
    pitch = np.deg2rad(pitch_start_deg + (pitch_end_deg - pitch_start_deg) * alpha)
    spd   = speed + (v_end - speed) * alpha
    hdg   = np.deg2rad(hdg_deg)
    v_h   = spd * np.cos(pitch)
    v_d   = -spd * np.sin(pitch)
    pos   = np.zeros((N, 3))
    pos[0] = entry
    for k in range(N - 1):
        pos[k+1, 0] = pos[k, 0] + v_h[k] * np.cos(hdg) * dt
        pos[k+1, 1] = pos[k, 1] + v_h[k] * np.sin(hdg) * dt
        pos[k+1, 2] = pos[k, 2] + v_d[k] * dt
    vel          = np.zeros((N, 3))
    vel[:, 0]    = v_h * np.cos(hdg)
    vel[:, 1]    = v_h * np.sin(hdg)
    vel[:, 2]    = v_d
    return pos, vel


def _takeoff(entry, hdg_deg, speed, pitch_deg, speed_end, duration_s, dt=0.1):
    """Generates the rotation/takeoff phase: pitch and speed ramp at fixed heading.

    Args:
        entry: Entry NED position, shape (3,) [m].
        hdg_deg: Heading [deg], held constant.
        speed: Initial speed [m/s].
        pitch_deg: Final pitch angle [deg], ramped from 0.
        speed_end: Final speed [m/s].
        duration_s: Duration of the rotation [s].
        dt: Sample interval [s]. Defaults to 0.1.

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]: (pos, vel), NED position
            and velocity arrays, each shape (N, 3).
    """
    return _pitch_transition(
        entry, hdg_deg, speed,
        pitch_start_deg=0.0, pitch_end_deg=pitch_deg,
        v_end=speed_end, dur=duration_s, dt=dt)


def _speed_ramp(entry, hdg_deg, v_start, v_end, alt_ned, dur=20.0, dt=0.1):
    """Generates a linear speed change at constant heading and altitude.

    Args:
        entry: Entry NED position (first two components used), shape
            (3,) [m].
        hdg_deg: Heading [deg], held constant.
        v_start: Initial speed [m/s].
        v_end: Final speed [m/s].
        alt_ned: Constant NED Down coordinate [m].
        dur: Duration of the ramp [s]. Defaults to 20.0.
        dt: Sample interval [s]. Defaults to 0.1.

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]: (pos, vel), NED position
            and velocity arrays, each shape (N, 3).
    """
    N    = max(2, int(dur / dt) + 1)
    t_   = np.arange(N) * dt
    spd  = v_start + (v_end - v_start) * t_ / t_[-1]
    hdg  = np.deg2rad(hdg_deg)
    pos  = np.zeros((N, 3))
    pos[0] = [entry[0], entry[1], alt_ned]
    for k in range(N - 1):
        pos[k+1, 0] = pos[k, 0] + spd[k] * np.cos(hdg) * dt
        pos[k+1, 1] = pos[k, 1] + spd[k] * np.sin(hdg) * dt
        pos[k+1, 2] = alt_ned
    vel       = np.zeros((N, 3))
    vel[:, 0] = spd * np.cos(hdg)
    vel[:, 1] = spd * np.sin(hdg)
    return pos, vel


def _geodetic_bearing(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    """Computes the forward azimuth from one geodetic point to another.

    Args:
        lat1_deg: Latitude of point 1 [deg].
        lon1_deg: Longitude of point 1 [deg].
        lat2_deg: Latitude of point 2 [deg].
        lon2_deg: Longitude of point 2 [deg].

    Returns:
        float: Forward azimuth from point 1 to point 2 [deg], in
            [0, 360).
    """
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1_deg, lon1_deg, lat2_deg, lon2_deg])
    dlon = lon2 - lon1
    x = np.sin(dlon) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
    return float(np.degrees(np.arctan2(x, y)) % 360)


def _geodetic_distance(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    """Computes great-circle distance between two geodetic points via the Haversine formula.

    Args:
        lat1_deg: Latitude of point 1 [deg].
        lon1_deg: Longitude of point 1 [deg].
        lat2_deg: Latitude of point 2 [deg].
        lon2_deg: Longitude of point 2 [deg].

    Returns:
        float: Great-circle distance [m], using the WGS-84 semi-major
            axis as the sphere radius.
    """
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1_deg, lon1_deg, lat2_deg, lon2_deg])
    dlat = lat2 - lat1; dlon = lon2 - lon1
    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    return 2.0 * WGS84_A * np.arcsin(np.sqrt(a))


def _approx_geodetic(pos_ned, lat0_deg, lon0_deg, alt0_msl):
    """Converts a flat-Earth NED offset to an approximate geodetic position.

    Args:
        pos_ned: NED offset from the reference point, shape (3,) [m].
        lat0_deg: Reference latitude [deg].
        lon0_deg: Reference longitude [deg].
        alt0_msl: Reference altitude [m MSL].

    Returns:
        tuple[float, float, float]: (lat_deg, lon_deg, alt_msl_m),
            approximate geodetic position.
    """
    lat = lat0_deg + np.degrees(pos_ned[0] / WGS84_A)
    lon = lon0_deg + np.degrees(pos_ned[1] / (WGS84_A * np.cos(np.radians(lat0_deg))))
    alt = alt0_msl - pos_ned[2]
    return lat, lon, alt


# =========================================================================
# 3. YAML-driven trajectory builder
# =========================================================================
class MissionTrajectory(NamedTuple):
    """Result of building a mission from its YAML definition.

    Attributes:
        truth: Truth trajectory over the whole mission.
        v_sprint: Sprint-leg speed [m/s] if a Mach-targeted speed_ramp
            phase is present, else None.
        R_turn: Turn radius [m] of the first coordinated turn
            encountered, else None.
    """

    truth: TruthTrajectory
    v_sprint: float | None
    R_turn: float | None


def build_trajectory(yaml_path: str, tau_i: float | None = None) -> MissionTrajectory:
    """Builds a phase-by-phase truth trajectory from a YAML mission definition.

    Each phase in the YAML maps to a helper function; fields omitted
    from a phase are inherited from the running state (heading, speed,
    altitude).

    Args:
        yaml_path: Path to the YAML mission-phase definition file.
        tau_i: Sample interval [s]. When None, defaults to the file's
            `simulation_time.dt_s` value.

    Returns:
        MissionTrajectory: (truth, v_sprint, R_turn); unpacks
            positionally as ``truth, v_sprint, R_turn = build_trajectory(...)``.

    Raises:
        ValueError: If a phase has an unrecognized `type`, or if a
            `speed_end_mach` speed_ramp phase appears before any
            `climb` phase.
    """
    with open(yaml_path) as fh:
        cfg = yaml.safe_load(fh)

    FT = 0.3048
    KT = 0.514444
    NM = 1852.0
    g0 = 9.80665

    dep    = cfg["departure"]
    sim    = cfg["simulation_time"]
    phases = cfg["phases"]

    lat0_deg     = float(dep["lat_deg"])
    lon0_deg     = float(dep["lon_deg"])
    alt0_msl     = float(dep["alt_ft"]) * FT
    nav_bank_deg = float(dep.get("nav_bank_angle_deg", 25.0))

    if tau_i is None:
        tau_i = float(sim["dt_s"])

    def _isa_speed(mach, alt_m_msl):
        T_isa = 288.15 - 0.0065 * alt_m_msl
        return mach * 340.294 * np.sqrt(T_isa / 288.15)

    ALT_CRUISE     = None
    ALT_NED_CRUISE = None
    v_sprint       = None
    R_turn_global  = None

    state = {
        "hdg_deg":  None,
        "speed":    0.0,
        "alt_ned":  0.0,
        "pos_last": None,
    }

    segs_pos = []
    segs_vel = []

    for phase in phases:
        ptype = phase["type"]

        if ptype == "ground_roll":
            v_final = float(phase["speed_final_kt"]) * KT
            pos, vel = _ground_roll(
                float(phase["heading_deg"]), v_final,
                float(phase["run_length_m"]), tau_i)
            state["hdg_deg"] = float(phase["heading_deg"])
            state["speed"]   = v_final

        elif ptype == "takeoff":
            v_end_val = float(phase["speed_kt"]) * KT
            pos, vel = _takeoff(
                state["pos_last"], float(phase["heading_deg"]), state["speed"],
                float(phase["pitch_deg"]), v_end_val,
                float(phase["duration_s"]), tau_i)
            state["hdg_deg"] = float(phase["heading_deg"])
            state["speed"]   = v_end_val

        elif ptype == "climb":
            ALT_CRUISE     = float(phase["to_altitude_ft"]) * FT
            ALT_NED_CRUISE = -(ALT_CRUISE - alt0_msl)
            climb_speed    = float(phase["speed_kt"]) * KT
            pos, vel = _climb(
                state["pos_last"], state["hdg_deg"], climb_speed,
                state["pos_last"][2], ALT_NED_CRUISE,
                pitch_deg=float(phase["pitch_deg"]), dt=tau_i)
            state["speed"]   = climb_speed
            state["alt_ned"] = ALT_NED_CRUISE

        elif ptype == "waypoint":
            wp_lat   = float(phase["lat_deg"])
            wp_lon   = float(phase["lon_deg"])
            wp_alt_m = float(phase["alt_ft"]) * FT
            # speed_kt is optional: without it the leg is flown at the
            # current speed (e.g. a sprint speed set by a preceding
            # speed_ramp), avoiding a nonphysical speed step at the
            # turn exit.
            wp_speed = (float(phase["speed_kt"]) * KT
                        if "speed_kt" in phase else state["speed"])
            wp_alt_ned = -(wp_alt_m - alt0_msl)

            curr_lat, curr_lon, _ = _approx_geodetic(
                state["pos_last"], lat0_deg, lon0_deg, alt0_msl)
            bearing_deg = _geodetic_bearing(curr_lat, curr_lon, wp_lat, wp_lon)
            dist_h_m    = _geodetic_distance(curr_lat, curr_lon, wp_lat, wp_lon)

            # Turn to waypoint bearing
            R_nav = state["speed"] ** 2 / (g0 * np.tan(np.radians(nav_bank_deg)))
            if R_turn_global is None:
                R_turn_global = R_nav
            pos_t, vel_t, exit_hdg = _turn(
                state["pos_last"], state["hdg_deg"], bearing_deg,
                state["speed"], state["alt_ned"], R_nav, tau_i)
            state["hdg_deg"] = exit_hdg

            # Fly to waypoint (with altitude change if needed)
            alt_diff = wp_alt_ned - state["alt_ned"]
            if abs(alt_diff) > 1.0:
                implied_pitch = float(np.degrees(np.arctan2(-alt_diff, max(dist_h_m, 1.0))))
                pos_s, vel_s = _climb(
                    pos_t[-1], exit_hdg, wp_speed,
                    state["alt_ned"], wp_alt_ned,
                    pitch_deg=implied_pitch, dt=tau_i)
            else:
                pos_s, vel_s = _straight(pos_t[-1], exit_hdg, dist_h_m, wp_speed, tau_i)

            ALT_CRUISE     = wp_alt_m
            state["speed"]   = wp_speed
            state["alt_ned"] = wp_alt_ned

            # Concatenate turn + transit into one block
            sub_pos = np.vstack([pos_t, pos_s[1:]])
            sub_vel = np.vstack([vel_t, vel_s[1:]])

            # Optional loiter on arrival
            loiter_cfg = phase.get("loiter")
            if loiter_cfg is not None:
                bank_rad = np.radians(float(loiter_cfg["bank_angle_deg"]))
                R_loit   = state["speed"] ** 2 / (g0 * np.tan(bank_rad))
                if R_turn_global is None:
                    R_turn_global = R_loit
                pos_l, vel_l = _loiter(
                    sub_pos[-1], state["hdg_deg"], state["speed"],
                    state["alt_ned"], int(loiter_cfg["n_turns"]),
                    R_loit, direction=loiter_cfg.get("direction", "right"), dt=tau_i)
                sub_pos = np.vstack([sub_pos, pos_l[1:]])
                sub_vel = np.vstack([sub_vel, vel_l[1:]])

            pos = sub_pos
            vel = sub_vel

        elif ptype == "speed_ramp":
            v_start = state["speed"]
            if "speed_end_mach" in phase:
                if ALT_CRUISE is None:
                    raise ValueError(
                        "speed_end_mach requires a prior 'climb' phase")
                v_end_val = _isa_speed(float(phase["speed_end_mach"]), ALT_CRUISE)
                v_sprint = v_end_val
            else:
                v_end_val = float(phase["speed_end_kt"]) * KT
            pos, vel = _speed_ramp(
                state["pos_last"], state["hdg_deg"],
                v_start, v_end_val, state["alt_ned"],
                dur=float(phase["duration_s"]), dt=tau_i)
            state["speed"] = v_end_val

        else:
            raise ValueError(f"Unknown phase type: {ptype!r}")

        if not segs_pos:
            segs_pos.append(pos)
            segs_vel.append(vel)
        else:
            segs_pos.append(pos[1:])
            segs_vel.append(vel[1:])
        state["pos_last"] = pos[-1]

    all_pos = np.vstack(segs_pos)
    all_vel = np.vstack(segs_vel)

    # Guard heading at t=0 against zero-velocity divide
    hdg0_rad = np.deg2rad(phases[0]["heading_deg"])
    all_vel[0] = np.array([np.cos(hdg0_rad), np.sin(hdg0_rad), 0.0]) * 1e-9

    # Smooth velocity to eliminate C1 kinks at phase boundaries.
    # Sigma ~2 s → roll/pitch transitions over 4–6 s, matching transport-category dynamics.
    _att_sigma = max(1, int(2.0 / tau_i))
    all_vel = gaussian_filter1d(all_vel, sigma=_att_sigma, axis=0)

    g_ref = normal_gravity(np.deg2rad(lat0_deg), alt0_msl)
    euler = _euler_from_velocity(all_vel, tau_i, g_ref)

    # The departure elevation is given above mean sea level; with no geoid
    # model it is used directly as the geodetic height h_b.
    truth = TruthTrajectory(all_pos, all_vel, euler, tau_i,
                            L_b0_deg=lat0_deg, lambda_b0_deg=lon0_deg,
                            h_b0=alt0_msl)
    return MissionTrajectory(truth, v_sprint, R_turn_global)
