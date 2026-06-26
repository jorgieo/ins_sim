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
class TruthTrajectory:
    """
    Truth state derived from the spline, expressed in a way that is
    self-consistent with rotating-Earth navigation kinematics.

    Computed at every sample:
      • NED position relative to start (from the spline)
      • NED velocity, NED inertial-acceleration  (from numerical derivatives)
      • Geodetic position (lat, lon, alt) integrated from start
      • Body Euler angles (heading and pitch from velocity, roll from
        coordinated-turn condition)
      • Body angular rate ω_ib_b that a perfect gyro would output —
        this includes Earth rate and transport rate, expressed in body
      • Specific force f_b that a perfect accelerometer would output —
        derived from the rotating-frame velocity equation so that a
        Coriolis-aware strapdown can recover the truth exactly
    """
    def __init__(self, path: NEDSplinePath, speed: float, dt: float,
                 lat0_deg: float = 38.97, lon0_deg: float = -76.49,
                 alt0: float = 100.0):
        self.dt = dt
        lat0 = np.deg2rad(lat0_deg)
        lon0 = np.deg2rad(lon0_deg)

        # ---- Time grid and position --------------------------------------
        T = path.length / speed
        self.t   = np.arange(0.0, T + dt, dt)
        s_of_t   = np.minimum(speed * self.t, path.length)
        self.pos_n = path.position(s_of_t)                          # (M, 3)
        M = len(self.t)

        # ---- Velocity & acceleration in NED ------------------------------
        # The numerical derivative in NED gives the rate of change of the
        # NED *components* — exactly the quantity the rotating-frame
        # velocity equation v̇_n = f_n − (2ω_ie + ω_en)×v_n + g_n refers to.
        self.vel_n = np.gradient(self.pos_n, dt, axis=0)
        self.acc_n = np.gradient(self.vel_n, dt, axis=0)

        # ---- Geodetic position by integration ----------------------------
        # Integrate (φ, λ, h) using current radii of curvature.
        # For typical short trajectories (< 100 km) this matters mostly
        # because Earth-rate components and gravity depend on latitude.
        lat = np.zeros(M); lon = np.zeros(M); alt = np.zeros(M)
        lat[0], lon[0], alt[0] = lat0, lon0, alt0
        for k in range(M - 1):
            R_M, R_N = wgs84_radii(lat[k])
            lat[k+1] = lat[k] + (self.vel_n[k, 0] / (R_M + alt[k])) * dt
            lon[k+1] = lon[k] + (self.vel_n[k, 1] /
                                 ((R_N + alt[k]) * np.cos(lat[k]))) * dt
            alt[k+1] = alt[k] - self.vel_n[k, 2] * dt
        self.lat, self.lon, self.alt = lat, lon, alt

        # ---- Euler angles -------------------------------------------------
        psi   = np.unwrap(np.arctan2(self.vel_n[:, 1], self.vel_n[:, 0]))
        v_h   = np.linalg.norm(self.vel_n[:, :2], axis=1)
        theta = np.arctan2(-self.vel_n[:, 2], v_h)
        psi_dot = np.gradient(psi, dt)
        # Coordinated-turn bank uses local gravity at the start; for short
        # trajectories using a mean g is well within the rounding error
        # of the heading-rate derivative itself.
        g_ref = normal_gravity(lat0, alt0)
        phi   = np.arctan2(v_h * psi_dot, g_ref)
        self.euler = np.column_stack([phi, theta, psi])

        # Body→NED rotation as a vectorized scipy Rotation stack
        self.R_b2n = Rot.from_euler('ZYX', np.column_stack([psi, theta, phi]))

        # Body rate of body wrt NED, in body — Euler kinematic transformation
        phi_dot   = np.gradient(phi, dt)
        theta_dot = np.gradient(theta, dt)
        sphi, cphi = np.sin(phi),   np.cos(phi)
        sth,  cth  = np.sin(theta), np.cos(theta)
        omega_nb_b = np.column_stack([
            phi_dot              -  sth * psi_dot,
            cphi * theta_dot     +  cth * sphi * psi_dot,
            -sphi * theta_dot    +  cth * cphi * psi_dot,
        ])

        # ---- Truth IMU outputs -------------------------------------------
        # Per sample:
        #   ω_ib_b = ω_nb_b + C_n^b · (ω_ie_n + ω_en_n)
        #   f_b    = C_n^b · [a_n + (2ω_ie_n + ω_en_n) × v_n − g_n]
        # The expression in brackets is the f_n that, when fed into
        # v̇_n = f_n − (2ω_ie + ω_en)×v_n + g_n, reproduces a_n exactly.
        omega_ib_b = np.zeros((M, 3))
        f_b        = np.zeros((M, 3))
        g_arr      = np.zeros(M)

        R_n2b = self.R_b2n.inv()
        for k in range(M):
            w_ie = earth_rate_n(lat[k])
            w_en = transport_rate_n(self.vel_n[k], lat[k], alt[k])
            g_k  = normal_gravity(lat[k], alt[k]); g_arr[k] = g_k
            g_n  = np.array([0.0, 0.0, g_k])

            f_n_k = self.acc_n[k] + np.cross(2.0 * w_ie + w_en, self.vel_n[k]) - g_n
            f_b[k]        = R_n2b[k].apply(f_n_k)
            omega_ib_b[k] = omega_nb_b[k] + R_n2b[k].apply(w_ie + w_en)

        self.omega_b = omega_ib_b
        self.f_b     = f_b
        self.g_loc   = g_arr


# =========================================================================
# 2. Phase helpers
# =========================================================================

def _ground_roll(hdg_deg, v_final, run_len, dt):
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
    """Coordinated horizontal turn; direction chosen as shortest arc."""
    omega = speed / R_turn
    hdg0  = np.deg2rad(hdg_start_deg)
    delta = np.deg2rad(hdg_end_deg) - hdg0
    delta = (delta + np.pi) % (2 * np.pi) - np.pi   # wrap to (−π, π]
    sign  = float(np.sign(delta)) if delta != 0.0 else 1.0
    N     = max(2, int(abs(delta) / omega / dt) + 1)
    t_    = np.arange(N) * dt
    hdg_t = hdg0 + sign * omega * t_
    vR    = speed / (sign * omega)
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
    """n complete horizontal circles."""
    omega = speed / R_turn
    sign  = 1.0 if direction == 'right' else -1.0
    hdg0  = np.deg2rad(hdg_deg)
    dur   = n_revs * 2.0 * np.pi / omega
    N     = max(2, int(dur / dt) + 1)
    t_    = np.arange(N) * dt
    hdg_t = hdg0 + sign * omega * t_
    vR    = speed / (sign * omega)
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
    """Smoothly ramp pitch (and optionally speed) over `dur` seconds."""
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
    """Rotation phase: pitch 0→pitch_deg, speed→speed_end, at fixed heading."""
    return _pitch_transition(
        entry, hdg_deg, speed,
        pitch_start_deg=0.0, pitch_end_deg=pitch_deg,
        v_end=speed_end, dur=duration_s, dt=dt)


def _speed_ramp(entry, hdg_deg, v_start, v_end, alt_ned, dur=20.0, dt=0.1):
    """Linear speed change at constant heading and altitude."""
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
    """Forward azimuth (deg, 0–360) from point 1 to point 2."""
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1_deg, lon1_deg, lat2_deg, lon2_deg])
    dlon = lon2 - lon1
    x = np.sin(dlon) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
    return float(np.degrees(np.arctan2(x, y)) % 360)


def _geodetic_distance(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    """Great-circle distance in metres (Haversine)."""
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1_deg, lon1_deg, lat2_deg, lon2_deg])
    dlat = lat2 - lat1; dlon = lon2 - lon1
    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    return 2.0 * WGS84_A * np.arcsin(np.sqrt(a))


def _approx_geodetic(pos_ned, lat0_deg, lon0_deg, alt0_msl):
    """Flat-Earth NED offset → approximate (lat_deg, lon_deg, alt_msl_m)."""
    lat = lat0_deg + np.degrees(pos_ned[0] / WGS84_A)
    lon = lon0_deg + np.degrees(pos_ned[1] / (WGS84_A * np.cos(np.radians(lat0_deg))))
    alt = alt0_msl - pos_ned[2]
    return lat, lon, alt


# =========================================================================
# 3. YAML-driven trajectory builder
# =========================================================================
def build_trajectory(yaml_path: str, dt: float = None): # type: ignore
    """
    Build a phase-by-phase truth trajectory from a YAML definition.

    Each phase in the YAML maps to a helper function; fields omitted from a
    phase are inherited from the running state (heading, speed, altitude).

    Returns (truth, v_sprint, R_turn) — same tuple as the former
    build_bqn_trajectory() for compatibility with existing callers.
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

    if dt is None:
        dt = float(sim["dt_s"])

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
                float(phase["run_length_m"]), dt)
            state["hdg_deg"] = float(phase["heading_deg"])
            state["speed"]   = v_final

        elif ptype == "takeoff":
            v_end_val = float(phase["speed_kt"]) * KT
            pos, vel = _takeoff(
                state["pos_last"], float(phase["heading_deg"]), state["speed"],
                float(phase["pitch_deg"]), v_end_val,
                float(phase["duration_s"]), dt)
            state["hdg_deg"] = float(phase["heading_deg"])
            state["speed"]   = v_end_val

        elif ptype == "climb":
            ALT_CRUISE     = float(phase["to_altitude_ft"]) * FT
            ALT_NED_CRUISE = -(ALT_CRUISE - alt0_msl)
            climb_speed    = float(phase["speed_kt"]) * KT
            pos, vel = _climb(
                state["pos_last"], state["hdg_deg"], climb_speed,
                state["pos_last"][2], ALT_NED_CRUISE,
                pitch_deg=float(phase["pitch_deg"]), dt=dt)
            state["speed"]   = climb_speed
            state["alt_ned"] = ALT_NED_CRUISE

        elif ptype == "waypoint":
            wp_lat   = float(phase["lat_deg"])
            wp_lon   = float(phase["lon_deg"])
            wp_alt_m = float(phase["alt_ft"]) * FT
            wp_speed = float(phase["speed_kt"]) * KT
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
                state["speed"], state["alt_ned"], R_nav, dt)
            state["hdg_deg"] = exit_hdg

            # Fly to waypoint (with altitude change if needed)
            alt_diff = wp_alt_ned - state["alt_ned"]
            if abs(alt_diff) > 1.0:
                implied_pitch = float(np.degrees(np.arctan2(-alt_diff, max(dist_h_m, 1.0))))
                pos_s, vel_s = _climb(
                    pos_t[-1], exit_hdg, wp_speed,
                    state["alt_ned"], wp_alt_ned,
                    pitch_deg=implied_pitch, dt=dt)
            else:
                pos_s, vel_s = _straight(pos_t[-1], exit_hdg, dist_h_m, wp_speed, dt)

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
                    R_loit, direction=loiter_cfg.get("direction", "right"), dt=dt)
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
                dur=float(phase["duration_s"]), dt=dt)
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
    M       = len(all_pos)
    t_arr   = np.arange(M) * dt

    # Guard heading at t=0 against zero-velocity divide
    hdg0_rad = np.deg2rad(phases[0]["heading_deg"])
    all_vel[0] = np.array([np.cos(hdg0_rad), np.sin(hdg0_rad), 0.0]) * 1e-9

    # Smooth velocity to eliminate C1 kinks at phase boundaries.
    # Sigma ~2 s → roll/pitch transitions over 4–6 s, matching transport-category dynamics.
    _att_sigma = max(1, int(2.0 / dt))
    all_vel = gaussian_filter1d(all_vel, sigma=_att_sigma, axis=0)

    # Acceleration (numerical; smooth after velocity smoothing)
    acc_n = np.gradient(all_vel, dt, axis=0)

    # Geodetic position by integration
    lat0 = np.deg2rad(lat0_deg)
    lon0 = np.deg2rad(lon0_deg)
    lat  = np.zeros(M); lon = np.zeros(M); alt = np.zeros(M)
    lat[0], lon[0], alt[0] = lat0, lon0, alt0_msl
    for k in range(M - 1):
        R_M, R_N = wgs84_radii(lat[k])
        lat[k+1] = lat[k] + (all_vel[k, 0] / (R_M + alt[k])) * dt
        lon[k+1] = lon[k] + (all_vel[k, 1] /
                              ((R_N + alt[k]) * np.cos(lat[k]))) * dt
        alt[k+1] = alt[k] - all_vel[k, 2] * dt

    # Euler angles
    psi     = np.unwrap(np.arctan2(all_vel[:, 1], all_vel[:, 0]))
    v_h     = np.linalg.norm(all_vel[:, :2], axis=1)
    theta   = np.arctan2(-all_vel[:, 2], np.maximum(v_h, 1e-6))
    psi_dot = np.gradient(psi, dt)
    g_ref   = normal_gravity(lat0, alt0_msl)
    phi     = np.arctan2(v_h * psi_dot, g_ref)
    euler   = np.column_stack([phi, theta, psi])

    R_b2n = Rot.from_euler('ZYX', np.column_stack([psi, theta, phi]))
    R_n2b = R_b2n.inv()

    # ----------------------------------------------------------------
    # Truth IMU — consistent with strapdown_navgrade (forward Euler).
    #
    # omega_ib_b[k]: exact discrete rotation vector R[k]→R[k+1] / dt
    #                plus Earth/transport rate in body frame.
    #                Gives exact attitude under forward rot-vec integration.
    #
    # f_b[k]:        forward-difference specific force in NED, rotated to body.
    #                  f_n[k] = (vel[k+1]-vel[k])/dt + Coriolis[k] - g[k]
    #                This is exactly what the forward-Euler strapdown needs
    #                to recover vel[k+1] from vel[k] with zero error.
    # ----------------------------------------------------------------

    # Exact omega_nb_b via discrete rotation vectors (vectorised)
    dR_stack        = R_b2n[:-1].inv() * R_b2n[1:]
    omega_nb_b_disc = dR_stack.as_rotvec() / dt          # (M-1, 3)

    # f_n[k] = forward-difference formula: consistent with forward-Euler strapdown
    f_n_truth = np.zeros((M, 3))
    for k in range(M - 1):
        w_ie_k = earth_rate_n(lat[k])
        w_en_k = transport_rate_n(all_vel[k], lat[k], alt[k])
        g_k    = normal_gravity(lat[k], alt[k])
        f_n_truth[k] = ((all_vel[k+1] - all_vel[k]) / dt
                        + np.cross(2.0*w_ie_k + w_en_k, all_vel[k])
                        - np.array([0.0, 0.0, g_k]))
    f_n_truth[-1] = f_n_truth[-2]   # last sample: copy neighbour

    # Build IMU arrays
    omega_ib_b = np.zeros((M, 3))
    f_b_arr    = np.zeros((M, 3))
    g_arr      = np.zeros(M)
    for k in range(M):
        w_ie_k = earth_rate_n(lat[k])
        w_en_k = transport_rate_n(all_vel[k], lat[k], alt[k])
        g_arr[k] = normal_gravity(lat[k], alt[k])
        f_b_arr[k]    = R_n2b[k].apply(f_n_truth[k])
        w_nb_b_k      = (omega_nb_b_disc[k] if k < M-1 else omega_nb_b_disc[-1])
        omega_ib_b[k] = w_nb_b_k + R_n2b[k].apply(w_ie_k + w_en_k)

    # Reproject truth position through the same linearized geodetic formula
    # that strapdown_navgrade uses, so error = strapdown_pos_ned - truth.pos_n
    # measures actual navigation error rather than flat-earth vs. curved-earth bias.
    R_M0, R_N0 = wgs84_radii(lat0)
    pos_n_geo        = np.zeros((M, 3))
    pos_n_geo[:, 0]  = (lat - lat0)    * (R_M0 + alt0_msl)
    pos_n_geo[:, 1]  = (lon - lon[0])  * (R_N0 + alt0_msl) * np.cos(lat0)
    pos_n_geo[:, 2]  = -(alt - alt0_msl)

    class _Truth:
        pass
    truth           = _Truth()
    truth.t         = t_arr
    truth.dt        = dt
    truth.pos_n     = pos_n_geo
    truth.vel_n     = all_vel
    truth.acc_n     = acc_n
    truth.lat       = lat
    truth.lon       = lon
    truth.alt       = alt
    truth.euler     = euler
    truth.R_b2n     = R_b2n
    truth.omega_b   = omega_ib_b
    truth.f_b       = f_b_arr
    truth.g_loc     = g_arr
    return truth, v_sprint, R_turn_global
