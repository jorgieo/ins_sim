"""
Navigation-grade IMU simulation pipeline.

    NED waypoints  →  C2 spline path  →  truth state in nav frame
                  →  ideal IMU samples (with Earth rate, transport rate,
                                        Coriolis-consistent specific force)
                  →  noisy IMU samples (nav-grade error model)
                  →  full strapdown INS in geodetic / nav frame
                  →  Monte Carlo ensemble
                  →  95th-percentile radial error envelope (tube)

Conventions
-----------
Frames     : ECI inertial (i), ECEF Earth-fixed (e), NED navigation (n),
             body (b). Position in geodetic (φ, λ, h), velocity in NED.
Attitude   : Aerospace ZYX intrinsic Euler (yaw, pitch, roll).
             R_b2n acts on a body-frame column vector and returns NED.
Gravity    : Somigliana normal gravity + free-air altitude correction.
             Local gravity vector g_n = [0, 0, +g(φ, h)].
Spec force : f = a_inertial − g_grav, so a stationary level IMU reads
             f_b ≈ [0, 0, −g(φ, h)].
Quaternions: scalar-last [x, y, z, w], scipy convention.
"""

import os
import yaml
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation as Rot
import matplotlib.pyplot as plt
import seaborn as sns
import folium
from dataclasses import dataclass
from datetime import datetime


# =========================================================================
# WGS-84 Earth model
# =========================================================================
# Defining constants of the WGS-84 ellipsoid.
WGS84_A   = 6378137.0                 # Semi-major axis [m]
WGS84_F   = 1.0 / 298.257223563       # Flattening
WGS84_E2  = WGS84_F * (2.0 - WGS84_F) # First eccentricity squared
WGS84_OMEGA = 7.2921151467e-5         # Earth rotation rate [rad/s]


def wgs84_radii(lat):
    """
    Radii of curvature at geodetic latitude `lat` [rad].

    R_M (meridian)        — used for north-south motion: dφ = v_N / (R_M + h) dt
    R_N (prime vertical)  — used for east-west motion:   dλ = v_E / ((R_N + h) cos φ) dt
    """
    sphi2 = np.sin(lat) ** 2
    den   = np.sqrt(1.0 - WGS84_E2 * sphi2)
    R_N   = WGS84_A / den                                     # transverse radius
    R_M   = WGS84_A * (1.0 - WGS84_E2) / (den ** 3)           # meridional radius
    return R_M, R_N


def earth_rate_n(lat):
    """Earth rotation rate ω_ie expressed in the local NED frame."""
    return np.array([WGS84_OMEGA * np.cos(lat), 0.0, -WGS84_OMEGA * np.sin(lat)])


def transport_rate_n(v_n, lat, h):
    """
    Rotation rate of the NED frame relative to ECEF (a.k.a. craft rate),
    expressed in NED. Comes from the vehicle moving over the curved Earth.
    """
    R_M, R_N = wgs84_radii(lat)
    vN, vE, _ = v_n
    return np.array([
         vE / (R_N + h),
        -vN / (R_M + h),
        -vE * np.tan(lat) / (R_N + h),
    ])


def normal_gravity(lat, h):
    """
    Somigliana formula for surface gravity plus a free-air altitude
    correction. Accurate to ~1 µg (10⁻⁸ m/s²) below 10 km, which is
    well below navigation-grade accelerometer bias.
    """
    g_e = 9.7803253359          # equatorial gravity
    k   = 1.93185265241e-3      # Somigliana ratio constant
    sphi2 = np.sin(lat) ** 2
    g0  = g_e * (1.0 + k * sphi2) / np.sqrt(1.0 - WGS84_E2 * sphi2)
    # Linear free-air correction: dg/dh ≈ −2g/a near the surface
    return g0 * (1.0 - 2.0 * h / WGS84_A)


# =========================================================================
# Spline path (slim version)
# =========================================================================
class NEDSplinePath:
    """C2 cubic spline through NED waypoints, parameterized by chord length."""
    def __init__(self, waypoints, bc_type="not-a-knot"):
        pts = np.asarray(waypoints, dtype=float)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.s_knots = np.r_[0.0, np.cumsum(seg)]
        self.length  = float(self.s_knots[-1])
        self.cs      = CubicSpline(self.s_knots, pts, bc_type=bc_type)

    def position(self, s):
        return self.cs(np.clip(s, 0.0, self.length))


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
# 2. IMU error model — navigation grade defaults
# =========================================================================
@dataclass
class IMUSpec:
    """
    Navigation-grade IMU error parameters.

    Reference numbers (typical fielded systems, e.g. ring-laser-gyro
    aircraft INS, marine-grade strapdown):
        Gyro ARW            ≈ 0.002 °/√hr
        Gyro bias instab.   ≈ 0.01  °/hr
        Gyro repeatability  ≈ 0.01  °/hr
        Accel VRW           ≈ 0.005 m/s/√hr
        Accel bias instab.  ≈ 5  µg
        Accel repeatability ≈ 25 µg
    """
    # Gyro
    gyro_arw:    float = 0.002 * np.pi/180 / 60          # rad / √s
    gyro_bi_std: float = 0.01  * np.pi/180 / 3600        # rad / s
    gyro_bi_tau: float = 3600.0                          # GM correlation time [s]
    gyro_br_std: float = 0.01  * np.pi/180 / 3600        # rad / s
    # Accel
    accel_vrw:    float = 0.005 / 60                     # (m/s) / √s
    accel_bi_std: float =  5e-6 * 9.80665                # 5 µg
    accel_bi_tau: float = 3600.0
    accel_br_std: float = 25e-6 * 9.80665                # 25 µg


def load_imu_spec(yaml_path: str) -> IMUSpec:
    """Load an IMUSpec from a YAML file expressed in datasheet units."""
    with open(yaml_path) as fh:
        cfg = yaml.safe_load(fh)
    g = cfg["gyro"]
    a = cfg["accel"]
    DEG_HR = np.pi / 180 / 3600
    return IMUSpec(
        gyro_arw     = g["arw_deg_per_rt_hr"]          * np.pi / 180 / 60,
        gyro_bi_std  = g["bias_instab_deg_per_hr"]     * DEG_HR,
        gyro_bi_tau  = g["bias_tau_s"],
        gyro_br_std  = g["repeatability_deg_per_hr"]   * DEG_HR,
        accel_vrw    = a["vrw_m_per_s_per_rt_hr"]      / 60,
        accel_bi_std = a["bias_instab_ug"]              * 1e-6 * 9.80665,
        accel_bi_tau = a["bias_tau_s"],
        accel_br_std = a["repeatability_ug"]            * 1e-6 * 9.80665,
    )


def generate_imu_samples(truth, spec: IMUSpec,
                         rng: np.random.Generator):
    """
    Per-axis IMU model:
        measurement = truth + b_repeat + b_drift(t) + η_white(t)

      b_repeat   ~ N(0, σ_BR²)               drawn once per run
      b_drift    1st-order Gauss-Markov, steady-state std σ_BI, τ_BI
      η_white    σ = ARW/√dt or VRW/√dt      per-sample white noise

    Discrete AR(1) form for the GM drift gives steady-state variance σ²
    independent of dt:    b[k+1] = a·b[k] + √(1−a²)·σ·w,   a = exp(−dt/τ).
    """
    M, dt = len(truth.t), truth.dt

    # Constant turn-on biases (per axis, per run)
    b_g_repeat = rng.normal(0.0, spec.gyro_br_std,  size=3)
    b_a_repeat = rng.normal(0.0, spec.accel_br_std, size=3)

    # Gauss-Markov drift histories
    a_g = np.exp(-dt / spec.gyro_bi_tau)
    a_a = np.exp(-dt / spec.accel_bi_tau)
    sg  = np.sqrt(1.0 - a_g * a_g) * spec.gyro_bi_std
    sa  = np.sqrt(1.0 - a_a * a_a) * spec.accel_bi_std
    b_g_drift = np.zeros((M, 3))
    b_a_drift = np.zeros((M, 3))
    for k in range(1, M):
        b_g_drift[k] = a_g * b_g_drift[k-1] + sg * rng.normal(size=3)
        b_a_drift[k] = a_a * b_a_drift[k-1] + sa * rng.normal(size=3)

    # White noise: ARW/√dt is the discrete-time σ that yields integrated
    # angle uncertainty growing as ARW · √t, independent of sample rate.
    eta_g = rng.normal(0.0, spec.gyro_arw  / np.sqrt(dt), size=(M, 3))
    eta_a = rng.normal(0.0, spec.accel_vrw / np.sqrt(dt), size=(M, 3))

    omega_meas = truth.omega_b + b_g_repeat + b_g_drift + eta_g
    f_meas     = truth.f_b     + b_a_repeat + b_a_drift + eta_a
    return omega_meas, f_meas


# =========================================================================
# 3. Navigation-grade strapdown mechanization
# =========================================================================
def strapdown_navgrade(omega_meas, f_meas, init_state, dt, alt_truth=None):
    """
    Local-level (NED) strapdown integration with full rotating-Earth
    corrections. Per step:

      1. Evaluate ω_ie_n, ω_en_n, g(φ,h) at the current state.
      2. Compute body-rate-relative-to-nav:
                 ω_nb_b = ω_ib_b − C_n^b · (ω_ie_n + ω_en_n)
      3. Update C_b^n via exact rotation-vector exponential of ω_nb_b·dt.
      4. Resolve specific force into NED (forward Euler).
      5. Apply rotating-frame velocity equation:
                 v̇_n = f_n − (2ω_ie_n + ω_en_n) × v_n + g_n
      6. Forward-Euler geodetic position update.

    alt_truth  : optional array (M,) of truth altitude [m MSL]. When
                 provided it is used as barometric altitude aiding, which
                 stabilises the inherently unstable vertical channel of a
                 free-inertial navigator (eigenvalue +ωs without aiding).
    init_state : (lat0, lon0, alt0, v_n0, q_b2n_0)   q in scalar-last form
    Returns    : pos_ned (M,3) relative to start, lat/lon/alt arrays,
                 vel_n, quat
    """
    M = len(omega_meas)
    lat = np.zeros(M); lon = np.zeros(M); alt = np.zeros(M)
    vel = np.zeros((M, 3)); quat = np.zeros((M, 4))
    pos_ned = np.zeros((M, 3))         # for plotting against truth NED

    lat[0], lon[0], alt[0], vel[0], quat[0] = init_state
    R_curr = Rot.from_quat(quat[0])
    lat0 = lat[0]                      # reference for local NED display
    alt0 = alt[0]

    for k in range(M - 1):
        # --- 1. Local rotating-Earth quantities at step k --------------
        w_ie = earth_rate_n(lat[k])
        w_en = transport_rate_n(vel[k], lat[k], alt[k])
        w_in = w_ie + w_en                                     # nav-frame rate
        g_k  = normal_gravity(lat[k], alt[k])
        g_n  = np.array([0.0, 0.0, g_k])

        # --- 2. Body-rate relative to nav frame -----------------------
        # The gyro saw ω_ib_b. Subtract the nav-frame's own rotation
        # (expressed in body) to get what drives the b→n attitude update.
        R_n2b = R_curr.inv()
        w_nb_b = omega_meas[k] - R_n2b.apply(w_in)

        # --- 3. Attitude update (exact rotation-vector exponential) ---
        dR     = Rot.from_rotvec(w_nb_b * dt)
        R_next = R_curr * dR                                   # right-mult: body Δ
        quat[k+1] = R_next.as_quat() # pyright: ignore[reportCallIssue]

        # --- 4. Specific force in NED (forward Euler — consistent with
        #        truth IMU which uses the same first-order scheme) --------
        f_n_k = R_curr.apply(f_meas[k])

        # --- 5. Rotating-frame velocity update ------------------------
        coriolis = np.cross(2.0 * w_ie + w_en, vel[k])
        a_n      = f_n_k - coriolis + g_n
        vel[k+1] = vel[k] + a_n * dt

        # --- 6. Geodetic position (forward Euler — matches truth) -----
        R_M, R_N = wgs84_radii(lat[k])
        lat[k+1] = lat[k] + (vel[k][0] / (R_M + alt[k])) * dt
        lon[k+1] = lon[k] + (vel[k][1] /
                             ((R_N + alt[k]) * np.cos(lat[k]))) * dt
        alt[k+1] = alt[k] - vel[k][2] * dt
        # Barometric altitude aiding: stabilises the vertical channel
        if alt_truth is not None:
            alt[k+1] = alt_truth[k+1]

        # Local NED position relative to start (for plotting against truth)
        # Linearized lat/lon-to-meters using current latitude radii.
        R_M0, R_N0 = wgs84_radii(lat0)
        pos_ned[k+1, 0] = (lat[k+1] - lat0) * (R_M0 + alt0)
        pos_ned[k+1, 1] = (lon[k+1] - lon[0]) * (R_N0 + alt0) * np.cos(lat0)
        pos_ned[k+1, 2] = -(alt[k+1] - alt0)

        R_curr = R_next

    return pos_ned, lat, lon, alt, vel, quat


# =========================================================================
# 4. Monte Carlo runner & percentile envelope
# =========================================================================
def run_monte_carlo(truth, spec: IMUSpec,
                    n_trials: int, seed: int = 0):
    """Run n_trials with independent IMU error realizations."""
    M = len(truth.t)
    pos_runs   = np.zeros((n_trials, M, 3))
    euler_runs = np.zeros((n_trials, M, 3))
    lat_runs   = np.zeros((n_trials, M))
    lon_runs   = np.zeros((n_trials, M))
    rng_master = np.random.default_rng(seed)

    init_state = (
        truth.lat[0], truth.lon[0], truth.alt[0],
        truth.vel_n[0].copy(),
        truth.R_b2n[0].as_quat(), # pyright: ignore[reportCallIssue]
    )

    for i in range(n_trials):
        rng_i = np.random.default_rng(rng_master.integers(0, 2**31))
        omega_m, f_m = generate_imu_samples(truth, spec, rng_i)
        pos_ned, lat_i, lon_i, _, _, quat_i = strapdown_navgrade(
            omega_m, f_m, init_state, truth.dt, alt_truth=truth.alt)
        pos_runs[i]   = pos_ned
        lat_runs[i]   = lat_i
        lon_runs[i]   = lon_i
        # as_euler('ZYX') → [psi, theta, phi]; reverse to [phi, theta, psi]
        euler_runs[i] = Rot.from_quat(quat_i).as_euler('ZYX')[:, ::-1]

    return pos_runs, euler_runs, lat_runs, lon_runs


def percentile_envelope(pos_runs, truth_pos, q=95):
    """Per-time-step q-th percentile of 3D radial error magnitude."""
    err  = pos_runs - truth_pos[None, :, :]
    rerr = np.linalg.norm(err, axis=2)
    return np.percentile(rerr, q, axis=0)


# =========================================================================
# 5. Visualization
# =========================================================================
def plot_error_tube(ax, truth_pos, r95, color="crimson",
                    alpha=0.20, n_circle=24):
    """Swept volume of the 95th-percentile radial error around truth."""
    T = np.gradient(truth_pos, axis=0)
    T = T / np.linalg.norm(T, axis=1, keepdims=True)

    seed = np.array([0.0, 0.0, 1.0])
    if abs(T[0] @ seed) > 0.95:
        seed = np.array([0.0, 1.0, 0.0])

    N1 = np.zeros_like(T); N2 = np.zeros_like(T)
    n  = seed - T[0] * (T[0] @ seed)
    N1[0] = n / np.linalg.norm(n)
    N2[0] = np.cross(T[0], N1[0])
    for k in range(1, len(T)):
        proj = N1[k-1] - T[k] * (T[k] @ N1[k-1])
        nn   = np.linalg.norm(proj)
        N1[k] = proj / nn if nn > 1e-10 else N1[k-1]
        N2[k] = np.cross(T[k], N1[k])

    th = np.linspace(0.0, 2*np.pi, n_circle)
    c, s = np.cos(th), np.sin(th)
    tube = (truth_pos[:, None, :]
            + r95[:, None, None] * (c[None, :, None] * N1[:, None, :]
                                  + s[None, :, None] * N2[:, None, :]))

    X = tube[..., 1]      # East
    Y = tube[..., 0]      # North
    Z = -tube[..., 2]     # Up
    ax.plot_surface(X, Y, Z, color=color, alpha=alpha,
                    edgecolor='none', linewidth=0)


# =========================================================================
# 6. Phase helpers
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
# 7. YAML-driven trajectory builder
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


if __name__ == "__main__":

    simulation_start = datetime.now() # for timing the whole script

    dt       = 0.1      # 10 Hz — adequate for long-range INS simulation
    n_trials = 20

    _dir  = os.path.dirname(__file__)
    truth, v_sprint, R_turn = build_trajectory(
        os.path.join(_dir, "bqn_departure.yaml"), dt=dt)

    NM = 1852.0
    total_dist = np.sum(np.linalg.norm(np.diff(truth.pos_n, axis=0), axis=1))
    print(f"Total path  : {total_dist/NM:.1f} nm  ({total_dist/1e3:.0f} km)")
    print(f"Duration    : {truth.t[-1]/60:.1f} min  ({truth.t[-1]:.0f} s)")
    print(f"Samples     : {len(truth.t):,}  (dt={dt} s)")
    print(f"Start g     : {truth.g_loc[0]:.5f} m/s²")

    # Diagnostic: zero-noise strapdown verifies truth IMU self-consistency
    zero_spec = IMUSpec(gyro_arw=0, gyro_bi_std=0, gyro_br_std=0,
                        accel_vrw=0, accel_bi_std=0, accel_br_std=0)
    zp, *_ = strapdown_navgrade(truth.omega_b, truth.f_b,
                                 (truth.lat[0], truth.lon[0], truth.alt[0],
                                  truth.vel_n[0].copy(), truth.R_b2n[0].as_quat()),
                                 truth.dt, alt_truth=truth.alt)
    
    zero_err = np.linalg.norm(zp - truth.pos_n, axis=1)

    print(f"Zero-noise error  start: {zero_err[0]:.3f} m  max: {zero_err.max():.1f} m  end: {zero_err[-1]:.1f} m")

    M_diag = len(truth.t)

    for pct in [10, 25, 50, 75, 100]:
        idx = int(pct/100 * (M_diag - 1))
        print(f"  t = {truth.t[idx]/60:.1f} min ({pct}%): err = {zero_err[idx]:.1f} m")

    print(f"Max |acc_n|: {np.linalg.norm(truth.acc_n, axis=1).max():.2f} m/s² \n"
          f"Max |f_b|: {np.linalg.norm(truth.f_b, axis=1).max():.2f} m/s²")

    spec     = load_imu_spec(os.path.join(_dir, "navgrade_imu.yaml"))
    pos_runs, euler_runs, lat_runs, lon_runs = run_monte_carlo(
        truth, spec, n_trials=n_trials, seed=42)

    r95      = percentile_envelope(pos_runs, truth.pos_n, q=95)
    print(f"95th-pct error  start: {r95[0]:6.3f} m   end: {r95[-1]:6.1f} m")

    # Simulation Time

    simulation_end = datetime.now()
    elapsed = simulation_end - simulation_start
    hours, rem = divmod(elapsed.total_seconds(), 3600)
    minutes, seconds = divmod(rem, 60)
    print(f"Monte Carlo simulation time: {int(hours)}h {int(minutes)}m {int(seconds)}s")

    P   = truth.pos_n
    t_m = truth.t / 60.0   # minutes for x-axis

    # ---- Light seaborn theme -----------------------------------------------
    sns.set_theme(
        style='whitegrid',
        palette='deep',
        font_scale=1.05,
        rc={
            'figure.facecolor': '#f5f5f5',
            'axes.facecolor':   'white',
            'grid.color':       '#dddddd',
            'axes.edgecolor':   '#aaaaaa',
            'text.color':       '#111111',
            'axes.labelcolor':  '#111111',
            'xtick.color':      '#333333',
            'ytick.color':      '#333333',
        },
    )

    step = max(1, len(P) // 4000)
    fig  = plt.figure(figsize=(22, 12), layout='constrained')
    gs   = fig.add_gridspec(2, 3)

    ax  = fig.add_subplot(gs[0, 0], projection='3d')
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[0, 2])
    ax4 = fig.add_subplot(gs[1, 0])   # Pitch
    ax5 = fig.add_subplot(gs[1, 1])   # Roll
    ax6 = fig.add_subplot(gs[1, 2])   # Heading

    # ---- 3-D ground track -------------------------------------------------
    ax.plot(P[::step, 1]/NM, P[::step, 0]/NM, -P[::step, 2]*3.28084/1000,
            'k-', lw=1.5, label='Truth')
    for i in range(min(6, n_trials)):
        Q = pos_runs[i]
        ax.plot(Q[::step, 1]/NM, Q[::step, 0]/NM, -Q[::step, 2]*3.28084/1000,
                '-', color='steelblue', alpha=0.35, lw=0.5)
    ax.set_xlabel('East (nm)')
    ax.set_ylabel('North (nm)')
    ax.set_zlabel('Alt (kft)')
    ax.set_title(f'BQN Departure — {n_trials}-trial Monte Carlo')
    ax.legend(loc='upper left')

    # ---- 3-D radial error vs time -----------------------------------------
    err_all = np.linalg.norm(pos_runs - P[None, :, :], axis=2)
    ax2.plot(t_m, err_all.T, color='steelblue', alpha=0.20, lw=0.5)
    ax2.plot(t_m, r95, color='crimson', lw=2.0, label='95th pct')
    ax2.fill_between(t_m, 0, r95, color='crimson', alpha=0.15)
    ax2.set_xlabel('Time (min)')
    ax2.set_ylabel('3-D position error (m)')
    ax2.set_title('Radial error envelope')
    ax2.legend()

    # ---- CEP vs time + linear fit -----------------------------------------
    horiz_err = np.linalg.norm(pos_runs[:, :, :2] - P[None, :, :2], axis=2)
    cep = np.percentile(horiz_err, 50, axis=0)

    mask60 = t_m <= 60.0
    coeffs = np.polyfit(t_m[mask60], cep[mask60], 1)
    cep_fit = np.polyval(coeffs, t_m)

    ax3.plot(t_m, horiz_err.T, color='steelblue', alpha=0.15, lw=0.5)
    ax3.plot(t_m, cep, color='navy', lw=2.0, label='CEP (50th pct)')
    ax3.fill_between(t_m, 0, cep, color='steelblue', alpha=0.18)
    ax3.plot(t_m, cep_fit, '--', color='darkorange', lw=1.8,
             label=f'Linear fit  {coeffs[0]*0.032397408:.2f} NM/hr')
    ax3.axvline(60.0, color='gray', lw=0.8, linestyle=':')
    ax3.set_xlabel('Time (min)')
    ax3.set_ylabel('Horizontal error (m)')
    ax3.set_title('CEP along path')
    ax3.legend()

    # ---- Attitude error envelopes -----------------------------------------
    euler_err = euler_runs - truth.euler[None, :, :]
    euler_err[:, :, 2] = (euler_err[:, :, 2] + np.pi) % (2 * np.pi) - np.pi
    p95_euler = np.rad2deg(np.percentile(np.abs(euler_err), 95, axis=0))

    for ax_att, col, title, ylabel in [
        (ax4, 1, 'Pitch',   'Pitch (deg)'),
        (ax5, 0, 'Roll',    'Roll (deg)'),
        (ax6, 2, 'Heading', 'Heading (deg)'),
    ]:
        truth_deg = np.rad2deg(truth.euler[:, col])
        err95     = p95_euler[:, col]
        ax_att.fill_between(t_m, truth_deg - err95, truth_deg + err95,
                            color='crimson', alpha=0.25, label='95th pct envelope')
        ax_att.plot(t_m, truth_deg, color='navy', lw=1.5, label='Truth')
        ax_att.set_xlabel('Time (min)')
        ax_att.set_ylabel(ylabel)
        ax_att.set_title(title)
        ax_att.legend(fontsize=8)

    # ---- Interactive folium map ---------------------------------------------
    lat_deg = np.rad2deg(truth.lat)
    lon_deg = np.rad2deg(truth.lon)

    p95_horiz = np.percentile(horiz_err, 95, axis=0)

    # Perpendicular envelope computed directly in geographic space.
    # Convert track gradients to approximate metres so the normal direction
    # is correct, then convert the resulting offset back to degrees.
    dlat_m = np.gradient(lat_deg) * 111320.0
    dlon_m = np.gradient(lon_deg) * 111320.0 * np.cos(np.deg2rad(lat_deg))
    seg_len = np.hypot(dlat_m, dlon_m) + 1e-6
    perp_lat = -dlon_m / seg_len
    perp_lon  =  dlat_m / seg_len

    upper_lat = lat_deg + p95_horiz * perp_lat / 111320.0
    upper_lon = lon_deg + p95_horiz * perp_lon / (111320.0 * np.cos(np.deg2rad(lat_deg)))
    lower_lat = lat_deg - p95_horiz * perp_lat / 111320.0
    lower_lon = lon_deg - p95_horiz * perp_lon / (111320.0 * np.cos(np.deg2rad(lat_deg)))

    mid = len(lat_deg) // 2
    fmap = folium.Map(location=[lat_deg[mid], lon_deg[mid]], zoom_start=9)

    for i in range(min(6, n_trials)):
        run_lat = np.rad2deg(lat_runs[i, ::step])
        run_lon = np.rad2deg(lon_runs[i, ::step])
        folium.PolyLine(
            list(zip(run_lat.tolist(), run_lon.tolist())),
            color='steelblue', weight=1, opacity=0.30,
        ).add_to(fmap)

    env_coords = (
        list(zip(upper_lat[::step].tolist(), upper_lon[::step].tolist()))
        + list(zip(lower_lat[::-step].tolist(), lower_lon[::-step].tolist()))
    )
    folium.Polygon(
        locations=env_coords,
        color='crimson', fill=True, fill_color='crimson', fill_opacity=0.30,
        tooltip='95th pct envelope',
    ).add_to(fmap)

    folium.PolyLine(
        list(zip(lat_deg[::step].tolist(), lon_deg[::step].tolist())),
        color='navy', weight=2, opacity=1.0, tooltip='Truth track',
    ).add_to(fmap)

    maps_dir = os.path.join(_dir, 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    map_path = os.path.join(maps_dir, 'trajectory_map.html')
    fmap.save(map_path)
    print(f"Interactive map saved: {map_path}")

    plt.show()