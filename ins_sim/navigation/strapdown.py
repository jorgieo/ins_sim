import numpy as np
from scipy.spatial.transform import Rotation as Rot

from ins_sim.core.earth_model import earth_rate_n, transport_rate_n, normal_gravity, wgs84_radii


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
