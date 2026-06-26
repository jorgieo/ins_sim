import numpy as np
from scipy.spatial.transform import Rotation as Rot

from ins_sim.sensors.imu import IMUSpec, generate_imu_samples
from ins_sim.navigation.strapdown import strapdown_navgrade


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
