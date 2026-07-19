import numpy as np
from scipy.spatial.transform import Rotation as Rot

from ins_sim.sensors.imu import (IMUSpec, draw_initial_misalignment,
                                 generate_imu_samples)
from ins_sim.navigation.strapdown import strapdown_navgrade


def run_monte_carlo(truth, spec: IMUSpec,
                    n_trials: int, seed: int = 0,
                    progress_callback=None):
    """Runs n_trials independent noisy-IMU strapdown realizations.

    Args:
        truth: Truth trajectory object exposing t, dt, lat, lon, alt,
            vel_n, R_b2n, omega_b (ω_ib_b), and f_b.
        spec: IMU error-parameter set used to generate noisy samples.
        n_trials: Number of independent Monte Carlo trials.
        seed: Seed for the master random generator, used to derive an
            independent child generator per trial. Defaults to 0.
        progress_callback: Optional callable invoked as
            progress_callback(n_done, n_trials) after each trial
            completes. Defaults to None (no reporting).

    Returns:
        tuple: (pos_runs, euler_runs, lat_runs, lon_runs, vel_runs)
            where:
            pos_runs (numpy.ndarray): NED position per trial, shape
                (n_trials, M, 3) [m].
            euler_runs (numpy.ndarray): Euler angles [φ, θ, ψ] per
                trial, shape (n_trials, M, 3) [rad].
            lat_runs (numpy.ndarray): Geodetic latitude per trial,
                shape (n_trials, M) [rad].
            lon_runs (numpy.ndarray): Geodetic longitude per trial,
                shape (n_trials, M) [rad].
            vel_runs (numpy.ndarray): NED velocity per trial, shape
                (n_trials, M, 3) [m/s].
    """
    M = len(truth.t)
    pos_runs   = np.zeros((n_trials, M, 3))
    euler_runs = np.zeros((n_trials, M, 3))
    lat_runs   = np.zeros((n_trials, M))
    lon_runs   = np.zeros((n_trials, M))
    vel_runs   = np.zeros((n_trials, M, 3))
    rng_master = np.random.default_rng(seed)

    R0_true = truth.R_b2n[0]

    for i in range(n_trials):
        rng_i = np.random.default_rng(rng_master.integers(0, 2**31))
        # Per-trial initial-alignment error: small NED tilts + azimuth
        # misalignment applied to the truth attitude (zero-mean draws;
        # exact truth init when the spec's alignment stds are zero).
        mis_rotvec = draw_initial_misalignment(spec, rng_i)
        R0_est = Rot.from_rotvec(mis_rotvec) * R0_true
        init_state = (
            truth.lat[0], truth.lon[0], truth.alt[0],
            truth.vel_n[0].copy(),
            R0_est.as_quat(), # pyright: ignore[reportCallIssue]
        )
        omega_m, f_m = generate_imu_samples(truth, spec, rng_i)
        pos_ned, lat_i, lon_i, _, vel_i, quat_i = strapdown_navgrade(
            omega_m, f_m, init_state, truth.dt, alt_truth=truth.alt)
        pos_runs[i]   = pos_ned
        lat_runs[i]   = lat_i
        lon_runs[i]   = lon_i
        vel_runs[i]   = vel_i
        # as_euler('ZYX') → [psi, theta, phi]; reverse to [phi, theta, psi]
        euler_runs[i] = Rot.from_quat(quat_i).as_euler('ZYX')[:, ::-1]
        if progress_callback is not None:
            progress_callback(i + 1, n_trials)

    return pos_runs, euler_runs, lat_runs, lon_runs, vel_runs


def percentile_envelope(pos_runs, truth_pos, q=95):
    """Computes the per-time-step q-th percentile of 3D radial position error.

    Args:
        pos_runs: Position per trial, shape (n_trials, M, 3) [m].
        truth_pos: Truth position, shape (M, 3) [m].
        q: Percentile to compute, in [0, 100]. Defaults to 95.

    Returns:
        numpy.ndarray: q-th percentile of 3D radial error magnitude at
            each time step, shape (M,) [m].
    """
    err  = pos_runs - truth_pos[None, :, :]
    rerr = np.linalg.norm(err, axis=2)
    return np.percentile(rerr, q, axis=0)
