from collections.abc import Callable
from typing import TYPE_CHECKING, NamedTuple

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from ins_sim.sensors.imu import (IMUSpec, draw_initial_misalignment,
                                 generate_imu_samples)
from ins_sim.navigation.strapdown import NavState, strapdown_navgrade

if TYPE_CHECKING:
    from ins_sim.trajectory.kinematics import TruthTrajectory


class MonteCarloResult(NamedTuple):
    """Navigation estimates from every Monte Carlo trial.

    Attributes:
        est_r_lb_l: Estimated position relative to the start point, per
            trial, shape (n_trials, M, 3) [m].
        est_euler_nb: Estimated Euler angles [roll, pitch, heading], per
            trial, shape (n_trials, M, 3) [rad].
        est_L_b: Estimated geodetic latitude, per trial, shape
            (n_trials, M) [rad].
        est_lambda_b: Estimated longitude, per trial, shape
            (n_trials, M) [rad].
        est_v_eb_n: Estimated velocity resolved in NED, per trial, shape
            (n_trials, M, 3) [m/s].
    """

    est_r_lb_l: np.ndarray
    est_euler_nb: np.ndarray
    est_L_b: np.ndarray
    est_lambda_b: np.ndarray
    est_v_eb_n: np.ndarray


def run_monte_carlo(truth: "TruthTrajectory", spec: IMUSpec,
                    n_trials: int, seed: int = 0,
                    progress_callback: Callable[[int, int], None] | None = None,
                    baro_aiding: bool = True) -> MonteCarloResult:
    """Runs n_trials independent noisy-IMU strapdown realizations.

    Args:
        truth: Truth trajectory exposing t, tau_i, L_b, lambda_b, h_b,
            v_eb_n, C_b_n, omega_ib_b and f_ib_b.
        spec: IMU error-parameter set used to generate noisy samples.
        n_trials: Number of independent Monte Carlo trials.
        seed: Seed for the master random generator, used to derive an
            independent child generator per trial. Defaults to 0.
        progress_callback: Optional callable invoked as
            progress_callback(n_done, n_trials) after each trial
            completes. Defaults to None (no reporting).
        baro_aiding: When True the strapdown runs with the baro-damped
            vertical channel (truth height as the baro measurement);
            when False the vertical channel is free-inertial and
            diverges. Defaults to True.

    Returns:
        MonteCarloResult: (est_r_lb_l, est_euler_nb, est_L_b,
            est_lambda_b, est_v_eb_n); unpacks positionally in that order.
    """
    M = len(truth.t)
    est_r_lb_l   = np.zeros((n_trials, M, 3))
    est_euler_nb = np.zeros((n_trials, M, 3))
    est_L_b      = np.zeros((n_trials, M))
    est_lambda_b = np.zeros((n_trials, M))
    est_v_eb_n   = np.zeros((n_trials, M, 3))
    rng_master = np.random.default_rng(seed)

    C_b_n0 = truth.C_b_n[0]

    for i in range(n_trials):
        rng_i = np.random.default_rng(rng_master.integers(0, 2**31))
        # Per-trial initial-alignment error delta_psi_nb^n: small NED tilts
        # plus an azimuth error, applied to the truth attitude (zero-mean
        # draws; exact truth initialization when the alignment stds are 0).
        delta_psi_nb_n0 = draw_initial_misalignment(spec, rng_i)
        est_C_b_n0 = Rot.from_rotvec(delta_psi_nb_n0) * C_b_n0
        init_state = NavState(
            truth.L_b[0], truth.lambda_b[0], truth.h_b[0],
            truth.v_eb_n[0].copy(),
            est_C_b_n0.as_quat(), # pyright: ignore[reportCallIssue]
        )
        meas_omega_ib_b, meas_f_ib_b = generate_imu_samples(truth, spec, rng_i)
        nav = strapdown_navgrade(
            meas_omega_ib_b, meas_f_ib_b, init_state, truth.tau_i,
            meas_h_b=truth.h_b if baro_aiding else None)
        est_r_lb_l[i]   = nav.r_lb_l
        est_L_b[i]      = nav.L_b
        est_lambda_b[i] = nav.lambda_b
        est_v_eb_n[i]   = nav.v_eb_n
        # as_euler('ZYX') → [psi, theta, phi]; reverse to [phi, theta, psi]
        est_euler_nb[i] = Rot.from_quat(nav.q_b_n).as_euler('ZYX')[:, ::-1]
        if progress_callback is not None:
            progress_callback(i + 1, n_trials)

    return MonteCarloResult(est_r_lb_l, est_euler_nb, est_L_b, est_lambda_b,
                            est_v_eb_n)


def percentile_envelope(est_r_lb_l: np.ndarray, true_r_lb_l: np.ndarray,
                        q: float = 95) -> np.ndarray:
    """Computes the per-time-step q-th percentile of 3D radial position error.

    Args:
        est_r_lb_l: Estimated position per trial, shape (n_trials, M, 3) [m].
        true_r_lb_l: Truth position, shape (M, 3) [m].
        q: Percentile to compute, in [0, 100]. Defaults to 95.

    Returns:
        numpy.ndarray: q-th percentile of 3D radial error magnitude at
            each time step, shape (M,) [m].
    """
    err  = est_r_lb_l - true_r_lb_l[None, :, :]
    rerr = np.linalg.norm(err, axis=2)
    return np.percentile(rerr, q, axis=0)
