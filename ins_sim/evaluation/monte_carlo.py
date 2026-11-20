"""Monte Carlo evaluation: many independent runs of the noisy IMU and the navigator.

`run_monte_carlo` repeats the sensor-error draw and the strapdown
navigation for a number of trials and collects every trial's estimates and
attitude error. The spread across trials is the simulator's measure of
navigation performance; `percentile_envelope` summarizes it as a
position-error percentile over time. See the
[Monte Carlo evaluation][monte-carlo-evaluation] theory page.
"""

from collections.abc import Callable
from typing import TYPE_CHECKING, NamedTuple

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from ins_sim.evaluation.errors import attitude_error
from ins_sim.sensors.baro import generate_baro_samples
from ins_sim.sensors.imu import (IMUSpec, draw_initial_misalignment,
                                 generate_imu_samples)
from ins_sim.navigation.strapdown import NavState, strapdown_navgrade

if TYPE_CHECKING:
    from ins_sim.trajectory.kinematics import TruthTrajectory


class MonteCarloResult(NamedTuple):
    """Navigation estimates and attitude errors from every Monte Carlo trial.

    Velocity and position errors are formed from these estimates with
    `ins_sim.evaluation.errors.velocity_error` and `position_error`.

    Attributes:
        est_r_lb_l: Estimated position relative to the start point, per
            trial, shape (n_trials, M, 3) [m]. Used to draw tracks.
        est_euler_nb: Estimated Euler angles [roll, pitch, heading], per
            trial, shape (n_trials, M, 3) [rad].
        est_L_b: Estimated geodetic latitude, per trial, shape
            (n_trials, M) [rad].
        est_lambda_b: Estimated longitude, per trial, shape
            (n_trials, M) [rad].
        est_v_eb_n: Estimated velocity resolved in NED, per trial, shape
            (n_trials, M, 3) [m/s].
        est_h_b: Estimated geodetic height, per trial, shape
            (n_trials, M) [m].
        delta_psi_nb_n: Attitude error (North tilt, East tilt, Down/heading),
            resolved in NED, per trial, shape (n_trials, M, 3) [rad].
    """

    est_r_lb_l: np.ndarray
    est_euler_nb: np.ndarray
    est_L_b: np.ndarray
    est_lambda_b: np.ndarray
    est_v_eb_n: np.ndarray
    est_h_b: np.ndarray
    delta_psi_nb_n: np.ndarray


def run_monte_carlo(truth: "TruthTrajectory", spec: IMUSpec,
                    n_trials: int, seed: int = 0,
                    progress_callback: Callable[[int, int], None] | None = None,
                    baro_aiding: bool = True) -> MonteCarloResult:
    """Runs n_trials independent noisy-IMU strapdown realizations.

    Each trial has its own random stream, spawned from `seed`, and each
    error source within a trial (initial alignment, IMU, barometer) draws
    from its own child stream. A trial's draws therefore do not depend on `n_trials`, and
    changing one error source's parameters leaves the other sources'
    realizations unchanged (common random numbers).

    Args:
        truth: Truth trajectory exposing t, tau_i, L_b, lambda_b, h_b,
            v_eb_n, C_b_n, omega_ib_b and f_ib_b.
        spec: IMU error-parameter set used to generate noisy samples.
        n_trials: Number of independent Monte Carlo trials.
        seed: Seed for the master random generator. Defaults to 0.
        progress_callback: Optional callable invoked as
            progress_callback(n_done, n_trials) after each trial
            completes. Defaults to None (no reporting).
        baro_aiding: When True the strapdown runs with the baro-damped
            vertical channel, aided by a barometer with the errors in
            `spec.baro` (the true height when `spec.baro` is None); when
            False the vertical channel is free-inertial and diverges.
            Defaults to True.

    Returns:
        MonteCarloResult: estimates and attitude errors for every trial.

    Examples:
        >>> from ins_sim.evaluation.monte_carlo import run_monte_carlo
        >>> from ins_sim.sensors.imu import IMUSpec
        >>> from ins_sim.trajectory.kinematics import TruthTrajectory
        >>> from ins_sim.trajectory.spline import NEDSplinePath
        >>> path = NEDSplinePath([[0.0, 0.0, 0.0], [3000.0, 1000.0, -50.0]])
        >>> truth = TruthTrajectory.from_spline(path, speed=150.0, tau_i=0.1)
        >>> mc = run_monte_carlo(truth, IMUSpec(), n_trials=3, seed=1)
        >>> mc.est_L_b.shape == (3, len(truth.t))
        True
    """
    M = len(truth.t)
    est_r_lb_l     = np.zeros((n_trials, M, 3))
    est_euler_nb   = np.zeros((n_trials, M, 3))
    est_L_b        = np.zeros((n_trials, M))
    est_lambda_b   = np.zeros((n_trials, M))
    est_v_eb_n     = np.zeros((n_trials, M, 3))
    est_h_b        = np.zeros((n_trials, M))
    delta_psi_nb_n = np.zeros((n_trials, M, 3))

    # --8<-- [start:trial]
    trial_rngs = np.random.default_rng(seed).spawn(n_trials)
    C_b_n0 = truth.C_b_n[0]

    for i, trial_rng in enumerate(trial_rngs):
        rng_alignment, rng_imu, rng_baro = trial_rng.spawn(3)
        # Per-trial initial-alignment error delta_psi_nb^n: small NED tilts
        # plus an azimuth error, applied to the truth attitude (zero-mean
        # draws; exact truth initialization when the alignment stds are 0).
        delta_psi_nb_n0 = draw_initial_misalignment(spec, rng_alignment)
        est_C_b_n0 = Rot.from_rotvec(delta_psi_nb_n0) * C_b_n0
        init_state = NavState(
            truth.L_b[0], truth.lambda_b[0], truth.h_b[0],
            truth.v_eb_n[0].copy(),
            est_C_b_n0.as_quat(), # pyright: ignore[reportCallIssue]
        )
        meas_omega_ib_b, meas_f_ib_b = generate_imu_samples(truth, spec, rng_imu)
        if not baro_aiding:
            meas_h_b = None                                  # free-inertial
        elif spec.baro is None:
            meas_h_b = truth.h_b                             # ideal barometer
        else:
            meas_h_b = generate_baro_samples(truth, spec.baro, rng_baro)
        nav = strapdown_navgrade(
            meas_omega_ib_b, meas_f_ib_b, init_state, truth.tau_i,
            meas_h_b=meas_h_b)
        # --8<-- [end:trial]
        est_r_lb_l[i]   = nav.r_lb_l
        est_L_b[i]      = nav.L_b
        est_lambda_b[i] = nav.lambda_b
        est_h_b[i]      = nav.h_b
        est_v_eb_n[i]   = nav.v_eb_n
        est_C_b_n = Rot.from_quat(nav.q_b_n)
        delta_psi_nb_n[i] = attitude_error(est_C_b_n, truth.C_b_n)
        # as_euler('ZYX') → [psi, theta, phi]; reverse to [phi, theta, psi]
        est_euler_nb[i] = est_C_b_n.as_euler('ZYX')[:, ::-1]
        if progress_callback is not None:
            progress_callback(i + 1, n_trials)

    return MonteCarloResult(est_r_lb_l, est_euler_nb, est_L_b, est_lambda_b,
                            est_v_eb_n, est_h_b, delta_psi_nb_n)


def percentile_envelope(delta_r_eb_n: np.ndarray, q: float = 95) -> np.ndarray:
    """Computes the per-time-step q-th percentile of 3D position error magnitude.

    Args:
        delta_r_eb_n: Position error per trial (see
            `ins_sim.evaluation.errors.position_error`), shape
            (n_trials, M, 3) [m].
        q: Percentile to compute, in [0, 100]. Defaults to 95.

    Returns:
        numpy.ndarray: q-th percentile of the 3D error magnitude across
            trials at each time step, shape (M,) [m].

    Examples:
        Five trials whose errors are 1 to 5 m at every sample:

        >>> import numpy as np
        >>> from ins_sim.evaluation.monte_carlo import percentile_envelope
        >>> delta_r = np.zeros((5, 2, 3))
        >>> delta_r[:, :, 0] = np.arange(1.0, 6.0)[:, None]
        >>> [float(x) for x in percentile_envelope(delta_r, q=50)]
        [3.0, 3.0]
    """
    # --8<-- [start:percentile]
    return np.percentile(np.linalg.norm(delta_r_eb_n, axis=2), q, axis=0)
    # --8<-- [end:percentile]
