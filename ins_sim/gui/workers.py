"""Background simulation worker for the INS simulator GUI.

SimulationWorker runs the full trajectory -> Monte Carlo -> envelope
pipeline (the same one main.py drives headlessly) inside a QThread,
reporting progress and results back to the GUI thread via Qt signals.
"""

import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PySide6.QtCore import QObject, Signal

from ins_sim.config import default_imu_spec_path
from ins_sim.evaluation.monte_carlo import percentile_envelope, run_monte_carlo
from ins_sim.sensors.imu import load_imu_spec
from ins_sim.trajectory.kinematics import build_trajectory

NM = 1852.0


@dataclass
class SimulationResult:
    """Everything a plotting layer needs from one completed Monte Carlo run."""

    truth: object
    pos_runs: np.ndarray      # (n_trials, M, 3) NED position [m]
    euler_runs: np.ndarray    # (n_trials, M, 3) [phi, theta, psi] [rad]
    lat_runs: np.ndarray      # (n_trials, M) latitude [rad]
    lon_runs: np.ndarray      # (n_trials, M) longitude [rad]
    vel_runs: np.ndarray      # (n_trials, M, 3) NED velocity [m/s]
    r95: np.ndarray           # (M,) 95th-pct radial error envelope [m]
    n_trials: int
    config: dict = field(default_factory=dict)
    elapsed_s: float = 0.0


class SimulationWorker(QObject):
    """Runs the Monte Carlo pipeline off the GUI thread.

    Construct with the config dict from MainWindow.current_config(),
    move to a QThread, and invoke run() via the thread's started signal.

    Signals:
        progress_updated (int): Percent complete, 0-100.
        log_message (str): Human-readable stage/progress lines.
        simulation_finished (object): A SimulationResult on success.
        error_occurred (str): Error description on failure.
    """

    progress_updated = Signal(int)
    log_message = Signal(str)
    simulation_finished = Signal(object)
    error_occurred = Signal(str)

    def __init__(self, config: dict, seed: int = 42, parent=None):
        super().__init__(parent)
        self._config = config
        self._seed = seed
        # TODO(phase-3): check self._cancel_requested in the progress
        # callback and abort cleanly once a Cancel button exists.
        self._cancel_requested = False

    def run(self) -> None:
        """Executes the full pipeline, emitting signals along the way."""
        try:
            result = self._run_pipeline()
        except Exception as e:
            self.error_occurred.emit(f"{type(e).__name__}: {e}")
            return
        self.simulation_finished.emit(result)

    def _run_pipeline(self) -> SimulationResult:
        start = time.monotonic()

        trajectory_path = self._config["trajectory_path"]
        if not trajectory_path:
            raise ValueError("No trajectory file selected")
        imu_spec_path = self._config.get("imu_spec_path") or str(default_imu_spec_path())
        dt = self._config.get("dt_s", 0.1)
        n_trials = self._config["n_iterations"]

        self.log_message.emit(f"Loading trajectory: {Path(trajectory_path).name} "
                              f"(dt = {dt} s)")
        truth, _, _ = build_trajectory(str(trajectory_path), dt=dt)

        total_dist = np.sum(np.linalg.norm(np.diff(truth.pos_n, axis=0), axis=1))
        self.log_message.emit(
            f"Trajectory: {total_dist/NM:.1f} nm, {truth.t[-1]/60:.1f} min, "
            f"{len(truth.t):,} samples")

        self.log_message.emit(f"Loading IMU spec: {Path(imu_spec_path).name}")
        spec = load_imu_spec(str(imu_spec_path))

        baro_aiding = self._config.get("baro_aiding", True)
        vertical = "baro-damped" if baro_aiding else "free-inertial"
        self.log_message.emit(f"Running {n_trials} Monte Carlo trials "
                              f"({vertical} vertical channel)...")
        pos_runs, euler_runs, lat_runs, lon_runs, vel_runs = run_monte_carlo(
            truth, spec, n_trials=n_trials, seed=self._seed,
            progress_callback=self._on_progress, baro_aiding=baro_aiding)
        r95 = percentile_envelope(pos_runs, truth.pos_n, q=95)

        elapsed = time.monotonic() - start
        self.log_message.emit(
            f"Done in {elapsed:.1f} s — 95th-pct error "
            f"start: {r95[0]:.3f} m, end: {r95[-1]:.1f} m")

        return SimulationResult(
            truth=truth, pos_runs=pos_runs, euler_runs=euler_runs,
            lat_runs=lat_runs, lon_runs=lon_runs, vel_runs=vel_runs, r95=r95,
            n_trials=n_trials, config=dict(self._config), elapsed_s=elapsed)

    def _on_progress(self, n_done: int, n_total: int) -> None:
        self.progress_updated.emit(n_done * 100 // n_total)
        self.log_message.emit(f"Trial {n_done}/{n_total} complete")
