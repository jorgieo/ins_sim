import os
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt

from ins_sim.trajectory.kinematics import build_trajectory
from ins_sim.sensors.imu import IMUSpec, load_imu_spec
from ins_sim.navigation.strapdown import strapdown_navgrade
from ins_sim.evaluation.monte_carlo import run_monte_carlo, percentile_envelope
from ins_sim.evaluation.visualization import build_summary_figure, build_folium_map
from ins_sim.config import default_trajectory_path, default_imu_spec_path

NM = 1852.0


def run_self_consistency_check(truth, dt):
    """Runs a zero-noise strapdown integration to verify truth IMU self-consistency.

    Feeds the truth trajectory's own ω_ib_b and f_b directly into
    strapdown_navgrade and compares the result against the truth
    position, verifying that the two are self-consistent under the
    same mechanization. Pure function, no I/O.

    Args:
        truth: Truth trajectory object exposing lat, lon, alt, vel_n,
            R_b2n, omega_b (ω_ib_b), f_b, and pos_n.
        dt: Sample interval [s].

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]: (zero_err, zp) where
            zero_err is the 3D position error norm at each time step,
            shape (M,) [m], and zp is the zero-noise strapdown NED
            position, shape (M, 3) [m].
    """
    zero_spec = IMUSpec(gyro_arw=0, gyro_bi_std=0, gyro_br_std=0,
                        accel_vrw=0, accel_bi_std=0, accel_br_std=0)
    init_state = (truth.lat[0], truth.lon[0], truth.alt[0],
                  truth.vel_n[0].copy(),
                  truth.R_b2n[0].as_quat()) # pyright: ignore[reportCallIssue]
    zp, *_ = strapdown_navgrade(truth.omega_b, truth.f_b, init_state, dt,
                                 alt_truth=truth.alt)
    zero_err = np.linalg.norm(zp - truth.pos_n, axis=1)
    return zero_err, zp


def simulate(truth, spec: IMUSpec, n_trials: int, seed: int = 0):
    """Runs the noisy-IMU Monte Carlo ensemble and the 95th-pct error envelope.

    Truth-agnostic -- works with any duck-typed truth object
    (build_trajectory- or TruthTrajectory-built).

    Args:
        truth: Truth trajectory object exposing t, dt, lat, lon, alt,
            vel_n, R_b2n, omega_b, f_b, and pos_n.
        spec: IMU error-parameter set.
        n_trials: Number of independent Monte Carlo trials.
        seed: Seed for the master random generator. Defaults to 0.

    Returns:
        tuple: (pos_runs, euler_runs, lat_runs, lon_runs, r95) — see
            run_monte_carlo and percentile_envelope for the shape and
            meaning of each element.
    """
    pos_runs, euler_runs, lat_runs, lon_runs = run_monte_carlo(
        truth, spec, n_trials=n_trials, seed=seed)
    r95 = percentile_envelope(pos_runs, truth.pos_n, q=95)
    return pos_runs, euler_runs, lat_runs, lon_runs, r95


def print_trajectory_summary(truth):
    """Prints a one-line summary of trajectory length, duration, and sampling.

    Args:
        truth: Truth trajectory object exposing pos_n, t, dt, and
            g_loc.

    Returns:
        None: Prints to stdout.
    """
    total_dist = np.sum(np.linalg.norm(np.diff(truth.pos_n, axis=0), axis=1))
    print(f"Total path  : {total_dist/NM:.1f} nm  ({total_dist/1e3:.0f} km)")
    print(f"Duration    : {truth.t[-1]/60:.1f} min  ({truth.t[-1]:.0f} s)")
    print(f"Samples     : {len(truth.t):,}  (dt={truth.dt} s)")
    print(f"Start g     : {truth.g_loc[0]:.5f} m/s²")


def print_self_consistency_summary(truth, zero_err):
    """Prints the zero-noise self-consistency error at several time fractions.

    Args:
        truth: Truth trajectory object exposing t, acc_n, and f_b.
        zero_err: Zero-noise position error norm at each time step,
            shape (M,) [m], as returned by run_self_consistency_check.

    Returns:
        None: Prints to stdout.
    """
    print(f"Zero-noise error  start: {zero_err[0]:.3f} m  max: {zero_err.max():.1f} m  end: {zero_err[-1]:.1f} m")

    M_diag = len(truth.t)
    for pct in [10, 25, 50, 75, 100]:
        idx = int(pct/100 * (M_diag - 1))
        print(f"  t = {truth.t[idx]/60:.1f} min ({pct}%): err = {zero_err[idx]:.1f} m")

    print(f"Max |acc_n|: {np.linalg.norm(truth.acc_n, axis=1).max():.2f} m/s² \n"
          f"Max |f_b|: {np.linalg.norm(truth.f_b, axis=1).max():.2f} m/s²")


def main():
    """Runs the end-to-end INS Monte Carlo simulation and renders its outputs.

    Builds the truth trajectory from the default mission YAML, verifies
    truth IMU self-consistency, runs the noisy-IMU Monte Carlo ensemble
    using the default IMU spec, then renders and saves the summary
    figure and an interactive Folium map.

    Returns:
        None.
    """
    simulation_start = datetime.now()

    dt       = 0.1      # 10 Hz — adequate for long-range INS simulation
    n_trials = 20

    truth, v_sprint, R_turn = build_trajectory(str(default_trajectory_path()), dt=dt)
    print_trajectory_summary(truth)

    # Diagnostic: zero-noise strapdown verifies truth IMU self-consistency
    zero_err, _ = run_self_consistency_check(truth, truth.dt)
    print_self_consistency_summary(truth, zero_err)

    spec = load_imu_spec(str(default_imu_spec_path()))
    pos_runs, euler_runs, lat_runs, lon_runs, r95 = simulate(
        truth, spec, n_trials=n_trials, seed=42)
    print(f"95th-pct error  start: {r95[0]:6.3f} m   end: {r95[-1]:6.1f} m")

    elapsed = datetime.now() - simulation_start
    hours, rem = divmod(elapsed.total_seconds(), 3600)
    minutes, seconds = divmod(rem, 60)
    print(f"Monte Carlo simulation time: {int(hours)}h {int(minutes)}m {int(seconds)}s")

    fig  = build_summary_figure(truth, pos_runs, euler_runs, r95, n_trials)
    fmap = build_folium_map(truth, pos_runs, lat_runs, lon_runs, n_trials)

    maps_dir = os.path.join(os.path.dirname(__file__), "maps")
    os.makedirs(maps_dir, exist_ok=True)
    map_path = os.path.join(maps_dir, "trajectory_map.html")
    fmap.save(map_path)
    print(f"Interactive map saved: {map_path}")

    plt.show()


if __name__ == "__main__":
    main()
