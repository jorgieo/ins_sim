import argparse
import os
from datetime import datetime
from types import SimpleNamespace

import numpy as np
import matplotlib.pyplot as plt

from ins_sim.trajectory.kinematics import build_trajectory
from ins_sim.sensors.imu import IMUSpec, load_imu_spec
from ins_sim.navigation.strapdown import NavState, strapdown_navgrade
from ins_sim.evaluation.monte_carlo import run_monte_carlo, percentile_envelope
from ins_sim.evaluation.visualization import build_summary_figure
from ins_sim.gui.figures import figure_map
from ins_sim.config import default_trajectory_path, default_imu_spec_path

NM = 1852.0


def run_self_consistency_check(truth, tau_i):
    """Runs a zero-noise strapdown integration to verify truth IMU self-consistency.

    Feeds the truth trajectory's own omega_ib^b and f_ib^b directly into
    strapdown_navgrade and compares the result against the truth
    position, verifying that the two are self-consistent under the
    same mechanization. Pure function, no I/O.

    Args:
        truth: Truth trajectory exposing L_b, lambda_b, h_b, v_eb_n,
            C_b_n, omega_ib_b, f_ib_b and r_lb_l.
        tau_i: Integration interval [s].

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]: (zero_err, zero_r_lb_l) where
            zero_err is the 3D position error norm at each time step,
            shape (M,) [m], and zero_r_lb_l is the zero-noise strapdown
            position relative to the start point, shape (M, 3) [m].
    """
    init_state = NavState(truth.L_b[0], truth.lambda_b[0], truth.h_b[0],
                          truth.v_eb_n[0].copy(),
                          truth.C_b_n[0].as_quat()) # pyright: ignore[reportCallIssue]
    zero_r_lb_l = strapdown_navgrade(truth.omega_ib_b, truth.f_ib_b, init_state,
                                     tau_i, meas_h_b=truth.h_b).r_lb_l
    zero_err = np.linalg.norm(zero_r_lb_l - truth.r_lb_l, axis=1)
    return zero_err, zero_r_lb_l


def simulate(truth, spec: IMUSpec, n_trials: int, seed: int = 0):
    """Runs the noisy-IMU Monte Carlo ensemble and the 95th-pct error envelope.

    Truth-agnostic -- works with any duck-typed truth object
    (build_trajectory- or TruthTrajectory-built).

    Args:
        truth: Truth trajectory exposing t, tau_i, L_b, lambda_b, h_b,
            v_eb_n, C_b_n, omega_ib_b, f_ib_b and r_lb_l.
        spec: IMU error-parameter set.
        n_trials: Number of independent Monte Carlo trials.
        seed: Seed for the master random generator. Defaults to 0.

    Returns:
        tuple: (est_r_lb_l, est_euler_nb, est_L_b, est_lambda_b, r95) —
            see run_monte_carlo and percentile_envelope for the shape and
            meaning of each element.
    """
    mc = run_monte_carlo(truth, spec, n_trials=n_trials, seed=seed)
    r95 = percentile_envelope(mc.est_r_lb_l, truth.r_lb_l, q=95)
    return mc.est_r_lb_l, mc.est_euler_nb, mc.est_L_b, mc.est_lambda_b, r95


def print_trajectory_summary(truth):
    """Prints a one-line summary of trajectory length, duration, and sampling.

    Args:
        truth: Truth trajectory exposing r_lb_l, t, tau_i and g.

    Returns:
        None: Prints to stdout.
    """
    total_dist = np.sum(np.linalg.norm(np.diff(truth.r_lb_l, axis=0), axis=1))
    print(f"Total path  : {total_dist/NM:.1f} nm  ({total_dist/1e3:.0f} km)")
    print(f"Duration    : {truth.t[-1]/60:.1f} min  ({truth.t[-1]:.0f} s)")
    print(f"Samples     : {len(truth.t):,}  (dt={truth.tau_i} s)")
    print(f"Start g     : {truth.g[0]:.5f} m/s²")


def print_self_consistency_summary(truth, zero_err):
    """Prints the zero-noise self-consistency error at several time fractions.

    Args:
        truth: Truth trajectory exposing t, v_eb_n_dot and f_ib_b.
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

    print(f"Max |dv_eb_n/dt|: {np.linalg.norm(truth.v_eb_n_dot, axis=1).max():.2f} m/s² \n"
          f"Max |f_ib_b|: {np.linalg.norm(truth.f_ib_b, axis=1).max():.2f} m/s²")


def parse_args(argv=None):
    """Parses the headless CLI arguments (defaults preserve legacy behavior).

    Args:
        argv: Argument list, or None for sys.argv.

    Returns:
        argparse.Namespace: config, imu_spec, trials, dt, seed.
    """
    parser = argparse.ArgumentParser(
        description="Headless INS Monte Carlo run: summary figure + "
                    "interactive ground-track map.")
    parser.add_argument("--config", default=str(default_trajectory_path()),
                        help="Trajectory mission YAML "
                             "(default: packaged BQN departure)")
    parser.add_argument("--imu-spec", default=str(default_imu_spec_path()),
                        help="IMU error spec YAML "
                             "(default: packaged navigation-grade spec)")
    parser.add_argument("--trials", type=int, default=20,
                        help="Monte Carlo trial count (default 20)")
    parser.add_argument("--dt", type=float, default=0.1,
                        help="Integration time step [s] (default 0.1)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Master RNG seed (default 42)")
    return parser.parse_args(argv)


def main(argv=None):
    """Runs the end-to-end INS Monte Carlo simulation and renders its outputs.

    Builds the truth trajectory from the mission YAML, verifies truth
    IMU self-consistency, runs the noisy-IMU Monte Carlo ensemble, then
    renders the summary figure and saves an interactive ground-track
    map.

    Args:
        argv: CLI argument list, or None for sys.argv.

    Returns:
        None.
    """
    args = parse_args(argv)
    simulation_start = datetime.now()

    truth, v_sprint, R_turn = build_trajectory(args.config, tau_i=args.dt)
    print_trajectory_summary(truth)

    # Diagnostic: zero-noise strapdown verifies truth IMU self-consistency
    zero_err, _ = run_self_consistency_check(truth, truth.tau_i)
    print_self_consistency_summary(truth, zero_err)

    n_trials = args.trials
    spec = load_imu_spec(args.imu_spec)
    est_r_lb_l, est_euler_nb, est_L_b, est_lambda_b, r95 = simulate(
        truth, spec, n_trials=n_trials, seed=args.seed)
    print(f"95th-pct error  start: {r95[0]:6.3f} m   end: {r95[-1]:6.1f} m")

    elapsed = datetime.now() - simulation_start
    hours, rem = divmod(elapsed.total_seconds(), 3600)
    minutes, seconds = divmod(rem, 60)
    print(f"Monte Carlo simulation time: {int(hours)}h {int(minutes)}m {int(seconds)}s")

    fig  = build_summary_figure(truth, est_r_lb_l, est_euler_nb, r95, n_trials)
    fmap = figure_map(SimpleNamespace(
        truth=truth, est_r_lb_l=est_r_lb_l, est_L_b=est_L_b,
        est_lambda_b=est_lambda_b, n_trials=n_trials))

    maps_dir = os.path.join(os.path.dirname(__file__), "maps")
    os.makedirs(maps_dir, exist_ok=True)
    map_path = os.path.join(maps_dir, "trajectory_map.html")
    fmap.write_html(map_path, include_plotlyjs=True,
                    config={"scrollZoom": True})
    print(f"Interactive map saved: {map_path}")

    plt.show()


if __name__ == "__main__":
    main()
