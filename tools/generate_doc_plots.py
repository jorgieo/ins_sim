"""Generates the documentation figure assets from the current codebase.

Runs the baseline Monte Carlo (packaged BQN departure trajectory +
navigation-grade IMU spec) and exports the GUI's own plotly figures as
high-resolution PNGs. Because the exports come from ins_sim.gui.figures,
the doc images are identical to what the app renders — regenerate after
figure or model changes so docs never drift:

    python tools/generate_doc_plots.py                      # full baseline
    python tools/generate_doc_plots.py --trials 5 --dt 1.0  # quick pass

Requires kaleido (dev extra) for plotly PNG export.
"""

import argparse
import sys
from pathlib import Path
from types import SimpleNamespace

# Guarantee the repo source tree wins over any installed ins_sim copy.
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from ins_sim.config import default_imu_spec_path, default_trajectory_path  # noqa: E402
from ins_sim.evaluation.monte_carlo import percentile_envelope, run_monte_carlo  # noqa: E402
from ins_sim.gui import figures  # noqa: E402
from ins_sim.sensors.imu import load_imu_spec  # noqa: E402
from ins_sim.trajectory.kinematics import build_trajectory  # noqa: E402

PLOTS_DIR = REPO_ROOT / "docs" / "assets" / "plots"

# 1400x900 logical px at scale 3 ≈ 300 DPI for a 14-inch-wide figure.
EXPORT_KW = dict(width=1400, height=900, scale=3)

SIM_FIGURES = [
    ("3d_trajectory.png",  figures.figure_trajectory_3d),
    ("position_error.png", figures.figure_position),
    ("velocity_error.png", figures.figure_velocity),
    ("attitude_error.png", figures.figure_attitude),
    ("cep.png",            figures.figure_cep),
]


def run_baseline(trials: int, dt: float, seed: int) -> SimpleNamespace:
    """Runs the baseline Monte Carlo and returns a figures-compatible result."""
    truth, _, _ = build_trajectory(str(default_trajectory_path()), dt=dt)
    spec = load_imu_spec(str(default_imu_spec_path()))
    print(f"Baseline: {trials} trials, dt={dt} s, "
          f"{len(truth.t):,} samples/trial", flush=True)
    pos_runs, euler_runs, lat_runs, lon_runs, vel_runs = run_monte_carlo(
        truth, spec, n_trials=trials, seed=seed,
        progress_callback=lambda done, total: print(
            f"  trial {done}/{total}", flush=True))
    r95 = percentile_envelope(pos_runs, truth.pos_n, q=95)
    return SimpleNamespace(
        truth=truth, pos_runs=pos_runs, euler_runs=euler_runs,
        lat_runs=lat_runs, lon_runs=lon_runs, vel_runs=vel_runs,
        r95=r95, n_trials=trials)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate the documentation figure assets.")
    parser.add_argument("--trials", type=int, default=50,
                        help="Monte Carlo trials (default 50)")
    parser.add_argument("--dt", type=float, default=0.1,
                        help="Integration time step [s] (default 0.1)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Master RNG seed (default 42)")
    args = parser.parse_args()

    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    result = run_baseline(args.trials, args.dt, args.seed)
    for filename, builder in SIM_FIGURES:
        path = PLOTS_DIR / filename
        builder(result).write_image(str(path), **EXPORT_KW)
        print(f"wrote {path.relative_to(REPO_ROOT)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
