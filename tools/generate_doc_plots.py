"""Generates the documentation figure assets from the current codebase.

Runs the baseline Monte Carlo (packaged BQN departure trajectory +
navigation-grade IMU spec) and exports the GUI's own plotly figures as
high-resolution PNGs. Because the exports come from ins_sim.gui.figures,
the doc images are identical to what the app renders — regenerate after
figure or model changes so docs never drift:

    python tools/generate_doc_plots.py                      # full baseline
    python tools/generate_doc_plots.py --trials 5 --dt 1.0  # quick pass
    python tools/generate_doc_plots.py --validation         # Validation page
    python tools/generate_doc_plots.py --theory             # Theory pages
    python tools/generate_doc_plots.py --guide              # Get started and guide

With --validation it instead runs the scenarios in validation_scenarios.py,
writes their figures to docs/assets/validation/, and writes the results
table (includes/validation_results.md) that the Validation page includes.
With --theory it draws the Theory pages' own figures (theory_figures.py)
into docs/assets/theory/. With --guide it takes the screenshots of the app
(gui_screenshots.py, which needs the gui extra, a display and an internet
connection for the map tiles) into docs/assets/gui/, and runs the "Your
first script" tutorial in docs/assets/get-started/ to save its figure there.

Requires kaleido (dev extra) for plotly PNG export.
"""

import argparse
import os
import runpy
import sys
from pathlib import Path
from types import SimpleNamespace

# Guarantee the repo source tree wins over any installed ins_sim copy.
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from ins_sim.config import default_imu_spec_path, default_trajectory_path  # noqa: E402
from ins_sim.evaluation.errors import position_error  # noqa: E402
from ins_sim.evaluation.monte_carlo import percentile_envelope, run_monte_carlo  # noqa: E402
from ins_sim.gui import figures  # noqa: E402
from ins_sim.sensors.imu import load_imu_spec  # noqa: E402
from ins_sim.trajectory.kinematics import build_trajectory  # noqa: E402

PLOTS_DIR = REPO_ROOT / "docs" / "assets" / "plots"
VALIDATION_DIR = REPO_ROOT / "docs" / "assets" / "validation"
VALIDATION_TABLE = REPO_ROOT / "includes" / "validation_results.md"
THEORY_DIR = REPO_ROOT / "docs" / "assets" / "theory"
GUI_DIR = REPO_ROOT / "docs" / "assets" / "gui"
GET_STARTED_DIR = REPO_ROOT / "docs" / "assets" / "get-started"
FIRST_SCRIPT = REPO_ROOT / "docs" / "examples" / "first_script.py"

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
    truth, _, _ = build_trajectory(str(default_trajectory_path()), tau_i=dt)
    spec = load_imu_spec(str(default_imu_spec_path()))
    print(f"Baseline: {trials} trials, dt={dt} s, "
          f"{len(truth.t):,} samples/trial", flush=True)
    mc = run_monte_carlo(
        truth, spec, n_trials=trials, seed=seed,
        progress_callback=lambda done, total: print(
            f"  trial {done}/{total}", flush=True))
    delta_r_eb_n = position_error(mc.est_L_b, mc.est_lambda_b, mc.est_h_b,
                                  truth.L_b, truth.lambda_b, truth.h_b)
    r95 = percentile_envelope(delta_r_eb_n, q=95)
    return SimpleNamespace(truth=truth, **mc._asdict(), r95=r95, n_trials=trials)


def build_guide_images() -> None:
    """Takes the app screenshots and saves the first-script tutorial's figure."""
    import gui_screenshots  # noqa: E402 (needs PySide6, so imported only here)

    GUI_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Screenshots into {GUI_DIR.relative_to(REPO_ROOT)}:", flush=True)
    gui_screenshots.take_screenshots(GUI_DIR)

    # The tutorial script saves its figure in the current folder, so the
    # image on its page is exactly what readers get by running it.
    GET_STARTED_DIR.mkdir(parents=True, exist_ok=True)
    os.chdir(GET_STARTED_DIR)
    runpy.run_path(str(FIRST_SCRIPT), run_name="__main__")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate the documentation figure assets.")
    parser.add_argument("--trials", type=int, default=50,
                        help="Monte Carlo trials (default 50)")
    parser.add_argument("--dt", type=float, default=0.1,
                        help="Integration time step [s] (default 0.1)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Master RNG seed (default 42)")
    parser.add_argument("--validation", action="store_true",
                        help="Generate the Validation page figures and results table instead")
    parser.add_argument("--theory", action="store_true",
                        help="Generate the Theory pages' figures instead")
    parser.add_argument("--guide", action="store_true",
                        help="Take the app screenshots and the tutorial figure instead")
    args = parser.parse_args()

    if args.validation:
        import validation_figures  # noqa: E402 (tools/ is on sys.path as the script's folder)
        validation_figures.build_all(VALIDATION_DIR, VALIDATION_TABLE)
        return 0
    if args.theory:
        import theory_figures  # noqa: E402 (tools/ is on sys.path as the script's folder)
        theory_figures.build_all(THEORY_DIR)
        return 0
    if args.guide:
        build_guide_images()
        return 0

    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    result = run_baseline(args.trials, args.dt, args.seed)
    for filename, builder in SIM_FIGURES:
        path = PLOTS_DIR / filename
        builder(result).write_image(str(path), **EXPORT_KW)
        print(f"wrote {path.relative_to(REPO_ROOT)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
