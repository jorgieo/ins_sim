"""Which error source dominates? The covariance analysis's error budget for the training flight."""

from pathlib import Path

import numpy as np

from ins_sim.config import default_imu_spec_path
from ins_sim.evaluation.covariance import error_budget
from ins_sim.evaluation.errors import position_error
from ins_sim.evaluation.monte_carlo import run_monte_carlo
from ins_sim.sensors.imu import load_imu_spec
from ins_sim.trajectory.kinematics import build_trajectory

MISSION = Path(__file__).parent / "missions" / "training_flight.yaml"

truth, _, _ = build_trajectory(str(MISSION), tau_i=2.0)
spec = load_imu_spec(str(default_imu_spec_path()))
budget = error_budget(truth, spec)


def horizontal_variance(part):
    """North plus East position variance at the end of the flight [m²]."""
    return part.var[-1, 6] + part.var[-1, 7]


print(f"After {truth.t[-1] / 60:.0f} min        CEP alone   share of the variance")
for source, part in budget.by_source.items():
    share = 100 * horizontal_variance(part) / horizontal_variance(budget.total)
    print(f"  {source:38s} {part.cep[-1]:5.0f} m   {share:5.1f}%")
print(f"  {'All sources together':38s} {budget.total.cep[-1]:5.0f} m")

# The same flight, flown 50 times with random errors.
mc = run_monte_carlo(truth, spec, n_trials=50, seed=42)
delta_r_eb_n = position_error(mc.est_L_b, mc.est_lambda_b, mc.est_h_b,
                              truth.L_b, truth.lambda_b, truth.h_b)
cep = np.median(np.hypot(delta_r_eb_n[:, -1, 0], delta_r_eb_n[:, -1, 1]))
print(f"  {'Monte Carlo, 50 trials':38s} {cep:5.0f} m")
