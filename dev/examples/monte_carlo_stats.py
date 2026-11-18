"""A small Monte Carlo run, summarized the way the GUI summarizes it."""

import numpy as np

from ins_sim.config import default_imu_spec_path
from ins_sim.evaluation.errors import position_error
from ins_sim.evaluation.monte_carlo import percentile_envelope, run_monte_carlo
from ins_sim.sensors.imu import load_imu_spec
from ins_sim.trajectory.kinematics import TruthTrajectory
from ins_sim.trajectory.spline import NEDSplinePath

# About 11 minutes at 150 m/s: 60 km north, then a gentle turn to the north-east.
path = NEDSplinePath([[0.0, 0.0, 0.0], [60e3, 0.0, 0.0], [90e3, 30e3, 0.0]])
truth = TruthTrajectory.from_spline(path, speed=150.0, tau_i=2.0)
spec = load_imu_spec(str(default_imu_spec_path()))

mc = run_monte_carlo(truth, spec, n_trials=25, seed=42)
delta_r_eb_n = position_error(mc.est_L_b, mc.est_lambda_b, mc.est_h_b,
                              truth.L_b, truth.lambda_b, truth.h_b)

n_trials = len(delta_r_eb_n)
final = delta_r_eb_n[:, -1, :]                       # every trial's error at the end
mean, sigma = final.mean(axis=0), final.std(axis=0, ddof=1)
print(f"{n_trials} trials, {truth.t[-1] / 60:.0f} min of flight. At the end:")
for axis, name in enumerate(("North", "East", "Down")):
    print(f"  {name:5s} error: mean {mean[axis]:7.1f} m, 3-sigma {3.0 * sigma[axis]:6.1f} m")
print(f"  (each sigma is uncertain by about {100 / np.sqrt(2 * (n_trials - 1)):.0f}% "
      f"with {n_trials} trials)")
horizontal = np.linalg.norm(final[:, :2], axis=1)
print(f"  CEP, the median horizontal error: {np.median(horizontal):.1f} m")
print(f"  95th percentile of the 3-D error: {percentile_envelope(delta_r_eb_n, q=95)[-1]:.1f} m")
