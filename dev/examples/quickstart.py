"""Monte Carlo run on the packaged mission: the 95th-percentile position error."""

import numpy as np

from ins_sim.config import default_imu_spec_path, default_trajectory_path
from ins_sim.evaluation.errors import position_error
from ins_sim.evaluation.monte_carlo import percentile_envelope, run_monte_carlo
from ins_sim.sensors.imu import load_imu_spec
from ins_sim.trajectory.kinematics import build_trajectory

# Truth trajectory at 1 s steps (tau_i) and the navigation-grade IMU spec.
truth, _, _ = build_trajectory(str(default_trajectory_path()), tau_i=1.0)
spec = load_imu_spec(str(default_imu_spec_path()))

# Ten independent trials of the noisy IMU and the strapdown navigator.
mc = run_monte_carlo(truth, spec, n_trials=10, seed=7)

# Position error (estimate minus truth) in metres along North, East, Down.
delta_r_eb_n = position_error(mc.est_L_b, mc.est_lambda_b, mc.est_h_b,
                              truth.L_b, truth.lambda_b, truth.h_b)
r95 = percentile_envelope(delta_r_eb_n, q=95)

one_hour = np.searchsorted(truth.t, 3600.0)
print(f"{truth.t[-1] / 60:.0f} min of flight, {mc.est_L_b.shape[0]} trials")
print(f"95th-percentile position error after 1 h: {r95[one_hour]:.0f} m")
