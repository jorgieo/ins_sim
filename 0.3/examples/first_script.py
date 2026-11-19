"""Your first script: a Monte Carlo run on the packaged mission, step by step."""

# --8<-- [start:imports]
import matplotlib.pyplot as plt
import numpy as np

from ins_sim.config import default_imu_spec_path, default_trajectory_path
from ins_sim.evaluation.errors import position_error
from ins_sim.evaluation.monte_carlo import run_monte_carlo
from ins_sim.sensors.imu import load_imu_spec
from ins_sim.trajectory.kinematics import build_trajectory
# --8<-- [end:imports]

# --8<-- [start:truth]
# 1. The truth: the packaged mission, sampled every second.
truth, _, _ = build_trajectory(str(default_trajectory_path()), tau_i=1.0)
print(f"Mission: {truth.t[-1] / 60:.1f} min, {len(truth.t)} samples")
# --8<-- [end:truth]

# --8<-- [start:spec]
# 2. The IMU: the packaged navigation-grade spec, converted to SI units.
spec = load_imu_spec(str(default_imu_spec_path()))
print(f"Gyro bias instability: {np.rad2deg(spec.gyro_bias_instability) * 3600:.3f} deg/h")
# --8<-- [end:spec]

# --8<-- [start:run]
# 3. Ten trials, each with its own draw of every sensor error.
mc = run_monte_carlo(truth, spec, n_trials=10, seed=42)
# --8<-- [end:run]

# --8<-- [start:errors]
# 4. Position error, estimate minus truth, in metres along North, East, Down.
delta_r_eb_n = position_error(mc.est_L_b, mc.est_lambda_b, mc.est_h_b,
                              truth.L_b, truth.lambda_b, truth.h_b)
horizontal = np.hypot(delta_r_eb_n[..., 0], delta_r_eb_n[..., 1])  # (trials, samples)
cep = np.median(horizontal, axis=0)

one_hour = np.searchsorted(truth.t, 3600.0)
print(f"CEP after 1 h: {cep[one_hour] / 1852:.2f} nmi")
# --8<-- [end:errors]

# --8<-- [start:plot]
# 5. Every trial's horizontal error, and their median, the CEP.
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(truth.t / 60, horizontal.T / 1852, color="0.75", linewidth=0.8)
ax.plot(truth.t / 60, cep / 1852, color="#2a78d6", linewidth=2.5, label="CEP")
ax.set(xlabel="Time (min)", ylabel="Horizontal error (nmi)",
       title="Navigation-grade IMU on the packaged mission, 10 trials")
ax.legend(loc="upper left")
fig.savefig("first_script.png", dpi=150, bbox_inches="tight")
print("Saved first_script.png")
# --8<-- [end:plot]
