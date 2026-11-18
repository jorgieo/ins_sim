"""What a perfect IMU measures along the packaged mission, and the zero-noise round trip."""

import numpy as np

from ins_sim.config import default_trajectory_path
from ins_sim.navigation.strapdown import NavState, strapdown_navgrade
from ins_sim.trajectory.kinematics import build_trajectory

np.set_printoptions(precision=3, suppress=True, floatmode="fixed")

truth, _, _ = build_trajectory(str(default_trajectory_path()), tau_i=1.0)
roll = truth.euler_nb[:, 0]

# Straight and level at Mach 0.8 (75 minutes in): the accelerometers sense
# the reaction to gravity, along the body z axis, and almost nothing else.
k = int(np.searchsorted(truth.t, 75 * 60.0))
print(f"t = {truth.t[k] / 60:2.0f} min, roll {np.rad2deg(roll[k]):4.1f}°: "
      f"f_ib^b = {truth.f_ib_b[k].round(3) + 0.0} m/s²")

# In the most steeply banked turn the specific force tilts with the
# aircraft, so it stays on the body z axis and grows to g / cos(roll).
k = int(np.argmax(np.abs(roll)))
print(f"t = {truth.t[k] / 60:2.0f} min, roll {np.rad2deg(roll[k]):4.1f}°: "
      f"f_ib^b = {truth.f_ib_b[k].round(3) + 0.0} m/s²")
print(f"g / cos(roll) = {truth.g[k] / np.cos(roll[k]):.3f} m/s²")

# The ideal outputs are the exact inverse of the navigator's update, so
# navigating with them reproduces the truth.
start = NavState(truth.L_b[0], truth.lambda_b[0], truth.h_b[0],
                 truth.v_eb_n[0], truth.C_b_n[0].as_quat())
nav = strapdown_navgrade(truth.omega_ib_b, truth.f_ib_b, start, truth.tau_i,
                         meas_h_b=truth.h_b)
largest = np.abs(nav.r_lb_l - truth.r_lb_l).max()
print(f"Round trip reproduces the truth to within a micrometre: {bool(largest < 1e-6)}")
