"""The navigator on a level IMU at rest at 45° N: perfect sensors, then a gyro bias."""

import numpy as np
from scipy.spatial.transform import Rotation

from ins_sim.core.earth_model import earth_rate_n, normal_gravity
from ins_sim.evaluation.errors import position_error
from ins_sim.navigation.strapdown import NavState, strapdown_navgrade

L_b, tau_i, M = np.deg2rad(45.0), 1.0, 3601          # one hour at 1 s
baro = np.zeros(M)                                    # an ideal barometer: h_b = 0

# At rest, a perfect IMU senses Earth rate and the reaction to gravity.
omega_ib_b = np.tile(earth_rate_n(L_b), (M, 1))
f_ib_b = np.tile([0.0, 0.0, -normal_gravity(L_b, 0.0)], (M, 1))
start = NavState(L_b, 0.0, 0.0, np.zeros(3), Rotation.identity().as_quat())

nav = strapdown_navgrade(omega_ib_b, f_ib_b, start, tau_i, meas_h_b=baro)
print(f"Perfect sensors: largest speed in 1 h = {np.abs(nav.v_eb_n).max():.1f} m/s")

# The same, with a 0.01 °/h bias on the gyro that points East (body y).
b_g = np.deg2rad(0.01) / 3600.0
nav = strapdown_navgrade(omega_ib_b + [0.0, b_g, 0.0], f_ib_b, start, tau_i, meas_h_b=baro)
delta_r_eb_n = position_error(nav.L_b, nav.lambda_b, nav.h_b, L_b, 0.0, 0.0)
print(f"0.01 °/h gyro bias: horizontal error after 1 h = "
      f"{np.hypot(*delta_r_eb_n[-1, :2]):.0f} m")
