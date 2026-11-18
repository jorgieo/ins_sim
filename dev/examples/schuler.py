"""The Schuler period, and the bounded position error that an initial tilt causes."""

import numpy as np
from scipy.spatial.transform import Rotation

from ins_sim.core.earth_model import earth_rate_n, normal_gravity, wgs84_radii
from ins_sim.evaluation.errors import position_error
from ins_sim.navigation.strapdown import NavState, strapdown_navgrade

L_b, h_b, tau_i = 0.0, 0.0, 1.0                     # the equator: no Foucault coupling
R_N, R_E = wgs84_radii(L_b)
g = normal_gravity(L_b, h_b)
omega_s = np.sqrt(g / (R_E + h_b))                  # East channel: radius R_E
T_s = 2.0 * np.pi / omega_s
print(f"Schuler period T_s = {T_s / 60:.2f} min")

# A level IMU at rest, perfect sensors and an ideal barometer, but the
# navigator starts with a 10 arcsecond tilt about North.
tilt = np.deg2rad(10.0 / 3600.0)
M = int(round(T_s / tau_i)) + 1                     # one Schuler period
start = NavState(L_b, 0.0, h_b, np.zeros(3), Rotation.from_rotvec([tilt, 0.0, 0.0]).as_quat())
nav = strapdown_navgrade(np.tile(earth_rate_n(L_b), (M, 1)),
                         np.tile([0.0, 0.0, -g], (M, 1)), start, tau_i,
                         meas_h_b=np.zeros(M))
delta_r_E = position_error(nav.L_b, nav.lambda_b, nav.h_b, L_b, 0.0, h_b)[:, 1]
print(f"Largest East error: {delta_r_E.max():.1f} m, "
      f"predicted 2 (R_E + h_b) tilt = {2.0 * (R_E + h_b) * tilt:.1f} m")
print(f"East error after one period: {abs(delta_r_E[-1]):.1f} m")
