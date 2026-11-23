"""A 1 m/s Down-velocity error, free-inertial and with an ideal barometer."""

import numpy as np
from scipy.spatial.transform import Rotation

from ins_sim.core.earth_model import WGS84_A, WGS84_OMEGA, earth_rate_n, normal_gravity
from ins_sim.navigation.strapdown import NavState, strapdown_navgrade

L_b, tau_i, M = 0.0, 1.0, 1801                       # 30 minutes at the equator
g = normal_gravity(L_b, 0.0)
omega_ib_b = np.tile(earth_rate_n(L_b), (M, 1))      # a level IMU at rest
f_ib_b = np.tile([0.0, 0.0, -g], (M, 1))
start = NavState(L_b, 0.0, 0.0, np.array([0.0, 0.0, 1.0]),   # v_D wrong by 1 m/s
                 Rotation.identity().as_quat())

free = strapdown_navgrade(omega_ib_b, f_ib_b, start, tau_i).h_b
aided = strapdown_navgrade(omega_ib_b, f_ib_b, start, tau_i, meas_h_b=np.zeros(M)).h_b

tau_v = 1.0 / np.sqrt(2.0 * g / WGS84_A - 4.0 * WGS84_OMEGA**2)   # free-inertial
tau = 100.0                                                       # baro loop
print(f"Free-inertial time constant tau_v = {tau_v:.1f} s")
print("Height error [m]  free-inertial           baro-aided")
print("t [min]           simulated  predicted    simulated  predicted")
for minutes in (2, 5, 10, 20, 30):
    t = minutes * 60.0
    k = int(t / tau_i)
    predicted_free = -tau_v * np.sinh(t / tau_v)
    predicted_aided = -t * np.exp(-t / tau) * (1.0 - t / (2.0 * tau))
    print(f"{minutes:7d}         {free[k]:10.1f} {predicted_free:10.1f}"
          f"   {round(aided[k], 1) + 0.0:10.1f} {round(predicted_aided, 1) + 0.0:10.1f}")
