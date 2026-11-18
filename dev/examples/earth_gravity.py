"""The WGS-84 Earth model: radii of curvature, gravity, Earth rate and transport rate."""

import numpy as np

from ins_sim.core.earth_model import (earth_rate_n, normal_gravity,
                                      transport_rate_n, wgs84_radii)

DEG_PER_HOUR = np.rad2deg(1.0) * 3600.0     # converts rad/s to °/h
MICRO_G = 9.80665e-6                        # 1 µg in m/s²
np.set_printoptions(precision=3, suppress=True, floatmode="fixed")

print("Latitude   R_N [km]   R_E [km]   g at h_b = 0 [m/s²]")
for lat_deg in (0, 30, 45, 60, 90):
    L_b = np.deg2rad(lat_deg)
    R_N, R_E = wgs84_radii(L_b)
    print(f"{lat_deg:7d}°  {R_N / 1e3:9.3f}  {R_E / 1e3:9.3f}  {normal_gravity(L_b, 0.0):12.6f}")

# Gravity weakens with height, by about 2g/R_0 per metre.
L_b = np.deg2rad(45.0)
weaker = normal_gravity(L_b, 0.0) - normal_gravity(L_b, 1000.0)
print(f"\nAt 45° N, gravity 1 km up is {weaker / MICRO_G:.0f} µg weaker")

# Earth rate, and the transport rate of an aircraft flying east at 250 m/s
# and 4572 m: the two are of the same size.
omega_ie_n = earth_rate_n(L_b)
omega_en_n = transport_rate_n([0.0, 250.0, 0.0], L_b, 4572.0)
print("At 45° N, in °/h:")
print("  Earth rate omega_ie^n                ", (omega_ie_n * DEG_PER_HOUR).round(3) + 0.0)
print("  transport rate omega_en^n, 250 m/s E ", (omega_en_n * DEG_PER_HOUR).round(3) + 0.0)
