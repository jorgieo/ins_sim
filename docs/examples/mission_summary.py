"""Checks a mission before a Monte Carlo run: how long, how far, how close to each waypoint."""

from pathlib import Path

import numpy as np
import yaml

from ins_sim.core.earth_model import wgs84_radii
from ins_sim.trajectory.kinematics import build_trajectory

MISSION = Path(__file__).parent / "missions" / "training_flight.yaml"
FT, KT, NM = 0.3048, 0.514444, 1852.0

truth, _, _ = build_trajectory(str(MISSION), tau_i=1.0)

speed = np.linalg.norm(truth.v_eb_n, axis=1)
flown = np.sum(np.linalg.norm(np.diff(truth.r_lb_l, axis=0), axis=1))
print(f"Duration           {truth.t[-1] / 60:7.1f} min")
print(f"Distance flown     {flown / NM:7.1f} nmi")
print(f"Highest altitude   {truth.h_b.max() / FT:7.0f} ft")
print(f"Top speed          {speed.max() / KT:7.0f} kt")
print(f"Steepest bank      {np.rad2deg(np.abs(truth.euler_nb[:, 0])).max():7.1f} deg")
print(f"Steepest pitch     {np.rad2deg(np.abs(truth.euler_nb[:, 1])).max():7.1f} deg")

# How close the track passes each waypoint, from the latitude and longitude
# differences converted to metres with the WGS-84 radii of curvature.
with open(MISSION, encoding="utf-8") as fh:
    waypoints = [p for p in yaml.safe_load(fh)["phases"] if p["type"] == "waypoint"]
for n, wp in enumerate(waypoints, start=1):
    L_wp, lambda_wp = np.deg2rad(wp["lat_deg"]), np.deg2rad(wp["lon_deg"])
    R_N, R_E = wgs84_radii(L_wp)
    north = (truth.L_b - L_wp) * R_N
    east = (truth.lambda_b - lambda_wp) * R_E * np.cos(L_wp)
    k = np.argmin(np.hypot(north, east))
    print(f"Waypoint {n}: passes {np.hypot(north[k], east[k]) / 1000:4.1f} km away "
          f"at {truth.t[k] / 60:4.1f} min, at {truth.h_b[k] / FT:5.0f} ft "
          f"(planned {wp['alt_ft']:.0f} ft)")
