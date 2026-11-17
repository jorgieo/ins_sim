"""Figures for the documentation's Theory pages.

Draws the simulator figures that the Theory pages show and the Validation
page does not, into docs/assets/theory/. Called by

    python tools/generate_doc_plots.py --theory

Chart conventions follow validation_figures.py: one series is blue, a
comparison of several uses the categorical order blue, orange, aqua, and
every line carries a direct label as well as a legend entry.
"""

from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

import validation_scenarios as v
from validation_figures import EXPORT, LAYOUT, SIM, SIM_2, _line, _save
from ins_sim.config import default_trajectory_path
from ins_sim.core.earth_model import WGS84_A, WGS84_F, normal_gravity
from ins_sim.trajectory.kinematics import build_trajectory

AQUA = "#1baf7a"         # third categorical series
GUIDE = "#8f8d87"        # reference lines and their labels

# WGS-84 constants for the full normal-gravity formula (NGA 2014)
GM = 3.986004418e14                  # Earth's gravitational constant [m³/s²]
OMEGA_WGS84 = 7.292115e-5            # WGS-84 defining rotation rate [rad/s]


def full_normal_gravity(L_b: np.ndarray, h_b: np.ndarray) -> np.ndarray:
    """WGS-84 normal gravity above the ellipsoid, to second order in height [m/s²]."""
    b = WGS84_A * (1.0 - WGS84_F)
    m = OMEGA_WGS84**2 * WGS84_A**2 * b / GM
    sin2_L = np.sin(L_b) ** 2
    g_0 = normal_gravity(L_b, 0.0)
    return g_0 * (1.0 - 2.0 / WGS84_A * (1.0 + WGS84_F + m - 2.0 * WGS84_F * sin2_L) * h_b
                  + 3.0 / WGS84_A**2 * h_b**2)


def truth_imu_figure(out_dir: Path) -> None:
    """The ideal gyro and accelerometer outputs along the packaged mission."""
    truth, _, _ = build_trajectory(str(default_trajectory_path()), tau_i=0.1)
    step = 10                                       # plot one point per second
    t_min = truth.t[::step] / 60.0
    gyro = np.rad2deg(truth.omega_ib_b[::step])     # °/s
    accel = truth.f_ib_b[::step]                    # m/s²
    axes = ["x (forward)", "y (right)", "z (down)"]
    titles = [title for axis in axes for title in (f"Gyro, {axis}", f"Accelerometer, {axis}")]
    fig = make_subplots(rows=3, cols=2, shared_xaxes=True, subplot_titles=titles,
                        vertical_spacing=0.09, horizontal_spacing=0.08)
    for row in range(3):
        fig.add_trace(_line(t_min, gyro[:, row], "Gyro", SIM, width=1.5), row=row + 1, col=1)
        fig.add_trace(_line(t_min, accel[:, row], "Accelerometer", SIM, width=1.5),
                      row=row + 1, col=2)
        fig.update_yaxes(title_text="°/s", row=row + 1, col=1)
        fig.update_yaxes(title_text="m/s²", row=row + 1, col=2)
    fig.update_xaxes(title_text="Time (min)", row=3)
    fig.update_layout(title="What a perfect IMU measures along the packaged mission",
                      showlegend=False, **{**LAYOUT, "margin": dict(l=80, r=30, t=110, b=60)})
    fig.write_image(str(out_dir / "truth_imu.png"), **{**EXPORT, "height": 900})
    print(f"wrote {out_dir.name}/truth_imu.png", flush=True)


def gravity_model_figure(out_dir: Path) -> None:
    """How far the linear free-air model departs from the full WGS-84 formula."""
    h_km = np.linspace(0.0, 15.0, 301)
    micro_g = 9.80665e-6
    fig = go.Figure()
    for lat_deg, color in ((0, SIM), (45, SIM_2), (90, AQUA)):
        L_b = np.deg2rad(lat_deg)
        error = (normal_gravity(L_b, h_km * 1e3) - full_normal_gravity(L_b, h_km * 1e3)) / micro_g
        name = f"{lat_deg}° latitude"
        fig.add_trace(_line(h_km, error, name, color))
        fig.add_annotation(x=h_km[-1], y=error[-1], text=name, showarrow=False,
                           xanchor="left", xshift=8, font=dict(color="#52514e"))
    cruise_km = 15000 * 0.3048 / 1e3
    fig.add_vline(x=cruise_km, line=dict(color=GUIDE, dash="dot", width=1.5),
                  annotation_text="Packaged mission cruise, 15,000 ft",
                  annotation_position="top right")
    fig.add_hline(y=0.0, line=dict(color=GUIDE, width=1))
    fig.update_layout(title="Gravity error of the linear free-air model, against the full WGS-84 formula",
                      xaxis_title="Height above the ellipsoid (km)",
                      yaxis_title="Model minus WGS-84 formula (µg)",
                      **{**LAYOUT, "margin": dict(l=80, r=150, t=90, b=60)})
    _save(fig, out_dir, "gravity_model.png")


def round_trip_figure(out_dir: Path) -> None:
    """Navigator minus truth when the navigator is fed the truth's own IMU outputs."""
    rt = v.zero_noise_round_trip()
    keep = rt.error > 0.0                           # a log axis cannot show exact zeros
    fig = go.Figure([_line(rt.t[keep] / 60.0, rt.error[keep], "Position difference", SIM, width=1.5)])
    fig.update_yaxes(type="log", exponentformat="power", dtick=1)
    fig.update_layout(title="Zero-noise round trip on the packaged mission (τ<sub>i</sub> = 1 s)",
                      xaxis_title="Time (min)",
                      yaxis_title="Navigator minus truth (m)", showlegend=False, **LAYOUT)
    _save(fig, out_dir, "round_trip.png")


def build_all(out_dir: Path) -> None:
    """Draws every Theory-page figure into out_dir."""
    out_dir.mkdir(parents=True, exist_ok=True)
    truth_imu_figure(out_dir)
    gravity_model_figure(out_dir)
    round_trip_figure(out_dir)
