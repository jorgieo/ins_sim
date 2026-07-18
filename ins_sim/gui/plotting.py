"""Figure builders for the GUI's embedded matplotlib canvases.

One builder per visualization checkbox (see VISUALIZATION_CHOICES in
widgets.py), each taking a SimulationResult and returning a
matplotlib.figure.Figure. Uses matplotlib.figure.Figure directly (not
pyplot) so no global figure manager state leaks into the Qt embedding.

Chart language matches evaluation/visualization.py: navy = truth,
steelblue = individual trials, crimson = error envelopes.
"""

import numpy as np
from matplotlib.figure import Figure

NM = 1852.0

GRID_KW = dict(color="#dddddd", linewidth=0.8)
MAX_PLOT_POINTS = 4000


def _step(n: int) -> int:
    return max(1, n // MAX_PLOT_POINTS)


def _style_axes(ax):
    ax.grid(True, **GRID_KW)
    ax.set_axisbelow(True)


def build_trajectory3d_figure(result) -> Figure:
    """3D ground track: truth, sample trials, and the 95th-pct error tube."""
    from ins_sim.evaluation.visualization import plot_error_tube

    truth = result.truth
    P = truth.pos_n
    s = _step(len(P))

    fig = Figure(layout="constrained")
    ax = fig.add_subplot(projection="3d")

    # Isotropic km units so the tube cross-section stays geometrically
    # correct (see build_summary_figure); downsampled for canvas speed.
    P_km = P[::s] / 1000.0
    r95_km = result.r95[::s] / 1000.0
    ax.plot(P_km[:, 1], P_km[:, 0], -P_km[:, 2], "k-", lw=1.5, label="Truth")
    for i in range(min(6, result.n_trials)):
        Q_km = result.pos_runs[i, ::s] / 1000.0
        ax.plot(Q_km[:, 1], Q_km[:, 0], -Q_km[:, 2],
                "-", color="steelblue", alpha=0.35, lw=0.5)
    plot_error_tube(ax, P_km, r95_km)
    ax.set_xlabel("East (km)")
    ax.set_ylabel("North (km)")
    ax.set_zlabel("Up (km)")
    ax.set_title(f"{result.n_trials}-trial Monte Carlo")
    ax.legend(loc="upper left")
    return fig


def build_attitude_figure(result) -> Figure:
    """Pitch/roll/heading truth with 95th-pct attitude error envelopes."""
    truth = result.truth
    t_m = truth.t / 60.0

    euler_err = result.euler_runs - truth.euler[None, :, :]
    euler_err[:, :, 2] = (euler_err[:, :, 2] + np.pi) % (2 * np.pi) - np.pi
    p95_euler = np.rad2deg(np.percentile(np.abs(euler_err), 95, axis=0))

    fig = Figure(layout="constrained")
    axes = fig.subplots(3, 1, sharex=True)
    for ax, col, title in [(axes[0], 1, "Pitch"),
                           (axes[1], 0, "Roll"),
                           (axes[2], 2, "Heading")]:
        truth_deg = np.rad2deg(truth.euler[:, col])
        err95 = p95_euler[:, col]
        ax.fill_between(t_m, truth_deg - err95, truth_deg + err95,
                        color="crimson", alpha=0.25, label="95th pct envelope")
        ax.plot(t_m, truth_deg, color="navy", lw=1.2, label="Truth")
        ax.set_ylabel(f"{title} (deg)")
        ax.set_title(title)
        ax.legend(fontsize=8)
        _style_axes(ax)
    axes[-1].set_xlabel("Time (min)")
    return fig


def build_velocity_figure(result) -> Figure:
    """Per-axis 95th-pct NED velocity error envelopes vs time."""
    truth = result.truth
    t_m = truth.t / 60.0

    vel_err = np.abs(result.vel_runs - truth.vel_n[None, :, :])
    p95_vel = np.percentile(vel_err, 95, axis=0)

    fig = Figure(layout="constrained")
    axes = fig.subplots(3, 1, sharex=True)
    for ax, col, title in [(axes[0], 0, "North"),
                           (axes[1], 1, "East"),
                           (axes[2], 2, "Down")]:
        ax.plot(t_m, p95_vel[:, col], color="crimson", lw=1.5,
                label="95th pct")
        ax.fill_between(t_m, 0, p95_vel[:, col], color="crimson", alpha=0.15)
        ax.set_ylabel(f"{title} (m/s)")
        ax.set_title(f"{title} velocity error")
        ax.legend(fontsize=8)
        _style_axes(ax)
    axes[-1].set_xlabel("Time (min)")
    return fig


def build_position_figure(result) -> Figure:
    """Radial 3D error envelope and horizontal CEP vs time."""
    truth = result.truth
    P = truth.pos_n
    t_m = truth.t / 60.0

    fig = Figure(layout="constrained")
    ax_r, ax_cep = fig.subplots(2, 1, sharex=True)

    err_all = np.linalg.norm(result.pos_runs - P[None, :, :], axis=2)
    ax_r.plot(t_m, err_all.T, color="steelblue", alpha=0.20, lw=0.5)
    ax_r.plot(t_m, result.r95, color="crimson", lw=2.0, label="95th pct")
    ax_r.fill_between(t_m, 0, result.r95, color="crimson", alpha=0.15)
    ax_r.set_ylabel("3-D position error (m)")
    ax_r.set_title("Radial error envelope")
    ax_r.legend()
    _style_axes(ax_r)

    horiz_err = np.linalg.norm(result.pos_runs[:, :, :2] - P[None, :, :2], axis=2)
    cep = np.percentile(horiz_err, 50, axis=0)
    ax_cep.plot(t_m, horiz_err.T, color="steelblue", alpha=0.15, lw=0.5)
    ax_cep.plot(t_m, cep, color="navy", lw=2.0, label="CEP (50th pct)")
    ax_cep.fill_between(t_m, 0, cep, color="steelblue", alpha=0.18)

    # Linear drift fit over the first hour, as in build_summary_figure
    mask60 = t_m <= 60.0
    if mask60.sum() >= 2:
        coeffs = np.polyfit(t_m[mask60], cep[mask60], 1)
        ax_cep.plot(t_m, np.polyval(coeffs, t_m), "--", color="darkorange",
                    lw=1.8, label=f"Linear fit  {coeffs[0]*0.032397408:.2f} NM/hr")
        ax_cep.axvline(60.0, color="gray", lw=0.8, linestyle=":")
    ax_cep.set_xlabel("Time (min)")
    ax_cep.set_ylabel("Horizontal error (m)")
    ax_cep.set_title("CEP along path")
    ax_cep.legend()
    _style_axes(ax_cep)
    return fig


#: slug (see VISUALIZATION_CHOICES) -> (tab title, figure builder)
PLOT_BUILDERS = {
    "trajectory_3d":   ("3D Trajectory", build_trajectory3d_figure),
    "attitude_errors": ("Attitude Errors", build_attitude_figure),
    "velocity_errors": ("Velocity Errors", build_velocity_figure),
    "position_errors": ("Position Errors", build_position_figure),
}
