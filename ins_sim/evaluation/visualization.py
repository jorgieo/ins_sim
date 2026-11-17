"""Matplotlib summary figure for the command-line tool (`main.py`).

The GUI builds its interactive plotly figures in `ins_sim.gui.figures`;
this module draws the static 2 × 3 summary the command-line run shows.
"""

from typing import TYPE_CHECKING

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

from ins_sim.evaluation.errors import position_error

if TYPE_CHECKING:
    from mpl_toolkits.mplot3d.axes3d import Axes3D

    from ins_sim.evaluation.monte_carlo import MonteCarloResult
    from ins_sim.trajectory.kinematics import TruthTrajectory

NM = 1852.0


def plot_error_tube(ax: "Axes3D", r_lb_l: np.ndarray, r95: np.ndarray, color: str = "crimson",
                    alpha: float = 0.20, n_circle: int = 24) -> None:
    """Draws the swept-volume tube of the radial error envelope around the truth track.

    Args:
        ax: Matplotlib 3D axes to draw on.
        r_lb_l: Truth position relative to the start point (or a
            consistently scaled copy), shape (M, 3).
        r95: Radial error envelope at each time step, shape (M,), in
            the same units as r_lb_l.
        color: Surface color. Defaults to "crimson".
        alpha: Surface opacity. Defaults to 0.20.
        n_circle: Number of points approximating the tube's circular
            cross-section. Defaults to 24.

    Returns:
        None: Draws directly onto `ax`.
    """
    T = np.gradient(r_lb_l, axis=0)
    T = T / np.linalg.norm(T, axis=1, keepdims=True)

    seed = np.array([0.0, 0.0, 1.0])
    if abs(T[0] @ seed) > 0.95:
        seed = np.array([0.0, 1.0, 0.0])

    N1 = np.zeros_like(T); N2 = np.zeros_like(T)
    n  = seed - T[0] * (T[0] @ seed)
    N1[0] = n / np.linalg.norm(n)
    N2[0] = np.cross(T[0], N1[0])
    for k in range(1, len(T)):
        proj = N1[k-1] - T[k] * (T[k] @ N1[k-1])
        nn   = np.linalg.norm(proj)
        N1[k] = proj / nn if nn > 1e-10 else N1[k-1]
        N2[k] = np.cross(T[k], N1[k])

    th = np.linspace(0.0, 2*np.pi, n_circle)
    c, s = np.cos(th), np.sin(th)
    tube = (r_lb_l[:, None, :]
            + r95[:, None, None] * (c[None, :, None] * N1[:, None, :]
                                  + s[None, :, None] * N2[:, None, :]))

    X = tube[..., 1]      # East
    Y = tube[..., 0]      # North
    Z = -tube[..., 2]     # Up
    ax.plot_surface(X, Y, Z, color=color, alpha=alpha,
                    edgecolor='none', linewidth=0)


def build_summary_figure(truth: "TruthTrajectory", mc: "MonteCarloResult",
                         r95: np.ndarray) -> plt.Figure:
    """Builds the 2x3 Monte Carlo summary figure.

    Figure panels: 3D ground track with swept 95th-pct error tube,
    3D position-error envelope, CEP, and the three components of the
    attitude error delta_psi_nb^n. Does not call plt.show() -- the
    caller decides.

    Note:
        Applies sns.set_theme(...) as a side effect (global rcParams
        mutation), matching the original script's behavior.

    Args:
        truth: Truth trajectory exposing r_lb_l, t, L_b, lambda_b and h_b.
        mc: MonteCarloResult from run_monte_carlo.
        r95: 95th-percentile 3D position-error envelope, shape (M,) [m].

    Returns:
        matplotlib.figure.Figure: The assembled summary figure.
    """
    sns.set_theme(
        style='whitegrid',
        palette='deep',
        font_scale=1.05,
        rc={
            'figure.facecolor': '#f5f5f5',
            'axes.facecolor':   'white',
            'grid.color':       '#dddddd',
            'axes.edgecolor':   '#aaaaaa',
            'text.color':       '#111111',
            'axes.labelcolor':  '#111111',
            'xtick.color':      '#333333',
            'ytick.color':      '#333333',
        },
    )

    n_trials = len(mc.est_r_lb_l)
    P   = truth.r_lb_l
    t_m = truth.t / 60.0   # minutes for x-axis
    delta_r_eb_n = position_error(mc.est_L_b, mc.est_lambda_b, mc.est_h_b,
                                  truth.L_b, truth.lambda_b, truth.h_b)

    step = max(1, len(P) // 4000)
    fig  = plt.figure(figsize=(22, 12), layout='constrained')
    gs   = fig.add_gridspec(2, 3)

    ax  = fig.add_subplot(gs[0, 0], projection='3d')
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[0, 2])
    ax4 = fig.add_subplot(gs[1, 0])   # North tilt
    ax5 = fig.add_subplot(gs[1, 1])   # East tilt
    ax6 = fig.add_subplot(gs[1, 2])   # Heading

    # ---- 3-D ground track + 95th-pct error tube ---------------------------
    # Isotropic km units (not the nm/kft mix used elsewhere) so the tube's
    # circular cross-section and tangent direction stay geometrically
    # correct: plot_error_tube offsets by a scalar radius (r95, in meters)
    # along normals derived from the track's own tangent, so all three axes
    # must share one linear scale factor.
    P_km = P / 1000.0
    r95_km = r95 / 1000.0
    ax.plot(P_km[::step, 1], P_km[::step, 0], -P_km[::step, 2],
            'k-', lw=1.5, label='Truth')
    for i in range(min(6, n_trials)):
        Q_km = mc.est_r_lb_l[i] / 1000.0
        ax.plot(Q_km[::step, 1], Q_km[::step, 0], -Q_km[::step, 2],
                '-', color='steelblue', alpha=0.35, lw=0.5)
    plot_error_tube(ax, P_km, r95_km)
    ax.set_xlabel('East (km)')
    ax.set_ylabel('North (km)')
    ax.set_zlabel('Up (km)')
    ax.set_title(f'{n_trials}-trial Monte Carlo')
    ax.legend(loc='upper left')

    # ---- 3-D position error vs time ---------------------------------------
    err_all = np.linalg.norm(delta_r_eb_n, axis=2)
    ax2.plot(t_m, err_all.T, color='steelblue', alpha=0.20, lw=0.5)
    ax2.plot(t_m, r95, color='crimson', lw=2.0, label='95th pct')
    ax2.fill_between(t_m, 0, r95, color='crimson', alpha=0.15)
    ax2.set_xlabel('Time (min)')
    ax2.set_ylabel('3-D position error (m)')
    ax2.set_title('Position error envelope')
    ax2.legend()

    # ---- CEP vs time + linear fit -----------------------------------------
    horiz_err = np.linalg.norm(delta_r_eb_n[:, :, :2], axis=2)
    cep = np.percentile(horiz_err, 50, axis=0)

    mask60 = t_m <= 60.0
    coeffs = np.polyfit(t_m[mask60], cep[mask60], 1)
    cep_fit = np.polyval(coeffs, t_m)

    ax3.plot(t_m, horiz_err.T, color='steelblue', alpha=0.15, lw=0.5)
    ax3.plot(t_m, cep, color='navy', lw=2.0, label='CEP (50th pct)')
    ax3.fill_between(t_m, 0, cep, color='steelblue', alpha=0.18)
    ax3.plot(t_m, cep_fit, '--', color='darkorange', lw=1.8,
             label=f'Linear fit  {coeffs[0]*0.032397408:.2f} NM/hr')
    ax3.axvline(60.0, color='gray', lw=0.8, linestyle=':')
    ax3.set_xlabel('Time (min)')
    ax3.set_ylabel('Horizontal error (m)')
    ax3.set_title('CEP along path')
    ax3.legend()

    # ---- Attitude error delta_psi_nb^n ------------------------------------
    delta_psi_deg = np.rad2deg(mc.delta_psi_nb_n)
    p95_abs = np.percentile(np.abs(delta_psi_deg), 95, axis=0)

    for ax_att, col, title in [
        (ax4, 0, 'North tilt error'),
        (ax5, 1, 'East tilt error'),
        (ax6, 2, 'Heading error'),
    ]:
        ax_att.plot(t_m, delta_psi_deg[:, :, col].T, color='steelblue',
                    alpha=0.20, lw=0.5)
        ax_att.fill_between(t_m, -p95_abs[:, col], p95_abs[:, col],
                            color='crimson', alpha=0.20,
                            label='95th pct |error|')
        ax_att.axhline(0.0, color='gray', lw=0.8)
        ax_att.set_xlabel('Time (min)')
        ax_att.set_ylabel(f'{title} (deg)')
        ax_att.set_title(title)
        ax_att.legend(fontsize=8)

    return fig
