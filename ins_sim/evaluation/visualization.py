import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

NM = 1852.0


def plot_error_tube(ax, truth_pos, r95, color="crimson",
                    alpha=0.20, n_circle=24):
    """Draws the swept-volume tube of the radial error envelope around the truth track.

    Args:
        ax: Matplotlib 3D axes to draw on.
        truth_pos: Truth NED (or consistently scaled) position, shape
            (M, 3).
        r95: Radial error envelope at each time step, shape (M,), in
            the same units as truth_pos.
        color: Surface color. Defaults to "crimson".
        alpha: Surface opacity. Defaults to 0.20.
        n_circle: Number of points approximating the tube's circular
            cross-section. Defaults to 24.

    Returns:
        None: Draws directly onto `ax`.
    """
    T = np.gradient(truth_pos, axis=0)
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
    tube = (truth_pos[:, None, :]
            + r95[:, None, None] * (c[None, :, None] * N1[:, None, :]
                                  + s[None, :, None] * N2[:, None, :]))

    X = tube[..., 1]      # East
    Y = tube[..., 0]      # North
    Z = -tube[..., 2]     # Up
    ax.plot_surface(X, Y, Z, color=color, alpha=alpha,
                    edgecolor='none', linewidth=0)


def build_summary_figure(truth, pos_runs, euler_runs, r95, n_trials):
    """Builds the 2x3 Monte Carlo summary figure.

    Figure panels: 3D ground track with swept 95th-pct error tube,
    radial error envelope, CEP plot, and pitch/roll/heading attitude
    envelopes. Does not call plt.show() -- the caller decides.

    Note:
        Applies sns.set_theme(...) as a side effect (global rcParams
        mutation), matching the original script's behavior.

    Args:
        truth: Truth trajectory object exposing pos_n, t, and euler
            ([φ, θ, ψ], shape (M, 3) [rad]).
        pos_runs: NED position per Monte Carlo trial, shape
            (n_trials, M, 3) [m].
        euler_runs: Euler angles [φ, θ, ψ] per trial, shape
            (n_trials, M, 3) [rad].
        r95: 95th-percentile radial position error envelope, shape
            (M,) [m].
        n_trials: Number of Monte Carlo trials represented in
            pos_runs.

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

    P   = truth.pos_n
    t_m = truth.t / 60.0   # minutes for x-axis

    step = max(1, len(P) // 4000)
    fig  = plt.figure(figsize=(22, 12), layout='constrained')
    gs   = fig.add_gridspec(2, 3)

    ax  = fig.add_subplot(gs[0, 0], projection='3d')
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[0, 2])
    ax4 = fig.add_subplot(gs[1, 0])   # Pitch
    ax5 = fig.add_subplot(gs[1, 1])   # Roll
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
        Q_km = pos_runs[i] / 1000.0
        ax.plot(Q_km[::step, 1], Q_km[::step, 0], -Q_km[::step, 2],
                '-', color='steelblue', alpha=0.35, lw=0.5)
    plot_error_tube(ax, P_km, r95_km)
    ax.set_xlabel('East (km)')
    ax.set_ylabel('North (km)')
    ax.set_zlabel('Up (km)')
    ax.set_title(f'{n_trials}-trial Monte Carlo')
    ax.legend(loc='upper left')

    # ---- 3-D radial error vs time -----------------------------------------
    err_all = np.linalg.norm(pos_runs - P[None, :, :], axis=2)
    ax2.plot(t_m, err_all.T, color='steelblue', alpha=0.20, lw=0.5)
    ax2.plot(t_m, r95, color='crimson', lw=2.0, label='95th pct')
    ax2.fill_between(t_m, 0, r95, color='crimson', alpha=0.15)
    ax2.set_xlabel('Time (min)')
    ax2.set_ylabel('3-D position error (m)')
    ax2.set_title('Radial error envelope')
    ax2.legend()

    # ---- CEP vs time + linear fit -----------------------------------------
    horiz_err = np.linalg.norm(pos_runs[:, :, :2] - P[None, :, :2], axis=2)
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

    # ---- Attitude error envelopes -----------------------------------------
    euler_err = euler_runs - truth.euler[None, :, :]
    euler_err[:, :, 2] = (euler_err[:, :, 2] + np.pi) % (2 * np.pi) - np.pi
    p95_euler = np.rad2deg(np.percentile(np.abs(euler_err), 95, axis=0))

    for ax_att, col, title, ylabel in [
        (ax4, 1, 'Pitch',   'Pitch (deg)'),
        (ax5, 0, 'Roll',    'Roll (deg)'),
        (ax6, 2, 'Heading', 'Heading (deg)'),
    ]:
        truth_deg = np.rad2deg(truth.euler[:, col])
        err95     = p95_euler[:, col]
        ax_att.fill_between(t_m, truth_deg - err95, truth_deg + err95,
                            color='crimson', alpha=0.25, label='95th pct envelope')
        ax_att.plot(t_m, truth_deg, color='navy', lw=1.5, label='Truth')
        ax_att.set_xlabel('Time (min)')
        ax_att.set_ylabel(ylabel)
        ax_att.set_title(title)
        ax_att.legend(fontsize=8)

    return fig
