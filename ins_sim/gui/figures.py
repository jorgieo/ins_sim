"""Plotly figure builders for the simulator's visualization tabs.

Qt-free by design: every builder takes a SimulationResult-shaped object
(truth, pos_runs, euler_runs, lat_runs, lon_runs, vel_runs, r95,
n_trials) and returns a plotly.graph_objects.Figure, so the module is
unit-testable headlessly and reusable from the CLI (main.py's saved map).

Chart language matches the rest of the project: navy = truth/mean,
steelblue = individual trials, crimson = uncertainty bounds, darkorange
= fits.

All Monte Carlo trials are always drawn. To keep figures responsive at
high trial counts, the N trial polylines of a figure are emitted as ONE
trace with NaN separators (single legend entry, no per-trial trace
overhead), and time is downsampled adaptively so a figure's trial
payload stays under ~MAX_TRIAL_POINTS points.
"""

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

NM = 1852.0

NAVY = "navy"
STEELBLUE_FAINT = "rgba(70, 130, 180, 0.30)"
CRIMSON_BAND = "rgba(220, 20, 60, 0.25)"
ORANGE = "darkorange"

#: Max points across all trials in one figure's trial-bundle trace.
MAX_TRIAL_POINTS = 500_000
#: Max points for single-line traces (truth tracks, envelopes).
MAX_LINE_POINTS = 4000
#: Rings along the 3D error tube's length.
MAX_TUBE_RINGS = 600

LAYOUT_KW = dict(template="plotly_white", margin=dict(l=60, r=20, t=50, b=50))


def _line_step(M: int) -> int:
    return max(1, M // MAX_LINE_POINTS)


def _trial_step(n_trials: int, M: int) -> int:
    return max(1, (n_trials * M) // MAX_TRIAL_POINTS)


def _nan_join(rows):
    """Concatenates the rows of a 2D array with NaN separators.

    Plotly draws NaN as a line break, so N polylines can ship as one
    trace (single legend entry, no per-trial trace overhead).

    Args:
        rows: Per-trial values, shape (N, K).

    Returns:
        numpy.ndarray: Flat array of shape (N*(K+1),).
    """
    N, K = rows.shape
    return np.concatenate(
        [np.append(rows[i], np.nan) for i in range(N)])


def _bundle(x, ys):
    """NaN-separated (x, y) arrays for N polylines sharing one x axis.

    Args:
        x: Shared x values, shape (K,).
        ys: Per-trial y values, shape (N, K).

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]: (xb, yb), each shape
            (N*(K+1),).
    """
    return np.tile(np.append(x, np.nan), len(ys)), _nan_join(ys)


def horizontal_error_nm(result):
    """Per-run horizontal position error in NM, shape (n_trials, M)."""
    return np.linalg.norm(
        result.pos_runs[:, :, :2] - result.truth.pos_n[None, :, :2],
        axis=2) / NM


# =========================================================================
# CEP
# =========================================================================

def figure_cep(result) -> go.Figure:
    """Every run's horizontal error [NM] vs decimal hours, + CEP drift fit."""
    truth = result.truth
    t_hr = truth.t / 3600.0
    horiz_err_nm = horizontal_error_nm(result)

    fig = go.Figure()

    s = _trial_step(result.n_trials, len(t_hr))
    xb, yb = _bundle(t_hr[::s], horiz_err_nm[:, ::s])
    fig.add_trace(go.Scatter(
        x=xb, y=yb, mode="lines",
        line=dict(color=STEELBLUE_FAINT, width=1),
        name="Individual runs",
        hovertemplate="t = %{x:.2f} hr<br>error = %{y:.2f} NM<extra></extra>",
    ))

    # Drift-rate fit to the ensemble CEP (pointwise 50th percentile across
    # trials) over the first hour; the CEP itself is tabulated separately.
    cep = np.percentile(horiz_err_nm, 50, axis=0)
    mask = t_hr <= 1.0
    if mask.sum() >= 2:
        coeffs = np.polyfit(t_hr[mask], cep[mask], 1)
        x_fit = np.array([t_hr[0], t_hr[-1]])
        fig.add_trace(go.Scatter(
            x=x_fit, y=np.polyval(coeffs, x_fit), mode="lines",
            line=dict(color=ORANGE, width=2, dash="dash"),
            name=f"CEP linear fit  {coeffs[0]:.2f} NM/hr",
            hoverinfo="skip",
        ))
        fig.add_vline(x=1.0, line_color="gray", line_width=1, line_dash="dot")

    fig.update_layout(
        title=f"Horizontal position error — {result.n_trials} runs",
        xaxis_title="Time (hr)", yaxis_title="Horizontal error (NM)",
        **LAYOUT_KW)
    return fig


# =========================================================================
# ±3σ band figures (attitude / velocity / position)
# =========================================================================

def _bands_figure(t_min, err_runs, panels, unit: str) -> go.Figure:
    """Three stacked panels of mean error with ±3σ bands.

    Args:
        t_min: Time axis [min], shape (M,).
        err_runs: Per-trial error, shape (n_trials, M, 3).
        panels: list of (column index, panel title) per subplot row.
        unit: Y-axis unit label.

    Returns:
        plotly.graph_objects.Figure: The assembled subplot figure.
    """
    mean = err_runs.mean(axis=0)
    sigma = err_runs.std(axis=0)
    s = _line_step(len(t_min))
    t = t_min[::s]

    fig = make_subplots(rows=len(panels), cols=1, shared_xaxes=True,
                        subplot_titles=[title for _, title in panels],
                        vertical_spacing=0.08)
    for row, (col, title) in enumerate(panels, start=1):
        lo = (mean[:, col] - 3 * sigma[:, col])[::s]
        hi = (mean[:, col] + 3 * sigma[:, col])[::s]
        fig.add_trace(go.Scatter(
            x=t, y=hi, mode="lines", line=dict(width=0),
            hoverinfo="skip", showlegend=False), row=row, col=1)
        fig.add_trace(go.Scatter(
            x=t, y=lo, mode="lines", line=dict(width=0),
            fill="tonexty", fillcolor=CRIMSON_BAND,
            name="±3σ", legendgroup="band", showlegend=(row == 1),
            hoverinfo="skip"), row=row, col=1)
        fig.add_trace(go.Scatter(
            x=t, y=mean[::s, col], mode="lines",
            line=dict(color=NAVY, width=1.5),
            name="Mean error", legendgroup="mean", showlegend=(row == 1),
            hovertemplate=("t = %{x:.1f} min<br>"
                           f"{title} = %{{y:.3g}} {unit}<extra></extra>"),
        ), row=row, col=1)
        fig.add_hline(y=0.0, line_color="gray", line_width=1,
                      line_dash="dot", row=row, col=1)
        fig.update_yaxes(title_text=f"{title} ({unit})", row=row, col=1)
    fig.update_xaxes(title_text="Time (min)", row=len(panels), col=1)
    fig.update_layout(**LAYOUT_KW)
    return fig


def figure_attitude(result) -> go.Figure:
    """Pitch/roll/heading Euler-angle errors with ±3σ bands [deg]."""
    truth = result.truth
    euler_err = result.euler_runs - truth.euler[None, :, :]
    euler_err[:, :, 2] = (euler_err[:, :, 2] + np.pi) % (2 * np.pi) - np.pi
    return _bands_figure(
        truth.t / 60.0, np.rad2deg(euler_err),
        [(1, "Pitch error"), (0, "Roll error"), (2, "Heading error")],
        "deg")


def figure_velocity(result) -> go.Figure:
    """NED velocity errors with ±3σ bands [m/s]."""
    truth = result.truth
    return _bands_figure(
        truth.t / 60.0, result.vel_runs - truth.vel_n[None, :, :],
        [(0, "North velocity error"), (1, "East velocity error"),
         (2, "Down velocity error")],
        "m/s")


def figure_position(result) -> go.Figure:
    """NED position errors with ±3σ bands [m]."""
    truth = result.truth
    return _bands_figure(
        truth.t / 60.0, result.pos_runs - truth.pos_n[None, :, :],
        [(0, "North position error"), (1, "East position error"),
         (2, "Down position error")],
        "m")


# =========================================================================
# 3D trajectory
# =========================================================================

def _tube_mesh(truth_pos, r95, n_circle=24) -> go.Mesh3d:
    """Builds the swept 95th-pct radial error tube as a crimson mesh.

    Ports plot_error_tube's parallel-transport frame: unit tangents along
    the track, a normal pair (N1, N2) propagated ring to ring, and a
    circle of radius r95 swept around each ring.

    Args:
        truth_pos: Truth NED position (consistently scaled), shape (M, 3).
        r95: Radial envelope per sample, shape (M,), same units.
        n_circle: Points per circular cross-section. Defaults to 24.

    Returns:
        plotly.graph_objects.Mesh3d: Tube mesh in (East, North, Up) axes.
    """
    T = np.gradient(truth_pos, axis=0)
    T = T / np.linalg.norm(T, axis=1, keepdims=True)

    seed = np.array([0.0, 0.0, 1.0])
    if abs(T[0] @ seed) > 0.95:
        seed = np.array([0.0, 1.0, 0.0])

    N1 = np.zeros_like(T)
    N2 = np.zeros_like(T)
    n = seed - T[0] * (T[0] @ seed)
    N1[0] = n / np.linalg.norm(n)
    N2[0] = np.cross(T[0], N1[0])
    for k in range(1, len(T)):
        proj = N1[k-1] - T[k] * (T[k] @ N1[k-1])
        nn = np.linalg.norm(proj)
        N1[k] = proj / nn if nn > 1e-10 else N1[k-1]
        N2[k] = np.cross(T[k], N1[k])

    th = np.linspace(0.0, 2 * np.pi, n_circle, endpoint=False)
    c, s = np.cos(th), np.sin(th)
    tube = (truth_pos[:, None, :]
            + r95[:, None, None] * (c[None, :, None] * N1[:, None, :]
                                    + s[None, :, None] * N2[:, None, :]))
    M = len(truth_pos)
    verts = tube.reshape(M * n_circle, 3)

    # Two triangles per quad between ring k and k+1, wrapping the circle.
    k_idx = np.repeat(np.arange(M - 1), n_circle)
    j_idx = np.tile(np.arange(n_circle), M - 1)
    j_next = (j_idx + 1) % n_circle
    a = k_idx * n_circle + j_idx
    b = k_idx * n_circle + j_next
    cc = (k_idx + 1) * n_circle + j_idx
    d = (k_idx + 1) * n_circle + j_next
    tri_i = np.concatenate([a, b])
    tri_j = np.concatenate([b, d])
    tri_k = np.concatenate([cc, cc])

    return go.Mesh3d(
        x=verts[:, 1], y=verts[:, 0], z=-verts[:, 2],   # East, North, Up
        i=tri_i, j=tri_j, k=tri_k,
        color="crimson", opacity=0.25, name="95th-pct error tube",
        showlegend=True, hoverinfo="skip")


def figure_trajectory_3d(result) -> go.Figure:
    """3D track: truth vs all INS trials, with the 95th-pct error tube."""
    truth = result.truth
    P_km = truth.pos_n / 1000.0
    M = len(P_km)

    fig = go.Figure()

    s_t = _trial_step(result.n_trials, M)
    runs_km = result.pos_runs[:, ::s_t] / 1000.0
    fig.add_trace(go.Scatter3d(
        x=_nan_join(runs_km[:, :, 1]), y=_nan_join(runs_km[:, :, 0]),
        z=_nan_join(-runs_km[:, :, 2]), mode="lines",
        line=dict(color=STEELBLUE_FAINT, width=1.5),
        name="INS estimates", hoverinfo="skip"))

    s = _line_step(M)
    fig.add_trace(go.Scatter3d(
        x=P_km[::s, 1], y=P_km[::s, 0], z=-P_km[::s, 2], mode="lines",
        line=dict(color="black", width=3), name="Truth",
        hovertemplate=("E = %{x:.1f} km<br>N = %{y:.1f} km<br>"
                       "Up = %{z:.2f} km<extra></extra>")))

    s_tube = max(1, M // MAX_TUBE_RINGS)
    fig.add_trace(_tube_mesh(P_km[::s_tube], result.r95[::s_tube] / 1000.0))

    fig.update_layout(
        title=f"{result.n_trials}-trial Monte Carlo",
        scene=dict(xaxis_title="East (km)", yaxis_title="North (km)",
                   zaxis_title="Up (km)"),
        legend=dict(x=0.01, y=0.99),
        template="plotly_white", margin=dict(l=0, r=0, t=50, b=0))
    return fig


# =========================================================================
# Ground-track map
# =========================================================================

def figure_map(result) -> go.Figure:
    """Interactive ground-track map: all trials, 95th-pct envelope, truth.

    MapLibre-based Scattermap on OpenStreetMap raster tiles (needs
    network for the basemap; overlays render regardless).
    """
    truth = result.truth
    P = truth.pos_n
    M = len(P)
    lat_deg = np.rad2deg(truth.lat)
    lon_deg = np.rad2deg(truth.lon)
    t_min = truth.t / 60.0

    horiz_err = np.linalg.norm(
        result.pos_runs[:, :, :2] - P[None, :, :2], axis=2)
    p95_horiz = np.percentile(horiz_err, 95, axis=0)

    # Perpendicular envelope in geographic space: track gradient ->
    # unit normal in metres -> offset by the 95th-pct horizontal error
    # -> back to degrees.
    dlat_m = np.gradient(lat_deg) * 111320.0
    dlon_m = np.gradient(lon_deg) * 111320.0 * np.cos(np.deg2rad(lat_deg))
    seg_len = np.hypot(dlat_m, dlon_m) + 1e-6
    perp_lat = -dlon_m / seg_len
    perp_lon = dlat_m / seg_len

    dlat = p95_horiz * perp_lat / 111320.0
    dlon = p95_horiz * perp_lon / (111320.0 * np.cos(np.deg2rad(lat_deg)))

    fig = go.Figure()

    s_t = _trial_step(result.n_trials, M)
    fig.add_trace(go.Scattermap(
        lat=_nan_join(np.rad2deg(result.lat_runs[:, ::s_t])),
        lon=_nan_join(np.rad2deg(result.lon_runs[:, ::s_t])),
        mode="lines",
        line=dict(color=STEELBLUE_FAINT, width=1),
        name="Trial tracks", hoverinfo="skip"))

    s = _line_step(M)
    env_lat = np.concatenate([(lat_deg + dlat)[::s], (lat_deg - dlat)[::-s]])
    env_lon = np.concatenate([(lon_deg + dlon)[::s], (lon_deg - dlon)[::-s]])
    fig.add_trace(go.Scattermap(
        lat=env_lat, lon=env_lon, mode="lines", fill="toself",
        fillcolor=CRIMSON_BAND, line=dict(color="crimson", width=1),
        name="95th-pct envelope", hoverinfo="skip"))

    customdata = np.column_stack(
        [t_min[::s], p95_horiz[::s], p95_horiz[::s] / NM])
    hover = ("t = %{customdata[0]:.1f} min<br>"
             "95th-pct error: %{customdata[1]:.0f} m "
             "(%{customdata[2]:.2f} NM)<extra></extra>")
    fig.add_trace(go.Scattermap(
        lat=lat_deg[::s], lon=lon_deg[::s], mode="lines",
        line=dict(color=NAVY, width=2), name="Truth track",
        customdata=customdata, hovertemplate=hover))

    # 5-minute tick markers along the truth track (same hover payload)
    idx = np.searchsorted(t_min, np.arange(0.0, t_min[-1], 5.0))
    fig.add_trace(go.Scattermap(
        lat=lat_deg[idx], lon=lon_deg[idx], mode="markers",
        marker=dict(size=9, color="white"),
        name="5-min markers",
        customdata=np.column_stack(
            [t_min[idx], p95_horiz[idx], p95_horiz[idx] / NM]),
        hovertemplate=hover))

    mid = M // 2
    fig.update_layout(
        map=dict(style="open-street-map",
                 center=dict(lat=float(lat_deg[mid]),
                             lon=float(lon_deg[mid])),
                 zoom=8),
        legend=dict(x=0.01, y=0.99, bgcolor="rgba(255,255,255,0.8)"),
        margin=dict(l=0, r=0, t=0, b=0))
    return fig
