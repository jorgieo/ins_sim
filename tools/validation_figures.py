"""Figures and the results table for the documentation's Validation page.

Runs every scenario in tools/validation_scenarios.py, draws one figure per
check into docs/assets/validation/, and writes includes/validation_results.md,
which the Validation page includes. Called by

    python tools/generate_doc_plots.py --validation

Chart conventions: simulated series are colored (blue, then orange when a
figure compares two simulations); predictions are dark-gray dashed lines,
and approximations gray dotted, so a reader can tell them apart without
color. The pair blue/orange passes the colorblind-separation and contrast
checks against the white chart surface.
"""

from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

import validation_scenarios as v
from ins_sim.config import default_imu_spec_path, default_trajectory_path
from ins_sim.sensors.imu import load_imu_spec
from ins_sim.trajectory.kinematics import build_trajectory

SIM = "#2a78d6"          # simulated series
SIM_2 = "#eb6834"        # second simulated series
PREDICTED = "#52514e"    # analytic prediction (dashed)
APPROX = "#8f8d87"       # approximation / component (dotted)

LAYOUT = dict(template="plotly_white", font=dict(size=15),
              margin=dict(l=80, r=30, t=90, b=60),
              legend=dict(orientation="h", yanchor="bottom", y=1.02,
                          xanchor="left", x=0.0))
PANEL_LAYOUT = {**LAYOUT, "margin": dict(l=80, r=30, t=80, b=140),
                "legend": dict(orientation="h", yanchor="top", y=-0.2, xanchor="left", x=0.0)}
EXPORT = dict(width=1400, height=600, scale=2)
ARCSEC_PER_RAD = 180.0 / np.pi * 3600.0


def _line(x, y, name, color, dash="solid", width=2.5):
    return go.Scatter(x=x, y=y, name=name, mode="lines",
                      line=dict(color=color, dash=dash, width=width))


def _save(fig, out_dir: Path, name: str) -> None:
    fig.write_image(str(out_dir / name), **EXPORT)
    print(f"wrote {out_dir.name}/{name}", flush=True)


def _pct(measured: float, predicted: float) -> str:
    percent = (measured / predicted - 1.0) * 100
    if abs(percent) < 0.005:
        return "0.00%"
    return f"{percent:+.2f}%".replace("-", "−")


def _num(value: float, fmt: str) -> str:
    """Formats a number with a typographic minus sign."""
    return format(value, fmt).replace("-", "−")


def _sci(value: float) -> str:
    """Formats a small positive number as a × 10^n in inline math."""
    mantissa, exponent = f"{value:.1e}".split("e")
    return rf"\({mantissa} \times 10^{{{int(exponent)}}}\)"


def build_all(out_dir: Path, table_path: Path) -> None:
    """Runs every validation scenario, writes the figures and the results table."""
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []

    # ---- Zero-noise round trip -----------------------------------------------
    rt = v.zero_noise_round_trip()
    rows.append((r"Zero-noise round trip, packaged mission (\(\tau_i\) = 1 s)",
                 "0 m", f"{_sci(rt.max_error)} m (largest)", "numerical precision"))

    # ---- Schuler oscillation -------------------------------------------------
    sch = v.schuler_tilt()
    t_min = sch.t / 60.0
    fig = go.Figure([_line(t_min, sch.simulated, "Simulated", SIM),
                     _line(t_min, sch.predicted, "Predicted, (R<sub>E</sub> + h)·δψ<sub>N</sub>·(1 − cos ω<sub>s</sub>t)", PREDICTED, "dash")])
    fig.add_vline(x=sch.period_predicted / 60.0, line=dict(color=APPROX, dash="dot", width=1.5),
                  annotation_text=f"T<sub>s</sub> = {sch.period_predicted / 60.0:.1f} min",
                  annotation_position="top left")
    fig.update_layout(title="East position error after a 20″ initial North tilt (equator)",
                      xaxis_title="Time (min)", yaxis_title="East position error (m)", **LAYOUT)
    _save(fig, out_dir, "schuler.png")
    rows.append(("Schuler period, from a 20″ North tilt",
                 f"{sch.period_predicted / 60:.2f} min", f"{sch.period_measured / 60:.2f} min",
                 _pct(sch.period_measured, sch.period_predicted)))
    rows.append(("Peak East position error from that tilt",
                 f"{sch.peak_predicted:,.1f} m", f"{sch.peak_measured:,.1f} m",
                 _pct(sch.peak_measured, sch.peak_predicted)))

    # ---- Accelerometer bias --------------------------------------------------
    acc = v.accel_bias()
    t_min = acc.t / 60.0
    fig = go.Figure([_line(t_min, acc.simulated, "Simulated", SIM),
                     _line(t_min, acc.predicted, "Predicted, (b<sub>a</sub>/ω<sub>s</sub><sup>2</sup>)·(1 − cos ω<sub>s</sub>t)", PREDICTED, "dash")])
    fig.update_layout(title="North position error from a 10 µg North accelerometer bias (equator)",
                      xaxis_title="Time (min)", yaxis_title="North position error (m)", **LAYOUT)
    _save(fig, out_dir, "accel_bias.png")
    rows.append(("Peak North error from a 10 µg accelerometer bias",
                 f"{acc.peak_predicted:.2f} m", f"{acc.peak_measured:.2f} m",
                 _pct(acc.peak_measured, acc.peak_predicted)))

    # ---- Gyro bias -----------------------------------------------------------
    gyr = v.gyro_bias()
    t_min = gyr.t / 60.0
    fig = go.Figure([_line(t_min, gyr.simulated, "Simulated", SIM),
                     _line(t_min, gyr.predicted, "Predicted, with the Earth-rate loop", PREDICTED, "dash"),
                     _line(t_min, gyr.single_channel, "Single-channel approximation", APPROX, "dot")])
    fig.update_layout(title="North position error from a 0.01 °/h East gyro bias (equator)",
                      xaxis_title="Time (min)", yaxis_title="North position error (m)", **LAYOUT)
    _save(fig, out_dir, "gyro_bias.png")
    rows.append(("North error after 3 h from a 0.01 °/h gyro bias",
                 f"{_num(gyr.end_predicted, ',.0f')} m", f"{_num(gyr.end_measured, ',.0f')} m",
                 _pct(gyr.end_measured, gyr.end_predicted)))

    # ---- Vertical channel ----------------------------------------------------
    vert = v.vertical_channel()
    t_min = vert.t / 60.0
    fig = make_subplots(rows=1, cols=2, horizontal_spacing=0.1,
                        subplot_titles=("Free-inertial: error magnitude (log scale)",
                                        "With an ideal barometer"))
    fig.add_trace(_line(t_min[1:], np.abs(vert.free[1:]), "Simulated", SIM), row=1, col=1)
    fig.add_trace(_line(t_min[1:], np.abs(vert.predicted[1:]),
                        "Predicted, δv<sub>D</sub>·τ<sub>v</sub>·sinh(t/τ<sub>v</sub>)", PREDICTED, "dash"),
                  row=1, col=1)
    fig.add_trace(go.Scatter(x=t_min, y=vert.aided, mode="lines", showlegend=False,
                             line=dict(color=SIM, width=2.5)), row=1, col=2)
    fig.update_yaxes(type="log", dtick=1, title_text="|Height error| (m)", row=1, col=1)
    fig.update_yaxes(title_text="Height error, estimate − truth (m)", row=1, col=2)
    fig.update_xaxes(title_text="Time (min)")
    fig.update_layout(title="Height error after a 1 m/s Down-velocity error (equator)", **PANEL_LAYOUT)
    _save(fig, out_dir, "vertical_channel.png")
    rows.append((r"Free-inertial vertical time constant \(\tau_v\)",
                 f"{vert.tau_v_predicted:.1f} s", f"{vert.tau_v_measured:.1f} s",
                 _pct(vert.tau_v_measured, vert.tau_v_predicted)))
    rows.append(("Largest height error with an ideal barometer", "bounded",
                 f"{np.abs(vert.aided).max():.1f} m, then {abs(vert.aided[-1]):.3f} m at 50 min",
                 "damped"))

    # ---- Barometer errors ----------------------------------------------------
    bias = v.baro_bias()
    drift = v.baro_drift()
    fig = make_subplots(rows=1, cols=2, horizontal_spacing=0.1,
                        subplot_titles=("A constant 10 m barometer error",
                                        "A barometer drift (10 m, correlated over 10 min)"))
    fig.add_trace(_line(bias.t / 60.0, bias.simulated, "Simulated INS height error", SIM),
                  row=1, col=1)
    fig.add_trace(_line(bias.t / 60.0, bias.predicted, "Predicted, barometer error through the loop",
                        PREDICTED, "dash"), row=1, col=1)
    fig.add_trace(_line(drift.t / 60.0, drift.drift, "Barometer error", APPROX, "dot"), row=1, col=2)
    fig.add_trace(go.Scatter(x=drift.t / 60.0, y=drift.simulated, mode="lines", showlegend=False,
                             line=dict(color=SIM, width=2.5)), row=1, col=2)
    fig.add_trace(go.Scatter(x=drift.t / 60.0, y=drift.predicted, mode="lines", showlegend=False,
                             line=dict(color=PREDICTED, dash="dash", width=2.5)), row=1, col=2)
    fig.update_yaxes(title_text="Height error, estimate − truth (m)", row=1, col=1)
    fig.update_xaxes(title_text="Time (min)")
    fig.update_layout(title="INS height error from barometer errors (IMU at rest, equator)",
                      **PANEL_LAYOUT)
    _save(fig, out_dir, "baro_tracking.png")
    rows.append(("Steady INS height error from a 10 m barometer bias", f"{bias.bias:.2f} m",
                 f"{bias.end_measured:.2f} m", _pct(bias.end_measured, bias.bias)))
    rows.append(("RMS lag of the INS height behind a barometer drift",
                 f"{drift.lag_rms_predicted:.2f} m", f"{drift.lag_rms_measured:.2f} m",
                 _pct(drift.lag_rms_measured, drift.lag_rms_predicted)))

    noise = v.baro_noise()
    fig = go.Figure([
        _line(noise.t / 60.0, noise.spread * 100.0, f"Simulated ({noise.n_runs} runs)", SIM),
        _line(noise.t / 60.0, np.full_like(noise.t, noise.predicted * 100.0),
              "Predicted, N·√∫h(t)²dt", PREDICTED, "dash"),
    ])
    fig.update_layout(title=("Spread of the INS height error from 0.1 m/√Hz of barometer noise "
                             f"({noise.per_sample * 100:.0f} cm per sample at τ<sub>i</sub> = 1 s)"),
                      xaxis_title="Time (min)", yaxis_title="Height-error spread (cm)",
                      yaxis_rangemode="tozero", **LAYOUT)
    _save(fig, out_dir, "baro_noise.png")
    rows.append(("INS height-error spread from 0.1 m/√Hz of barometer noise",
                 f"{noise.predicted * 100:.2f} cm", f"{noise.measured * 100:.2f} cm",
                 _pct(noise.measured, noise.predicted)))

    # ---- Precision mechanization ---------------------------------------------
    con = v.coning()
    scu = v.sculling()
    fig = make_subplots(rows=1, cols=2, horizontal_spacing=0.1,
                        subplot_titles=("Coning: heading error",
                                        "Sculling: East velocity error"))
    for col, (r, sim_b, sim_p, scale) in enumerate(
            [(con, con.heading_basic, con.heading_precision, ARCSEC_PER_RAD),
             (scu, scu.v_E_basic, scu.v_E_precision, 1e3)], start=1):
        legend = col == 1
        fig.add_trace(go.Scatter(x=r.t, y=sim_b * scale, name="Basic mechanization", mode="lines",
                                 showlegend=legend, line=dict(color=SIM_2, width=2.5)),
                      row=1, col=col)
        fig.add_trace(go.Scatter(x=r.t, y=sim_p * scale, name="Precision mechanization", mode="lines",
                                 showlegend=legend, line=dict(color=SIM, width=2.5)),
                      row=1, col=col)
        for rate, name in ((r.predicted_basic, "Predicted drift"),
                           (r.predicted_precision, "Predicted drift")):
            fig.add_trace(go.Scatter(x=r.t, y=rate * r.t * scale, name=name, mode="lines",
                                     showlegend=bool(legend and rate == r.predicted_basic),
                                     line=dict(color=PREDICTED, dash="dash", width=2)),
                          row=1, col=col)
    fig.update_yaxes(title_text="Heading error (″)", row=1, col=1)
    fig.update_yaxes(title_text="East velocity error (mm/s)", row=1, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_layout(title=("Classical coning (1° cone at 2 Hz) and sculling (1° yaw with 1 g "
                             "at 2 Hz), IMU sampled at 100 Hz"), **PANEL_LAYOUT)
    _save(fig, out_dir, "coning_sculling.png")
    to_deg_h = 1.0 / v.DEG_PER_HOUR
    rows.append(("Heading drift from coning, basic mechanization",
                 f"{_num(con.predicted_basic * to_deg_h, '.3f')} °/h",
                 f"{_num(con.rate_basic * to_deg_h, '.3f')} °/h",
                 _pct(con.rate_basic, con.predicted_basic)))
    rows.append(("Heading drift from coning, precision mechanization",
                 f"{_num(con.predicted_precision * to_deg_h, '.5f')} °/h",
                 f"{_num(con.rate_precision * to_deg_h, '.5f')} °/h",
                 _pct(con.rate_precision, con.predicted_precision)))
    rows.append(("East acceleration error from sculling, basic mechanization",
                 f"{_num(scu.predicted_basic / v.MICRO_G, '.2f')} µg",
                 f"{_num(scu.rate_basic / v.MICRO_G, '.2f')} µg",
                 _pct(scu.rate_basic, scu.predicted_basic)))
    rows.append(("East acceleration error from sculling, precision mechanization",
                 f"{_num(scu.predicted_precision / v.MICRO_G, '.4f')} µg",
                 f"{_num(scu.rate_precision / v.MICRO_G, '.4f')} µg",
                 _pct(scu.rate_precision, scu.predicted_precision)))

    mis = v.mission_mechanizations()
    fig = go.Figure([_line(mis.t / 60.0, mis.basic, "Basic mechanization", SIM_2),
                     _line(mis.t / 60.0, mis.precision, "Precision mechanization", SIM)])
    fig.update_layout(title=(f"Horizontal position error with perfect sensors, packaged mission "
                             f"(τ<sub>i</sub> = {mis.tau_i} s, truth oversampled {mis.oversample}×)"),
                      xaxis_title="Time (min)", yaxis_title="Horizontal error (m)",
                      yaxis_type="log", **LAYOUT)
    _save(fig, out_dir, "mission_mechanizations.png")

    conv = v.mechanization_convergence()
    fig = go.Figure([
        go.Scatter(x=conv.tau, y=conv.basic, name="Basic mechanization", mode="lines+markers",
                   line=dict(color=SIM_2, width=2.5), marker=dict(size=10)),
        go.Scatter(x=conv.tau, y=conv.precision, name="Precision mechanization",
                   mode="lines+markers", line=dict(color=SIM, width=2.5),
                   marker=dict(size=10, symbol="square")),
        _line(conv.tau, conv.basic[0] * conv.tau / conv.tau[0], "First order, ∝ τ<sub>i</sub>",
              APPROX, "dot"),
        _line(conv.tau, conv.precision[0] * (conv.tau / conv.tau[0]) ** 2,
              "Second order, ∝ τ<sub>i</sub>²", APPROX, "dash"),
    ])
    fig.update_layout(title=("Largest horizontal error with perfect sensors, packaged mission "
                             f"(truth built at {conv.tau_fine} s)"),
                      xaxis_title="IMU interval τ<sub>i</sub> (s)",
                      yaxis_title="Largest horizontal error (m)",
                      xaxis_type="log", yaxis_type="log", **LAYOUT)
    _save(fig, out_dir, "mechanization_convergence.png")
    order = lambda e: float(np.log(e[0] / e[1]) / np.log(conv.tau[0] / conv.tau[1]))
    rows.append((rf"Order of the basic mechanization's error, \(\tau_i\) = {conv.tau[0]:g} → {conv.tau[1]:g} s",
                 "1", f"{order(conv.basic):.2f}", "first order"))
    rows.append((rf"Order of the precision mechanization's error, \(\tau_i\) = {conv.tau[0]:g} → {conv.tau[1]:g} s",
                 "≥ 2", f"{order(conv.precision):.2f}", "higher order"))

    # ---- Angle random walk ---------------------------------------------------
    arw_fast = v.angle_random_walk(tau_i=0.1)
    arw_slow = v.angle_random_walk(tau_i=1.0)
    fig = go.Figure([
        _line(arw_fast.t / 60.0, arw_fast.simulated * ARCSEC_PER_RAD, "Simulated, τ<sub>i</sub> = 0.1 s", SIM),
        _line(arw_slow.t / 60.0, arw_slow.simulated * ARCSEC_PER_RAD, "Simulated, τ<sub>i</sub> = 1 s", SIM_2),
        _line(arw_fast.t / 60.0, arw_fast.predicted * ARCSEC_PER_RAD, "Predicted, ARW·√t", PREDICTED, "dash"),
    ])
    fig.update_layout(title="Spread of the integrated gyro white noise (ARW 0.002 °/√h, 300 runs)",
                      xaxis_title="Time (min)", yaxis_title="Integrated-angle spread (arcsec)", **LAYOUT)
    _save(fig, out_dir, "arw.png")
    for r, label in ((arw_fast, "0.1 s"), (arw_slow, "1 s")):
        rows.append((rf"Integrated-angle spread after 30 min, \(\tau_i\) = {label}",
                     f"{r.predicted[-1] * ARCSEC_PER_RAD:.2f}″", f"{r.simulated[-1] * ARCSEC_PER_RAD:.2f}″",
                     _pct(r.simulated[-1], r.predicted[-1])))

    # ---- Gauss-Markov bias ---------------------------------------------------
    gm = v.gauss_markov()
    to_deg_h = 1.0 / v.DEG_PER_HOUR
    fig = make_subplots(rows=1, cols=2, horizontal_spacing=0.12,
                        subplot_titles=("Spread across runs (predicted: σ)",
                                        "Correlation with the first sample (predicted: exp(−Δt/τ<sub>c</sub>))"))
    fig.add_trace(_line(gm.t / 60.0, gm.spread * to_deg_h, "Simulated", SIM), row=1, col=1)
    fig.add_trace(_line(gm.t / 60.0, np.full_like(gm.t, gm.sigma * to_deg_h),
                        "Predicted", PREDICTED, "dash"), row=1, col=1)
    fig.add_trace(go.Scatter(x=gm.lag / 60.0, y=gm.autocorr, mode="lines", showlegend=False,
                             line=dict(color=SIM, width=2.5)), row=1, col=2)
    fig.add_trace(go.Scatter(x=gm.lag / 60.0, y=gm.autocorr_predicted, mode="lines", showlegend=False,
                             line=dict(color=PREDICTED, dash="dash", width=2.5)), row=1, col=2)
    fig.update_yaxes(title_text="Bias spread (°/h)", rangemode="tozero", row=1, col=1)
    fig.update_yaxes(title_text="Correlation", row=1, col=2)
    fig.update_xaxes(title_text="Time (min)", row=1, col=1)
    fig.update_xaxes(title_text="Lag (min)", row=1, col=2)
    fig.update_layout(title="Gauss–Markov gyro bias: σ = 0.01 °/h, τ<sub>c</sub> = 10 min (3,000 samples)",
                      **PANEL_LAYOUT)
    _save(fig, out_dir, "gauss_markov.png")
    rows.append(("Gauss–Markov bias spread at the first sample", "σ",
                 f"{gm.spread[0] / gm.sigma:.3f} σ", _pct(gm.spread[0], gm.sigma)))
    rho = float(np.interp(600.0, gm.lag, gm.autocorr))
    std_error = (1.0 - np.exp(-2.0)) / np.sqrt(gm.autocorr_samples)
    rows.append((r"Gauss–Markov correlation at a lag of \(\tau_c\)", r"\(e^{-1}\) = 0.368",
                 f"{rho:.3f}", f"{_num(rho - np.exp(-1.0), '+.3f')} (standard error {std_error:.3f})"))

    # ---- Allan deviation -----------------------------------------------------
    al = v.gyro_allan()
    fig = go.Figure([
        go.Scatter(x=al.T, y=al.simulated * to_deg_h, name="Simulated (10 runs × 100 h)", mode="lines+markers",
                   line=dict(color=SIM, width=2.5), marker=dict(size=8, color=SIM)),
        _line(al.T, al.predicted * to_deg_h, "Predicted (white noise + Gauss–Markov)", PREDICTED, "dash"),
        _line(al.T, al.white * to_deg_h, "White-noise term, ARW/√T", APPROX, "dot"),
        _line(al.T, al.gauss_markov * to_deg_h, "Gauss–Markov term", APPROX, "dashdot"),
    ])
    fig.add_annotation(x=np.log10(al.gm_peak_T), y=np.log10(al.gm_peak * to_deg_h),
                       text=(f"Gauss–Markov peak = datasheet bias instability, "
                             f"{al.gm_peak * to_deg_h:.4f} °/h at {al.gm_peak_T / 3600:.2f} h"),
                       showarrow=True, arrowhead=2, ax=-40, ay=-60)
    fig.update_xaxes(type="log", dtick=1, title_text="Averaging time T (s)")
    fig.update_yaxes(type="log", dtick=1, exponentformat="power", title_text="Allan deviation (°/h)")
    fig.update_layout(title="Allan deviation of a simulated stationary gyro (packaged navigation-grade spec)",
                      **LAYOUT)
    _save(fig, out_dir, "allan.png")
    k1 = int(np.argmin(np.abs(al.T - 1.0)))
    kp = int(np.argmin(np.abs(al.T - al.gm_peak_T)))
    rows.append(("Allan deviation at T = 1 s", f"{al.predicted[k1] * to_deg_h:.4f} °/h",
                 f"{al.simulated[k1] * to_deg_h:.4f} °/h", _pct(al.simulated[k1], al.predicted[k1])))
    rows.append((f"Allan deviation at the bias-instability peak (datasheet 0.01 °/h, plus white noise; "
                 f"T = {al.T[kp] / 3600:.1f} h)",
                 f"{al.predicted[kp] * to_deg_h:.4f} °/h", f"{al.simulated[kp] * to_deg_h:.4f} °/h",
                 _pct(al.simulated[kp], al.predicted[kp])))

    # ---- Covariance analysis ----------------------------------------------------
    spec = load_imu_spec(str(default_imu_spec_path()))
    single = v.linear_model_single_trial(v.turning_truth(0.1), spec)
    gap = np.abs(single.navigator - single.linear).max(axis=0) / np.abs(single.navigator).max(axis=0)
    rows.append((r"Largest gap between the linear error model and the navigator, one trial of "
                 r"constant errors, turning flight (\(\tau_i\) = 0.1 s), as a share of each "
                 "error's largest value", "0 (to first order)", f"{100 * gap.max():.2f}%",
                 "within 1%"))

    mission = build_trajectory(str(default_trajectory_path()), tau_i=1.0).truth
    cmp = v.covariance_vs_monte_carlo(mission, spec, n_trials=200)
    t_min = cmp.t / 60.0
    colors = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")
    fig = go.Figure()
    for (label, mc, cov), color in zip(
            [("North σ", cmp.sigma_mc[:, 6], cmp.sigma_cov[:, 6]),
             ("East σ", cmp.sigma_mc[:, 7], cmp.sigma_cov[:, 7]),
             ("CEP", cmp.cep_mc, cmp.cep_cov)], colors):
        fig.add_trace(_line(t_min, mc / 1852.0, f"{label}, {cmp.n_trials} trials", color))
        fig.add_trace(_line(t_min, cov / 1852.0, f"{label}, predicted", color, "dash"))
    fig.update_layout(title=("Horizontal position spread, packaged mission (τ<sub>i</sub> = 1 s): "
                             "Monte Carlo and covariance analysis"),
                      xaxis_title="Time (min)", yaxis_title="Position spread (NM)", **LAYOUT)
    _save(fig, out_dir, "covariance_position.png")

    checkpoints = [20, 40, 60, int(t_min[-1])]
    names = ["North tilt", "East tilt", "Heading", "North vel.", "East vel.", "Down vel.",
             "North pos.", "East pos.", "Down pos."]
    fig = go.Figure()
    fig.add_hrect(y0=1 - cmp.band, y1=1 + cmp.band, fillcolor="rgba(82, 81, 78, 0.12)",
                  line_width=0)
    fig.add_hline(y=1.0, line_color=PREDICTED, line_dash="dash", line_width=1.5)
    ratios = []
    for minute, color, symbol in zip(checkpoints, colors, ("circle", "square", "diamond", "triangle-up")):
        k = int(np.searchsorted(cmp.t, minute * 60.0))
        ratio = cmp.sigma_mc[k] / cmp.sigma_cov[k]
        ratios.append(ratio)
        fig.add_trace(go.Scatter(x=names, y=ratio, mode="markers", name=f"{minute} min",
                                 marker=dict(color=color, size=13, symbol=symbol,
                                             line=dict(color="white", width=2))))
    fig.update_layout(title=(f"Monte Carlo σ over predicted σ, {cmp.n_trials} trials; "
                             f"shaded: the 95% sampling band, ±{cmp.band:.3f}"),
                      xaxis_title="Error component", yaxis_title="σ Monte Carlo / σ predicted",
                      **LAYOUT)
    _save(fig, out_dir, "covariance_ratio.png")
    ratios = np.array(ratios)
    inside = int(np.sum(np.abs(ratios - 1) < cmp.band))
    rows.append((f"Monte Carlo σ over predicted σ, nine errors at {len(checkpoints)} times, "
                 f"packaged mission, {cmp.n_trials} trials",
                 f"1 ± {cmp.band:.3f} (95%)",
                 f"{_num(ratios.min(), '.3f')} to {_num(ratios.max(), '.3f')}",
                 f"{inside} of {ratios.size} inside"))
    inside = cmp.cep_lo[-1] <= cmp.cep_cov[-1] <= cmp.cep_hi[-1]
    rows.append((f"CEP at the end ({t_min[-1]:.0f} min): predicted, and the Monte Carlo median "
                 f"with its 95% confidence interval",
                 f"{cmp.cep_cov[-1]:.0f} m",
                 f"{cmp.cep_mc[-1]:.0f} m ({cmp.cep_lo[-1]:.0f} to {cmp.cep_hi[-1]:.0f} m)",
                 f"{_pct(cmp.cep_mc[-1], cmp.cep_cov[-1])}, prediction "
                 f"{'inside' if inside else 'outside'} the interval"))

    # ---- Results table ---------------------------------------------------------
    lines = ["<!-- Generated by tools/generate_doc_plots.py --validation; do not edit by hand. -->",
             "",
             "| Check | Prediction | Simulated | Difference |",
             "| ----- | ---------- | --------- | ---------- |"]
    lines += [f"| {check} | {pred} | {sim} | {diff} |" for check, pred, sim, diff in rows]
    table_path.parent.mkdir(parents=True, exist_ok=True)
    table_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {table_path.name}", flush=True)
