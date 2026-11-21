"""Unit tests for the Qt-free plotly figure builders (ins_sim.gui.figures).

Uses a tiny synthetic SimulationResult-shaped namespace so no Qt, no
event loop, and no real Monte Carlo run are needed.
"""

from types import SimpleNamespace

import numpy as np
import plotly.graph_objects as go

from ins_sim.gui import figures

N_TRIALS = 4
M = 240


def _synthetic_result(seed=0):
    """Builds a 4-trial, 2-hour synthetic result with linear error growth."""
    rng = np.random.default_rng(seed)
    t = np.linspace(0.0, 7200.0, M)               # 2 h -> fit window is half

    heading = np.linspace(0.0, np.pi / 2, M)
    speed = 150.0
    pos_n = np.column_stack([
        speed * t * np.cos(heading),
        speed * t * np.sin(heading),
        -3000.0 * np.ones(M),
    ])
    vel_n = np.gradient(pos_n, t[1] - t[0], axis=0)
    euler = np.column_stack([0.1 * np.sin(t / 500.0),
                             0.05 * np.ones(M), heading])
    lat = np.deg2rad(18.5) + pos_n[:, 0] / 6.378e6
    lon = np.deg2rad(-67.1) + pos_n[:, 1] / 6.0e6

    h_b = -pos_n[:, 2]                                   # 3000 m, constant
    truth = SimpleNamespace(t=t, r_lb_l=pos_n, v_eb_n=vel_n, euler_nb=euler,
                            L_b=lat, lambda_b=lon, h_b=h_b)

    # Per-trial error: distinct linear drift rates + noise, so the
    # ensemble median differs from every individual trial's fit.
    drift = np.linspace(0.5, 4.0, N_TRIALS)[:, None]           # m/s
    err = drift * t[None, :] + rng.normal(0.0, 20.0, (N_TRIALS, M))
    pos_runs = pos_n[None, :, :] + np.stack(
        [err, 0.5 * err, 0.1 * err], axis=2)
    vel_runs = vel_n[None, :, :] + rng.normal(0.0, 0.5, (N_TRIALS, M, 3))
    euler_runs = euler[None, :, :] + rng.normal(0.0, 0.01, (N_TRIALS, M, 3))
    lat_runs = lat[None, :] + pos_runs[:, :, 0] / 6.378e6 - pos_n[None, :, 0] / 6.378e6
    lon_runs = lon[None, :] + pos_runs[:, :, 1] / 6.0e6 - pos_n[None, :, 1] / 6.0e6
    h_runs = h_b[None, :] - (pos_runs[:, :, 2] - pos_n[None, :, 2])
    delta_psi = rng.normal(0.0, 1e-4, (N_TRIALS, M, 3))
    r95 = np.percentile(
        np.linalg.norm(pos_runs - pos_n[None, :, :], axis=2), 95, axis=0)

    return SimpleNamespace(
        truth=truth, est_r_lb_l=pos_runs, est_euler_nb=euler_runs,
        est_L_b=lat_runs, est_lambda_b=lon_runs, est_v_eb_n=vel_runs,
        est_h_b=h_runs, delta_psi_nb_n=delta_psi,
        r95=r95, n_trials=N_TRIALS, config={})


def _segments(arr):
    """Counts NaN-separated polyline segments in a bundled coordinate array."""
    return int(np.count_nonzero(np.isnan(np.asarray(arr, dtype=float))))


def test_cep_figure_bundles_all_trials_and_fits_ensemble_median():
    result = _synthetic_result()
    fig = figures.figure_cep(result)

    runs_trace = next(t for t in fig.data if t.name == "Individual runs")
    assert _segments(runs_trace.y) == N_TRIALS

    fit_trace = next(t for t in fig.data if "NM/hr" in (t.name or ""))
    shown_slope = float(fit_trace.name.split()[-2])

    t_hr = result.truth.t / 3600.0
    cep = np.median(figures.horizontal_error_nm(result), axis=0)
    mask = t_hr <= 1.0
    ens_slope = np.polyfit(t_hr[mask], cep[mask], 1)[0]
    trial_slopes = [
        np.polyfit(t_hr[mask], figures.horizontal_error_nm(result)[i, mask], 1)[0]
        for i in range(N_TRIALS)]

    assert abs(shown_slope - ens_slope) < 0.005
    # The synthetic drifts are all distinct, so no single trial's fit
    # may masquerade as the ensemble fit.
    assert all(abs(s - ens_slope) > 0.05 for s in trial_slopes
               if abs(s - ens_slope) > 0.005)


def test_band_figures_have_three_panels_with_bands_and_mean():
    result = _synthetic_result()
    for builder in (figures.figure_attitude, figures.figure_velocity,
                    figures.figure_position):
        fig = builder(result)
        assert len(fig.data) == 9          # 3 panels x (hi, lo-fill, mean)
        fills = [t for t in fig.data if t.fill == "tonexty"]
        means = [t for t in fig.data if t.line.color == figures.NAVY]
        assert len(fills) == 3
        assert len(means) == 3


def test_trajectory_3d_has_all_trials_truth_and_tube():
    result = _synthetic_result()
    fig = figures.figure_trajectory_3d(result)

    types = {type(t) for t in fig.data}
    assert go.Scatter3d in types and go.Mesh3d in types

    trials = next(t for t in fig.data if t.name == "INS estimates")
    assert _segments(trials.x) == N_TRIALS
    assert any(t.name == "Truth" for t in fig.data)


def test_map_has_all_trials_envelope_truth_and_markers():
    result = _synthetic_result()
    fig = figures.figure_map(result)

    assert all(isinstance(t, go.Scattermap) for t in fig.data)

    trials = next(t for t in fig.data if t.name == "Trial tracks")
    assert _segments(trials.lat) == N_TRIALS

    envelope = next(t for t in fig.data if t.name == "95th-pct envelope")
    assert envelope.fill == "toself"

    truthtrace = next(t for t in fig.data if t.name == "Truth track")
    assert "95th-pct error" in truthtrace.hovertemplate

    markers = next(t for t in fig.data if t.name == "5-min markers")
    # 2-hour flight -> 24 five-minute marks starting at t=0
    assert len(markers.lat) == 24

    assert fig.layout.map.style == "open-street-map"


def test_bands_use_unbiased_spread_and_tolerate_one_trial():
    result = _synthetic_result()
    two = SimpleNamespace(**{**vars(result), "n_trials": 2,
                             "delta_psi_nb_n": result.delta_psi_nb_n[:2]})
    fig = figures.figure_attitude(two)
    upper = np.asarray(fig.data[0].y, dtype=float)      # first panel, mean + 3σ
    deg = np.rad2deg(two.delta_psi_nb_n[:, :, 0])
    step = figures._line_step(M)
    expected = (deg.mean(axis=0) + 3 * deg.std(axis=0, ddof=1))[::step]
    np.testing.assert_allclose(upper, expected)

    one = SimpleNamespace(**{**vars(result), "n_trials": 1,
                             "delta_psi_nb_n": result.delta_psi_nb_n[:1]})
    fig = figures.figure_attitude(one)
    assert np.all(np.isfinite(np.asarray(fig.data[0].y, dtype=float)))


def _with_budget(result):
    """Adds a synthetic covariance ErrorBudget: sources' variances grow with time."""
    from ins_sim.evaluation.covariance import ERROR_SOURCES, CovarianceResult, ErrorBudget
    t = result.truth.t[::4]
    parts = {}
    for i, name in enumerate(ERROR_SOURCES):
        var = np.outer((t / 100.0) ** 2 * (i + 1), np.ones(9))
        parts[name] = CovarianceResult(t, var, np.zeros(len(t)))
    total = CovarianceResult(t, sum(p.var for p in parts.values()), np.zeros(len(t)))
    result.budget = ErrorBudget(total, parts)
    return result


def test_band_figures_draw_the_predicted_bounds_only_with_a_budget():
    plain = _synthetic_result()
    assert not any(t.name == "Predicted ±3σ (covariance)" for t in figures.figure_position(plain).data)
    fig = figures.figure_position(_with_budget(_synthetic_result()))
    predicted = [t for t in fig.data if t.name == "Predicted ±3σ (covariance)"]
    assert len(predicted) == 6                    # ±, in each of the three panels
    assert sum(t.showlegend for t in predicted) == 1


def test_cep_figure_draws_the_predicted_cep_with_a_budget():
    fig = figures.figure_cep(_with_budget(_synthetic_result()))
    assert any(t.name == "Predicted CEP (covariance)" for t in fig.data)


def test_error_budget_shares_fill_one_hundred_percent():
    result = _with_budget(_synthetic_result())
    shares = figures.horizontal_shares(result.budget)
    np.testing.assert_allclose(shares[:, 1:].sum(axis=0), 100.0)
    fig = figures.figure_error_budget(result)
    assert [t.name for t in fig.data] == list(result.budget.by_source)
