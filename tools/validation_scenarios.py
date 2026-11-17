"""Validation scenarios: single-error-source runs with analytic predictions.

Each function sets up one textbook situation, usually a stationary IMU with
exactly one error source, runs the simulator's own code on it, and returns
the simulated response together with the analytic prediction. The tests in
tests/validation/ assert the agreement; tools/generate_doc_plots.py
--validation draws the figures and writes the results table that the
Validation page includes.

The single-channel inertial checks run at the equator. There the vertical
component of Earth rate is zero, so the Foucault effect (Earth rate slowly
swapping error between the North and East channels) vanishes and the
textbook single-axis formulas apply exactly.
"""

from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from ins_sim.config import default_trajectory_path
from ins_sim.core.earth_model import WGS84_A, WGS84_OMEGA, normal_gravity, wgs84_radii
from ins_sim.evaluation.errors import attitude_error, position_error
from ins_sim.navigation.strapdown import NavState, strapdown_navgrade
from ins_sim.sensors.imu import GM_ALLAN_PEAK, IMUSpec, generate_imu_samples
from ins_sim.trajectory.kinematics import TruthTrajectory, build_trajectory

#: All error sources off; scenarios switch on exactly the one they test.
QUIET = dict(gyro_arw=0.0, gyro_bias_instability=0.0, gyro_br_std=0.0,
             accel_vrw=0.0, accel_bias_instability=0.0, accel_br_std=0.0)

DEG_PER_HOUR = np.pi / 180.0 / 3600.0      # 1 °/h in rad/s
MICRO_G = 9.80665e-6                       # 1 µg in m/s²


def stationary_truth(duration_s: float, tau_i: float, L_b_deg: float = 0.0,
                     h_b: float = 0.0) -> TruthTrajectory:
    """A level IMU at rest, facing North, at a fixed point on the Earth."""
    M = int(round(duration_s / tau_i)) + 1
    return TruthTrajectory(np.zeros((M, 3)), np.zeros((M, 3)), np.zeros((M, 3)),
                           tau_i, L_b0_deg=L_b_deg, lambda_b0_deg=0.0, h_b0=h_b)


def navigate(truth: TruthTrajectory, *, delta_psi_nb_n0=(0.0, 0.0, 0.0),
             delta_v_eb_n0=(0.0, 0.0, 0.0), accel_bias_b=(0.0, 0.0, 0.0),
             gyro_bias_b=(0.0, 0.0, 0.0), baro: bool = True) -> SimpleNamespace:
    """Runs the strapdown navigator on the truth's own IMU outputs plus chosen errors.

    Returns:
        SimpleNamespace: t, nav (NavSolution), delta_r_eb_n, delta_psi_nb_n.
    """
    # --8<-- [start:navigate]
    est_C_b_n0 = Rot.from_rotvec(delta_psi_nb_n0) * truth.C_b_n[0]
    start = NavState(truth.L_b[0], truth.lambda_b[0], truth.h_b[0],
                     truth.v_eb_n[0] + np.asarray(delta_v_eb_n0, dtype=float),
                     est_C_b_n0.as_quat())
    nav = strapdown_navgrade(truth.omega_ib_b + np.asarray(gyro_bias_b, dtype=float),
                             truth.f_ib_b + np.asarray(accel_bias_b, dtype=float),
                             start, truth.tau_i,
                             meas_h_b=truth.h_b if baro else None)
    # --8<-- [end:navigate]
    delta_r_eb_n = position_error(nav.L_b, nav.lambda_b, nav.h_b,
                                  truth.L_b, truth.lambda_b, truth.h_b)
    delta_psi_nb_n = attitude_error(Rot.from_quat(nav.q_b_n), truth.C_b_n)
    return SimpleNamespace(t=truth.t, nav=nav, delta_r_eb_n=delta_r_eb_n,
                           delta_psi_nb_n=delta_psi_nb_n)


def schuler_frequency(R: float, L_b: float = 0.0, h_b: float = 0.0) -> float:
    """omega_s = sqrt(g / (R + h_b)) [rad/s] for a channel with radius of curvature R."""
    return float(np.sqrt(normal_gravity(L_b, h_b) / (R + h_b)))


# --------------------------------------------------------------------------
# Inertial error propagation
# --------------------------------------------------------------------------

def zero_noise_round_trip(tau_i: float = 1.0) -> SimpleNamespace:
    """The packaged mission, navigated with its own perfect IMU outputs."""
    truth, _, _ = build_trajectory(str(default_trajectory_path()), tau_i=tau_i)
    run = navigate(truth)
    error = np.linalg.norm(run.nav.r_lb_l - truth.r_lb_l, axis=1)
    return SimpleNamespace(t=truth.t, error=error, max_error=float(error.max()))


def schuler_tilt(delta_psi_N: float = 1e-4, duration_s: float = 3 * 3600.0,
                 tau_i: float = 1.0) -> SimpleNamespace:
    """An initial North tilt, with no other error, at the equator.

    The tilt misresolves gravity into the East channel; the Schuler loop
    turns that into an oscillation of the East position error:
    delta_r_E = (R_E + h)·delta_psi_N·(1 − cos omega_s t).
    """
    truth = stationary_truth(duration_s, tau_i)
    run = navigate(truth, delta_psi_nb_n0=(delta_psi_N, 0.0, 0.0))
    R_E = float(wgs84_radii(0.0).R_E)
    omega_s = schuler_frequency(R_E)
    predicted = R_E * delta_psi_N * (1.0 - np.cos(omega_s * run.t))
    simulated = run.delta_r_eb_n[:, 1]
    # Period: time between the first two peaks (near T_s/2 and 3·T_s/2),
    # each refined with a parabola through its neighbours.
    T_s = 2.0 * np.pi / omega_s
    peaks = [_peak_time(run.t, simulated, (n - 0.25) * T_s, (n + 0.25) * T_s)
             for n in (0.5, 1.5)]
    return SimpleNamespace(t=run.t, simulated=simulated, predicted=predicted,
                           period_predicted=T_s, period_measured=peaks[1] - peaks[0],
                           peak_predicted=2.0 * R_E * delta_psi_N,
                           peak_measured=float(np.max(simulated)))


def _peak_time(t: np.ndarray, y: np.ndarray, t_from: float, t_to: float) -> float:
    """Time of the maximum of y within [t_from, t_to], refined with a parabola."""
    window = np.flatnonzero((t >= t_from) & (t <= t_to))
    k = int(window[np.argmax(y[window])])
    y0, y1, y2 = y[k - 1:k + 2]
    curvature = y0 - 2.0 * y1 + y2
    offset = 0.5 * (y0 - y2) / curvature if curvature else 0.0
    return float(t[k] + offset * (t[1] - t[0]))


def accel_bias(b_a: float = 10.0 * MICRO_G, duration_s: float = 3 * 3600.0,
               tau_i: float = 1.0) -> SimpleNamespace:
    """A North accelerometer bias, with no other error, at the equator.

    delta_r_N = (b_a / omega_s²)·(1 − cos omega_s t): a Schuler oscillation
    about b_a/omega_s², never a growing error.
    """
    truth = stationary_truth(duration_s, tau_i)
    run = navigate(truth, accel_bias_b=(b_a, 0.0, 0.0))
    omega_s = schuler_frequency(float(wgs84_radii(0.0).R_N))
    predicted = b_a / omega_s**2 * (1.0 - np.cos(omega_s * run.t))
    simulated = run.delta_r_eb_n[:, 0]
    return SimpleNamespace(t=run.t, simulated=simulated, predicted=predicted,
                           peak_predicted=2.0 * b_a / omega_s**2,
                           peak_measured=float(np.max(simulated)))


def gyro_bias(b_g: float = 0.01 * DEG_PER_HOUR, duration_s: float = 3 * 3600.0,
              tau_i: float = 1.0) -> SimpleNamespace:
    """An East-axis gyro bias, with no other error, at the equator.

    The tilt it builds is bounded by the Schuler loop, but the North
    position error keeps growing, at first at the rate R_N·b_g. Over hours
    the Earth-rate loop bends it over: the North error is a latitude error,
    which mis-resolves Earth rate into a heading error that feeds back into
    the tilt. Solving the coupled tilt, heading, velocity and latitude
    error equations at the equator gives

        delta_r_N = −R_N·b_g·ω_s²/(ω_s² − ω_ie²)·(sin(ω_ie t)/ω_ie − sin(ω_s t)/ω_s),

    which reduces to −R_N·b_g·(t − sin(ω_s t)/ω_s) while ω_ie·t is small.
    """
    truth = stationary_truth(duration_s, tau_i)
    run = navigate(truth, gyro_bias_b=(0.0, b_g, 0.0))
    R_N = float(wgs84_radii(0.0).R_N)
    omega_s = schuler_frequency(R_N)
    t = run.t
    predicted = (-R_N * b_g * omega_s**2 / (omega_s**2 - WGS84_OMEGA**2)
                 * (np.sin(WGS84_OMEGA * t) / WGS84_OMEGA - np.sin(omega_s * t) / omega_s))
    single_channel = -R_N * b_g * (t - np.sin(omega_s * t) / omega_s)
    simulated = run.delta_r_eb_n[:, 0]
    return SimpleNamespace(t=t, simulated=simulated, predicted=predicted,
                           single_channel=single_channel,
                           drift_rate_predicted=R_N * b_g,
                           end_predicted=float(predicted[-1]),
                           end_measured=float(simulated[-1]))


def vertical_channel(delta_v_D: float = 1.0, duration_s: float = 3000.0,
                     tau_i: float = 1.0) -> SimpleNamespace:
    """A 1 m/s Down-velocity error, free-inertial and with an ideal barometer.

    Free-inertial, the free-air gravity gradient makes the height error
    obey δḧ = (2g/R_0)·δh. Coriolis also couples the Down and East velocity
    errors, which slows the divergence slightly, to
    τ_v = 1/sqrt(2g/R_0 − 4·ω_ie²·cos²L_b), and δh = −δv_D·τ_v·sinh(t/τ_v).
    With an ideal barometer the third-order damping loop holds the error
    to a bounded transient.
    """
    truth = stationary_truth(duration_s, tau_i)
    free = navigate(truth, delta_v_eb_n0=(0.0, 0.0, delta_v_D), baro=False)
    aided = navigate(truth, delta_v_eb_n0=(0.0, 0.0, delta_v_D), baro=True)
    tau_v = float(1.0 / np.sqrt(2.0 * normal_gravity(0.0, 0.0) / WGS84_A
                                - 4.0 * WGS84_OMEGA**2))
    predicted = -delta_v_D * tau_v * np.sinh(free.t / tau_v)
    # e-folding time from the slope of ln|δh| over the last third of the run
    late = free.t > 2.0 * duration_s / 3.0
    slope = np.polyfit(free.t[late], np.log(np.abs(free.nav.h_b[late] - truth.h_b[late])), 1)[0]
    return SimpleNamespace(t=free.t, free=free.nav.h_b - truth.h_b,
                           aided=aided.nav.h_b - truth.h_b, predicted=predicted,
                           tau_v_predicted=tau_v, tau_v_measured=float(1.0 / slope))


# --------------------------------------------------------------------------
# Sensor noise models
# --------------------------------------------------------------------------

def _quiet_imu(n_samples: int, tau_i: float) -> SimpleNamespace:
    """A stand-in truth whose ideal gyro and accelerometer outputs are all zero."""
    return SimpleNamespace(t=np.arange(n_samples) * tau_i, tau_i=tau_i,
                           omega_ib_b=np.zeros((n_samples, 3)),
                           f_ib_b=np.zeros((n_samples, 3)))


def angle_random_walk(arw_deg_rt_h: float = 0.002, tau_i: float = 0.1,
                      duration_s: float = 1800.0, n_runs: int = 300,
                      seed: int = 0) -> SimpleNamespace:
    """Spread of the integrated gyro white noise across runs: ARW·√t."""
    arw = arw_deg_rt_h * np.pi / 180.0 / 60.0                      # rad/√s
    spec = IMUSpec(**{**QUIET, "gyro_arw": arw})
    n = int(round(duration_s / tau_i))
    imu = _quiet_imu(n, tau_i)
    # Keep about 200 checkpoints per run rather than every sample.
    keep = np.unique(np.linspace(0, n - 1, 200).astype(int))
    rngs = np.random.default_rng(seed).spawn(n_runs)
    angles = np.array([
        (np.cumsum(generate_imu_samples(imu, spec, rng).meas_omega_ib_b, axis=0) * tau_i)[keep]
        for rng in rngs])                                          # (runs, checkpoints, 3)
    t = (keep + 1) * tau_i
    simulated = angles.transpose(1, 0, 2).reshape(len(keep), -1).std(axis=1)
    return SimpleNamespace(t=t, simulated=simulated, predicted=arw * np.sqrt(t))


def gauss_markov(sigma: float = 0.01 * DEG_PER_HOUR, tau_c: float = 600.0,
                 tau_i: float = 10.0, duration_s: float = 3600.0,
                 n_runs: int = 1000, seed: int = 0) -> SimpleNamespace:
    """Ensemble spread over time and autocorrelation of the dynamic gyro bias.

    `sigma` is the Gauss-Markov spread; the spec is given the datasheet bias
    instability that produces it, sigma·GM_ALLAN_PEAK.
    """
    spec = IMUSpec(**{**QUIET, "gyro_bias_instability": sigma * GM_ALLAN_PEAK,
                      "gyro_bias_tau": tau_c})
    n = int(round(duration_s / tau_i)) + 1
    imu = _quiet_imu(n, tau_i)
    rngs = np.random.default_rng(seed).spawn(n_runs)
    bias = np.array([generate_imu_samples(imu, spec, rng).meas_omega_ib_b for rng in rngs])
    samples = bias.transpose(1, 0, 2).reshape(n, -1)               # (n, runs·3)
    lags = np.arange(0, n // 2)
    autocorr = np.array([np.corrcoef(samples[0], samples[k])[0, 1] for k in lags])
    return SimpleNamespace(t=imu.t, spread=samples.std(axis=1), sigma=sigma,
                           lag=lags * tau_i, autocorr=autocorr,
                           autocorr_predicted=np.exp(-lags * tau_i / tau_c),
                           autocorr_samples=samples.shape[1])


def allan_deviation(rate: np.ndarray, tau0: float, cluster_sizes: np.ndarray) -> np.ndarray:
    """Overlapping Allan deviation of a rate signal, per axis.

    Args:
        rate: Sampled rate, shape (N, k).
        tau0: Sample interval [s].
        cluster_sizes: Averaging-window lengths m, in samples.

    Returns:
        numpy.ndarray: Allan deviation, shape (len(cluster_sizes), k).
    """
    theta = np.vstack([np.zeros((1, rate.shape[1])), np.cumsum(rate, axis=0) * tau0])
    out = []
    for m in cluster_sizes:
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        out.append(np.sqrt(np.mean(d**2, axis=0) / (2.0 * (m * tau0) ** 2)))
    return np.array(out)


def gauss_markov_allan_variance(T: np.ndarray, sigma: float, tau_c: float) -> np.ndarray:
    """Analytic Allan variance of a first-order Gauss-Markov process."""
    x = T / tau_c
    return sigma**2 / x**2 * (2.0 * x - 3.0 + 4.0 * np.exp(-x) - np.exp(-2.0 * x))


def gyro_allan(arw_deg_rt_h: float = 0.002, bias_instab_deg_h: float = 0.01,
               tau_c: float = 3600.0, tau_i: float = 1.0, duration_h: float = 100.0,
               n_runs: int = 10, seed: int = 0) -> SimpleNamespace:
    """Allan deviation of a simulated stationary gyro (white noise + Gauss-Markov bias).

    The spec is given the datasheet values directly, as an IMU YAML would:
    the bias instability `bias_instab_deg_h` should come out as the peak of
    the Gauss-Markov term of the Allan deviation.
    """
    arw = arw_deg_rt_h * np.pi / 180.0 / 60.0
    bias_instability = bias_instab_deg_h * DEG_PER_HOUR
    spec = IMUSpec(**{**QUIET, "gyro_arw": arw, "gyro_bias_instability": bias_instability,
                      "gyro_bias_tau": tau_c})
    sigma = spec.gyro_gm_std
    n = int(round(duration_h * 3600.0 / tau_i))
    imu = _quiet_imu(n, tau_i)
    m = np.unique(np.round(np.logspace(0, np.log10(n / 10), 40)).astype(int))
    avar = np.zeros(len(m))
    for rng in np.random.default_rng(seed).spawn(n_runs):
        rate = generate_imu_samples(imu, spec, rng).meas_omega_ib_b
        avar += np.mean(allan_deviation(rate, tau_i, m) ** 2, axis=1)
    T = m * tau_i
    simulated = np.sqrt(avar / n_runs)
    white = arw / np.sqrt(T)
    gm = np.sqrt(gauss_markov_allan_variance(T, sigma, tau_c))
    # Where the Gauss-Markov term peaks, and how high, on a fine grid
    T_fine = np.logspace(1, 5, 4000)
    gm_fine = np.sqrt(gauss_markov_allan_variance(T_fine, sigma, tau_c))
    peak = int(np.argmax(gm_fine))
    return SimpleNamespace(T=T, simulated=simulated, white=white, gauss_markov=gm,
                           predicted=np.sqrt(white**2 + gm**2), sigma=sigma, arw=arw,
                           bias_instability=bias_instability,
                           gm_peak=float(gm_fine[peak]), gm_peak_T=float(T_fine[peak]))
