"""Covariance analysis: the navigation-error spread predicted by a linearized error model.

Where `run_monte_carlo` runs the navigator many times with random sensor
errors, `covariance_analysis` propagates the covariance of those errors
through the navigator's linearized error equations, along the same truth
trajectory, in one pass. The error state mirrors the simulator's error
sources one for one:

| States | Symbols | Count |
| ------ | ------- | ----- |
| Attitude, velocity, position errors | delta_psi_nb^n, delta_v_eb^n, delta_L_b, delta_lambda_b, delta_h_b | 9 |
| Integral state of the baro-damping loop | delta_a_c | 1 |
| Accelerometer static and Gauss-Markov biases | b_as, b_ad | 6 |
| Gyro static and Gauss-Markov biases | b_gs, b_gd | 6 |
| Accelerometer and gyro scale factor and misalignment | M_a, M_g | 18 |
| Barometer scale factor, bias and drift | s_b, b_bs, b_bd | 3 |

The model is linear, so each error source's contribution to the variance
adds to the others': `error_budget` returns them separately. The model is
derived on the [Covariance analysis][covariance-analysis] theory page.
"""

from dataclasses import dataclass
from functools import cache
from typing import TYPE_CHECKING, NamedTuple

import numpy as np
from scipy.linalg import expm
from scipy.special import erf

from ins_sim.core.earth_model import (WGS84_A, WGS84_E2, WGS84_OMEGA,
                                      normal_gravity, wgs84_radii)
from ins_sim.sensors.imu import IMUSpec

if TYPE_CHECKING:
    from ins_sim.trajectory.kinematics import TruthTrajectory

# ---- State layout ----------------------------------------------------------
PSI = slice(0, 3)      # delta_psi_nb^n [rad]
VEL = slice(3, 6)      # delta_v_eb^n [m/s]
LAT, LON, HGT = 6, 7, 8  # delta_L_b, delta_lambda_b [rad]; delta_h_b [m]
ACC = 9                # baro-loop integral state [m/s²]
B_AS = slice(10, 13)   # accelerometer static bias [m/s²]
B_AD = slice(13, 16)   # accelerometer Gauss-Markov bias [m/s²]
B_GS = slice(16, 19)   # gyro static bias [rad/s]
B_GD = slice(19, 22)   # gyro Gauss-Markov bias [rad/s]
M_A = slice(22, 31)    # accelerometer scale factor and misalignment, row by row
M_G = slice(31, 40)    # gyro scale factor and misalignment, row by row
S_B, B_BS, B_BD = 40, 41, 42  # barometer scale factor, bias, drift
N_STATES = 43

#: The error sources of `error_budget`, in the order it reports them.
ERROR_SOURCES = (
    "Initial alignment",
    "Gyro biases",
    "Gyro scale factor and misalignment",
    "Accelerometer biases",
    "Accel. scale factor and misalignment",
    "Sensor noise (ARW and VRW)",
    "Barometer",
)

#: Gains of the baro-damping loop, as `strapdown_navgrade` sets them.
_BARO_TAU = 100.0


@dataclass(frozen=True)
class CovarianceResult:
    """Predicted spread of the navigation errors over time.

    The errors are those of `ins_sim.evaluation.errors`: estimate minus
    truth, resolved in NED at the true position.

    Attributes:
        t: Times of the prediction, shape (K,) [s]: every `step_s` along
            the truth trajectory.
        var: Variances of the nine navigation errors, in the order North
            tilt, East tilt, heading [rad²]; North, East, Down velocity
            [(m/s)²]; North, East, Down position [m²]. Shape (K, 9).
        cov_NE: Covariance of the North and East position errors, shape
            (K,) [m²].
    """

    t: np.ndarray
    var: np.ndarray
    cov_NE: np.ndarray

    @property
    def sigma_delta_psi_nb_n(self) -> np.ndarray:
        """Standard deviation of the attitude error delta_psi_nb^n, shape (K, 3) [rad]."""
        return np.sqrt(self.var[:, 0:3])

    @property
    def sigma_delta_v_eb_n(self) -> np.ndarray:
        """Standard deviation of the velocity error delta_v_eb^n, shape (K, 3) [m/s]."""
        return np.sqrt(self.var[:, 3:6])

    @property
    def sigma_delta_r_eb_n(self) -> np.ndarray:
        """Standard deviation of the position error delta_r_eb^n, shape (K, 3) [m]."""
        return np.sqrt(self.var[:, 6:9])

    @property
    def cep(self) -> np.ndarray:
        """Circular error probable of the horizontal position, shape (K,) [m].

        The exact median of the horizontal error's two-dimensional Gaussian
        distribution; see `gaussian_cep`.
        """
        return gaussian_cep(self.var[:, 6], self.var[:, 7], self.cov_NE)

    def __add__(self, other: "CovarianceResult") -> "CovarianceResult":
        """Combines independent error sources: their variances add."""
        return CovarianceResult(self.t, self.var + other.var,
                                self.cov_NE + other.cov_NE)


class ErrorBudget(NamedTuple):
    """Each error source's share of the predicted navigation-error spread.

    Attributes:
        total: The prediction with every error source.
        by_source: The prediction with each source of `ERROR_SOURCES` on its
            own. Their variances add up to the total's.
    """

    total: CovarianceResult
    by_source: dict[str, CovarianceResult]


# ---- CEP of a two-dimensional Gaussian -------------------------------------

@cache
def _cep_table(n: int = 401) -> tuple[np.ndarray, np.ndarray]:
    """CEP of a unit-major-axis Gaussian, against the minor-to-major axis ratio.

    For X, Y independent standard normal, finds c(r) with
    P(X² + r²·Y² ≤ c²) = 1/2 by bisection. The probability is integrated
    over x = c·sin θ, which makes the integrand smooth.
    """
    r = np.linspace(0.0, 1.0, n)
    theta, w = np.polynomial.legendre.leggauss(64)
    theta = theta * np.pi / 2; w = w * np.pi / 2

    def probability(c):
        x = c[:, None] * np.sin(theta)
        half_width = c[:, None] * np.cos(theta)
        with np.errstate(divide="ignore", invalid="ignore"):
            inner = np.where(r[:, None] > 0,
                             erf(half_width / (np.maximum(r[:, None], 1e-300) * np.sqrt(2))),
                             1.0)
        density = np.exp(-0.5 * x * x) / np.sqrt(2 * np.pi)
        return (density * inner * half_width) @ w

    lo, hi = np.zeros(n), np.full(n, 2.0)
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        below = probability(mid) < 0.5
        lo = np.where(below, mid, lo)
        hi = np.where(below, hi, mid)
    return r, 0.5 * (lo + hi)


def gaussian_cep(var_N: np.ndarray, var_E: np.ndarray,
                 cov_NE: np.ndarray) -> np.ndarray:
    """Circular error probable of a zero-mean two-dimensional Gaussian.

    The CEP is the radius of the circle that holds half the probability.
    It is computed exactly, from the principal axes of the covariance,
    rather than with the 0.59·(σ_N + σ_E) approximation.

    Args:
        var_N: Variance of the North error [m²], any shape.
        var_E: Variance of the East error [m²], the same shape.
        cov_NE: Covariance of the North and East errors [m²], the same shape.

    Returns:
        numpy.ndarray: CEP [m], the same shape.

    Examples:
        A circular distribution has CEP = σ·√(2 ln 2) ≈ 1.1774·σ, and a
        one-dimensional one the median of |X|, ≈ 0.6745·σ:

        >>> from ins_sim.evaluation.covariance import gaussian_cep
        >>> [round(float(c), 4) for c in gaussian_cep([1.0, 1.0], [1.0, 0.0], [0.0, 0.0])]
        [1.1774, 0.6745]
    """
    var_N, var_E, cov_NE = (np.asarray(x, dtype=float) for x in (var_N, var_E, cov_NE))
    mean = 0.5 * (var_N + var_E)
    spread = np.sqrt((0.5 * (var_N - var_E)) ** 2 + cov_NE ** 2)
    major = np.maximum(mean + spread, 0.0)
    minor = np.clip(mean - spread, 0.0, None)
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = np.where(major > 0, np.sqrt(minor / np.where(major > 0, major, 1.0)), 1.0)
    r, c = _cep_table()
    return np.sqrt(major) * np.interp(ratio, r, c)


# ---- The linearized error model --------------------------------------------

def _skew(x: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrices [x ∧] of a stack of vectors, shape (..., 3, 3)."""
    S = np.zeros(x.shape[:-1] + (3, 3))
    S[..., 0, 1], S[..., 0, 2] = -x[..., 2], x[..., 1]
    S[..., 1, 0], S[..., 1, 2] = x[..., 2], -x[..., 0]
    S[..., 2, 0], S[..., 2, 1] = -x[..., 1], x[..., 0]
    return S


def _radii_derivatives(L_b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Derivatives of the meridian and transverse radii with latitude [m/rad]."""
    s, c = np.sin(L_b), np.cos(L_b)
    den2 = 1.0 - WGS84_E2 * s * s
    dR_E = WGS84_A * WGS84_E2 * s * c / den2 ** 1.5
    dR_N = 3.0 * WGS84_A * (1.0 - WGS84_E2) * WGS84_E2 * s * c / den2 ** 2.5
    return dR_N, dR_E


def error_dynamics(truth: "TruthTrajectory", idx: np.ndarray,
                   spec: IMUSpec, baro_aiding: bool = True) -> np.ndarray:
    """System matrix F of the linearized error model at samples of the truth.

    The error state x obeys dx/dt = F·x + noise. F is the exact first-order
    expansion of the navigator's continuous-time equations (Groves 2013,
    Chapter 14, NED form), including the variation of the radii of
    curvature and of gravity with position, and the baro-damping loop of
    `strapdown_navgrade`.

    Args:
        truth: Truth trajectory exposing L_b, h_b, v_eb_n, C_b_n,
            omega_ib_b and f_ib_b.
        idx: Indices of the truth samples at which to evaluate F, shape (K,).
        spec: IMU error-parameter set; sets the Gauss-Markov correlation
            times.
        baro_aiding: Whether the baro-damping loop closes the vertical
            channel, as in `run_monte_carlo`. Defaults to True.

    Returns:
        numpy.ndarray: F at each sample, shape (K, 43, 43).
    """
    L = np.asarray(truth.L_b)[idx]; h = np.asarray(truth.h_b)[idx]
    v = np.asarray(truth.v_eb_n)[idx]
    C = truth.C_b_n[idx].as_matrix()
    f_b = np.asarray(truth.f_ib_b)[idx]; w_b = np.asarray(truth.omega_ib_b)[idx]
    K = len(idx)
    vN, vE = v[:, 0], v[:, 1]
    sL, cL, tL = np.sin(L), np.cos(L), np.tan(L)
    R_N, R_E = wgs84_radii(L)
    dR_N, dR_E = _radii_derivatives(L)
    rN, rE = R_N + h, R_E + h
    O = WGS84_OMEGA

    # --8<-- [start:frame_rate_errors]
    # Frame rates and their partial derivatives with respect to the
    # velocity, latitude and height errors.
    omega_ie = np.stack([O * cL, 0 * L, -O * sL], axis=-1)
    omega_en = np.stack([vE / rE, -vN / rN, -vE * tL / rE], axis=-1)
    d_ie_dL = np.stack([-O * sL, 0 * L, -O * cL], axis=-1)
    d_en_dv = np.zeros((K, 3, 3))
    d_en_dv[:, 0, 1] = 1 / rE
    d_en_dv[:, 1, 0] = -1 / rN
    d_en_dv[:, 2, 1] = -tL / rE
    d_en_dL = np.stack([-vE * dR_E / rE**2,
                        vN * dR_N / rN**2,
                        -vE / (cL**2 * rE) + vE * tL * dR_E / rE**2], axis=-1)
    d_en_dh = np.stack([-vE / rE**2, vN / rN**2, vE * tL / rE**2], axis=-1)
    # --8<-- [end:frame_rate_errors]

    # Gravity varies with latitude and height; differentiate the model itself.
    eps = 1e-6
    dg_dL = (normal_gravity(L + eps, h) - normal_gravity(L - eps, h)) / (2 * eps)
    dg_dh = normal_gravity(L, h + 1.0) - normal_gravity(L, h)   # exactly linear in h

    F = np.zeros((K, N_STATES, N_STATES))
    v_skew = _skew(v)
    f_n = np.einsum("kij,kj->ki", C, f_b)

    # --8<-- [start:attitude_error_dynamics]
    # Attitude: d(delta_psi)/dt = −omega_in × delta_psi − delta_omega_in + C_b^n·delta_omega_ib^b
    F[:, PSI, PSI] = -_skew(omega_ie + omega_en)
    F[:, PSI, VEL] = -d_en_dv
    F[:, PSI, LAT] = -(d_ie_dL + d_en_dL)
    F[:, PSI, HGT] = -d_en_dh
    # --8<-- [end:attitude_error_dynamics]

    # --8<-- [start:velocity_error_dynamics]
    # Velocity: d(delta_v)/dt = −(C_b^n f) × delta_psi + C_b^n·delta_f_ib^b
    #   − (2·omega_ie + omega_en) × delta_v + v × (2·delta_omega_ie + delta_omega_en)
    #   + delta_g
    F[:, VEL, PSI] = -_skew(f_n)
    F[:, VEL, VEL] = -_skew(2 * omega_ie + omega_en) + v_skew @ d_en_dv
    F[:, VEL, LAT] = np.einsum("kij,kj->ki", v_skew, 2 * d_ie_dL + d_en_dL)
    F[:, VEL, HGT] = np.einsum("kij,kj->ki", v_skew, d_en_dh)
    F[:, 5, LAT] += dg_dL
    F[:, 5, HGT] += dg_dh
    # --8<-- [end:velocity_error_dynamics]

    # --8<-- [start:position_error_dynamics]
    # Curvilinear position: the derivatives of L_b, lambda_b and h_b rates.
    F[:, LAT, 3] = 1 / rN
    F[:, LAT, LAT] = -vN * dR_N / rN**2
    F[:, LAT, HGT] = -vN / rN**2
    F[:, LON, 4] = 1 / (rE * cL)
    F[:, LON, LAT] = vE * (sL / (rE * cL**2) - dR_E / (rE**2 * cL))
    F[:, LON, HGT] = -vE / (rE**2 * cL)
    F[:, HGT, 5] = -1.0
    # --8<-- [end:position_error_dynamics]

    # --8<-- [start:sensor_error_coupling]
    # Sensor errors: delta_f_ib^b = b_as + b_ad + M_a·f_ib^b, and likewise
    # for the gyros; the white noise enters through the process noise.
    # d(M·x)/dm_ij = x_j along axis i, so C_b^n·M·x has column (i, j) = C[:, i]·x_j.
    F[:, VEL, B_AS] = C
    F[:, VEL, B_AD] = C
    F[:, VEL, M_A] = np.einsum("kai,kj->kaij", C, f_b).reshape(K, 3, 9)
    F[:, PSI, B_GS] = C
    F[:, PSI, B_GD] = C
    F[:, PSI, M_G] = np.einsum("kai,kj->kaij", C, w_b).reshape(K, 3, 9)
    for b, tau in ((B_AD, spec.accel_bias_tau), (B_GD, spec.gyro_bias_tau)):
        F[:, b, b] = -np.eye(3) / tau
    # --8<-- [end:sensor_error_coupling]

    if baro_aiding:
        # --8<-- [start:baro_loop_errors]
        # The height residual rho = delta_h_b − s_b·(h_b − h_b0) − b_bs − b_bd
        # (minus the barometer's white noise) drives the loop:
        # d(delta_h)/dt −= K1·rho, d(delta_v_D)/dt += K2·rho + a_c, da_c/dt = K3·rho.
        K1, K2, K3 = _baro_gains()
        rho = np.zeros((K, N_STATES))
        rho[:, HGT] = 1.0
        if spec.baro is not None:
            rho[:, S_B] = -(h - truth.h_b[0])
            rho[:, B_BS] = -1.0
            rho[:, B_BD] = -1.0
        F[:, HGT] -= K1 * rho
        F[:, 5] += K2 * rho
        F[:, 5, ACC] += 1.0
        F[:, ACC] += K3 * rho
        # --8<-- [end:baro_loop_errors]
    if spec.baro is not None:
        F[:, B_BD, B_BD] = -1.0 / spec.baro.drift_tau
    return F


def _baro_gains() -> tuple[float, float, float]:
    """The baro-damping gains K1, K2, K3 of `strapdown_navgrade`."""
    return 3.0 / _BARO_TAU, 3.0 / _BARO_TAU ** 2, 1.0 / _BARO_TAU ** 3


def _source_terms(spec: IMUSpec, baro_aiding: bool) -> list[tuple[np.ndarray, np.ndarray]]:
    """Initial covariance P0 and process-noise density Q of each error source."""
    terms = []

    def new():
        return np.zeros((N_STATES, N_STATES)), np.zeros((N_STATES, N_STATES))

    def error_matrix(sf_std, ma_std):
        var = np.full((3, 3), ma_std ** 2)
        np.fill_diagonal(var, sf_std ** 2)
        return np.diag(var.ravel())

    # --8<-- [start:source_terms]
    P0, Q = new()                                        # Initial alignment
    P0[PSI, PSI] = np.diag([spec.align_tilt_std, spec.align_tilt_std,
                            spec.align_heading_std]) ** 2
    terms.append((P0, Q))

    P0, Q = new()                                        # Gyro biases
    P0[B_GS, B_GS] = np.eye(3) * spec.gyro_br_std ** 2
    P0[B_GD, B_GD] = np.eye(3) * spec.gyro_gm_std ** 2
    Q[B_GD, B_GD] = np.eye(3) * 2 * spec.gyro_gm_std ** 2 / spec.gyro_bias_tau
    terms.append((P0, Q))

    P0, Q = new()                                        # Gyro scale factor, misalignment
    P0[M_G, M_G] = error_matrix(spec.gyro_sf_std, spec.gyro_ma_std)
    terms.append((P0, Q))

    P0, Q = new()                                        # Accelerometer biases
    P0[B_AS, B_AS] = np.eye(3) * spec.accel_br_std ** 2
    P0[B_AD, B_AD] = np.eye(3) * spec.accel_gm_std ** 2
    Q[B_AD, B_AD] = np.eye(3) * 2 * spec.accel_gm_std ** 2 / spec.accel_bias_tau
    terms.append((P0, Q))

    P0, Q = new()                                        # Accel. scale factor, misalignment
    P0[M_A, M_A] = error_matrix(spec.accel_sf_std, spec.accel_ma_std)
    terms.append((P0, Q))

    # White noise, rotated into NED by C_b^n; its density is the same on
    # every axis, so the rotation leaves it unchanged.
    P0, Q = new()                                        # Sensor noise
    Q[PSI, PSI] = np.eye(3) * spec.gyro_arw ** 2
    Q[VEL, VEL] = np.eye(3) * spec.accel_vrw ** 2
    terms.append((P0, Q))

    P0, Q = new()                                        # Barometer
    baro = spec.baro
    if baro is not None:
        P0[S_B, S_B] = baro.scale_factor_std ** 2
        P0[B_BS, B_BS] = baro.bias_std ** 2
        P0[B_BD, B_BD] = baro.drift_std ** 2
        Q[B_BD, B_BD] = 2 * baro.drift_std ** 2 / baro.drift_tau
        if baro_aiding:
            # The white noise w_b enters the residual with a minus sign.
            K1, K2, K3 = _baro_gains()
            g = np.zeros(N_STATES); g[HGT], g[5], g[ACC] = K1, -K2, -K3
            Q += np.outer(g, g) * baro.noise_density ** 2
    terms.append((P0, Q))
    # --8<-- [end:source_terms]
    return terms


def error_budget(truth: "TruthTrajectory", spec: IMUSpec,
                 baro_aiding: bool = True, step_s: float = 1.0) -> ErrorBudget:
    """Predicts the navigation-error spread, and each error source's share of it.

    Propagates the covariance of the 43-state error model of `error_dynamics`
    along the truth trajectory:

        P(k+1) = Phi·P(k)·Phi^T + Q_d,   Phi = exp(F·step),
        Q_d = (Phi·Q·Phi^T + Q)·step/2,

    with F averaged over the step. Each error source is propagated on its
    own; since the error sources are independent and the model is linear,
    the total is their sum.

    Args:
        truth: Truth trajectory exposing t, tau_i, L_b, lambda_b, h_b,
            v_eb_n, C_b_n, omega_ib_b and f_ib_b.
        spec: IMU error-parameter set, as for `run_monte_carlo`.
        baro_aiding: Whether the barometer aids the vertical channel, as
            for `run_monte_carlo`. Defaults to True.
        step_s: Propagation step [s], rounded to a whole number of truth
            samples (at least one). Defaults to 1.0.

    Returns:
        ErrorBudget: The total prediction and each source's share.

    Examples:
        On a short straight flight, a heading error barely matters (it
        only misresolves accelerations), and the accelerometer biases
        dominate. The sources' variances add up to the total:

        >>> from ins_sim.evaluation.covariance import error_budget
        >>> from ins_sim.sensors.imu import IMUSpec
        >>> from ins_sim.trajectory.kinematics import TruthTrajectory
        >>> from ins_sim.trajectory.spline import NEDSplinePath
        >>> path = NEDSplinePath([[0.0, 0.0, 0.0], [30000.0, 0.0, 0.0]])
        >>> truth = TruthTrajectory.from_spline(path, speed=150.0, tau_i=0.5)
        >>> budget = error_budget(truth, IMUSpec(align_heading_std=1e-3))
        >>> ceps = {name: float(part.cep[-1]) for name, part in budget.by_source.items()}
        >>> max(ceps, key=ceps.get)
        'Accelerometer biases'
        >>> north = sum(part.var[-1, 6] for part in budget.by_source.values())
        >>> round(float(north / budget.total.var[-1, 6]), 9)
        1.0
    """
    stride = max(1, int(round(step_s / truth.tau_i)))
    idx = np.arange(0, len(truth.t), stride)
    step = stride * truth.tau_i
    terms = _source_terms(spec, baro_aiding)
    P = np.stack([P0 for P0, _ in terms])                  # (S, n, n)
    Q = np.stack([Q for _, Q in terms])

    L = np.asarray(truth.L_b)[idx]; h = np.asarray(truth.h_b)[idx]
    R_N, R_E = wgs84_radii(L)
    # Converts (delta_L, delta_lambda, delta_h) to North, East, Down metres.
    scale = np.ones((len(idx), 9))
    scale[:, 6], scale[:, 7], scale[:, 8] = R_N + h, (R_E + h) * np.cos(L), -1.0

    var = np.zeros((len(terms), len(idx), 9))
    cov_NE = np.zeros((len(terms), len(idx)))
    diag = np.arange(9)

    def record(k):
        var[:, k] = P[:, diag, diag] * scale[k] ** 2
        cov_NE[:, k] = P[:, LAT, LON] * scale[k, 6] * scale[k, 7]

    record(0)
    chunk = 512
    for start in range(0, len(idx) - 1, chunk):
        nodes = idx[start:min(start + chunk, len(idx) - 1) + 1]
        F = error_dynamics(truth, nodes, spec, baro_aiding)
        # --8<-- [start:propagation]
        Phi = expm(0.5 * (F[:-1] + F[1:]) * step)           # (n_steps, n, n)
        for j, Phi_j in enumerate(Phi):
            PhiQ = Phi_j @ Q @ Phi_j.T
            P = Phi_j @ P @ Phi_j.T + 0.5 * (PhiQ + Q) * step
            record(start + j + 1)
        # --8<-- [end:propagation]

    t = np.asarray(truth.t)[idx]
    by_source = {name: CovarianceResult(t, var[s], cov_NE[s])
                 for s, name in enumerate(ERROR_SOURCES)}
    total = CovarianceResult(t, var.sum(axis=0), cov_NE.sum(axis=0))
    return ErrorBudget(total, by_source)


def covariance_analysis(truth: "TruthTrajectory", spec: IMUSpec,
                        baro_aiding: bool = True,
                        step_s: float = 1.0) -> CovarianceResult:
    """Predicts the spread of the navigation errors with a linearized error model.

    The prediction a Monte Carlo run with `run_monte_carlo` converges to as
    the number of trials grows, as long as the errors stay small enough for
    the model to be linear. See `error_budget` for the method.

    Args:
        truth: Truth trajectory, as for `run_monte_carlo`.
        spec: IMU error-parameter set.
        baro_aiding: Whether the barometer aids the vertical channel.
            Defaults to True.
        step_s: Propagation step [s]. Defaults to 1.0.

    Returns:
        CovarianceResult: The predicted variances over time.
    """
    return error_budget(truth, spec, baro_aiding, step_s).total
