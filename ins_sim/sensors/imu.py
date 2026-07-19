from dataclasses import dataclass

import numpy as np
import yaml
from scipy.signal import lfilter


@dataclass
class IMUSpec:
    """Navigation-grade IMU error parameters.

    Reference numbers (typical fielded systems, e.g. ring-laser-gyro
    aircraft INS, marine-grade strapdown):
        Gyro ARW            ≈ 0.002 °/√hr
        Gyro bias instab.   ≈ 0.01  °/hr
        Gyro repeatability  ≈ 0.01  °/hr
        Accel VRW           ≈ 0.005 m/s/√hr
        Accel bias instab.  ≈ 5  µg
        Accel repeatability ≈ 25 µg

    Attributes:
        gyro_arw: Gyro angular random walk [rad/√s].
        gyro_bi_std: Gyro bias-instability steady-state std dev of the
            Gauss-Markov process [rad/s].
        gyro_bi_tau: Gyro bias-instability Gauss-Markov correlation
            time [s].
        gyro_br_std: Gyro turn-on bias repeatability std dev [rad/s].
        gyro_sf_std: Gyro scale-factor error std dev, drawn once per
            run per axis [dimensionless, e.g. 5e-6 = 5 ppm].
        gyro_ma_std: Gyro axis-misalignment std dev, drawn once per
            run per off-diagonal element [rad].
        accel_vrw: Accelerometer velocity random walk [(m/s)/√s].
        accel_bi_std: Accelerometer bias-instability steady-state std
            dev of the Gauss-Markov process [m/s²].
        accel_bi_tau: Accelerometer bias-instability Gauss-Markov
            correlation time [s].
        accel_br_std: Accelerometer turn-on bias repeatability std
            dev [m/s²].
        accel_sf_std: Accelerometer scale-factor error std dev
            [dimensionless].
        accel_ma_std: Accelerometer axis-misalignment std dev [rad].
        align_tilt_std: Initial-alignment tilt (N/E) misalignment std
            dev [rad]. Physically ≈ accel repeatability / g.
        align_heading_std: Initial-alignment heading (azimuth)
            misalignment std dev [rad]. Physically ≈ gyrocompass
            limit, gyro repeatability / (Ω_ie·cos φ).

    Note:
        The multiplicative (scale factor, misalignment) and
        initial-alignment terms default to zero — they apply only when
        specified — so zero-noise diagnostic specs and legacy YAMLs
        keep their exact-passthrough behavior.
    """
    # Gyro
    gyro_arw:    float = 0.002 * np.pi/180 / 60          # rad / √s
    gyro_bi_std: float = 0.01  * np.pi/180 / 3600        # rad / s
    gyro_bi_tau: float = 3600.0                          # GM correlation time [s]
    gyro_br_std: float = 0.01  * np.pi/180 / 3600        # rad / s
    gyro_sf_std: float = 0.0                             # dimensionless
    gyro_ma_std: float = 0.0                             # rad
    # Accel
    accel_vrw:    float = 0.005 / 60                     # (m/s) / √s
    accel_bi_std: float =  5e-6 * 9.80665                # 5 µg
    accel_bi_tau: float = 3600.0
    accel_br_std: float = 25e-6 * 9.80665                # 25 µg
    accel_sf_std: float = 0.0                            # dimensionless
    accel_ma_std: float = 0.0                            # rad
    # Initial alignment
    align_tilt_std:    float = 0.0                       # rad
    align_heading_std: float = 0.0                       # rad


def load_imu_spec(yaml_path: str) -> IMUSpec:
    """Loads an IMUSpec from a YAML file expressed in datasheet units.

    Args:
        yaml_path: Path to a YAML file with `gyro` and `accel` sections
            in datasheet units (deg/√hr, deg/hr, µg, etc.).

    Returns:
        IMUSpec: Equivalent error-parameter set converted to SI units.
    """
    with open(yaml_path) as fh:
        cfg = yaml.safe_load(fh)
    g = cfg["gyro"]
    a = cfg["accel"]
    al = cfg.get("alignment", {})
    DEG_HR = np.pi / 180 / 3600
    return IMUSpec(
        gyro_arw     = g["arw_deg_per_rt_hr"]          * np.pi / 180 / 60,
        gyro_bi_std  = g["bias_instab_deg_per_hr"]     * DEG_HR,
        gyro_bi_tau  = g["bias_tau_s"],
        gyro_br_std  = g["repeatability_deg_per_hr"]   * DEG_HR,
        gyro_sf_std  = g.get("scale_factor_ppm", 0.0)   * 1e-6,
        gyro_ma_std  = g.get("misalignment_urad", 0.0)  * 1e-6,
        accel_vrw    = a["vrw_m_per_s_per_rt_hr"]      / 60,
        accel_bi_std = a["bias_instab_ug"]              * 1e-6 * 9.80665,
        accel_bi_tau = a["bias_tau_s"],
        accel_br_std = a["repeatability_ug"]            * 1e-6 * 9.80665,
        accel_sf_std = a.get("scale_factor_ppm", 0.0)   * 1e-6,
        accel_ma_std = a.get("misalignment_urad", 0.0)  * 1e-6,
        align_tilt_std    = al.get("tilt_std_deg", 0.0)    * np.pi / 180,
        align_heading_std = al.get("heading_std_deg", 0.0) * np.pi / 180,
    )


def _error_matrix(rng: np.random.Generator, sf_std: float,
                  ma_std: float) -> np.ndarray:
    """Draws a per-run multiplicative sensor-error matrix M.

    The sensed vector is (I + M)·truth: the diagonal of M holds
    per-axis scale-factor errors, the off-diagonal elements hold axis
    misalignments (6 independent small angles).

    Args:
        rng: Seeded NumPy random generator.
        sf_std: Scale-factor error std dev [dimensionless].
        ma_std: Axis-misalignment std dev [rad].

    Returns:
        numpy.ndarray: Error matrix M, shape (3, 3).
    """
    M = np.diag(rng.normal(0.0, sf_std, size=3))
    off = rng.normal(0.0, ma_std, size=6)
    M[0, 1], M[0, 2], M[1, 0], M[1, 2], M[2, 0], M[2, 1] = off
    return M


def draw_initial_misalignment(spec: IMUSpec,
                              rng: np.random.Generator) -> np.ndarray:
    """Draws a per-run initial-alignment attitude error rotation vector.

    Models the residual error of a stationary gyrocompass alignment:
    small tilts about North/East (limited by accelerometer bias over g)
    and a larger azimuth error about Down (limited by east-gyro bias
    over Ω_ie·cos φ). Apply as R_init = exp([ψ×])·R_true with ψ in NED.

    Args:
        spec: IMU error-parameter set (align_tilt_std,
            align_heading_std).
        rng: Seeded NumPy random generator.

    Returns:
        numpy.ndarray: NED rotation vector [δ_N, δ_E, δ_D], shape
            (3,) [rad].
    """
    return rng.normal(0.0, [spec.align_tilt_std,
                            spec.align_tilt_std,
                            spec.align_heading_std])


def generate_imu_samples(truth, spec: IMUSpec,
                         rng: np.random.Generator):
    """Generates noisy IMU samples from a truth trajectory and an error spec.

    Per-axis IMU model:
        measurement = (I + M)·truth + b_repeat + b_drift(t) + η_white(t)

      M          scale factor (diag) + misalignment (off-diag),
                 drawn once per run — see _error_matrix
      b_repeat   ~ N(0, σ_BR²)               drawn once per run
      b_drift    1st-order Gauss-Markov, steady-state std σ_BI, τ_BI
      η_white    σ = ARW/√dt or VRW/√dt      per-sample white noise

    Discrete AR(1) form for the GM drift gives steady-state variance σ²
    independent of dt:    b[k+1] = a·b[k] + √(1−a²)·σ·w,   a = exp(−dt/τ).

    Args:
        truth: Truth trajectory object exposing `t`, `dt`, `omega_b`
            (ω_ib_b, shape (M, 3) [rad/s]), and `f_b` (shape (M, 3)
            [m/s²]).
        spec: IMU error-parameter set.
        rng: Seeded NumPy random generator.

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]: (omega_meas, f_meas), the
            noisy gyro and accelerometer outputs, each shape (M, 3).
    """
    M, dt = len(truth.t), truth.dt

    # Per-run multiplicative errors (scale factor + misalignment). With
    # zero stds these are exact identity transforms, preserving the
    # zero-noise passthrough property.
    M_g = _error_matrix(rng, spec.gyro_sf_std,  spec.gyro_ma_std)
    M_a = _error_matrix(rng, spec.accel_sf_std, spec.accel_ma_std)
    omega_true = truth.omega_b @ (np.eye(3) + M_g).T
    f_true     = truth.f_b     @ (np.eye(3) + M_a).T

    # Constant turn-on biases (per axis, per run)
    b_g_repeat = rng.normal(0.0, spec.gyro_br_std,  size=3)
    b_a_repeat = rng.normal(0.0, spec.accel_br_std, size=3)

    # Gauss-Markov drift histories: discrete AR(1) recursion b[k] = a*b[k-1]
    # + sg*w[k] with b[0] = 0, implemented as an IIR filter over white noise
    # that is zeroed at index 0 (so the filter's zero initial condition
    # reproduces b[0] = 0 exactly).
    a_g = np.exp(-dt / spec.gyro_bi_tau)
    a_a = np.exp(-dt / spec.accel_bi_tau)
    sg  = np.sqrt(1.0 - a_g * a_g) * spec.gyro_bi_std
    sa  = np.sqrt(1.0 - a_a * a_a) * spec.accel_bi_std
    w_g = rng.normal(size=(M, 3)); w_g[0] = 0.0
    w_a = rng.normal(size=(M, 3)); w_a[0] = 0.0
    b_g_drift = lfilter([1.0], [1.0, -a_g], sg * w_g, axis=0)
    b_a_drift = lfilter([1.0], [1.0, -a_a], sa * w_a, axis=0)

    # White noise: ARW/√dt is the discrete-time σ that yields integrated
    # angle uncertainty growing as ARW · √t, independent of sample rate.
    eta_g = rng.normal(0.0, spec.gyro_arw  / np.sqrt(dt), size=(M, 3))
    eta_a = rng.normal(0.0, spec.accel_vrw / np.sqrt(dt), size=(M, 3))

    omega_meas = omega_true + b_g_repeat + b_g_drift + eta_g
    f_meas     = f_true     + b_a_repeat + b_a_drift + eta_a
    return omega_meas, f_meas
