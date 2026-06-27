from dataclasses import dataclass

import numpy as np
import yaml


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
        accel_vrw: Accelerometer velocity random walk [(m/s)/√s].
        accel_bi_std: Accelerometer bias-instability steady-state std
            dev of the Gauss-Markov process [m/s²].
        accel_bi_tau: Accelerometer bias-instability Gauss-Markov
            correlation time [s].
        accel_br_std: Accelerometer turn-on bias repeatability std
            dev [m/s²].
    """
    # Gyro
    gyro_arw:    float = 0.002 * np.pi/180 / 60          # rad / √s
    gyro_bi_std: float = 0.01  * np.pi/180 / 3600        # rad / s
    gyro_bi_tau: float = 3600.0                          # GM correlation time [s]
    gyro_br_std: float = 0.01  * np.pi/180 / 3600        # rad / s
    # Accel
    accel_vrw:    float = 0.005 / 60                     # (m/s) / √s
    accel_bi_std: float =  5e-6 * 9.80665                # 5 µg
    accel_bi_tau: float = 3600.0
    accel_br_std: float = 25e-6 * 9.80665                # 25 µg


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
    DEG_HR = np.pi / 180 / 3600
    return IMUSpec(
        gyro_arw     = g["arw_deg_per_rt_hr"]          * np.pi / 180 / 60,
        gyro_bi_std  = g["bias_instab_deg_per_hr"]     * DEG_HR,
        gyro_bi_tau  = g["bias_tau_s"],
        gyro_br_std  = g["repeatability_deg_per_hr"]   * DEG_HR,
        accel_vrw    = a["vrw_m_per_s_per_rt_hr"]      / 60,
        accel_bi_std = a["bias_instab_ug"]              * 1e-6 * 9.80665,
        accel_bi_tau = a["bias_tau_s"],
        accel_br_std = a["repeatability_ug"]            * 1e-6 * 9.80665,
    )


def generate_imu_samples(truth, spec: IMUSpec,
                         rng: np.random.Generator):
    """Generates noisy IMU samples from a truth trajectory and an error spec.

    Per-axis IMU model:
        measurement = truth + b_repeat + b_drift(t) + η_white(t)

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

    # Constant turn-on biases (per axis, per run)
    b_g_repeat = rng.normal(0.0, spec.gyro_br_std,  size=3)
    b_a_repeat = rng.normal(0.0, spec.accel_br_std, size=3)

    # Gauss-Markov drift histories
    a_g = np.exp(-dt / spec.gyro_bi_tau)
    a_a = np.exp(-dt / spec.accel_bi_tau)
    sg  = np.sqrt(1.0 - a_g * a_g) * spec.gyro_bi_std
    sa  = np.sqrt(1.0 - a_a * a_a) * spec.accel_bi_std
    b_g_drift = np.zeros((M, 3))
    b_a_drift = np.zeros((M, 3))
    for k in range(1, M):
        b_g_drift[k] = a_g * b_g_drift[k-1] + sg * rng.normal(size=3)
        b_a_drift[k] = a_a * b_a_drift[k-1] + sa * rng.normal(size=3)

    # White noise: ARW/√dt is the discrete-time σ that yields integrated
    # angle uncertainty growing as ARW · √t, independent of sample rate.
    eta_g = rng.normal(0.0, spec.gyro_arw  / np.sqrt(dt), size=(M, 3))
    eta_a = rng.normal(0.0, spec.accel_vrw / np.sqrt(dt), size=(M, 3))

    omega_meas = truth.omega_b + b_g_repeat + b_g_drift + eta_g
    f_meas     = truth.f_b     + b_a_repeat + b_a_drift + eta_a
    return omega_meas, f_meas
