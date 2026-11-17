from dataclasses import dataclass
from typing import TYPE_CHECKING, NamedTuple

import numpy as np
import yaml
from scipy.signal import lfilter

if TYPE_CHECKING:
    from ins_sim.trajectory.kinematics import TruthTrajectory


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

    The field names are datasheet parameters; each one sets the spread of
    a Groves (2013) error term (see the Notation & conventions page).

    Attributes:
        gyro_arw: Gyro angular random walk, the density of the white
            noise w_g [rad/√s].
        gyro_bi_std: Gyro bias-instability steady-state std dev of the
            Gauss-Markov dynamic bias b_gd [rad/s].
        gyro_bi_tau: Gyro bias-instability Gauss-Markov correlation
            time [s].
        gyro_br_std: Gyro turn-on bias repeatability, the std dev of the
            static bias b_gs [rad/s].
        gyro_sf_std: Gyro scale-factor error std dev (diagonal of M_g),
            drawn once per run per axis [dimensionless, e.g. 5e-6 = 5 ppm].
        gyro_ma_std: Gyro axis-misalignment std dev (off-diagonal of M_g),
            drawn once per run per element [rad].
        accel_vrw: Accelerometer velocity random walk, the density of the
            white noise w_a [(m/s)/√s].
        accel_bi_std: Accelerometer bias-instability steady-state std
            dev of the Gauss-Markov dynamic bias b_ad [m/s²].
        accel_bi_tau: Accelerometer bias-instability Gauss-Markov
            correlation time [s].
        accel_br_std: Accelerometer turn-on bias repeatability, the std
            dev of the static bias b_as [m/s²].
        accel_sf_std: Accelerometer scale-factor error std dev (diagonal
            of M_a) [dimensionless].
        accel_ma_std: Accelerometer axis-misalignment std dev
            (off-diagonal of M_a) [rad].
        align_tilt_std: Initial-alignment tilt (N/E) error std dev [rad].
            Physically ≈ accel repeatability / g.
        align_heading_std: Initial-alignment heading (azimuth) error std
            dev [rad]. Physically ≈ gyrocompass limit, gyro repeatability
            / (omega_ie·cos L_b).

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


class IMUSamples(NamedTuple):
    """Simulated IMU outputs, one row per sample interval tau_i.

    Attributes:
        meas_omega_ib_b: Gyro output, the measured angular rate of the body
            relative to inertial space, resolved in body axes, shape
            (M, 3) [rad/s].
        meas_f_ib_b: Accelerometer output, the measured specific force,
            resolved in body axes, shape (M, 3) [m/s²].
    """

    meas_omega_ib_b: np.ndarray
    meas_f_ib_b: np.ndarray


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
    """Draws a per-run initial-alignment attitude error delta_psi_nb^n.

    Models the residual error of a stationary gyrocompass alignment:
    small tilts about North/East (limited by accelerometer bias over g)
    and a larger azimuth error about Down (limited by east-gyro bias
    over omega_ie·cos L_b). Apply as C_b^n(estimated) =
    exp([delta_psi_nb^n ^])·C_b^n(true).

    Args:
        spec: IMU error-parameter set (align_tilt_std,
            align_heading_std).
        rng: Seeded NumPy random generator.

    Returns:
        numpy.ndarray: Attitude error rotation vector resolved in NED,
            [North tilt, East tilt, Down (heading)], shape (3,) [rad].
    """
    return rng.normal(0.0, [spec.align_tilt_std,
                            spec.align_tilt_std,
                            spec.align_heading_std])


def _triad_measurement(rng: np.random.Generator, truth_triad: np.ndarray,
                       tau_i: float, sf_std: float, ma_std: float,
                       bs_std: float, bd_std: float, bd_tau: float,
                       noise_density: float) -> np.ndarray:
    """Applies the error model to one sensor triad (gyros or accelerometers).

    Draws, in this fixed order from the triad's own random stream: the
    scale-factor/misalignment matrix M, the static bias b_s, the initial
    dynamic bias b_d[0], the Gauss-Markov driving noise, and the white
    noise w. Every draw happens whatever the parameter values, so changing
    one spread never reshuffles the others.

    Args:
        rng: The triad's random generator.
        truth_triad: True sensed vector, shape (M, 3).
        tau_i: Sample interval [s].
        sf_std: Scale-factor error std dev [dimensionless].
        ma_std: Axis-misalignment std dev [rad].
        bs_std: Static (turn-on) bias std dev.
        bd_std: Dynamic (Gauss-Markov) bias steady-state std dev.
        bd_tau: Gauss-Markov correlation time [s].
        noise_density: White-noise density (ARW or VRW), per √s.

    Returns:
        numpy.ndarray: Measured triad b_s + b_d + (I + M)·truth + w, shape
            (M, 3).
    """
    n_samples = len(truth_triad)

    # --8<-- [start:scale_misalignment]
    # Per-run multiplicative errors (scale factor + misalignment). With
    # zero stds these are exact identity transforms, preserving the
    # zero-noise passthrough property.
    M = _error_matrix(rng, sf_std, ma_std)
    scaled = truth_triad @ (np.eye(3) + M).T          # (I + M)·truth
    # --8<-- [end:scale_misalignment]

    # --8<-- [start:static_bias]
    # Static (turn-on) bias, per axis, constant over the run
    b_s = rng.normal(0.0, bs_std, size=3)
    # --8<-- [end:static_bias]

    # --8<-- [start:dynamic_bias]
    # Dynamic (Gauss-Markov) bias: discrete AR(1) recursion
    #   b_d[k+1] = a·b_d[k] + √(1−a²)·σ·u[k+1],   a = exp(−tau_i/τ),
    # started in its stationary distribution, b_d[0] ~ N(0, σ²), so its
    # variance is σ² from the first sample on. The IIR filter produces the
    # zero-start response to u; the decaying initial state is added to it.
    a = np.exp(-tau_i / bd_tau)
    s = np.sqrt(1.0 - a * a) * bd_std
    b_d0 = rng.normal(0.0, bd_std, size=3)
    u = rng.normal(size=(n_samples, 3)); u[0] = 0.0
    b_d = (lfilter([1.0], [1.0, -a], s * u, axis=0)
           + a ** np.arange(n_samples)[:, None] * b_d0)
    # --8<-- [end:dynamic_bias]

    # --8<-- [start:white_noise]
    # White noise: density/√tau_i is the discrete-time σ that makes the
    # integrated angle (velocity) uncertainty grow as density·√t,
    # independent of the sample rate.
    w = rng.normal(0.0, noise_density / np.sqrt(tau_i), size=(n_samples, 3))
    # --8<-- [end:white_noise]

    return scaled + b_s + b_d + w


def generate_imu_samples(truth: "TruthTrajectory", spec: IMUSpec,
                         rng: np.random.Generator) -> IMUSamples:
    """Generates noisy IMU samples from a truth trajectory and an error spec.

    Per-axis IMU model (Groves notation):
        meas_f_ib_b     = b_a + (I + M_a)·f_ib_b     + w_a
        meas_omega_ib_b = b_g + (I + M_g)·omega_ib_b + w_g

      M_a, M_g   scale factor (diagonal) + misalignment (off-diagonal),
                 drawn once per run — see _error_matrix
      b_as, b_gs static (turn-on) bias ~ N(0, σ_BR²), drawn once per run
      b_ad, b_gd dynamic bias: 1st-order Gauss-Markov, steady-state std
                 σ_BI, correlation time τ_BI, stationary from t = 0
      w_a, w_g   white noise, σ = VRW/√tau_i or ARW/√tau_i per sample

    The gyro and accelerometer triads draw from separate random streams
    spawned from `rng`, so changing one triad's parameters leaves the
    other's error realization unchanged.

    Args:
        truth: Truth trajectory exposing `t`, `tau_i`, `omega_ib_b`
            (shape (M, 3) [rad/s]) and `f_ib_b` (shape (M, 3) [m/s²]).
        spec: IMU error-parameter set.
        rng: Seeded NumPy random generator. Two child streams are spawned
            from it, one per sensor triad.

    Returns:
        IMUSamples: (meas_omega_ib_b, meas_f_ib_b), the noisy gyro and
            accelerometer outputs, each shape (M, 3).
    """
    tau_i = truth.tau_i
    rng_gyro, rng_accel = rng.spawn(2)
    meas_omega_ib_b = _triad_measurement(
        rng_gyro, truth.omega_ib_b, tau_i, spec.gyro_sf_std, spec.gyro_ma_std,
        spec.gyro_br_std, spec.gyro_bi_std, spec.gyro_bi_tau, spec.gyro_arw)
    meas_f_ib_b = _triad_measurement(
        rng_accel, truth.f_ib_b, tau_i, spec.accel_sf_std, spec.accel_ma_std,
        spec.accel_br_std, spec.accel_bi_std, spec.accel_bi_tau, spec.accel_vrw)
    return IMUSamples(meas_omega_ib_b, meas_f_ib_b)
