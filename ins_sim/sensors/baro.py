"""Barometric altimeter error model: the height measurement that aids the vertical channel.

A pressure altimeter converts static pressure to height through a standard
atmosphere, so its errors come from the atmosphere differing from that
standard, not from the sensor alone (Groves 2013, Chapter 6). A `BaroSpec`
sets the spread of each error term in the model

    meas_h_b = h_b + s_b·(h_b − h_b0) + b_bs + b_bd(t) + w_b

and `generate_baro_samples` draws one realization of it along a truth
trajectory. The model is explained on the [Vertical channel][barometer-error-model]
theory page.
"""

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

from ins_sim.sensors.noise import gauss_markov

if TYPE_CHECKING:
    from ins_sim.trajectory.kinematics import TruthTrajectory


@dataclass
class BaroSpec:
    """Barometric altimeter error parameters, in SI units.

    Every spread defaults to zero, so `BaroSpec()` is an ideal barometer
    that reports the true height.

    Attributes:
        scale_factor_std: Std dev of the scale-factor error s_b, drawn once
            per run [dimensionless, e.g. 0.02 = 2%]. Models a non-standard
            air temperature: the altimeter's height above the departure
            point is off by s_b times that height.
        bias_std: Std dev of the static bias b_bs, drawn once per run [m].
            Models the error in the altimeter setting (QNH) and the
            sensor's own offset.
        drift_std: Steady-state std dev of the drift b_bd, a first-order
            Gauss-Markov process [m]. Models the weather: the sea-level
            pressure changing along the route and over time.
        drift_tau: Correlation time of the drift [s], above 0.
        noise_density: Density of the white noise w_b [m·√s, i.e. m/√Hz].
            The per-sample std dev is noise_density/√tau_i, so the noise
            the vertical channel sees does not depend on the time step.
    """

    scale_factor_std: float = 0.0
    bias_std: float = 0.0
    drift_std: float = 0.0
    drift_tau: float = 3600.0
    noise_density: float = 0.0


def baro_spec_from_yaml(cfg: dict[str, Any]) -> BaroSpec:
    """Converts the `baro:` block of an IMU spec YAML file to a BaroSpec.

    Args:
        cfg: The parsed block, in datasheet units. Every key is optional
            and defaults to no error: `scale_factor_pct` [%], `bias_m` [m],
            `drift_m` [m], `drift_tau_s` [s, default 3600] and
            `noise_m_per_rt_hz` [m/√Hz].

    Returns:
        BaroSpec: The same parameters in SI units.

    Examples:
        >>> from ins_sim.sensors.baro import baro_spec_from_yaml
        >>> baro_spec_from_yaml({"scale_factor_pct": 2.0, "bias_m": 10.0})
        BaroSpec(scale_factor_std=0.02, bias_std=10.0, drift_std=0.0, drift_tau=3600.0, noise_density=0.0)
    """
    # --8<-- [start:datasheet_units]
    return BaroSpec(
        scale_factor_std = cfg.get("scale_factor_pct", 0.0) / 100.0,
        bias_std         = cfg.get("bias_m", 0.0),
        drift_std        = cfg.get("drift_m", 0.0),
        drift_tau        = cfg.get("drift_tau_s", 3600.0),
        noise_density    = cfg.get("noise_m_per_rt_hz", 0.0),
    )
    # --8<-- [end:datasheet_units]


def generate_baro_samples(truth: "TruthTrajectory", spec: BaroSpec,
                          rng: np.random.Generator) -> np.ndarray:
    """Generates barometric height measurements from a truth trajectory.

    Measurement model:

        meas_h_b = h_b + s_b·(h_b − h_b0) + b_bs + b_bd(t) + w_b

    - s_b: scale-factor error ~ N(0, scale_factor_std²), drawn once per
      run. It scales the height above the departure point h_b0, where the
      altimeter setting makes the barometer read correctly apart from b_bs.
    - b_bs: static bias ~ N(0, bias_std²), drawn once per run.
    - b_bd: drift, a stationary first-order Gauss-Markov process with
      spread drift_std and correlation time drift_tau.
    - w_b: white noise, with σ = noise_density/√tau_i per sample.

    The draws come from `rng` in a fixed order (s_b, b_bs, the drift, w_b)
    whatever the parameter values, so changing one spread never reshuffles
    the others.

    Args:
        truth: Truth trajectory exposing `h_b` (shape (M,) [m]) and `tau_i`.
        spec: Barometer error-parameter set.
        rng: Seeded NumPy random generator.

    Returns:
        numpy.ndarray: Measured height meas_h_b, shape (M,) [m].

    Examples:
        An ideal barometer reports the true height:

        >>> import numpy as np
        >>> from types import SimpleNamespace
        >>> from ins_sim.sensors.baro import BaroSpec, generate_baro_samples
        >>> truth = SimpleNamespace(h_b=np.array([100.0, 150.0, 200.0]), tau_i=1.0)
        >>> generate_baro_samples(truth, BaroSpec(), np.random.default_rng(0))
        array([100., 150., 200.])
    """
    h_b = np.asarray(truth.h_b, dtype=float)
    tau_i = truth.tau_i
    # --8<-- [start:baro_errors]
    s_b = rng.normal(0.0, spec.scale_factor_std)
    b_bs = rng.normal(0.0, spec.bias_std)
    b_bd = gauss_markov(rng, len(h_b), tau_i, spec.drift_std, spec.drift_tau,
                        n_axes=1)[:, 0]
    w_b = rng.normal(0.0, spec.noise_density / np.sqrt(tau_i), size=len(h_b))
    return h_b + s_b * (h_b - h_b[0]) + b_bs + b_bd + w_b
    # --8<-- [end:baro_errors]
