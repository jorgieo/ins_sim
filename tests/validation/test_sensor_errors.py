"""Validation: the sensor noise models against their defining statistics."""

import numpy as np
import pytest

from tools.validation_scenarios import (angle_random_walk, gauss_markov,
                                        gauss_markov_allan_variance, gyro_allan)


@pytest.mark.parametrize("tau_i", [0.1, 1.0])
def test_angle_random_walk_grows_as_root_t_at_any_sample_rate(tau_i):
    r = angle_random_walk(tau_i=tau_i)
    ratio = r.simulated / r.predicted
    late = r.t > r.t[-1] / 2
    assert ratio[late].mean() == pytest.approx(1.0, abs=0.03)
    assert ratio[-1] == pytest.approx(1.0, abs=0.06)


def test_gauss_markov_bias_is_stationary_and_exponentially_correlated():
    r = gauss_markov()
    assert r.spread[0] == pytest.approx(r.sigma, rel=0.05)
    assert r.spread[-1] == pytest.approx(r.sigma, rel=0.05)
    at_tau_c = np.interp(600.0, r.lag, r.autocorr)
    assert at_tau_c == pytest.approx(np.exp(-1.0), abs=0.06)


def test_gauss_markov_allan_variance_formula_has_the_textbook_peak():
    tau_c, sigma = 3600.0, 1.0
    T = np.logspace(1, 5, 20000)
    adev = np.sqrt(gauss_markov_allan_variance(T, sigma, tau_c))
    k = int(np.argmax(adev))
    assert adev[k] == pytest.approx(0.618, abs=0.002)
    assert T[k] / tau_c == pytest.approx(1.89, abs=0.02)


def test_simulated_gyro_allan_deviation_matches_the_noise_model():
    r = gyro_allan(duration_h=60.0, n_runs=4)
    # The datasheet bias instability is the peak of the Gauss-Markov term.
    assert r.gm_peak == pytest.approx(r.bias_instability, rel=1e-6)
    ratio = r.simulated / r.predicted
    assert ratio[np.argmin(np.abs(r.T - 1.0))] == pytest.approx(1.0, abs=0.03)
    assert ratio[np.argmin(np.abs(r.T - r.gm_peak_T))] == pytest.approx(1.0, abs=0.10)
