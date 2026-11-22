"""Validation: the covariance analysis against the navigator and the Monte Carlo.

Each test runs one scenario from tools/validation_scenarios.py (the same
scenarios the Validation page uses), on a 20-minute flight with turns both
ways and a climb, with every error source of the packaged IMU spec.
"""

import numpy as np
import pytest

from ins_sim.config import default_imu_spec_path
from ins_sim.sensors.imu import load_imu_spec
from tools.validation_scenarios import (covariance_vs_monte_carlo,
                                        linear_model_single_trial, turning_truth)


@pytest.fixture(scope="module")
def spec():
    return load_imu_spec(str(default_imu_spec_path()))


def test_the_linear_model_follows_one_trial_of_the_navigator(spec):
    """Fed the same constant errors, the linear model and the nonlinear
    navigator agree to first order: within 1% of each error's largest value."""
    r = linear_model_single_trial(turning_truth(0.1), spec)
    largest = np.abs(r.navigator).max(axis=0)
    assert np.all(np.abs(r.navigator - r.linear).max(axis=0) < 0.01 * largest)


def test_the_monte_carlo_spread_matches_the_predicted_spread(spec):
    """At 5-minute checkpoints, all nine errors' Monte Carlo spreads lie within
    the 95% sampling band of the prediction, and the predicted CEP lies
    within the 95% confidence interval of the Monte Carlo median."""
    r = covariance_vs_monte_carlo(turning_truth(1.0), spec, n_trials=100)
    k = np.searchsorted(r.t, [300.0, 600.0, 900.0, 1200.0])
    ratio = r.sigma_mc[k] / r.sigma_cov[k]
    assert np.all(np.abs(ratio - 1.0) < r.band)
    assert np.all((r.cep_lo[k] <= r.cep_cov[k]) & (r.cep_cov[k] <= r.cep_hi[k]))
