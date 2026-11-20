"""Validation: barometer errors passing through the vertical channel's damping loop.

Each test runs one scenario from tools/validation_scenarios.py (the same
scenarios the Validation page plots) and compares the INS height error with
the loop's linearized transfer function from barometer error to height error.
"""

import numpy as np
import pytest

from tools.validation_scenarios import baro_bias, baro_drift, baro_noise


def test_a_barometer_bias_ends_up_in_full_in_the_ins_height():
    r = baro_bias()
    assert r.end_measured == pytest.approx(r.bias, rel=1e-3)
    assert np.abs(r.simulated - r.predicted).max() < 1e-2 * r.bias


def test_the_ins_height_tracks_a_barometer_drift_through_the_loop():
    r = baro_drift()
    assert r.max_difference < 1e-2 * np.abs(r.drift).max()
    assert r.lag_rms_measured == pytest.approx(r.lag_rms_predicted, rel=0.05)


def test_the_loop_attenuates_white_barometer_noise_as_predicted():
    r = baro_noise(n_runs=30, duration_s=2000.0)
    assert r.measured == pytest.approx(r.predicted, rel=0.08)
    assert r.measured < 0.2 * r.per_sample
