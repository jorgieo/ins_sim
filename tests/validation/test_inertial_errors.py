"""Validation: inertial error propagation against textbook predictions.

Each test runs one scenario from tools/validation_scenarios.py (the same
scenarios the Validation page plots) and asserts agreement with the
analytic prediction.
"""

import numpy as np
import pytest

from tools.validation_scenarios import (accel_bias, gyro_bias, schuler_tilt,
                                        vertical_channel, zero_noise_round_trip)


def test_zero_noise_round_trip_on_the_packaged_mission():
    assert zero_noise_round_trip().max_error < 1e-6


def test_schuler_oscillation_from_an_initial_tilt():
    r = schuler_tilt()
    assert r.period_measured == pytest.approx(r.period_predicted, rel=2e-3)
    assert r.peak_measured == pytest.approx(r.peak_predicted, rel=1e-2)
    assert np.abs(r.simulated - r.predicted).max() < 1e-2 * r.peak_predicted


def test_accelerometer_bias_gives_a_bounded_schuler_oscillation():
    r = accel_bias()
    assert r.peak_measured == pytest.approx(r.peak_predicted, rel=1e-2)
    assert np.abs(r.simulated - r.predicted).max() < 1e-2 * r.peak_predicted


def test_gyro_bias_gives_a_growing_error_bent_by_the_earth_rate_loop():
    r = gyro_bias()
    scale = np.abs(r.predicted).max()
    assert np.abs(r.simulated - r.predicted).max() < 5e-3 * scale
    # The single-channel formula, which ignores Earth rate, overstates the
    # error by about 10% after three hours: the coupling is real physics.
    assert abs(r.single_channel[-1]) > 1.05 * abs(r.end_measured)


def test_vertical_channel_diverges_free_inertial_and_is_held_by_the_baro_loop():
    r = vertical_channel()
    assert r.tau_v_measured == pytest.approx(r.tau_v_predicted, rel=5e-3)
    assert r.free[-1] == pytest.approx(r.predicted[-1], rel=1e-2)
    assert np.abs(r.aided).max() < 60.0
    assert abs(r.aided[-1]) < 1.0
