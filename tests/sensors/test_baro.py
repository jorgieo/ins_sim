from types import SimpleNamespace

import numpy as np
import pytest

from ins_sim.config import default_imu_spec_path
from ins_sim.sensors.baro import BaroSpec, baro_spec_from_yaml, generate_baro_samples
from ins_sim.sensors.imu import load_imu_spec
from ins_sim.sensors.noise import gauss_markov


def _climb(M=4000, tau_i=0.5, h0=100.0, rate=2.0):
    """A truth that climbs steadily from h0."""
    return SimpleNamespace(h_b=h0 + rate * tau_i * np.arange(M), tau_i=tau_i)


def test_ideal_barometer_reports_the_true_height_exactly():
    truth = _climb()
    np.testing.assert_array_equal(
        generate_baro_samples(truth, BaroSpec(), np.random.default_rng(0)), truth.h_b)


def test_scale_factor_scales_the_height_above_the_departure_point():
    truth = _climb()
    error = generate_baro_samples(truth, BaroSpec(scale_factor_std=0.02),
                                  np.random.default_rng(4)) - truth.h_b
    s_b = error[-1] / (truth.h_b[-1] - truth.h_b[0])
    assert error[0] == 0.0
    np.testing.assert_allclose(error, s_b * (truth.h_b - truth.h_b[0]), atol=1e-9)
    assert 1e-4 < abs(s_b) < 0.1


def test_static_bias_is_constant_over_the_run():
    truth = _climb()
    error = generate_baro_samples(truth, BaroSpec(bias_std=10.0),
                                  np.random.default_rng(2)) - truth.h_b
    np.testing.assert_allclose(error, error[0], atol=1e-9)
    assert error[0] != 0.0


@pytest.mark.parametrize("tau_i", [0.1, 1.0])
def test_white_noise_spread_follows_the_density_at_any_time_step(tau_i):
    truth = _climb(M=20000, tau_i=tau_i)
    error = generate_baro_samples(truth, BaroSpec(noise_density=0.1),
                                  np.random.default_rng(1)) - truth.h_b
    assert error.std() == pytest.approx(0.1 / np.sqrt(tau_i), rel=0.03)


def test_drift_is_the_shared_gauss_markov_process():
    truth = _climb(M=500)
    spec = BaroSpec(drift_std=10.0, drift_tau=600.0)
    error = generate_baro_samples(truth, spec, np.random.default_rng(7)) - truth.h_b
    # Same draw order as the generator: s_b, b_bs, then the drift.
    rng = np.random.default_rng(7)
    rng.normal(), rng.normal()
    expected = gauss_markov(rng, 500, truth.tau_i, 10.0, 600.0, n_axes=1)[:, 0]
    np.testing.assert_allclose(error, expected, atol=1e-9)


def test_adding_noise_leaves_the_other_errors_unchanged():
    truth = _climb()
    quiet = BaroSpec(scale_factor_std=0.02, bias_std=10.0, drift_std=5.0)
    noisy = BaroSpec(scale_factor_std=0.02, bias_std=10.0, drift_std=5.0, noise_density=0.1)
    a = generate_baro_samples(truth, quiet, np.random.default_rng(9))
    b = generate_baro_samples(truth, noisy, np.random.default_rng(9))
    # The difference is the white noise alone: zero mean, 0.1/√0.5 m per sample.
    assert abs((b - a).mean()) < 0.01
    assert (b - a).std() == pytest.approx(0.1 / np.sqrt(0.5), rel=0.05)


def test_yaml_block_converts_to_si():
    spec = baro_spec_from_yaml({"scale_factor_pct": 2.0, "bias_m": 10.0, "drift_m": 5.0,
                                "drift_tau_s": 7200.0, "noise_m_per_rt_hz": 0.1})
    assert spec == BaroSpec(scale_factor_std=0.02, bias_std=10.0, drift_std=5.0,
                            drift_tau=7200.0, noise_density=0.1)
    assert baro_spec_from_yaml({}) == BaroSpec()


def test_packaged_spec_has_a_barometer_and_a_spec_without_one_is_ideal(tmp_path):
    spec = load_imu_spec(str(default_imu_spec_path()))
    assert spec.baro == BaroSpec(scale_factor_std=0.02, bias_std=10.0, drift_std=10.0,
                                 drift_tau=7200.0, noise_density=0.1)
    text = default_imu_spec_path().read_text(encoding="utf-8").split("\nbaro:")[0]
    path = tmp_path / "no_baro.yaml"
    path.write_text(text, encoding="utf-8")
    assert load_imu_spec(str(path)).baro is None
