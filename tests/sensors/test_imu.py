from types import SimpleNamespace

import numpy as np
import pytest

from ins_sim.config import default_imu_spec_path
from ins_sim.sensors.imu import (GM_ALLAN_PEAK, IMUSpec, draw_initial_misalignment,
                                 generate_imu_samples, load_imu_spec)


def test_load_imu_spec_matches_default_dataclass_values():
    spec = load_imu_spec(str(default_imu_spec_path()))
    default = IMUSpec()

    assert spec.gyro_arw == pytest.approx(default.gyro_arw)
    assert spec.gyro_bias_instability == pytest.approx(default.gyro_bias_instability)
    assert spec.gyro_bias_tau == pytest.approx(default.gyro_bias_tau)
    assert spec.gyro_br_std == pytest.approx(default.gyro_br_std)
    assert spec.accel_vrw == pytest.approx(default.accel_vrw)
    assert spec.accel_bias_instability == pytest.approx(default.accel_bias_instability)
    assert spec.accel_bias_tau == pytest.approx(default.accel_bias_tau)
    assert spec.accel_br_std == pytest.approx(default.accel_br_std)


def _stub_truth(M=200, dt=0.1):
    return SimpleNamespace(
        t=np.arange(M) * dt,
        tau_i=dt,
        omega_ib_b=np.full((M, 3), 0.01),
        f_ib_b=np.tile([0.0, 0.0, -9.81], (M, 1)),
    )


def test_generate_imu_samples_zero_noise_is_exact_passthrough():
    truth = _stub_truth()
    zero_spec = IMUSpec(gyro_arw=0, gyro_bias_instability=0, gyro_br_std=0,
                        accel_vrw=0, accel_bias_instability=0, accel_br_std=0)
    rng = np.random.default_rng(0)
    meas_omega_ib_b, meas_f_ib_b = generate_imu_samples(truth, zero_spec, rng)
    np.testing.assert_array_equal(meas_omega_ib_b, truth.omega_ib_b)
    np.testing.assert_array_equal(meas_f_ib_b, truth.f_ib_b)


def test_generate_imu_samples_shapes():
    truth = _stub_truth(M=500)
    spec = IMUSpec()
    rng = np.random.default_rng(1)
    meas_omega_ib_b, meas_f_ib_b = generate_imu_samples(truth, spec, rng)
    assert meas_omega_ib_b.shape == (500, 3)
    assert meas_f_ib_b.shape == (500, 3)
    assert np.all(np.isfinite(meas_omega_ib_b))
    assert np.all(np.isfinite(meas_f_ib_b))


def test_generate_imu_samples_reproducible_with_same_seed():
    truth = _stub_truth()
    spec = IMUSpec()
    om1, fm1 = generate_imu_samples(truth, spec, np.random.default_rng(42))
    om2, fm2 = generate_imu_samples(truth, spec, np.random.default_rng(42))
    np.testing.assert_array_equal(om1, om2)
    np.testing.assert_array_equal(fm1, fm2)


def test_load_imu_spec_converts_multiplicative_and_alignment_fields():
    spec = load_imu_spec(str(default_imu_spec_path()))
    assert spec.gyro_sf_std == pytest.approx(5.0e-6)
    assert spec.gyro_ma_std == pytest.approx(50.0e-6)
    assert spec.accel_sf_std == pytest.approx(100.0e-6)
    assert spec.accel_ma_std == pytest.approx(50.0e-6)
    assert spec.align_tilt_std == pytest.approx(np.deg2rad(0.0015))
    assert spec.align_heading_std == pytest.approx(np.deg2rad(0.04))


def test_scale_factor_error_is_multiplicative():
    # Only gyro scale factor active: measurement/truth ratio must be a
    # per-axis constant of plausible magnitude, identical at every sample.
    truth = _stub_truth()
    spec = IMUSpec(gyro_arw=0, gyro_bias_instability=0, gyro_br_std=0,
                   accel_vrw=0, accel_bias_instability=0, accel_br_std=0,
                   gyro_sf_std=100e-6)
    rng = np.random.default_rng(3)
    meas_omega_ib_b, meas_f_ib_b = generate_imu_samples(truth, spec, rng)

    ratio = meas_omega_ib_b / truth.omega_ib_b
    np.testing.assert_allclose(
        ratio, np.broadcast_to(ratio[0], ratio.shape), rtol=0, atol=1e-12)
    assert np.all(ratio[0] != 1.0)
    assert np.all(np.abs(ratio[0] - 1.0) < 5 * 100e-6)
    np.testing.assert_array_equal(meas_f_ib_b, truth.f_ib_b)


def test_misalignment_couples_axes():
    # Truth rate along x only; with misalignment active the y/z gyros
    # must pick up a proportional constant component.
    truth = _stub_truth()
    truth.omega_ib_b = np.tile([0.02, 0.0, 0.0], (len(truth.t), 1))
    spec = IMUSpec(gyro_arw=0, gyro_bias_instability=0, gyro_br_std=0,
                   accel_vrw=0, accel_bias_instability=0, accel_br_std=0,
                   gyro_ma_std=100e-6)
    rng = np.random.default_rng(4)
    meas_omega_ib_b, _ = generate_imu_samples(truth, spec, rng)

    assert np.all(meas_omega_ib_b[:, 1] != 0.0)
    assert np.all(meas_omega_ib_b[:, 2] != 0.0)
    assert np.abs(meas_omega_ib_b[0, 1]) < 0.02 * 5 * 100e-6
    # x-axis sees only its own (zero) scale factor -- unchanged
    np.testing.assert_array_equal(meas_omega_ib_b[:, 0], truth.omega_ib_b[:, 0])


def test_draw_initial_misalignment_statistics_and_zero_spec():
    spec = IMUSpec(align_tilt_std=1e-4, align_heading_std=1e-3)
    rng = np.random.default_rng(5)
    draws = np.array([draw_initial_misalignment(spec, rng)
                      for _ in range(4000)])
    assert draws[:, 0].std() == pytest.approx(1e-4, rel=0.1)
    assert draws[:, 1].std() == pytest.approx(1e-4, rel=0.1)
    assert draws[:, 2].std() == pytest.approx(1e-3, rel=0.1)

    zero_spec = IMUSpec()
    np.testing.assert_array_equal(
        draw_initial_misalignment(zero_spec, np.random.default_rng(6)),
        np.zeros(3))


def test_generate_imu_samples_white_noise_statistics():
    # Isolate the white-noise contribution by removing bias terms entirely,
    # then check its std against the ARW/VRW/sqrt(dt) formula over many
    # samples (generous tolerance -- this is a statistical check).
    M, dt = 20000, 0.1
    truth = _stub_truth(M=M, dt=dt)
    spec = IMUSpec(gyro_bias_instability=0, gyro_br_std=0, accel_bias_instability=0, accel_br_std=0)
    rng = np.random.default_rng(7)
    meas_omega_ib_b, meas_f_ib_b = generate_imu_samples(truth, spec, rng)

    gyro_noise_std = np.std(meas_omega_ib_b - truth.omega_ib_b)
    accel_noise_std = np.std(meas_f_ib_b - truth.f_ib_b)

    expected_gyro_std = spec.gyro_arw / np.sqrt(dt)
    expected_accel_std = spec.accel_vrw / np.sqrt(dt)

    assert gyro_noise_std == pytest.approx(expected_gyro_std, rel=0.05)
    assert accel_noise_std == pytest.approx(expected_accel_std, rel=0.05)


def test_gauss_markov_bias_is_stationary_from_the_first_sample():
    # Only the gyro dynamic bias is active. Its spread across independent
    # runs must equal the steady-state σ at the first sample (b_d[0] is drawn
    # from the stationary distribution) and stay there later on.
    sigma, tau = 1e-5, 20.0
    spec = IMUSpec(gyro_arw=0, gyro_br_std=0, accel_vrw=0, accel_bias_instability=0,
                   accel_br_std=0, gyro_bias_instability=sigma * GM_ALLAN_PEAK,
                   gyro_bias_tau=tau)
    truth = _stub_truth(M=60, dt=1.0)
    bias = np.array([
        generate_imu_samples(truth, spec, np.random.default_rng(seed)).meas_omega_ib_b
        - truth.omega_ib_b
        for seed in range(2000)
    ])                                                  # (runs, M, 3)
    assert bias[:, 0].std() == pytest.approx(sigma, rel=0.06)
    assert bias[:, -1].std() == pytest.approx(sigma, rel=0.06)
    # Lag-k correlation of a first-order Gauss-Markov process is exp(-k·dt/τ).
    k = 10
    corr = np.corrcoef(bias[:, 0].ravel(), bias[:, k].ravel())[0, 1]
    assert corr == pytest.approx(np.exp(-k * 1.0 / tau), abs=0.05)


def test_gyro_errors_do_not_depend_on_the_accelerometer_spec():
    # Each triad draws from its own stream: changing accelerometer
    # parameters must leave the gyro realization bit-identical.
    truth = _stub_truth()
    spec_a = IMUSpec()
    spec_b = IMUSpec(accel_vrw=3 * spec_a.accel_vrw, accel_bias_instability=0.0,
                     accel_sf_std=1e-4, accel_ma_std=1e-4)
    a = generate_imu_samples(truth, spec_a, np.random.default_rng(9))
    b = generate_imu_samples(truth, spec_b, np.random.default_rng(9))
    np.testing.assert_array_equal(a.meas_omega_ib_b, b.meas_omega_ib_b)
    assert not np.array_equal(a.meas_f_ib_b, b.meas_f_ib_b)


def test_yaml_bias_instability_is_the_datasheet_value():
    # The YAML's bias_instab_deg_per_hr is stored as given (in SI units) as
    # the datasheet bias instability; the model's Gauss-Markov spread is
    # derived from it so that the Allan deviation peaks at that value.
    spec = load_imu_spec(str(default_imu_spec_path()))
    datasheet = 0.01 * np.pi / 180.0 / 3600.0
    assert spec.gyro_bias_instability == pytest.approx(datasheet)
    assert spec.gyro_gm_std == pytest.approx(datasheet / GM_ALLAN_PEAK)
    assert spec.accel_gm_std == pytest.approx(5e-6 * 9.80665 / GM_ALLAN_PEAK)


def test_allan_peak_ratio_matches_the_gauss_markov_formula():
    x = np.linspace(1.0, 3.0, 200001)                         # T / tau_c
    adev = np.sqrt((2 * x - 3 + 4 * np.exp(-x) - np.exp(-2 * x)) / x**2)
    assert adev.max() == pytest.approx(GM_ALLAN_PEAK, abs=1e-9)
    assert x[np.argmax(adev)] == pytest.approx(1.8926, abs=1e-4)
