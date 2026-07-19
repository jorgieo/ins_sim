from types import SimpleNamespace

import numpy as np
import pytest

from ins_sim.config import default_imu_spec_path
from ins_sim.sensors.imu import (IMUSpec, draw_initial_misalignment,
                                 generate_imu_samples, load_imu_spec)


def test_load_imu_spec_matches_default_dataclass_values():
    spec = load_imu_spec(str(default_imu_spec_path()))
    default = IMUSpec()

    assert spec.gyro_arw == pytest.approx(default.gyro_arw)
    assert spec.gyro_bi_std == pytest.approx(default.gyro_bi_std)
    assert spec.gyro_bi_tau == pytest.approx(default.gyro_bi_tau)
    assert spec.gyro_br_std == pytest.approx(default.gyro_br_std)
    assert spec.accel_vrw == pytest.approx(default.accel_vrw)
    assert spec.accel_bi_std == pytest.approx(default.accel_bi_std)
    assert spec.accel_bi_tau == pytest.approx(default.accel_bi_tau)
    assert spec.accel_br_std == pytest.approx(default.accel_br_std)


def _stub_truth(M=200, dt=0.1):
    return SimpleNamespace(
        t=np.arange(M) * dt,
        dt=dt,
        omega_b=np.full((M, 3), 0.01),
        f_b=np.tile([0.0, 0.0, -9.81], (M, 1)),
    )


def test_generate_imu_samples_zero_noise_is_exact_passthrough():
    truth = _stub_truth()
    zero_spec = IMUSpec(gyro_arw=0, gyro_bi_std=0, gyro_br_std=0,
                        accel_vrw=0, accel_bi_std=0, accel_br_std=0)
    rng = np.random.default_rng(0)
    omega_meas, f_meas = generate_imu_samples(truth, zero_spec, rng)
    np.testing.assert_array_equal(omega_meas, truth.omega_b)
    np.testing.assert_array_equal(f_meas, truth.f_b)


def test_generate_imu_samples_shapes():
    truth = _stub_truth(M=500)
    spec = IMUSpec()
    rng = np.random.default_rng(1)
    omega_meas, f_meas = generate_imu_samples(truth, spec, rng)
    assert omega_meas.shape == (500, 3)
    assert f_meas.shape == (500, 3)
    assert np.all(np.isfinite(omega_meas))
    assert np.all(np.isfinite(f_meas))


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
    spec = IMUSpec(gyro_arw=0, gyro_bi_std=0, gyro_br_std=0,
                   accel_vrw=0, accel_bi_std=0, accel_br_std=0,
                   gyro_sf_std=100e-6)
    rng = np.random.default_rng(3)
    omega_meas, f_meas = generate_imu_samples(truth, spec, rng)

    ratio = omega_meas / truth.omega_b
    np.testing.assert_allclose(
        ratio, np.broadcast_to(ratio[0], ratio.shape), rtol=0, atol=1e-12)
    assert np.all(ratio[0] != 1.0)
    assert np.all(np.abs(ratio[0] - 1.0) < 5 * 100e-6)
    np.testing.assert_array_equal(f_meas, truth.f_b)


def test_misalignment_couples_axes():
    # Truth rate along x only; with misalignment active the y/z gyros
    # must pick up a proportional constant component.
    truth = _stub_truth()
    truth.omega_b = np.tile([0.02, 0.0, 0.0], (len(truth.t), 1))
    spec = IMUSpec(gyro_arw=0, gyro_bi_std=0, gyro_br_std=0,
                   accel_vrw=0, accel_bi_std=0, accel_br_std=0,
                   gyro_ma_std=100e-6)
    rng = np.random.default_rng(4)
    omega_meas, _ = generate_imu_samples(truth, spec, rng)

    assert np.all(omega_meas[:, 1] != 0.0)
    assert np.all(omega_meas[:, 2] != 0.0)
    assert np.abs(omega_meas[0, 1]) < 0.02 * 5 * 100e-6
    # x-axis sees only its own (zero) scale factor -- unchanged
    np.testing.assert_array_equal(omega_meas[:, 0], truth.omega_b[:, 0])


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
    spec = IMUSpec(gyro_bi_std=0, gyro_br_std=0, accel_bi_std=0, accel_br_std=0)
    rng = np.random.default_rng(7)
    omega_meas, f_meas = generate_imu_samples(truth, spec, rng)

    gyro_noise_std = np.std(omega_meas - truth.omega_b)
    accel_noise_std = np.std(f_meas - truth.f_b)

    expected_gyro_std = spec.gyro_arw / np.sqrt(dt)
    expected_accel_std = spec.accel_vrw / np.sqrt(dt)

    assert gyro_noise_std == pytest.approx(expected_gyro_std, rel=0.05)
    assert accel_noise_std == pytest.approx(expected_accel_std, rel=0.05)
