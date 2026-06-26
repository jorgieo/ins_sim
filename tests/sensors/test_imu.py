from types import SimpleNamespace

import numpy as np
import pytest

from ins_sim.config import default_imu_spec_path
from ins_sim.sensors.imu import IMUSpec, load_imu_spec, generate_imu_samples


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
