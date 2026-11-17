import numpy as np
import pytest
from scipy.spatial.transform import Rotation as Rot

from ins_sim.core.earth_model import wgs84_radii
from ins_sim.evaluation.errors import attitude_error, position_error, velocity_error


def test_attitude_error_recovers_the_applied_small_rotation():
    # est = exp([delta ^]) · true, so the error must return delta exactly.
    rng = np.random.default_rng(0)
    C_b_n = Rot.from_rotvec(rng.normal(0.0, 1.0, (50, 3)))
    delta = rng.normal(0.0, 1e-3, (50, 3))
    est_C_b_n = Rot.from_rotvec(delta) * C_b_n
    np.testing.assert_allclose(attitude_error(est_C_b_n, C_b_n), delta, atol=1e-12)


def test_attitude_error_is_zero_for_identical_attitudes():
    C_b_n = Rot.from_euler("ZYX", [[30.0, 5.0, -2.0]], degrees=True)
    np.testing.assert_allclose(attitude_error(C_b_n, C_b_n), 0.0, atol=1e-15)


def test_position_error_matches_the_curvilinear_conversion():
    L_b = np.deg2rad(np.array([10.0, 45.0, 70.0]))
    lambda_b = np.deg2rad(np.array([-60.0, 0.0, 120.0]))
    h_b = np.array([0.0, 3000.0, 10000.0])
    dL, dlam, dh = 1e-6, -2e-6, 5.0
    err = position_error(L_b + dL, lambda_b + dlam, h_b + dh, L_b, lambda_b, h_b)

    R_N, R_E = wgs84_radii(L_b)
    np.testing.assert_allclose(err[:, 0], (R_N + h_b) * dL)
    np.testing.assert_allclose(err[:, 1], (R_E + h_b) * np.cos(L_b) * dlam)
    np.testing.assert_allclose(err[:, 2], -dh)


def test_position_error_broadcasts_over_trials_and_is_zero_at_truth():
    L_b = np.deg2rad(np.linspace(15.0, 20.0, 7))
    lambda_b = np.deg2rad(np.linspace(-68.0, -66.0, 7))
    h_b = np.full(7, 4572.0)
    est = np.broadcast_to(L_b, (3, 7)), np.broadcast_to(lambda_b, (3, 7)), np.broadcast_to(h_b, (3, 7))
    err = position_error(*est, L_b, lambda_b, h_b)
    assert err.shape == (3, 7, 3)
    np.testing.assert_array_equal(err, 0.0)


def test_position_error_wraps_longitude_across_the_antimeridian():
    L_b, h_b = np.array([0.0]), np.array([0.0])
    true_lambda = np.array([np.pi - 1e-7])
    est_lambda = np.array([-np.pi + 1e-7])        # 2e-7 rad east, across ±180°
    err = position_error(L_b, est_lambda, h_b, L_b, true_lambda, h_b)
    R_E = wgs84_radii(0.0).R_E
    assert err[0, 1] == pytest.approx(R_E * 2e-7, rel=1e-6)


def test_velocity_error_is_estimate_minus_truth():
    v = np.array([[100.0, 5.0, -1.0]])
    np.testing.assert_allclose(velocity_error(v + [0.1, -0.2, 0.3], v), [[0.1, -0.2, 0.3]],
                               atol=1e-12)
