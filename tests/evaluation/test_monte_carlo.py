import numpy as np
import pytest

from ins_sim.evaluation.errors import position_error
from ins_sim.evaluation.monte_carlo import percentile_envelope, run_monte_carlo
from ins_sim.sensors.imu import IMUSpec, draw_initial_misalignment
from ins_sim.trajectory.kinematics import TruthTrajectory
from ins_sim.trajectory.spline import NEDSplinePath

ZERO_SENSORS = dict(gyro_arw=0.0, gyro_bias_instability=0.0, gyro_br_std=0.0,
                    accel_vrw=0.0, accel_bias_instability=0.0, accel_br_std=0.0)


def _short_truth():
    path = NEDSplinePath([[0.0, 0.0, 0.0], [2000.0, 800.0, -50.0], [4000.0, 1000.0, -80.0]])
    return TruthTrajectory.from_spline(path, speed=200.0, tau_i=0.1)


def test_initial_attitude_error_is_the_alignment_draw_of_each_trial():
    """Trial i's alignment error comes from its own alignment stream."""
    truth = _short_truth()
    spec = IMUSpec(**ZERO_SENSORS, align_tilt_std=1e-4, align_heading_std=1e-3)
    mc = run_monte_carlo(truth, spec, n_trials=4, seed=11)

    trial_rngs = np.random.default_rng(11).spawn(4)
    for i, trial_rng in enumerate(trial_rngs):
        rng_alignment, _ = trial_rng.spawn(2)
        expected = draw_initial_misalignment(spec, rng_alignment)
        np.testing.assert_allclose(mc.delta_psi_nb_n[i, 0], expected, atol=1e-12)


def test_alignment_errors_do_not_depend_on_the_imu_spec():
    """Common random numbers: changing sensor errors leaves alignment draws alone."""
    truth = _short_truth()
    quiet = IMUSpec(**ZERO_SENSORS, align_tilt_std=1e-4, align_heading_std=1e-3)
    noisy = IMUSpec(align_tilt_std=1e-4, align_heading_std=1e-3)   # default sensor errors
    a = run_monte_carlo(truth, quiet, n_trials=3, seed=5)
    b = run_monte_carlo(truth, noisy, n_trials=3, seed=5)
    np.testing.assert_array_equal(a.delta_psi_nb_n[:, 0], b.delta_psi_nb_n[:, 0])


def test_zero_errors_reproduce_the_truth():
    truth = _short_truth()
    mc = run_monte_carlo(truth, IMUSpec(**ZERO_SENSORS), n_trials=2, seed=0)
    delta_r = position_error(mc.est_L_b, mc.est_lambda_b, mc.est_h_b,
                             truth.L_b, truth.lambda_b, truth.h_b)
    assert np.abs(mc.delta_psi_nb_n).max() < 1e-9
    assert np.abs(delta_r).max() < 1e-6


def test_trial_draws_do_not_depend_on_the_number_of_trials():
    truth = _short_truth()
    spec = IMUSpec(align_tilt_std=1e-4, align_heading_std=1e-3)
    two = run_monte_carlo(truth, spec, n_trials=2, seed=3)
    four = run_monte_carlo(truth, spec, n_trials=4, seed=3)
    np.testing.assert_array_equal(two.est_L_b, four.est_L_b[:2])


def test_percentile_envelope_of_known_errors():
    # Trials with 3D error magnitudes 1..5 m at every sample.
    mags = np.arange(1.0, 6.0)
    delta_r = np.zeros((5, 4, 3))
    delta_r[:, :, 0] = mags[:, None]
    np.testing.assert_allclose(percentile_envelope(delta_r, q=50), 3.0)
    np.testing.assert_allclose(percentile_envelope(delta_r, q=100), 5.0)
    assert percentile_envelope(delta_r).shape == (4,)


@pytest.mark.parametrize("baro_aiding", [True, False])
def test_result_shapes(baro_aiding):
    truth = _short_truth()
    mc = run_monte_carlo(truth, IMUSpec(), n_trials=2, seed=1, baro_aiding=baro_aiding)
    M = len(truth.t)
    assert mc.est_h_b.shape == (2, M)
    assert mc.delta_psi_nb_n.shape == (2, M, 3)
    assert np.all(np.isfinite(mc.delta_psi_nb_n))
