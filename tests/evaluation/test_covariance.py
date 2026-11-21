import operator
from dataclasses import replace
from functools import reduce

import numpy as np
import pytest

from ins_sim.evaluation.covariance import (ERROR_SOURCES, covariance_analysis,
                                           error_budget, gaussian_cep)
from ins_sim.sensors.baro import BaroSpec
from ins_sim.sensors.imu import IMUSpec
from ins_sim.trajectory.kinematics import TruthTrajectory
from ins_sim.trajectory.spline import NEDSplinePath

ZERO_SENSORS = dict(gyro_arw=0.0, gyro_bias_instability=0.0, gyro_br_std=0.0,
                    accel_vrw=0.0, accel_bias_instability=0.0, accel_br_std=0.0)


@pytest.fixture(scope="module")
def truth():
    path = NEDSplinePath([[0.0, 0.0, 0.0], [20000.0, 5000.0, -500.0], [40000.0, 0.0, -800.0]])
    return TruthTrajectory.from_spline(path, speed=200.0, tau_i=0.5)


def test_gaussian_cep_matches_a_sampled_distribution():
    rng = np.random.default_rng(0)
    cov = np.array([[400.0, 150.0], [150.0, 100.0]])
    xy = rng.multivariate_normal([0.0, 0.0], cov, size=400_000)
    sampled = np.median(np.hypot(xy[:, 0], xy[:, 1]))
    assert gaussian_cep(cov[0, 0], cov[1, 1], cov[0, 1]) == pytest.approx(sampled, rel=5e-3)


def test_gaussian_cep_is_zero_without_spread():
    assert gaussian_cep(0.0, 0.0, 0.0) == 0.0


def test_the_sources_add_up_to_the_total(truth):
    spec = IMUSpec(gyro_sf_std=5e-6, accel_ma_std=5e-5, align_heading_std=7e-4,
                   baro=BaroSpec(bias_std=10.0, noise_density=0.1))
    budget = error_budget(truth, spec)
    assert list(budget.by_source) == list(ERROR_SOURCES)
    summed = reduce(operator.add, budget.by_source.values())
    np.testing.assert_allclose(summed.var, budget.total.var, rtol=1e-12)
    np.testing.assert_allclose(budget.total.var,
                               covariance_analysis(truth, spec).var, rtol=1e-12)


def test_an_error_free_imu_predicts_no_error(truth):
    cov = covariance_analysis(truth, IMUSpec(**ZERO_SENSORS))
    assert np.all(cov.var == 0.0)
    assert np.all(cov.cep == 0.0)


def test_only_the_error_source_that_is_on_contributes(truth):
    budget = error_budget(truth, IMUSpec(**{**ZERO_SENSORS, "gyro_br_std": 1e-7}))
    for name, part in budget.by_source.items():
        assert (part.var[-1].max() > 0.0) == (name == "Gyro biases")


def test_the_initial_spread_is_the_alignment_spread(truth):
    spec = IMUSpec(**ZERO_SENSORS, align_tilt_std=1e-5, align_heading_std=7e-4)
    cov = covariance_analysis(truth, spec)
    np.testing.assert_allclose(cov.sigma_delta_psi_nb_n[0], [1e-5, 1e-5, 7e-4])


def test_the_step_sets_the_prediction_times(truth):
    cov = covariance_analysis(truth, IMUSpec(), step_s=2.0)
    np.testing.assert_allclose(np.diff(cov.t), 2.0)


def test_without_baro_aiding_an_accelerometer_bias_grows_the_height_error(truth):
    """Free-inertial, a vertical accelerometer bias b gives a height error of
    about b·t²/2 (the gravity feedback adds a few percent in this time);
    the barometer holds it far below that."""
    b = 1e-4
    spec = IMUSpec(**{**ZERO_SENSORS, "accel_br_std": b})
    free = covariance_analysis(truth, spec, baro_aiding=False)
    aided = covariance_analysis(truth, spec)
    t = free.t[-1]
    # The body is close to level, so the vertical axis sees about one axis's bias.
    assert free.sigma_delta_r_eb_n[-1, 2] == pytest.approx(0.5 * b * t**2, rel=0.1)
    assert aided.sigma_delta_r_eb_n[-1, 2] < free.sigma_delta_r_eb_n[-1, 2] / 5


def test_an_ideal_barometer_contributes_nothing(truth):
    budget = error_budget(truth, replace(IMUSpec(), baro=BaroSpec()))
    assert np.all(budget.by_source["Barometer"].var == 0.0)
