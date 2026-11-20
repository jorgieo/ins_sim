"""Validation: the precision mechanization against coning, sculling and a real mission.

Each test runs one scenario from tools/validation_scenarios.py (the same
scenarios the Validation page plots). The IMU outputs are those of an
integrating IMU: the averages of the angular rate and specific force over
each interval, as a real IMU reports them.
"""

import pytest

from ins_sim.trajectory.kinematics import TruthTrajectory
from ins_sim.trajectory.spline import NEDSplinePath
from tools.validation_scenarios import (coning, mechanization_convergence,
                                        mission_mechanizations, sculling)


def test_coning_drifts_the_heading_at_the_predicted_rate_and_the_correction_removes_it():
    r = coning()
    assert r.rate_basic == pytest.approx(r.predicted_basic, rel=1e-2)
    assert r.rate_precision == pytest.approx(r.predicted_precision, rel=2e-2)
    assert abs(r.rate_precision) < abs(r.rate_basic) / 100


def test_sculling_drifts_the_velocity_at_the_predicted_rate_and_the_correction_removes_it():
    r = sculling()
    assert r.rate_basic == pytest.approx(r.predicted_basic, rel=1e-2)
    assert r.rate_precision == pytest.approx(r.predicted_precision, rel=2e-2)
    assert abs(r.rate_precision) < abs(r.rate_basic) / 100


def test_the_precision_mechanization_is_second_order_and_the_basic_first_order():
    path = NEDSplinePath([[0.0, 0.0, 0.0], [6000.0, 0.0, 0.0],
                          [9000.0, 3000.0, -200.0], [9000.0, 9000.0, -200.0]])
    r = mechanization_convergence(
        (0.5, 0.25), tau_fine=0.005,
        build=lambda tau_i, m: TruthTrajectory.from_spline(
            path, speed=150.0, tau_i=tau_i, oversample=m))
    assert r.basic[0] / r.basic[1] == pytest.approx(2.0, rel=0.1)
    assert r.precision[0] / r.precision[1] == pytest.approx(4.0, rel=0.1)
    assert r.precision[1] < r.basic[1] / 100


def test_on_the_packaged_mission_the_precision_mechanization_removes_the_turn_errors():
    r = mission_mechanizations(tau_i=1.0, oversample=10)
    assert r.precision.max() < r.basic.max() / 50
