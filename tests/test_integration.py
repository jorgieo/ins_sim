import time

import numpy as np

from ins_sim.trajectory.spline import NEDSplinePath
from ins_sim.trajectory.kinematics import TruthTrajectory
from ins_sim.sensors.imu import IMUSpec

import main as main_module


def _tiny_truth():
    """~10s synthetic trajectory -- fast stand-in for the full BQN scenario
    (measured at >12s just to build), used to exercise main.py's pipeline
    wiring (truth -> noisy IMU -> strapdown -> Monte Carlo -> envelope)
    without depending on BQN-specific YAML content or phase-helper geometry
    (those are covered separately in tests/trajectory/test_trajectory.py)."""
    waypoints = np.array([
        [0.0, 0.0, 0.0],
        [1000.0, 500.0, -50.0],
        [3000.0, 1500.0, -100.0],
        [5000.0, 2000.0, -100.0],
    ])
    path = NEDSplinePath(waypoints)
    return TruthTrajectory(path, speed=500.0, dt=0.1)   # ~10.8 s of flight


def test_pipeline_via_main_functions_runs_fast_and_consistently():
    truth = _tiny_truth()
    assert truth.t[-1] < 15.0   # confirms this really is a short trajectory

    spec = IMUSpec()  # default nav-grade spec

    t0 = time.time()

    zero_err, _ = main_module.run_self_consistency_check(truth, truth.dt)
    assert np.all(np.isfinite(zero_err))

    n_trials = 2
    pos_runs, euler_runs, lat_runs, lon_runs, r95 = main_module.simulate(
        truth, spec, n_trials=n_trials, seed=0)

    elapsed = time.time() - t0

    M = len(truth.t)
    assert pos_runs.shape == (n_trials, M, 3)
    assert euler_runs.shape == (n_trials, M, 3)
    assert lat_runs.shape == (n_trials, M)
    assert lon_runs.shape == (n_trials, M)
    assert r95.shape == (M,)

    assert np.all(np.isfinite(pos_runs))
    assert np.all(np.isfinite(euler_runs))
    assert np.all(r95 >= 0.0)

    # Generous headroom over the measured ~0.3s cost for this scenario size.
    assert elapsed < 10.0
