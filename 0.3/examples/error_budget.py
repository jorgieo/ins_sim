"""Which error source dominates? The training flight with one source at a time."""

from dataclasses import replace
from pathlib import Path

import numpy as np

from ins_sim.config import default_imu_spec_path
from ins_sim.evaluation.errors import position_error
from ins_sim.evaluation.monte_carlo import run_monte_carlo
from ins_sim.sensors.imu import load_imu_spec
from ins_sim.trajectory.kinematics import build_trajectory

MISSION = Path(__file__).parent / "missions" / "training_flight.yaml"

# The IMUSpec fields that make up each error source.
SOURCES = {
    "Initial alignment": ["align_tilt_std", "align_heading_std"],
    "Gyro biases": ["gyro_br_std", "gyro_bias_instability"],
    "Gyro scale factor and misalignment": ["gyro_sf_std", "gyro_ma_std"],
    "Accelerometer biases": ["accel_br_std", "accel_bias_instability"],
    "Accel. scale factor and misalignment": ["accel_sf_std", "accel_ma_std"],
    "Sensor noise (ARW and VRW)": ["gyro_arw", "accel_vrw"],
}

truth, _, _ = build_trajectory(str(MISSION), tau_i=2.0)
spec = load_imu_spec(str(default_imu_spec_path()))
no_errors = replace(spec, **{field: 0.0 for fields in SOURCES.values() for field in fields})


def final_cep(spec):
    """CEP at the end of the flight, from 10 trials with the GUI's seed."""
    mc = run_monte_carlo(truth, spec, n_trials=10, seed=42)
    delta_r_eb_n = position_error(mc.est_L_b, mc.est_lambda_b, mc.est_h_b,
                                  truth.L_b, truth.lambda_b, truth.h_b)
    return np.median(np.hypot(delta_r_eb_n[:, -1, 0], delta_r_eb_n[:, -1, 1]))


print(f"CEP after {truth.t[-1] / 60:.0f} min, each source on its own")
for source, fields in SOURCES.items():
    only_this = replace(no_errors, **{field: getattr(spec, field) for field in fields})
    print(f"  {source:38s} {final_cep(only_this):5.0f} m")
print(f"  {'All sources together':38s} {final_cep(spec):5.0f} m")
