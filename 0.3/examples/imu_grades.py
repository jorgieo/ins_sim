"""How fast three grades of IMU lose position on the same flight."""

from pathlib import Path

import numpy as np

from ins_sim.config import default_imu_spec_path
from ins_sim.evaluation.errors import position_error
from ins_sim.evaluation.monte_carlo import run_monte_carlo
from ins_sim.sensors.imu import load_imu_spec
from ins_sim.trajectory.kinematics import build_trajectory

EXAMPLES = Path(__file__).parent
SPECS = {
    "Navigation": default_imu_spec_path(),
    "Tactical": EXAMPLES / "imu" / "tactical_grade.yaml",
    "MEMS": EXAMPLES / "imu" / "mems_grade.yaml",
}
MINUTES = [1, 5, 10, 25]


def distance(metres):
    """Formats a distance in metres below 1 km, in kilometres above."""
    return f"{metres:.0f} m" if metres < 1000 else f"{metres / 1000:.1f} km"


# A 2 s time step keeps the comparison quick: the grades differ far more
# than the time step changes the result.
truth, _, _ = build_trajectory(str(EXAMPLES / "missions" / "training_flight.yaml"),
                               tau_i=2.0)
samples = np.searchsorted(truth.t, [60.0 * m for m in MINUTES])

print("CEP of 10 trials on the training flight")
print(f"{'IMU grade':12s}" + "".join(f"{f'{m} min':>10s}" for m in MINUTES))
for grade, path in SPECS.items():
    spec = load_imu_spec(str(path))
    mc = run_monte_carlo(truth, spec, n_trials=10, seed=42)
    delta_r_eb_n = position_error(mc.est_L_b, mc.est_lambda_b, mc.est_h_b,
                                  truth.L_b, truth.lambda_b, truth.h_b)
    cep = np.median(np.hypot(delta_r_eb_n[..., 0], delta_r_eb_n[..., 1]), axis=0)
    print(f"{grade:12s}" + "".join(f"{distance(cep[k]):>10s}" for k in samples))
