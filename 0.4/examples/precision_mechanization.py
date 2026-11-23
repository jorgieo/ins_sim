"""Both mechanizations on an integrating IMU's outputs, with perfect sensors."""

import numpy as np

from ins_sim.config import default_trajectory_path
from ins_sim.evaluation.errors import position_error
from ins_sim.navigation.strapdown import NavState, strapdown_navgrade
from ins_sim.trajectory.kinematics import build_trajectory

for oversample in (None, 10):
    # The packaged mission at 1 s steps; oversampled, the IMU outputs are
    # averages over each step of a mission built ten times as finely.
    truth = build_trajectory(str(default_trajectory_path()), tau_i=1.0,
                             oversample=oversample).truth
    start = NavState(truth.L_b[0], truth.lambda_b[0], truth.h_b[0],
                     truth.v_eb_n[0], truth.C_b_n[0].as_quat())
    print(f"Truth oversampling: {oversample or 'off'}")
    for mechanization in ("basic", "precision"):
        nav = strapdown_navgrade(truth.omega_ib_b, truth.f_ib_b, start, truth.tau_i,
                                 meas_h_b=truth.h_b, mechanization=mechanization)
        delta_r_eb_n = position_error(nav.L_b, nav.lambda_b, nav.h_b,
                                      truth.L_b, truth.lambda_b, truth.h_b)
        largest = np.hypot(delta_r_eb_n[:, 0], delta_r_eb_n[:, 1]).max()
        print(f"  {mechanization:9s}  largest horizontal error {largest:7.0f} m")
