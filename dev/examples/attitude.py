"""One attitude written three ways, and an attitude error resolved in NED."""

import numpy as np
from scipy.spatial.transform import Rotation

from ins_sim.evaluation.errors import attitude_error

np.set_printoptions(precision=4, suppress=True, floatmode="fixed")

# Heading 30°, pitch 5°, roll 10°. C_b^n turns through the heading about
# Down, then the pitch about the new y axis, then the roll about the new x axis.
C_b_n = Rotation.from_euler("ZYX", [30.0, 5.0, 10.0], degrees=True)
print("C_b^n =")
print(C_b_n.as_matrix())
print("q_b^n, stored scalar-last [x, y, z, w]:", C_b_n.as_quat())

# C_b^n resolves a vector given in body axes along North, East and Down:
# here the body x axis, the direction the nose points.
print("Nose direction in NED:", C_b_n.apply([1.0, 0.0, 0.0]))

# A 0.1° pitch error. At a heading of 30° the attitude error vector
# delta_psi_nb^n splits it between the North and East tilts.
est_C_b_n = Rotation.from_euler("ZYX", [30.0, 5.1, 10.0], degrees=True)
delta_psi_nb_n = attitude_error(est_C_b_n, C_b_n)
print("delta_psi_nb^n [deg]:", np.rad2deg(delta_psi_nb_n).round(4) + 0.0)
