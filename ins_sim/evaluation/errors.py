"""Navigation error definitions, in Groves (2013) notation.

Every error is the **estimate minus the truth**. All three are resolved in
the local NED axes at the true position, so attitude, velocity and
position errors share one frame. They are also what the linearized error
model (covariance analysis) propagates. The definitions are spelled out on
[Notation & conventions][notation-conventions].
"""

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation

from ins_sim.core.earth_model import wgs84_radii


def attitude_error(est_C_b_n: Rotation, C_b_n: Rotation) -> np.ndarray:
    """Computes the attitude error delta_psi_nb^n.

    The small rotation that takes the true attitude to the estimated one:
    C_b^n(estimated) = exp([delta_psi_nb^n ^]) · C_b^n(true), so
    delta_psi_nb^n is the rotation vector of C_b^n(estimated)·C_n^b(true).
    Its components are the North tilt, the East tilt and the Down
    (heading) error, resolved in NED.

    Args:
        est_C_b_n: Estimated body-to-NED attitude (scipy Rotation, single
            or stacked).
        C_b_n: True body-to-NED attitude, with the same stacking.

    Returns:
        numpy.ndarray: delta_psi_nb^n, shape (3,) or (M, 3) [rad].

    Examples:
        A 1 mrad heading error appears on the Down component:

        >>> from scipy.spatial.transform import Rotation
        >>> from ins_sim.evaluation.errors import attitude_error
        >>> C_b_n = Rotation.from_euler("ZYX", [30.0, 2.0, 5.0], degrees=True)
        >>> est_C_b_n = Rotation.from_rotvec([0.0, 0.0, 1e-3]) * C_b_n
        >>> [round(float(x), 6) for x in attitude_error(est_C_b_n, C_b_n)]
        [0.0, 0.0, 0.001]
    """
    return (est_C_b_n * C_b_n.inv()).as_rotvec()


def position_error(est_L_b: npt.ArrayLike, est_lambda_b: npt.ArrayLike,
                   est_h_b: npt.ArrayLike, L_b: npt.ArrayLike,
                   lambda_b: npt.ArrayLike, h_b: npt.ArrayLike) -> np.ndarray:
    """Computes the position error delta_r_eb^n, resolved in NED at the true position.

    The curvilinear errors (delta_L_b, delta_lambda_b, delta_h_b) are
    converted to metres along the local North, East and Down axes:

        delta_r_eb^n = [(R_N + h_b)·delta_L_b,
                        (R_E + h_b)·cos(L_b)·delta_lambda_b,
                        −delta_h_b]

    with the radii of curvature evaluated at the true latitude.

    Args:
        est_L_b: Estimated latitude [rad], shape (..., M).
        est_lambda_b: Estimated longitude [rad], shape (..., M).
        est_h_b: Estimated height [m], shape (..., M).
        L_b: True latitude [rad], shape (M,), broadcast against the
            estimates.
        lambda_b: True longitude [rad], shape (M,).
        h_b: True height [m], shape (M,).

    Returns:
        numpy.ndarray: delta_r_eb^n, shape (..., M, 3) [m].

    Examples:
        A 1 µrad latitude error at 45° N is about 6.4 m North; a height
        estimate 10 m too high is a −10 m Down error:

        >>> import numpy as np
        >>> from ins_sim.evaluation.errors import position_error
        >>> L_b = np.deg2rad(45.0)
        >>> delta_r = position_error(L_b + 1e-6, 0.0, 10.0, L_b, 0.0, 0.0)
        >>> [round(float(x), 2) for x in delta_r]
        [6.37, 0.0, -10.0]
    """
    L_b = np.asarray(L_b)
    h_b = np.asarray(h_b)
    delta_L_b = np.asarray(est_L_b) - L_b
    # Wrap the longitude difference to (−π, π] so a track crossing the
    # antimeridian does not show a spurious 2π jump.
    delta_lambda_b = np.mod(np.asarray(est_lambda_b) - lambda_b + np.pi, 2.0 * np.pi) - np.pi
    delta_h_b = np.asarray(est_h_b) - h_b
    R_N, R_E = wgs84_radii(L_b)
    return np.stack([
        (R_N + h_b) * delta_L_b,
        (R_E + h_b) * np.cos(L_b) * delta_lambda_b,
        -delta_h_b,
    ], axis=-1)


def velocity_error(est_v_eb_n: npt.ArrayLike, v_eb_n: npt.ArrayLike) -> np.ndarray:
    """Computes the velocity error delta_v_eb^n.

    Args:
        est_v_eb_n: Estimated velocity resolved in NED, shape (..., M, 3)
            [m/s].
        v_eb_n: True velocity resolved in NED, shape (M, 3) [m/s].

    Returns:
        numpy.ndarray: delta_v_eb^n, shape (..., M, 3) [m/s].
    """
    return np.asarray(est_v_eb_n) - np.asarray(v_eb_n)
