"""Smooth paths through waypoints, for building test and example trajectories.

`NEDSplinePath` interpolates waypoints given in metres along North, East
and Down from a start point; `TruthTrajectory.from_spline` flies such a
path at constant speed.
"""

import numpy as np
import numpy.typing as npt
from scipy.interpolate import CubicSpline


class NEDSplinePath:
    """C2-continuous cubic spline through NED waypoints.

    The spline is parameterized by cumulative chord length so that
    position(s) is evaluated at arc-length s along the path rather than
    by waypoint index.

    Attributes:
        s_knots (numpy.ndarray): Cumulative chord length at each waypoint
            [m], shape (N,).
        length (float): Total chord length of the path [m].
        cs (scipy.interpolate.CubicSpline): Underlying cubic spline,
            mapping arc-length s to NED position.

    Examples:
        >>> from ins_sim.trajectory.spline import NEDSplinePath
        >>> path = NEDSplinePath([[0.0, 0.0, 0.0], [3.0, 4.0, 0.0], [3.0, 4.0, -12.0]])
        >>> float(path.length)                  # 5 m + 12 m of chords
        17.0
    """
    def __init__(self, waypoints: npt.ArrayLike, bc_type: str = "not-a-knot"):
        """Initializes the spline from a sequence of NED waypoints.

        Args:
            waypoints: NED waypoints [m] to interpolate, shape (N, 3).
            bc_type: Boundary condition passed to
                scipy.interpolate.CubicSpline. Defaults to "not-a-knot".
        """
        pts = np.asarray(waypoints, dtype=float)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.s_knots = np.r_[0.0, np.cumsum(seg)]
        self.length  = float(self.s_knots[-1])
        self.cs      = CubicSpline(self.s_knots, pts, bc_type=bc_type)

    def position(self, s: npt.ArrayLike) -> np.ndarray:
        """Evaluates NED position at arc-length(s) along the path.

        Args:
            s: Arc-length parameter(s) [m], clipped to [0, self.length].

        Returns:
            numpy.ndarray: NED position(s) [m] at the requested
                arc-length(s).
        """
        return self.cs(np.clip(s, 0.0, self.length))
