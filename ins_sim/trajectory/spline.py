import numpy as np
from scipy.interpolate import CubicSpline


class NEDSplinePath:
    """C2 cubic spline through NED waypoints, parameterized by chord length."""
    def __init__(self, waypoints, bc_type="not-a-knot"):
        pts = np.asarray(waypoints, dtype=float)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.s_knots = np.r_[0.0, np.cumsum(seg)]
        self.length  = float(self.s_knots[-1])
        self.cs      = CubicSpline(self.s_knots, pts, bc_type=bc_type)

    def position(self, s):
        return self.cs(np.clip(s, 0.0, self.length))
