"""
WGS-84 Earth model.

Frames: ECI inertial (i), ECEF Earth-fixed (e), NED navigation (n), body (b).
Position is curvilinear: geodetic latitude L_b, longitude lambda_b and
geodetic height h_b. Velocity v_eb^n is resolved in NED.
Gravity: Somigliana normal gravity + free-air height correction, with the
local gravity vector g_b^n = [0, 0, +g(L_b, h_b)] (Down positive).

Symbols follow Groves (2013); see [Notation & conventions][notation-conventions].
"""

from typing import NamedTuple

import numpy as np
import numpy.typing as npt

# Defining constants of the WGS-84 ellipsoid.
WGS84_A   = 6378137.0                 # Semi-major axis R_0 [m]
WGS84_F   = 1.0 / 298.257223563       # Flattening f
WGS84_E2  = WGS84_F * (2.0 - WGS84_F) # First eccentricity squared e^2
WGS84_OMEGA = 7.2921151467e-5         # Earth rotation rate omega_ie [rad/s]


class Radii(NamedTuple):
    """Radii of curvature of the WGS-84 ellipsoid at a given latitude.

    Attributes:
        R_N: Meridian radius of curvature [m], which governs north-south
            motion: dL_b/dt = v_N / (R_N + h_b).
        R_E: Transverse (prime-vertical) radius of curvature [m], which
            governs east-west motion: dlambda_b/dt = v_E / ((R_E + h_b)·cos L_b).
    """

    R_N: float | np.ndarray
    R_E: float | np.ndarray


def wgs84_radii(L_b: npt.ArrayLike) -> Radii:
    """Computes the WGS-84 radii of curvature at a given geodetic latitude.

    Args:
        L_b: Geodetic latitude, scalar or shape (M,) [rad].

    Returns:
        Radii: (R_N, R_E), the meridian and transverse radii of curvature
            [m], each with the shape of `L_b`. Unpacks positionally as
            ``R_N, R_E = wgs84_radii(L_b)``.

    Examples:
        At the equator the transverse radius is the semi-major axis and
        the meridian radius is at its smallest:

        >>> from ins_sim.core.earth_model import wgs84_radii
        >>> R_N, R_E = wgs84_radii(0.0)
        >>> round(float(R_N)), round(float(R_E))
        (6335439, 6378137)
    """
    # --8<-- [start:radii]
    sin2_L = np.sin(L_b) ** 2
    den    = np.sqrt(1.0 - WGS84_E2 * sin2_L)
    R_E    = WGS84_A / den                                    # transverse radius
    R_N    = WGS84_A * (1.0 - WGS84_E2) / (den ** 3)          # meridian radius
    # --8<-- [end:radii]
    return Radii(R_N, R_E)


def earth_rate_n(L_b: npt.ArrayLike) -> np.ndarray:
    """Computes Earth's rotation rate resolved in the local NED frame.

    Args:
        L_b: Geodetic latitude, scalar or shape (M,) [rad].

    Returns:
        numpy.ndarray: omega_ie^n, the Earth rotation rate vector [rad/s]
            resolved in NED. Shape (3,) for scalar `L_b`, shape (M, 3)
            for `L_b` of shape (M,).

    Examples:
        At 45° N the Earth's 15.04 °/h rotation splits equally between
        the North and (negative) Down axes:

        >>> import numpy as np
        >>> from ins_sim.core.earth_model import earth_rate_n
        >>> omega_ie_n = earth_rate_n(np.deg2rad(45.0))
        >>> [round(float(x), 3) for x in np.rad2deg(omega_ie_n) * 3600]   # °/h
        [10.636, 0.0, -10.636]
    """
    L_b = np.asarray(L_b)
    zeros = np.zeros_like(L_b)
    # --8<-- [start:earth_rate]
    return np.stack(
        [WGS84_OMEGA * np.cos(L_b), zeros, -WGS84_OMEGA * np.sin(L_b)], axis=-1)
    # --8<-- [end:earth_rate]


def transport_rate_n(v_eb_n: npt.ArrayLike, L_b: npt.ArrayLike,
                     h_b: npt.ArrayLike) -> np.ndarray:
    """Computes the transport rate of the NED frame relative to the Earth frame.

    The rotation rate of the local navigation frame caused by the
    vehicle's motion over the curved Earth, resolved in NED.

    Args:
        v_eb_n: Velocity of the body relative to the Earth, resolved in
            NED [v_N, v_E, v_D]. Shape (3,) or (M, 3) [m/s].
        L_b: Geodetic latitude, scalar or shape (M,) [rad].
        h_b: Geodetic height, scalar or shape (M,) [m].

    Returns:
        numpy.ndarray: omega_en^n, the transport rate vector [rad/s]
            resolved in NED. Shape (3,) for scalar inputs, shape (M, 3)
            for inputs of shape (M, 3)/(M,)/(M,).

    Examples:
        Flying due north at 250 m/s turns the local NED frame about its
        East axis (negative sign: the frame pitches nose-down to stay level):

        >>> import numpy as np
        >>> from ins_sim.core.earth_model import transport_rate_n
        >>> omega_en_n = transport_rate_n([250.0, 0.0, 0.0], np.deg2rad(45.0), 4572.0)
        >>> [round(float(x), 3) for x in np.rad2deg(omega_en_n) * 3600]   # °/h
        [0.0, -8.09, 0.0]
    """
    v_eb_n = np.asarray(v_eb_n)
    L_b = np.asarray(L_b)
    h_b = np.asarray(h_b)
    R_N, R_E = wgs84_radii(L_b)
    v_N, v_E = v_eb_n[..., 0], v_eb_n[..., 1]
    # --8<-- [start:transport_rate]
    return np.stack([
         v_E / (R_E + h_b),
        -v_N / (R_N + h_b),
        -v_E * np.tan(L_b) / (R_E + h_b),
    ], axis=-1)
    # --8<-- [end:transport_rate]


def normal_gravity(L_b: npt.ArrayLike, h_b: npt.ArrayLike) -> float | np.ndarray:
    """Computes normal gravity via the Somigliana formula with a free-air correction.

    Args:
        L_b: Geodetic latitude, scalar or shape (M,) [rad].
        h_b: Geodetic height, scalar or shape (M,) [m].

    Returns:
        float | numpy.ndarray: Local gravity magnitude g(L_b, h_b)
            [m/s²], with the shape of the inputs.

    Examples:
        >>> import numpy as np
        >>> from ins_sim.core.earth_model import normal_gravity
        >>> round(float(normal_gravity(0.0, 0.0)), 7)       # equator, sea level
        9.7803253
        >>> round(float(normal_gravity(np.deg2rad(45.0), 10000.0)), 4)
        9.7754
    """
    g_e = 9.7803253359          # equatorial gravity
    k   = 1.93185265241e-3      # Somigliana ratio constant
    # --8<-- [start:gravity]
    sin2_L = np.sin(L_b) ** 2
    g0  = g_e * (1.0 + k * sin2_L) / np.sqrt(1.0 - WGS84_E2 * sin2_L)
    # Linear free-air correction: dg/dh ≈ −2g/R_0 near the surface
    return g0 * (1.0 - 2.0 * h_b / WGS84_A)
    # --8<-- [end:gravity]
