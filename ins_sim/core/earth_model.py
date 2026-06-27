"""
WGS-84 Earth model.

Frames: ECI inertial (i), ECEF Earth-fixed (e), NED navigation (n), body (b).
Position in geodetic (φ, λ, h), velocity in NED.
Gravity: Somigliana normal gravity + free-air altitude correction.
Local gravity vector g_n = [0, 0, +g(φ, h)].
"""

import numpy as np

# Defining constants of the WGS-84 ellipsoid.
WGS84_A   = 6378137.0                 # Semi-major axis [m]
WGS84_F   = 1.0 / 298.257223563       # Flattening
WGS84_E2  = WGS84_F * (2.0 - WGS84_F) # First eccentricity squared
WGS84_OMEGA = 7.2921151467e-5         # Earth rotation rate [rad/s]


def wgs84_radii(lat):
    """Computes the WGS-84 radii of curvature at a given geodetic latitude.

    Args:
        lat: Geodetic latitude φ [rad].

    Returns:
        tuple[float, float]: (R_M, R_N) where:
            R_M: Meridian radius of curvature [m], used for north-south
                motion: dφ = v_N / (R_M + h) · dt.
            R_N: Prime-vertical radius of curvature [m], used for
                east-west motion: dλ = v_E / ((R_N + h) · cos φ) · dt.
    """
    sphi2 = np.sin(lat) ** 2
    den   = np.sqrt(1.0 - WGS84_E2 * sphi2)
    R_N   = WGS84_A / den                                     # transverse radius
    R_M   = WGS84_A * (1.0 - WGS84_E2) / (den ** 3)           # meridional radius
    return R_M, R_N


def earth_rate_n(lat):
    """Computes Earth's rotation rate expressed in the local NED frame.

    Args:
        lat: Geodetic latitude φ [rad].

    Returns:
        numpy.ndarray: ω_ie_n, shape (3,), Earth rotation rate vector
            [rad/s] resolved in the NED navigation frame.
    """
    return np.array([WGS84_OMEGA * np.cos(lat), 0.0, -WGS84_OMEGA * np.sin(lat)])


def transport_rate_n(v_n, lat, h):
    """Computes the transport rate (craft rate) of the NED frame relative to ECEF.

    Rotation rate of the local-level NED frame caused by the vehicle's
    motion over the curved Earth, expressed in NED.

    Args:
        v_n: NED velocity [v_N, v_E, v_D], shape (3,) [m/s].
        lat: Geodetic latitude φ [rad].
        h: Geodetic altitude [m].

    Returns:
        numpy.ndarray: ω_en_n, shape (3,), transport rate vector [rad/s]
            resolved in the NED navigation frame.
    """
    R_M, R_N = wgs84_radii(lat)
    vN, vE, _ = v_n
    return np.array([
         vE / (R_N + h),
        -vN / (R_M + h),
        -vE * np.tan(lat) / (R_N + h),
    ])


def normal_gravity(lat, h):
    """Computes normal gravity via the Somigliana formula with a free-air correction.

    Args:
        lat: Geodetic latitude φ [rad].
        h: Geodetic altitude [m].

    Returns:
        float: Local gravity magnitude g(φ, h) [m/s²]. Accurate to ~1 µg
            (10⁻⁸ m/s²) below 10 km altitude, well below navigation-grade
            accelerometer bias.
    """
    g_e = 9.7803253359          # equatorial gravity
    k   = 1.93185265241e-3      # Somigliana ratio constant
    sphi2 = np.sin(lat) ** 2
    g0  = g_e * (1.0 + k * sphi2) / np.sqrt(1.0 - WGS84_E2 * sphi2)
    # Linear free-air correction: dg/dh ≈ −2g/a near the surface
    return g0 * (1.0 - 2.0 * h / WGS84_A)
