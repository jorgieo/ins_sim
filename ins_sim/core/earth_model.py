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
    """
    Radii of curvature at geodetic latitude `lat` [rad].

    R_M (meridian)        — used for north-south motion: dφ = v_N / (R_M + h) dt
    R_N (prime vertical)  — used for east-west motion:   dλ = v_E / ((R_N + h) cos φ) dt
    """
    sphi2 = np.sin(lat) ** 2
    den   = np.sqrt(1.0 - WGS84_E2 * sphi2)
    R_N   = WGS84_A / den                                     # transverse radius
    R_M   = WGS84_A * (1.0 - WGS84_E2) / (den ** 3)           # meridional radius
    return R_M, R_N


def earth_rate_n(lat):
    """Earth rotation rate ω_ie expressed in the local NED frame."""
    return np.array([WGS84_OMEGA * np.cos(lat), 0.0, -WGS84_OMEGA * np.sin(lat)])


def transport_rate_n(v_n, lat, h):
    """
    Rotation rate of the NED frame relative to ECEF (a.k.a. craft rate),
    expressed in NED. Comes from the vehicle moving over the curved Earth.
    """
    R_M, R_N = wgs84_radii(lat)
    vN, vE, _ = v_n
    return np.array([
         vE / (R_N + h),
        -vN / (R_M + h),
        -vE * np.tan(lat) / (R_N + h),
    ])


def normal_gravity(lat, h):
    """
    Somigliana formula for surface gravity plus a free-air altitude
    correction. Accurate to ~1 µg (10⁻⁸ m/s²) below 10 km, which is
    well below navigation-grade accelerometer bias.
    """
    g_e = 9.7803253359          # equatorial gravity
    k   = 1.93185265241e-3      # Somigliana ratio constant
    sphi2 = np.sin(lat) ** 2
    g0  = g_e * (1.0 + k * sphi2) / np.sqrt(1.0 - WGS84_E2 * sphi2)
    # Linear free-air correction: dg/dh ≈ −2g/a near the surface
    return g0 * (1.0 - 2.0 * h / WGS84_A)
