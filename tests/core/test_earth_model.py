import numpy as np
import pytest

from ins_sim.core.earth_model import (
    WGS84_A, WGS84_E2, WGS84_OMEGA,
    wgs84_radii, earth_rate_n, transport_rate_n, normal_gravity,
)


def test_normal_gravity_equator_sea_level_is_exact_equatorial_constant():
    # At lat=0, h=0 the Somigliana sin^2(phi) term and altitude correction
    # both vanish, leaving the bare equatorial gravity constant.
    assert normal_gravity(0.0, 0.0) == 9.7803253359


def test_normal_gravity_pole_sea_level():
    g_pole = normal_gravity(np.pi / 2, 0.0)
    assert g_pole == pytest.approx(9.832184937859015, rel=1e-9)
    # Gravity increases from equator to pole.
    assert g_pole > normal_gravity(0.0, 0.0)


def test_normal_gravity_decreases_with_altitude():
    lat = np.deg2rad(45.0)
    g0 = normal_gravity(lat, 0.0)
    g1 = normal_gravity(lat, 1000.0)
    assert g1 < g0


def test_wgs84_radii_at_equator():
    R_M, R_N = wgs84_radii(0.0)
    assert R_N == pytest.approx(WGS84_A)
    assert R_M == pytest.approx(WGS84_A * (1.0 - WGS84_E2))


def test_earth_rate_n_equator():
    w = earth_rate_n(0.0)
    assert w[0] == pytest.approx(WGS84_OMEGA)
    assert w[1] == pytest.approx(0.0)
    assert w[2] == pytest.approx(0.0, abs=1e-15)
    assert np.linalg.norm(w) == pytest.approx(WGS84_OMEGA)


def test_earth_rate_n_pole():
    w = earth_rate_n(np.pi / 2)
    assert w[0] == pytest.approx(0.0, abs=1e-15)
    assert w[1] == pytest.approx(0.0)
    assert w[2] == pytest.approx(-WGS84_OMEGA)
    assert np.linalg.norm(w) == pytest.approx(WGS84_OMEGA)


@pytest.mark.parametrize("lat_deg,h", [(0.0, 0.0), (38.97, 100.0), (-45.0, 5000.0)])
def test_transport_rate_n_zero_velocity_is_zero(lat_deg, h):
    lat = np.deg2rad(lat_deg)
    w_en = transport_rate_n(np.zeros(3), lat, h)
    np.testing.assert_array_equal(w_en, np.zeros(3))


def test_transport_rate_n_nonzero_velocity():
    lat = np.deg2rad(38.97)
    h = 100.0
    R_M, R_N = wgs84_radii(lat)
    v_n = np.array([10.0, 5.0, 0.0])
    w_en = transport_rate_n(v_n, lat, h)
    assert w_en[0] == pytest.approx(5.0 / (R_N + h))
    assert w_en[1] == pytest.approx(-10.0 / (R_M + h))
    assert w_en[2] == pytest.approx(-5.0 * np.tan(lat) / (R_N + h))


def test_earth_rate_n_array_matches_scalar_loop():
    lats = np.deg2rad(np.array([0.0, 38.97, -45.0, 89.0]))
    batched = earth_rate_n(lats)
    looped = np.array([earth_rate_n(lat) for lat in lats])
    assert batched.shape == (len(lats), 3)
    np.testing.assert_allclose(batched, looped)


def test_transport_rate_n_array_matches_scalar_loop():
    lats = np.deg2rad(np.array([0.0, 38.97, -45.0, 89.0]))
    alts = np.array([0.0, 100.0, 5000.0, -10.0])
    v_n = np.array([
        [10.0, 5.0, 0.0],
        [0.0, 0.0, 0.0],
        [-20.0, 30.0, 1.0],
        [5.0, -5.0, 2.0],
    ])
    batched = transport_rate_n(v_n, lats, alts)
    looped = np.array([
        transport_rate_n(v_n[k], lats[k], alts[k]) for k in range(len(lats))
    ])
    assert batched.shape == (len(lats), 3)
    np.testing.assert_allclose(batched, looped)
