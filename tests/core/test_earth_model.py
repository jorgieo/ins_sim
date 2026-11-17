import numpy as np
import pytest

from ins_sim.core.earth_model import (
    WGS84_A, WGS84_E2, WGS84_OMEGA,
    wgs84_radii, earth_rate_n, transport_rate_n, normal_gravity,
)


def test_normal_gravity_equator_sea_level_is_exact_equatorial_constant():
    # At L_b=0, h_b=0 the Somigliana sin^2(L_b) term and the height
    # correction both vanish, leaving the bare equatorial gravity constant.
    assert normal_gravity(0.0, 0.0) == 9.7803253359


def test_normal_gravity_pole_sea_level():
    g_pole = normal_gravity(np.pi / 2, 0.0)
    assert g_pole == pytest.approx(9.832184937859015, rel=1e-9)
    # Gravity increases from equator to pole.
    assert g_pole > normal_gravity(0.0, 0.0)


def test_normal_gravity_decreases_with_height():
    L_b = np.deg2rad(45.0)
    g0 = normal_gravity(L_b, 0.0)
    g1 = normal_gravity(L_b, 1000.0)
    assert g1 < g0


def test_wgs84_radii_at_equator():
    # Groves notation: R_N is the meridian radius, R_E the transverse one.
    R_N, R_E = wgs84_radii(0.0)
    assert R_E == pytest.approx(WGS84_A)
    assert R_N == pytest.approx(WGS84_A * (1.0 - WGS84_E2))


def test_wgs84_radii_named_fields_are_meridian_and_transverse():
    radii = wgs84_radii(np.deg2rad(45.0))
    assert radii.R_N == radii[0]
    assert radii.R_E == radii[1]
    # Away from the poles the meridian radius is the smaller of the two.
    assert radii.R_N < radii.R_E


def test_earth_rate_n_equator():
    omega_ie_n = earth_rate_n(0.0)
    assert omega_ie_n[0] == pytest.approx(WGS84_OMEGA)
    assert omega_ie_n[1] == pytest.approx(0.0)
    assert omega_ie_n[2] == pytest.approx(0.0, abs=1e-15)
    assert np.linalg.norm(omega_ie_n) == pytest.approx(WGS84_OMEGA)


def test_earth_rate_n_pole():
    omega_ie_n = earth_rate_n(np.pi / 2)
    assert omega_ie_n[0] == pytest.approx(0.0, abs=1e-15)
    assert omega_ie_n[1] == pytest.approx(0.0)
    assert omega_ie_n[2] == pytest.approx(-WGS84_OMEGA)
    assert np.linalg.norm(omega_ie_n) == pytest.approx(WGS84_OMEGA)


@pytest.mark.parametrize("L_b_deg,h_b", [(0.0, 0.0), (38.97, 100.0), (-45.0, 5000.0)])
def test_transport_rate_n_zero_velocity_is_zero(L_b_deg, h_b):
    L_b = np.deg2rad(L_b_deg)
    omega_en_n = transport_rate_n(np.zeros(3), L_b, h_b)
    np.testing.assert_array_equal(omega_en_n, np.zeros(3))


def test_transport_rate_n_nonzero_velocity():
    L_b = np.deg2rad(38.97)
    h_b = 100.0
    R_N, R_E = wgs84_radii(L_b)
    v_eb_n = np.array([10.0, 5.0, 0.0])
    omega_en_n = transport_rate_n(v_eb_n, L_b, h_b)
    assert omega_en_n[0] == pytest.approx(5.0 / (R_E + h_b))
    assert omega_en_n[1] == pytest.approx(-10.0 / (R_N + h_b))
    assert omega_en_n[2] == pytest.approx(-5.0 * np.tan(L_b) / (R_E + h_b))


def test_earth_rate_n_array_matches_scalar_loop():
    L_b = np.deg2rad(np.array([0.0, 38.97, -45.0, 89.0]))
    batched = earth_rate_n(L_b)
    looped = np.array([earth_rate_n(L) for L in L_b])
    assert batched.shape == (len(L_b), 3)
    np.testing.assert_allclose(batched, looped)


def test_transport_rate_n_array_matches_scalar_loop():
    L_b = np.deg2rad(np.array([0.0, 38.97, -45.0, 89.0]))
    h_b = np.array([0.0, 100.0, 5000.0, -10.0])
    v_eb_n = np.array([
        [10.0, 5.0, 0.0],
        [0.0, 0.0, 0.0],
        [-20.0, 30.0, 1.0],
        [5.0, -5.0, 2.0],
    ])
    batched = transport_rate_n(v_eb_n, L_b, h_b)
    looped = np.array([
        transport_rate_n(v_eb_n[k], L_b[k], h_b[k]) for k in range(len(L_b))
    ])
    assert batched.shape == (len(L_b), 3)
    np.testing.assert_allclose(batched, looped)
