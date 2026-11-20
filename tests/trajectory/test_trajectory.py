import os

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation as Rot

from ins_sim.trajectory.spline import NEDSplinePath
from ins_sim.trajectory.kinematics import (
    TruthTrajectory, build_trajectory,
    _ground_roll, _geodetic_bearing, _geodetic_distance,
)

FIXTURE_YAML = os.path.join(os.path.dirname(__file__), "..", "fixtures", "tiny_phases.yaml")


# =========================================================================
# NEDSplinePath
# =========================================================================
def test_spline_hits_waypoints_at_their_arc_length_knots():
    waypoints = np.array([
        [0.0, 0.0, 0.0],
        [100.0, 0.0, 0.0],
        [100.0, 100.0, -10.0],
    ])
    path = NEDSplinePath(waypoints)
    for s_knot, wp in zip(path.s_knots, waypoints):
        np.testing.assert_allclose(path.position(s_knot), wp, atol=1e-9)


def test_spline_length_matches_sum_of_chord_segments():
    waypoints = np.array([
        [0.0, 0.0, 0.0],
        [3.0, 4.0, 0.0],      # chord length 5
        [3.0, 4.0, -12.0],    # chord length 12
    ])
    path = NEDSplinePath(waypoints)
    assert path.length == pytest.approx(17.0)


def test_spline_clips_out_of_range_arc_length():
    waypoints = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    path = NEDSplinePath(waypoints)
    np.testing.assert_allclose(path.position(-5.0), path.position(0.0))
    np.testing.assert_allclose(path.position(path.length + 5.0), path.position(path.length))


# =========================================================================
# TruthTrajectory
# =========================================================================
def test_truth_trajectory_straight_line_heading_and_quaternion_convention():
    # Straight line due east: heading should be constant ~90 deg, and at
    # zero bank/pitch the body->NED rotation should be (near) identity, so
    # quat ~= [0, 0, 0, 1] -- scalar-last [x, y, z, w] convention.
    waypoints = [[0.0, 0.0, 0.0], [0.0, 5000.0, 0.0]]
    path = NEDSplinePath(waypoints)
    truth = TruthTrajectory.from_spline(path, speed=100.0, tau_i=0.1)

    interior = slice(20, -20)   # avoid spline-endpoint derivative artifacts
    heading_deg = np.rad2deg(truth.euler_nb[interior, 2])
    np.testing.assert_allclose(heading_deg, 90.0, atol=0.5)

    quat = truth.C_b_n.as_quat()
    assert quat.shape == (len(truth.t), 4)
    # scalar-last: the w (last) component should be near cos(0/2)=1 for this
    # near-level, near-zero-heading-rate... but heading=90 deg here, so check
    # against an explicit Rotation built the same way instead.
    expected = Rot.from_euler('ZYX', np.column_stack([
        truth.euler_nb[interior, 2], truth.euler_nb[interior, 1], truth.euler_nb[interior, 0],
    ])).as_quat()
    np.testing.assert_allclose(quat[interior], expected, atol=1e-9)


def test_truth_trajectory_level_zero_heading_quat_is_near_identity():
    # heading=0 (due north), no turn, no climb -> phi=theta=psi~0 -> quat ~= [0,0,0,1].
    waypoints = [[0.0, 0.0, 0.0], [5000.0, 0.0, 0.0]]
    path = NEDSplinePath(waypoints)
    truth = TruthTrajectory.from_spline(path, speed=100.0, tau_i=0.1)
    mid = len(truth.t) // 2
    quat_mid = truth.C_b_n[mid].as_quat()
    np.testing.assert_allclose(quat_mid, [0.0, 0.0, 0.0, 1.0], atol=1e-6)


def test_truth_trajectory_rotations_are_orthonormal():
    waypoints = [[0.0, 0.0, 0.0], [1000.0, 500.0, -50.0], [3000.0, 1500.0, -100.0]]
    path = NEDSplinePath(waypoints)
    truth = TruthTrajectory.from_spline(path, speed=80.0, tau_i=0.1)
    R = truth.C_b_n.as_matrix()
    should_be_identity = np.einsum('kij,kil->kjl', R, R)
    eye = np.tile(np.eye(3), (len(truth.t), 1, 1))
    np.testing.assert_allclose(should_be_identity, eye, atol=1e-9)
    dets = np.linalg.det(R)
    np.testing.assert_allclose(dets, 1.0, atol=1e-9)


# =========================================================================
# build_trajectory (YAML-driven)
# =========================================================================
def test_build_trajectory_fixture_yaml_has_expected_attributes_and_shape():
    truth, v_sprint, R_turn = build_trajectory(FIXTURE_YAML)

    expected_attrs = {"t", "tau_i", "r_lb_l", "v_eb_n", "v_eb_n_dot", "L_b",
                      "lambda_b", "h_b", "euler_nb", "C_b_n", "omega_ib_b",
                      "f_ib_b", "g"}
    assert expected_attrs.issubset(set(dir(truth)))

    M = len(truth.t)
    assert M > 1
    assert truth.r_lb_l.shape == (M, 3)
    assert truth.omega_ib_b.shape == (M, 3)
    assert truth.f_ib_b.shape == (M, 3)
    assert np.all(np.diff(truth.t) > 0)
    np.testing.assert_allclose(truth.r_lb_l[0], [0.0, 0.0, 0.0], atol=1e-6)
    assert np.all(np.isfinite(truth.r_lb_l))
    assert np.all(np.isfinite(truth.omega_ib_b))
    assert np.all(np.isfinite(truth.f_ib_b))


def test_build_trajectory_overrides_dt():
    truth, *_ = build_trajectory(FIXTURE_YAML, tau_i=0.25)
    assert truth.tau_i == 0.25


def test_oversampled_truth_keeps_the_imu_interval_and_whole_intervals():
    truth, *_ = build_trajectory(FIXTURE_YAML, tau_i=0.5, oversample=5)
    fine, *_ = build_trajectory(FIXTURE_YAML, tau_i=0.1)
    assert truth.tau_i == 0.5 and truth.oversample == 5
    np.testing.assert_allclose(np.diff(truth.t), 0.5)
    assert len(truth.t) == (len(fine.t) - 1) // 5 + 1
    for name in ("r_lb_l", "v_eb_n", "omega_ib_b", "f_ib_b", "euler_nb"):
        assert getattr(truth, name).shape[0] == len(truth.t)
    # The states are the fine profile's, every fifth sample.
    np.testing.assert_array_equal(truth.v_eb_n, fine.v_eb_n[::5][:len(truth.t)])


def test_oversampled_outputs_are_interval_averages_for_steady_flight():
    """In straight, level, unaccelerated flight the rates barely change, so
    the averaged outputs equal the instantaneous ones (the transport rate
    drifts by about 1e-9 rad/s per second as the latitude changes)."""
    path = NEDSplinePath([[0.0, 0.0, -1000.0], [10000.0, 0.0, -1000.0]])
    exact = TruthTrajectory.from_spline(path, speed=200.0, tau_i=0.5)
    averaged = TruthTrajectory.from_spline(path, speed=200.0, tau_i=0.5, oversample=10)
    n = min(len(exact.t), len(averaged.t)) - 2
    np.testing.assert_allclose(averaged.f_ib_b[1:n], exact.f_ib_b[1:n], atol=1e-6)
    np.testing.assert_allclose(averaged.omega_ib_b[1:n], exact.omega_ib_b[1:n], atol=1e-9)


def test_gamma_is_the_mean_rotation_over_the_interval():
    from ins_sim.trajectory.kinematics import _gamma_apply
    psi = np.array([[0.3, -0.2, 0.5], [1e-6, 0.0, 2e-6]])
    x = np.array([[1.0, 2.0, 3.0], [0.5, -1.0, 2.0]])
    s = (np.arange(4000) + 0.5) / 4000                 # midpoint rule over [0, 1]
    for p, v in zip(psi, x):
        mean = Rot.from_rotvec(s[:, None] * p).apply(v).mean(axis=0)
        np.testing.assert_allclose(_gamma_apply(p[None], v[None])[0], mean, atol=1e-7)


FT = 0.3048


def _mission(tmp_path, phases):
    """Writes a mission YAML that departs like the fixture and flies `phases`."""
    with open(FIXTURE_YAML) as fh:
        cfg = yaml.safe_load(fh)
    cfg["phases"] = phases
    path = tmp_path / "mission.yaml"
    path.write_text(yaml.safe_dump(cfg))
    return str(path)


ROLL_AND_TAKEOFF = [
    {"type": "ground_roll", "heading_deg": 90.0, "run_length_m": 200.0,
     "speed_final_kt": 100.0},
    {"type": "takeoff", "heading_deg": 90.0, "pitch_deg": 5.0,
     "speed_kt": 120.0, "duration_s": 5.0},
    {"type": "climb", "pitch_deg": 8.0, "speed_kt": 150.0,
     "to_altitude_ft": 3000.0},
]


def test_waypoint_leg_descends_to_the_waypoint_altitude(tmp_path):
    path = _mission(tmp_path, ROLL_AND_TAKEOFF + [
        {"type": "waypoint", "lat_deg": 18.50, "lon_deg": -66.95,
         "alt_ft": 1000.0, "speed_kt": 150.0}])
    truth, *_ = build_trajectory(path, tau_i=0.5)
    assert truth.h_b.max() == pytest.approx(3000.0 * FT, abs=20.0)
    assert truth.h_b[-1] == pytest.approx(1000.0 * FT, abs=20.0)
    assert np.all(truth.v_eb_n[-50:, 2] > 0.0)      # still descending at the end


@pytest.mark.parametrize("pitch_deg", [5.0, -5.0])
def test_climb_phase_descends_to_a_lower_target(tmp_path, pitch_deg):
    path = _mission(tmp_path, ROLL_AND_TAKEOFF + [
        {"type": "climb", "pitch_deg": pitch_deg, "speed_kt": 150.0,
         "to_altitude_ft": 1500.0}])
    truth, *_ = build_trajectory(path, tau_i=0.5)
    assert truth.h_b[-1] == pytest.approx(1500.0 * FT, abs=20.0)


def test_first_phase_must_be_a_ground_roll(tmp_path):
    path = _mission(tmp_path, ROLL_AND_TAKEOFF[1:])
    with pytest.raises(ValueError, match="first phase must be a 'ground_roll'"):
        build_trajectory(path, tau_i=0.5)


def test_only_the_first_phase_can_be_a_ground_roll(tmp_path):
    path = _mission(tmp_path, ROLL_AND_TAKEOFF + ROLL_AND_TAKEOFF[:1])
    with pytest.raises(ValueError, match="Only the first phase"):
        build_trajectory(path, tau_i=0.5)


def test_mach_speed_ramp_needs_an_altitude(tmp_path):
    path = _mission(tmp_path, ROLL_AND_TAKEOFF[:2] + [
        {"type": "speed_ramp", "speed_end_mach": 0.3, "duration_s": 10.0}])
    with pytest.raises(ValueError, match="speed_end_mach needs an earlier"):
        build_trajectory(path, tau_i=0.5)


# =========================================================================
# Maneuver helpers
# =========================================================================
def test_ground_roll_reaches_target_speed_and_distance():
    pos, vel = _ground_roll(hdg_deg=0.0, v_final=50.0, run_len=500.0, dt=0.05)
    assert vel[-1, 0] == pytest.approx(50.0, rel=1e-2)
    assert pos[-1, 0] == pytest.approx(500.0, rel=1e-2)
    np.testing.assert_allclose(pos[:, 1], 0.0)


def test_geodetic_bearing_known_directions():
    assert _geodetic_bearing(0.0, 0.0, 1.0, 0.0) == pytest.approx(0.0, abs=1e-6)    # due north
    assert _geodetic_bearing(0.0, 0.0, 0.0, 1.0) == pytest.approx(90.0, abs=1e-6)   # due east


def test_geodetic_distance_one_degree_latitude():
    # _geodetic_distance is a haversine on a sphere of radius WGS84_A (not
    # the true ellipsoidal meridian arc), so the expected value is the
    # spherical great-circle distance for 1 deg of arc at that radius.
    from ins_sim.core.earth_model import WGS84_A
    dist = _geodetic_distance(0.0, 0.0, 1.0, 0.0)
    assert dist == pytest.approx(2 * np.pi * WGS84_A / 360.0, rel=1e-6)
