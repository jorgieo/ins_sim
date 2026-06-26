import os

import numpy as np
import pytest
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
    truth = TruthTrajectory(path, speed=100.0, dt=0.1)

    interior = slice(20, -20)   # avoid spline-endpoint derivative artifacts
    heading_deg = np.rad2deg(truth.euler[interior, 2])
    np.testing.assert_allclose(heading_deg, 90.0, atol=0.5)

    quat = truth.R_b2n.as_quat()
    assert quat.shape == (len(truth.t), 4)
    # scalar-last: the w (last) component should be near cos(0/2)=1 for this
    # near-level, near-zero-heading-rate... but heading=90 deg here, so check
    # against an explicit Rotation built the same way instead.
    expected = Rot.from_euler('ZYX', np.column_stack([
        truth.euler[interior, 2], truth.euler[interior, 1], truth.euler[interior, 0],
    ])).as_quat()
    np.testing.assert_allclose(quat[interior], expected, atol=1e-9)


def test_truth_trajectory_level_zero_heading_quat_is_near_identity():
    # heading=0 (due north), no turn, no climb -> phi=theta=psi~0 -> quat ~= [0,0,0,1].
    waypoints = [[0.0, 0.0, 0.0], [5000.0, 0.0, 0.0]]
    path = NEDSplinePath(waypoints)
    truth = TruthTrajectory(path, speed=100.0, dt=0.1)
    mid = len(truth.t) // 2
    quat_mid = truth.R_b2n[mid].as_quat()
    np.testing.assert_allclose(quat_mid, [0.0, 0.0, 0.0, 1.0], atol=1e-6)


def test_truth_trajectory_rotations_are_orthonormal():
    waypoints = [[0.0, 0.0, 0.0], [1000.0, 500.0, -50.0], [3000.0, 1500.0, -100.0]]
    path = NEDSplinePath(waypoints)
    truth = TruthTrajectory(path, speed=80.0, dt=0.1)
    R = truth.R_b2n.as_matrix()
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

    expected_attrs = {"t", "dt", "pos_n", "vel_n", "acc_n", "lat", "lon", "alt",
                      "euler", "R_b2n", "omega_b", "f_b", "g_loc"}
    assert expected_attrs.issubset(set(dir(truth)))

    M = len(truth.t)
    assert M > 1
    assert truth.pos_n.shape == (M, 3)
    assert truth.omega_b.shape == (M, 3)
    assert truth.f_b.shape == (M, 3)
    assert np.all(np.diff(truth.t) > 0)
    np.testing.assert_allclose(truth.pos_n[0], [0.0, 0.0, 0.0], atol=1e-6)
    assert np.all(np.isfinite(truth.pos_n))
    assert np.all(np.isfinite(truth.omega_b))
    assert np.all(np.isfinite(truth.f_b))


def test_build_trajectory_overrides_dt():
    truth, *_ = build_trajectory(FIXTURE_YAML, dt=0.25)
    assert truth.dt == 0.25


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
