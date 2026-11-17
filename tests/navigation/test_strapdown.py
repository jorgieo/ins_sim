import numpy as np
from scipy.spatial.transform import Rotation as Rot

from ins_sim.core.earth_model import earth_rate_n, normal_gravity
from ins_sim.navigation.strapdown import NavSolution, NavState, strapdown_navgrade
from ins_sim.trajectory.spline import NEDSplinePath
from ins_sim.trajectory.kinematics import TruthTrajectory


def test_stationary_strapdown_is_exactly_stationary():
    """
    A body at rest at fixed (L_b0, h_b0) with level (identity) attitude:
    the gyro reads the constant Earth rate for that latitude, and the
    accelerometer reads -g (a stationary level IMU's specific force).
    With v_eb^n = 0 throughout, transport_rate_n is exactly zero every
    step, and since L_b/h_b never move, earth_rate_n(L_b) and
    normal_gravity(L_b, h_b) recomputed each step are bit-identical to
    the constants fed in. Every integrated rate is therefore exactly
    zero -- forward Euler introduces no drift, so velocity, attitude,
    and position stay exactly constant.
    """
    L_b0 = np.deg2rad(38.97)
    lambda_b0 = np.deg2rad(-76.49)
    h_b0 = 100.0
    tau_i = 0.1
    M = 1000

    meas_omega_ib_b = np.tile(earth_rate_n(L_b0), (M, 1))
    g0 = normal_gravity(L_b0, h_b0)
    meas_f_ib_b = np.tile(np.array([0.0, 0.0, -g0]), (M, 1))

    init_state = NavState(L_b0, lambda_b0, h_b0, np.zeros(3), Rot.identity().as_quat())
    nav = strapdown_navgrade(meas_omega_ib_b, meas_f_ib_b, init_state, tau_i)

    assert isinstance(nav, NavSolution)
    assert np.all(nav.L_b == L_b0)
    assert np.all(nav.lambda_b == lambda_b0)
    assert np.all(nav.h_b == h_b0)
    assert np.all(nav.v_eb_n == 0.0)
    np.testing.assert_array_equal(nav.r_lb_l, np.zeros((M, 3)))
    np.testing.assert_array_equal(nav.q_b_n, np.tile(nav.q_b_n[0], (M, 1)))


def test_vertical_channel_damped_when_aided_diverges_when_free():
    """
    The vertical channel of a free-inertial navigator is unstable: the
    free-air gravity gradient (dg/dh = -2g/R) makes the height error obey
    d²(δh)/dt² = (2g/R)·δh, diverging with time constant √(R/2g) ≈ 570 s.
    With baro aiding, the third-order damping loop must instead drive an
    injected error back to zero (triple pole at -1/baro_tau).

    Same stationary setup as above, but with a 1 m/s initial Down
    velocity error. Free-inertial, that error grows to
    δh(t) ≈ δv·τ_s·sinh(t/τ_s) ≈ 1.6 km at t = 1000 s. Baro-aided
    (τ = 100 s), the transient peaks near δv·τ/e ≈ 37 m and both the
    height and vertical-velocity errors decay to ~zero within ~10 τ.
    """
    L_b0 = np.deg2rad(38.97)
    lambda_b0 = np.deg2rad(-76.49)
    h_b0 = 100.0
    tau_i = 0.5
    M = 2000                       # 1000 s of flight

    meas_omega_ib_b = np.tile(earth_rate_n(L_b0), (M, 1))
    g0 = normal_gravity(L_b0, h_b0)
    meas_f_ib_b = np.tile(np.array([0.0, 0.0, -g0]), (M, 1))
    init_state = NavState(L_b0, lambda_b0, h_b0, np.array([0.0, 0.0, 1.0]),
                          Rot.identity().as_quat())

    free = strapdown_navgrade(meas_omega_ib_b, meas_f_ib_b, init_state, tau_i)
    aided = strapdown_navgrade(meas_omega_ib_b, meas_f_ib_b, init_state, tau_i,
                               meas_h_b=np.full(M, h_b0))

    # Free-inertial: divergence well beyond linear dead-reckoning growth
    assert abs(free.h_b[-1] - h_b0) > 1000.0

    # Baro-damped: bounded transient, then decay of both states
    aided_err = np.abs(aided.h_b - h_b0)
    assert aided_err.max() < 60.0
    assert aided_err[-1] < 1.0
    assert abs(aided.v_eb_n[-1, 2]) < 0.05
    assert abs(free.h_b[-1] - h_b0) > 20.0 * aided_err.max()


def test_strapdown_zero_noise_self_consistency():
    """
    Feeding a TruthTrajectory's own (noiseless) omega_ib_b/f_ib_b back into
    strapdown_navgrade with height aiding should reproduce the truth
    position bit-exactly. TruthTrajectory derives omega_ib_b/f_ib_b using
    the same exact-discrete forward-difference formulas (rotation-vector
    exponential for attitude, forward-Euler for velocity/position) that
    strapdown_navgrade's recursion implements, for both the spline-based
    (`from_spline`) and YAML-phase-based (`build_trajectory`) construction
    paths -- so there is no structural truth/integrator mismatch to bound
    against; any nonzero residual here would indicate a real bug.
    """
    waypoints = [
        [0.0, 0.0, 0.0],
        [1000.0, 500.0, -50.0],
        [3000.0, 1500.0, -100.0],
        [5000.0, 2000.0, -100.0],
    ]
    path = NEDSplinePath(waypoints)
    truth = TruthTrajectory.from_spline(path, speed=100.0, tau_i=0.1)

    init_state = NavState(truth.L_b[0], truth.lambda_b[0], truth.h_b[0],
                          truth.v_eb_n[0].copy(), truth.C_b_n[0].as_quat())
    nav = strapdown_navgrade(
        truth.omega_ib_b, truth.f_ib_b, init_state, truth.tau_i, meas_h_b=truth.h_b)

    err = np.linalg.norm(nav.r_lb_l - truth.r_lb_l, axis=1)
    assert err.max() < 1e-6


def test_init_state_accepts_a_plain_tuple():
    """NavState is a NamedTuple, so callers may still pass a plain 5-tuple."""
    L_b0 = np.deg2rad(10.0)
    meas_omega_ib_b = np.tile(earth_rate_n(L_b0), (5, 1))
    meas_f_ib_b = np.tile([0.0, 0.0, -normal_gravity(L_b0, 0.0)], (5, 1))
    as_tuple = (L_b0, 0.0, 0.0, np.zeros(3), Rot.identity().as_quat())
    a = strapdown_navgrade(meas_omega_ib_b, meas_f_ib_b, as_tuple, 0.1)
    b = strapdown_navgrade(meas_omega_ib_b, meas_f_ib_b, NavState(*as_tuple), 0.1)
    for x, y in zip(a, b):
        np.testing.assert_array_equal(x, y)
