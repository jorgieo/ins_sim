import numpy as np
from scipy.spatial.transform import Rotation as Rot

from ins_sim.core.earth_model import earth_rate_n, normal_gravity
from ins_sim.navigation.strapdown import strapdown_navgrade
from ins_sim.trajectory.spline import NEDSplinePath
from ins_sim.trajectory.kinematics import TruthTrajectory


def test_stationary_strapdown_is_exactly_stationary():
    """
    A body at rest at fixed (lat0, alt0) with level (identity) attitude:
    the gyro reads the constant Earth rate for that latitude, and the
    accelerometer reads -g (a stationary level IMU's specific force).
    With v=0 throughout, transport_rate_n is exactly zero every step, and
    since lat/alt never move, earth_rate_n(lat) and normal_gravity(lat,alt)
    recomputed each step are bit-identical to the constants fed in. Every
    integrated rate is therefore exactly zero -- forward Euler introduces
    no drift, so velocity, attitude, and position stay exactly constant.
    """
    lat0 = np.deg2rad(38.97)
    lon0 = np.deg2rad(-76.49)
    alt0 = 100.0
    dt = 0.1
    M = 1000

    omega_meas = np.tile(earth_rate_n(lat0), (M, 1))
    g0 = normal_gravity(lat0, alt0)
    f_meas = np.tile(np.array([0.0, 0.0, -g0]), (M, 1))

    init_state = (lat0, lon0, alt0, np.zeros(3), Rot.identity().as_quat())
    pos_ned, lat, lon, alt, vel, quat = strapdown_navgrade(
        omega_meas, f_meas, init_state, dt)

    assert np.all(lat == lat0)
    assert np.all(lon == lon0)
    assert np.all(alt == alt0)
    assert np.all(vel == 0.0)
    np.testing.assert_array_equal(pos_ned, np.zeros((M, 3)))
    np.testing.assert_array_equal(quat, np.tile(quat[0], (M, 1)))


def test_vertical_channel_damped_when_aided_diverges_when_free():
    """
    The vertical channel of a free-inertial navigator is unstable: the
    free-air gravity gradient (dg/dh = -2g/R) makes altitude error obey
    d²(δh)/dt² = (2g/R)·δh, diverging with time constant √(R/2g) ≈ 570 s.
    With baro aiding, the third-order damping loop must instead drive an
    injected error back to zero (triple pole at -1/baro_tau).

    Same stationary setup as above, but with a 1 m/s initial Down
    velocity error. Free-inertial, that error grows to
    δh(t) ≈ δv·τ_s·sinh(t/τ_s) ≈ 1.6 km at t = 1000 s. Baro-aided
    (τ = 100 s), the transient peaks near δv·τ/e ≈ 37 m and both the
    altitude and vertical-velocity errors decay to ~zero within ~10 τ.
    """
    lat0 = np.deg2rad(38.97)
    lon0 = np.deg2rad(-76.49)
    alt0 = 100.0
    dt = 0.5
    M = 2000                       # 1000 s of flight

    omega_meas = np.tile(earth_rate_n(lat0), (M, 1))
    g0 = normal_gravity(lat0, alt0)
    f_meas = np.tile(np.array([0.0, 0.0, -g0]), (M, 1))
    init_state = (lat0, lon0, alt0, np.array([0.0, 0.0, 1.0]),
                  Rot.identity().as_quat())

    *_, alt_free, vel_free, _ = strapdown_navgrade(
        omega_meas, f_meas, init_state, dt)
    *_, alt_aid, vel_aid, _ = strapdown_navgrade(
        omega_meas, f_meas, init_state, dt,
        alt_truth=np.full(M, alt0))

    # Free-inertial: divergence well beyond linear dead-reckoning growth
    assert abs(alt_free[-1] - alt0) > 1000.0

    # Baro-damped: bounded transient, then decay of both states
    aid_err = np.abs(alt_aid - alt0)
    assert aid_err.max() < 60.0
    assert aid_err[-1] < 1.0
    assert abs(vel_aid[-1, 2]) < 0.05
    assert abs(alt_free[-1] - alt0) > 20.0 * aid_err.max()


def test_strapdown_zero_noise_self_consistency():
    """
    Feeding a TruthTrajectory's own (noiseless) omega_b/f_b back into
    strapdown_navgrade with altitude aiding should reproduce the truth
    position bit-exactly. TruthTrajectory derives omega_b/f_b using the
    same exact-discrete forward-difference formulas (rotation-vector
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
    truth = TruthTrajectory.from_spline(path, speed=100.0, dt=0.1)

    init_state = (truth.lat[0], truth.lon[0], truth.alt[0],
                 truth.vel_n[0].copy(), truth.R_b2n[0].as_quat())
    pos_ned, *_ = strapdown_navgrade(
        truth.omega_b, truth.f_b, init_state, truth.dt, alt_truth=truth.alt)

    err = np.linalg.norm(pos_ned - truth.pos_n, axis=1)
    assert err.max() < 1e-6
