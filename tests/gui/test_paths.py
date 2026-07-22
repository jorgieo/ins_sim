"""Unit tests for the writable user-folder helpers (ins_sim.gui.paths).

Qt-free: ``ins_sim.gui.paths`` only depends on ``ins_sim.config`` and the
standard library, so no QApplication or event loop is needed. ``app_base_dir``
resolves to the current working directory when not frozen, so each test runs
inside a fresh ``tmp_path`` via ``monkeypatch.chdir``.
"""

from ins_sim.gui import paths


def test_app_base_dir_is_cwd_when_not_frozen(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert paths.app_base_dir() == tmp_path


def test_dirs_are_created_distinct_and_named(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    traj = paths.trajectories_dir()
    imu = paths.imu_specs_dir()

    assert traj == tmp_path / paths.TRAJECTORIES_DIRNAME
    assert imu == tmp_path / paths.IMU_SPECS_DIRNAME
    assert traj.is_dir() and imu.is_dir()
    assert traj != imu


def test_dirs_are_seeded_with_packaged_defaults(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    traj = paths.trajectories_dir()
    imu = paths.imu_specs_dir()

    assert (traj / "bqn_departure.yaml").is_file()
    assert (imu / "imu_spec.yaml").is_file()


def test_seeding_never_overwrites_user_edits(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    traj = paths.trajectories_dir()
    edited = traj / "bqn_departure.yaml"
    edited.write_text("phases: [] # my edit\n", encoding="utf-8")

    # A second call must not re-seed over the edited file.
    assert paths.trajectories_dir() == traj
    assert edited.read_text(encoding="utf-8") == "phases: [] # my edit\n"
