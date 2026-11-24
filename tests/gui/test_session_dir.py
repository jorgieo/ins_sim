"""Tests for the per-session plot folders and the removal of abandoned ones."""

import os
import time

from ins_sim.gui import session_dir
from ins_sim.gui.session_dir import (LOCK_NAME, SESSION_PREFIX, SessionDir,
                                     remove_abandoned)

HOUR = 3600.0


def _make_old(path, age_s=2 * HOUR):
    stamp = time.time() - age_s
    os.utime(path, (stamp, stamp))


def _legacy_folder(parent, name="ins_sim_ab12cd_4", files=("plotly.min.js", "cep.html")):
    folder = parent / name
    folder.mkdir()
    for f in files:
        (folder / f).write_text("x")
    return folder


def _crashed_session(parent):
    """A session folder whose process died: lock released, folder still there."""
    session = SessionDir(parent)
    (session.path / "cep.html").write_text("x")
    session_dir._unlock(session._lock)
    session._lock.close()
    session._lock = None
    return session.path


def test_session_dir_is_locked_then_removed_on_close(tmp_path):
    session = SessionDir(tmp_path)
    assert session.path.name.startswith(SESSION_PREFIX)
    assert (session.path / LOCK_NAME).is_file()
    session.close()
    session.close()                                  # second call is a no-op
    assert not session.path.exists()


def test_running_session_is_kept(tmp_path):
    with SessionDir(tmp_path) as session:
        _make_old(session.path)
        assert remove_abandoned(tmp_path, min_age_s=0) == []
        assert session.path.exists()


def test_crashed_session_is_removed(tmp_path):
    folder = _crashed_session(tmp_path)
    assert remove_abandoned(tmp_path) == [folder]
    assert not folder.exists()


def test_old_legacy_folder_is_removed(tmp_path):
    folder = _legacy_folder(tmp_path)
    _make_old(folder)
    assert remove_abandoned(tmp_path) == [folder]


def test_recent_lockless_folders_are_kept(tmp_path):
    _legacy_folder(tmp_path)
    (tmp_path / f"{SESSION_PREFIX}abc").mkdir()      # lock not written yet
    assert remove_abandoned(tmp_path) == []


def test_old_lockless_session_folder_is_removed(tmp_path):
    folder = tmp_path / f"{SESSION_PREFIX}abc"
    folder.mkdir()
    _make_old(folder)
    assert remove_abandoned(tmp_path) == [folder]


def test_unrecognised_folders_and_files_are_kept(tmp_path):
    keep = [
        _legacy_folder(tmp_path, files=("cep.html", "notes.txt")),  # foreign file
        _legacy_folder(tmp_path, name="ins_sim_screenshots_abc12345"),
        _legacy_folder(tmp_path, name="other_ab12cd_4"),
    ]
    stray_file = tmp_path / "ins_sim_abcdefgh"
    stray_file.write_text("x")
    for path in [*keep, stray_file]:
        _make_old(path)
    assert remove_abandoned(tmp_path) == []
    assert all(p.exists() for p in [*keep, stray_file])


def test_missing_parent_is_not_an_error(tmp_path):
    assert remove_abandoned(tmp_path / "missing") == []
