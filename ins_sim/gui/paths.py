"""Writable user folders for trajectory and IMU-spec YAMLs.

These folders live *next to the running application* — the executable's
directory when frozen by PyInstaller, else the current working directory
when run from source — so a portable copy of the app carries the user's
mission and IMU files alongside it.

Each folder is created on first access and seeded with the packaged
default YAML (copied only if absent, so user edits are never
overwritten). The read-only packaged config folder is thus only the
pristine seed source; the editable defaults the user sees live here.
"""

import shutil
import sys
from importlib.resources import as_file
from pathlib import Path

from ins_sim.config import default_imu_spec_path, default_trajectory_path

TRAJECTORIES_DIRNAME = "trajectories"
IMU_SPECS_DIRNAME = "imu_specs"


def app_base_dir() -> Path:
    """Directory the app treats as 'next to itself'.

    Returns:
        pathlib.Path: The folder containing the executable when frozen
            (PyInstaller onedir), otherwise the current working
            directory.
    """
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path.cwd()


def _ensure_dir(name: str) -> Path:
    """Returns app_base_dir()/name, created if missing.

    Falls back to ``~/ins_sim/<name>`` if the app's own directory is not
    writable (e.g. installed under a read-only location), so startup
    never fails on directory creation.
    """
    try:
        directory = app_base_dir() / name
        directory.mkdir(parents=True, exist_ok=True)
    except OSError:
        directory = Path.home() / "ins_sim" / name
        directory.mkdir(parents=True, exist_ok=True)
    return directory


def _seed(directory: Path, default_resource) -> None:
    """Copies a packaged default YAML into directory if not already present.

    Uses ``importlib.resources.as_file`` so the source resolves to a real
    filesystem path even inside a frozen bundle. Existing files are left
    untouched, so user edits survive across launches. Any OSError is
    swallowed — a missing seed leaves an empty folder, not a crash.
    """
    try:
        with as_file(default_resource) as src:
            dest = directory / src.name
            if not dest.exists():
                shutil.copyfile(src, dest)
    except OSError:
        pass


def trajectories_dir() -> Path:
    """User trajectories folder next to the app (created and seeded)."""
    directory = _ensure_dir(TRAJECTORIES_DIRNAME)
    _seed(directory, default_trajectory_path())
    return directory


def imu_specs_dir() -> Path:
    """User IMU-spec folder next to the app (created and seeded)."""
    directory = _ensure_dir(IMU_SPECS_DIRNAME)
    _seed(directory, default_imu_spec_path())
    return directory
