"""Version and notice information shown by the GUI's About dialog."""

import sys
from importlib import metadata
from pathlib import Path


def version_for(distribution: str) -> str:
    """Return an installed distribution's version, or a clear fallback."""
    try:
        return metadata.version(distribution)
    except metadata.PackageNotFoundError:
        return "not installed"


def component_versions(qt_version: str | None = None) -> list[tuple[str, str]]:
    """Return versions without importing optional or heavyweight packages."""
    versions = [
        ("ins_sim", version_for("ins_sim")),
        ("Python", sys.version.split()[0]),
        ("PySide6", version_for("PySide6")),
        ("NumPy", version_for("numpy")),
        ("SciPy", version_for("scipy")),
        ("Plotly", version_for("plotly")),
    ]
    if qt_version is not None:
        versions.insert(3, ("Qt", qt_version))
    return versions


def notices_path() -> Path:
    """Find the notice in a source checkout or PyInstaller one-folder build."""
    if getattr(sys, "frozen", False):
        bundle_dir = Path(sys._MEIPASS)  # PyInstaller's read-only data root
        return bundle_dir / "THIRD-PARTY-NOTICES.txt"
    return Path(__file__).resolve().parents[2] / "THIRD-PARTY-NOTICES.txt"


def third_party_notices() -> str:
    """Read the notices while keeping the dialog usable if data is missing."""
    try:
        return notices_path().read_text(encoding="utf-8")
    except OSError as exc:
        return f"Third-party notices could not be loaded: {exc}"
