"""Keeps the release build's pinned versions, the notices and the license texts in step.

The downloadable app bundles third-party packages at the exact versions in
.github/release-constraints.txt, and the Python versions pinned in
.github/workflows/release.yml. THIRD-PARTY-NOTICES.txt, which ships with the
app, must name those same versions, and licenses/ must hold the license text
of each one, or the shipped notices would describe a different app.
"""

import re
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONSTRAINTS = ROOT / ".github" / "release-constraints.txt"
RELEASE_WORKFLOW = ROOT / ".github" / "workflows" / "release.yml"
NOTICES = (ROOT / "THIRD-PARTY-NOTICES.txt").read_text(encoding="utf-8")
LICENSES = ROOT / "licenses"

# Qt for Python is covered by licenses/LGPL-3.0.txt, and the notices name it
# as "PySide6 <version>, shiboken6 <version>".
QT_PACKAGES = {"pyside6", "pyside6_essentials", "pyside6_addons", "shiboken6"}

# A row of the notices' table of permissively licensed libraries:
# "  NumPy 1.26.3   BSD 3-Clause   licenses/numpy-1.26.3.LICENSE.txt".
NOTICE_ROW = re.compile(r"^  (\S+) (\d+(?:\.\d+)+) .*\blicenses/(\S+)$", re.MULTILINE)

# Any license file the notices name, such as "licenses/LGPL-3.0.txt".
LICENSE_REFERENCE = re.compile(r"\blicenses/([\w.+-]+\.txt)\b")


def _pins() -> dict[str, tuple[str, str]]:
    """Maps each pinned package's lowercase name to (name, version)."""
    pins = {}
    for line in CONSTRAINTS.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            name, version = (part.strip() for part in line.split("=="))
            pins[name.lower()] = (name, version)
    return pins


PINS = _pins()
# Every package other than Qt for Python has its own license file.
OWN_LICENSE = [pin for key, pin in PINS.items() if key not in QT_PACKAGES]


def test_release_build_pins_the_bundled_packages():
    assert QT_PACKAGES <= set(PINS), "Qt for Python must be pinned"
    assert OWN_LICENSE, "the other bundled packages must be pinned"
    assert "pyinstaller" in PINS, "PyInstaller must be pinned: its launcher ships in the app"


@pytest.mark.parametrize("name, version", list(PINS.values()), ids=list(PINS))
def test_notices_name_the_pinned_version(name, version):
    shown = "PySide6" if name.lower().startswith("pyside6") else name
    assert f"{shown.lower()} {version}" in NOTICES.lower(), (
        f"THIRD-PARTY-NOTICES.txt should name {shown} {version}, the version the release bundles")


@pytest.mark.parametrize("name, version", OWN_LICENSE, ids=[name for name, _ in OWN_LICENSE])
def test_license_text_ships_for_the_pinned_version(name, version):
    path = LICENSES / f"{name}-{version}.LICENSE.txt"
    assert path.is_file(), f"licenses/ should hold {path.name}"


def test_qt_license_texts_ship():
    assert (LICENSES / "LGPL-3.0.txt").is_file()
    assert (LICENSES / "GPL-3.0.txt").is_file()


def test_every_library_in_the_notices_is_pinned_at_that_version():
    rows = NOTICE_ROW.findall(NOTICES)
    assert rows, "the notices should list the permissively licensed libraries"
    for shown, version, license_file in rows:
        pinned = PINS.get(shown.lower())
        assert pinned is not None, f"{shown} is in the notices but not pinned for the release build"
        assert pinned[1] == version, f"{shown}: notices say {version}, release pins {pinned[1]}"
        assert (LICENSES / license_file).is_file(), f"licenses/{license_file} is missing"


def test_every_license_file_the_notices_name_exists():
    named = set(LICENSE_REFERENCE.findall(NOTICES))
    assert named, "the notices should name their license files"
    missing = sorted(name for name in named if not (LICENSES / name).is_file())
    assert not missing, f"the notices name license files that licenses/ lacks: {missing}"


def test_every_license_file_is_named_in_the_notices():
    named = set(LICENSE_REFERENCE.findall(NOTICES))
    unnamed = sorted(path.name for path in LICENSES.glob("*.txt") if path.name not in named)
    assert not unnamed, f"licenses/ holds files the notices do not name: {unnamed}"


def test_notices_name_the_python_each_download_bundles():
    workflow = yaml.safe_load(RELEASE_WORKFLOW.read_text(encoding="utf-8"))
    builds = workflow["jobs"]["build"]["strategy"]["matrix"]["include"]
    versions = {build["os"]: str(build["python"]) for build in builds}
    assert all(re.fullmatch(r"3\.\d+\.\d+", v) for v in versions.values()), (
        f"each release build should pin an exact Python version: {versions}")
    for runner, version in versions.items():
        assert f"Python {version}" in NOTICES, (
            f"THIRD-PARTY-NOTICES.txt should name Python {version}, which the {runner} build bundles")
