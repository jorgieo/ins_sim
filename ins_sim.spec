# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller spec for the INS Monte Carlo Simulator GUI.

Build with:  pyinstaller ins_sim.spec --noconfirm

Produces a one-folder bundle at dist/ins_sim/ (onedir is the reliable
mode for QtWebEngine apps — no per-launch self-extraction).

Architecture invariants (why this spec looks the way it does):
- The GUI renders exclusively through plotly HTML in QWebEngineView
  pages; matplotlib/seaborn are CLI-only (main.py's summary figure)
  and stay excluded. Do NOT add matplotlib.backends.* hiddenimports:
  nothing in the frozen import graph uses them, and Qt-backend probing
  can drag PyQt5 in alongside PySide6 (a hard PyInstaller error).
- All core engines (evaluation.monte_carlo, navigation, sensors,
  trajectory, and their scipy submodules) are imported statically — no
  dynamic import machinery — so Analysis discovers them without
  hiddenimports. scipy must never be excluded.
- PySide6/QtWebEngine (Chromium runtime, platform plugins qwindows/
  qoffscreen/qxcb/wayland, resources) and plotly package data are
  handled by PyInstaller's bundled hooks; binaries=[] stays empty.
  The Qt Quick, QML and Positioning bindings are excluded, and unused
  Qt plugins filtered out, so the bundle holds only the Qt the app uses
  (see the comments at excludes and _unused_qt_plugin).
- On Linux the bundle holds only the Python runtime and the pinned
  packages: the system libraries PyInstaller would copy from the build
  machine are left out, and the app uses the user's own, as other
  desktop apps do (see _system_library).
- The mission/IMU YAMLs are bundled at ins_sim/config/ so the app's
  importlib.resources lookups resolve identically when frozen (onedir
  places them under _internal/, which is on sys.path).
- The release smoke test (INS_SIM_SMOKE=1) runs a real Monte Carlo in
  the frozen bundle and requires Qt WebEngine to draw a plotly figure —
  it guards every assumption above on each release build.
- THIRD-PARTY-NOTICES.txt must credit every compiled file the bundle
  holds: after the build, tools/check_bundle.py fails on any file that
  .github/bundle-inventory.txt does not list.
"""

import sys
from pathlib import Path

# Anchor to the spec's own directory (SPECPATH is provided by
# PyInstaller) so builds work from any cwd, and fail at build time —
# not at first launch — if the packaged configs are missing.
spec_dir = Path(SPECPATH)
config_files = sorted((spec_dir / "ins_sim" / "config").glob("*.yaml"))
if not config_files:
    raise SystemExit("ins_sim.spec: no YAML configs found in ins_sim/config/")
config_datas = [(str(p), "ins_sim/config") for p in config_files]

# LGPL v3 compliance: the bundle redistributes PySide6/Qt (LGPL-3.0), so the
# THIRD-PARTY-NOTICES and the full third-party license texts must travel with
# every release. Ship the notice at the bundle root and the texts under
# _internal/licenses/. Fail the build if either is missing rather than emit a
# non-compliant binary.
notice_file = spec_dir / "THIRD-PARTY-NOTICES.txt"
license_files = sorted((spec_dir / "licenses").glob("*.txt"))
if not notice_file.exists() or not license_files:
    raise SystemExit(
        "ins_sim.spec: THIRD-PARTY-NOTICES.txt and licenses/*.txt are required "
        "for LGPL compliance but were not found"
    )
license_datas = [(str(notice_file), ".")]
license_datas += [(str(p), "licenses") for p in license_files]
icon_files = sorted((spec_dir / "ins_sim" / "gui" / "assets").glob("*"))
icon_datas = [(str(p), "ins_sim/gui/assets") for p in icon_files if p.suffix in {".svg", ".png", ".ico"}]

a = Analysis(
    [str(spec_dir / "launcher.py")],
    pathex=[],
    binaries=[],
    datas=config_datas + license_datas + icon_datas,
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # matplotlib/seaborn are CLI-only; IPython/pandas/PIL are plotly
    # soft imports; pytest/jupyter live in the CI build env but must
    # never ship. PyQt5/PyQt6 must never be collected alongside PySide6.
    # readline serves interactive prompts, which the app never shows (on
    # Linux it would also need the system's GNU readline, GPL-3.0).
    #
    # The app imports only the Qt Core, Gui, Widgets and WebEngineWidgets
    # bindings (which load Network, PrintSupport, WebChannel and
    # WebEngineCore). PyInstaller would also add the Qt Quick, QML and
    # Positioning bindings, because Qt WebEngine links against those
    # libraries, and then collect every QML plugin and every Qt module
    # they use, some of which Qt licenses only under the GPL-3.0. Leaving
    # the bindings out keeps the libraries WebEngine links against and
    # drops the rest.
    excludes=[
        "matplotlib", "seaborn", "tkinter",
        "PyQt5", "PyQt6",
        "PySide6.QtQml", "PySide6.QtQuick", "PySide6.QtQuickWidgets",
        "PySide6.QtPositioning",
        "readline",
        "IPython", "jupyter", "notebook", "pandas", "PIL",
        "pytest", "_pytest", "py",
    ],
    noarchive=False,
)


def _unused_qt_plugin(dest_name):
    """Qt plugins the app never loads, with the files only they need.

    - The TLS backends, and the OpenSSL copy PyInstaller finds for them on
      PATH: the app makes no TLS connections through Qt (Qt WebEngine has
      its own TLS).
    - The virtual keyboard input method, which Qt licenses only under the
      GPL-3.0.
    """
    dest = dest_name.replace("\\", "/").lower()
    return ("/plugins/tls/" in dest
            or "virtualkeyboard" in dest
            or dest.split("/")[-1] in ("libssl-3-x64.dll", "libcrypto-3-x64.dll"))


a.binaries = [entry for entry in a.binaries if not _unused_qt_plugin(entry[0])]


def _system_library(entry):
    """Whether a collected Linux binary is a library from the build machine.

    PyInstaller copies every shared library the bundled code links against,
    including the system's graphics, font, sound, D-Bus, NSS and C++
    libraries, and puts them at the top of the _internal folder. Nothing
    else lands there as a plain binary except the Python runtime itself:
    the libraries the wheels include stay in their packages' folders, with
    symbolic links at the top.
    """
    dest_name, _source, typecode = entry
    return (typecode == "BINARY" and "/" not in dest_name.replace("\\", "/")
            and not dest_name.startswith("libpython"))


# On Linux, leave the build machine's system libraries out: the app uses the
# user's own, as other desktop apps do, so the bundle holds only the Python
# runtime and the pinned packages that THIRD-PARTY-NOTICES.txt credits. (On
# Windows, the system's DLLs are never collected.)
if sys.platform.startswith("linux"):
    system_libraries = sorted(entry[0] for entry in a.binaries if _system_library(entry))
    print(f"ins_sim.spec: leaving out {len(system_libraries)} system libraries: "
          + " ".join(system_libraries))
    a.binaries = [entry for entry in a.binaries if not _system_library(entry)]

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="ins_sim",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,      # UPX corrupts Qt/Chromium DLLs — keep off
    console=False,  # windowed on Windows; inert on Linux. The smoke
                    # test's contract is the exit code, not stdout.
    icon=str(spec_dir / "ins_sim" / "gui" / "assets" / "app.ico") if sys.platform == "win32" else None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    name="ins_sim",
)
