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
- The mission/IMU YAMLs are bundled at ins_sim/config/ so the app's
  importlib.resources lookups resolve identically when frozen (onedir
  places them under _internal/, which is on sys.path).
- The release smoke test (INS_SIM_SMOKE=1) runs a real Monte Carlo in
  the frozen bundle and requires the plotly tabs to render — it guards
  every assumption above on each release build.
"""

from pathlib import Path

# Anchor to the spec's own directory (SPECPATH is provided by
# PyInstaller) so builds work from any cwd, and fail at build time —
# not at first launch — if the packaged configs are missing.
spec_dir = Path(SPECPATH)
config_files = sorted((spec_dir / "ins_sim" / "config").glob("*.yaml"))
if not config_files:
    raise SystemExit("ins_sim.spec: no YAML configs found in ins_sim/config/")
config_datas = [(str(p), "ins_sim/config") for p in config_files]

a = Analysis(
    [str(spec_dir / "launcher.py")],
    pathex=[],
    binaries=[],
    datas=config_datas,
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # matplotlib/seaborn are CLI-only; IPython/pandas/PIL are plotly
    # soft imports; pytest/jupyter live in the CI build env but must
    # never ship. PyQt5/PyQt6 must never be collected alongside PySide6.
    excludes=[
        "matplotlib", "seaborn", "tkinter",
        "PyQt5", "PyQt6",
        "IPython", "jupyter", "notebook", "pandas", "PIL",
        "pytest", "_pytest", "py",
    ],
    noarchive=False,
)

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
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    name="ins_sim",
)
