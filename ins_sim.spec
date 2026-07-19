# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller spec for the INS Monte Carlo Simulator GUI.

Build with:  pyinstaller ins_sim.spec --noconfirm

Produces a one-folder bundle at dist/ins_sim/ (onedir is the reliable
mode for QtWebEngine apps — no per-launch self-extraction).

Notes:
- The packaged mission/IMU YAMLs are bundled at ins_sim/config/ so the
  app's importlib.resources lookups resolve identically when frozen.
- PySide6/QtWebEngine (Chromium runtime, plugins, resources) and plotly
  (plotly.js and package data) are handled by PyInstaller's bundled
  hooks; nothing manual is needed for them.
- matplotlib/seaborn are CLI-only (main.py's summary figure) and are
  excluded to keep the GUI bundle lean — the GUI renders exclusively
  through plotly. The frozen smoke test guards this assumption.
"""

from pathlib import Path

config_dir = Path("ins_sim") / "config"
config_datas = [(str(p), "ins_sim/config") for p in config_dir.glob("*.yaml")]

a = Analysis(
    ["launcher.py"],
    pathex=[],
    binaries=[],
    datas=config_datas,
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # matplotlib/seaborn are CLI-only; the rest are optional integrations
    # (plotly/IPython/pandas soft imports) that would bloat the bundle —
    # and PyQt5/PyQt6 must never be collected alongside PySide6.
    excludes=["matplotlib", "seaborn", "tkinter",
              "PyQt5", "PyQt6", "IPython", "pandas", "PIL"],
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
    upx=False,
    console=False,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    name="ins_sim",
)
