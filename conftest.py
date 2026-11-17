"""Repository-wide pytest configuration.

`--doctest-modules` (see pyproject.toml) imports every module under
`ins_sim/` to collect doctests. The modules below need PySide6, which is an
optional extra, and `ins_sim/gui/__main__.py` launches the application when
imported, so they are excluded from collection. The GUI's Qt-free modules
(`figures.py`, `paths.py`) are still collected and tested.
"""

collect_ignore = [
    "ins_sim/gui/__main__.py",
    "ins_sim/gui/canvas.py",
    "ins_sim/gui/main_window.py",
    "ins_sim/gui/widgets.py",
    "ins_sim/gui/workers.py",
]
