"""Desktop GUI for the INS Monte Carlo simulator (requires the ``gui`` extra).

The Qt-dependent submodules (main_window, canvas, widgets, workers)
import PySide6 at module level, so they need the ``gui`` extra; the core
``ins_sim`` library, ``figures`` and ``paths`` do not.
Launch with ``python -m ins_sim.gui``.
"""
