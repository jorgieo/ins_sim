"""Closing the window during a run stops the run and exits cleanly (#10).

The window tests start the app offscreen in a subprocess, so the exit code
and Qt's warnings can be checked. All tests are skipped without PySide6.
"""

import os
import subprocess
import sys
import textwrap

import pytest

pytest.importorskip("PySide6.QtWebEngineWidgets")

from ins_sim.gui.workers import SimulationWorker  # noqa: E402

# Starts a 5-trial run at dt = 1 s and closes the window 1.5 s later, during
# the first trial. ANSWERS lists the replies to the "Stop it and quit?"
# question, one per close attempt; a refusal is followed by another close.
_SCRIPT = textwrap.dedent("""
    import sys
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication
    from ins_sim.gui.main_window import MainWindow

    answers = {answers!r}
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    asked = []

    def confirm():
        asked.append(answers[len(asked)])
        if not asked[-1]:
            QTimer.singleShot(500, w.close)          # try again
        return asked[-1]

    w._confirm_stop = confirm
    w.iterations_spinbox.setValue(5)
    w.dt_spinbox.setValue(1.0)
    QTimer.singleShot(0, w.run_button.click)
    QTimer.singleShot(1500, w.close)
    QTimer.singleShot(60_000, lambda: app.exit(3))   # watchdog: never hang CI
    code = app.exec()
    print("asked", len(asked), "result", w.last_result is not None, flush=True)
    sys.exit(code)
""")


def _run_app(answers):
    env = {**os.environ, "QT_QPA_PLATFORM": "offscreen"}
    return subprocess.run(
        [sys.executable, "-c", _SCRIPT.format(answers=answers)],
        capture_output=True, text=True, timeout=120, env=env)


def test_close_during_run_stops_and_exits_cleanly():
    proc = _run_app([True])
    assert proc.returncode == 0, proc.stderr
    assert "QThread" not in proc.stderr
    assert "asked 1 result False" in proc.stdout


def test_refusing_keeps_the_window_open():
    proc = _run_app([False, True])
    assert proc.returncode == 0, proc.stderr
    assert "asked 2 result False" in proc.stdout


def test_cancel_requested_before_run_emits_cancelled():
    from ins_sim.config import default_trajectory_path

    worker = SimulationWorker({"trajectory_path": str(default_trajectory_path()),
                               "dt_s": 1.0, "n_iterations": 2})
    events = []
    worker.simulation_cancelled.connect(lambda: events.append("cancelled"))
    worker.simulation_finished.connect(lambda _r: events.append("finished"))
    worker.error_occurred.connect(events.append)
    worker.request_cancel()
    worker.run()
    assert events == ["cancelled"]
