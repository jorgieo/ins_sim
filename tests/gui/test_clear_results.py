"""The Clear results button returns the window to its start state (#41).

The app runs offscreen in a subprocess: a short run to completion, then a
run cancelled at once (so the progress bar reads "Cancelled"), then Clear.
Skipped without PySide6.
"""

import json
import os
import subprocess
import sys
import textwrap

import pytest

pytest.importorskip("PySide6.QtWebEngineWidgets")

_SCRIPT = textwrap.dedent("""
    import json, sys
    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication
    from ins_sim.gui.main_window import MainWindow

    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    w.iterations_spinbox.setValue(2)
    w.dt_spinbox.setValue(1.0)

    def snapshot():
        tabs = w.visualization_panel
        return {
            "result": w.last_result is not None,
            "tabs": [tabs.tabText(i) for i in range(tabs.count())],
            "log": w.log_view.toPlainText(),
            "progress": [w.progress_bar.value(), w.progress_bar.format()],
            "clear_enabled": w.clear_button.isEnabled(),
        }

    report = {"start": snapshot()}

    def later(fn):
        QTimer.singleShot(100, fn)

    def start_run():
        w.run_button.click()
        report["clear_enabled_running"] = w.clear_button.isEnabled()
        w.log_view.appendPlainText("marker")
        w.clear_button.click()                 # disabled: must do nothing
        w.clear_results()                      # and so must a direct call
        report["cleared_while_running"] = "marker" not in w.log_view.toPlainText()
        later(first_run_done)

    def first_run_done():
        if w.is_running():
            return later(first_run_done)
        w.run_button.click()
        QTest.keyClick(w, Qt.Key.Key_Escape)
        later(cancelled)

    def cancelled():
        if w.is_running():
            return later(cancelled)
        report["before_clear"] = snapshot()
        w.clear_button.click()
        report["after_clear"] = snapshot()
        app.quit()

    QTimer.singleShot(0, start_run)
    QTimer.singleShot(90_000, lambda: app.exit(3))   # watchdog: never hang CI
    code = app.exec()
    print("REPORT", json.dumps(report), flush=True)
    sys.exit(code)
""")


def test_clear_results_restores_the_start_state():
    env = {**os.environ, "QT_QPA_PLATFORM": "offscreen"}
    proc = subprocess.run([sys.executable, "-c", _SCRIPT],
                          capture_output=True, text=True, timeout=180, env=env)
    assert proc.returncode == 0, proc.stderr
    line = next(l for l in proc.stdout.splitlines() if l.startswith("REPORT "))
    report = json.loads(line.removeprefix("REPORT "))

    start = report["start"]
    assert start == {"result": False, "tabs": ["Results"], "log": "",
                     "progress": [0, "%p%"], "clear_enabled": True}
    assert report["clear_enabled_running"] is False
    assert report["cleared_while_running"] is False

    before = report["before_clear"]
    assert before["result"] and len(before["tabs"]) > 1
    assert before["log"] and before["progress"] == [0, "Cancelled"]
    assert report["after_clear"] == start
